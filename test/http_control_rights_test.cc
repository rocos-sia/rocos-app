#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "src/robot_http_server.hpp"
#include "src/robot_info_json.hpp"
#include "test/doctest.h"

namespace rocos {

// 只启动 HTTP 服务，不创建 Robot；调整时间避免测试等待真实 TTL。
struct RobotHttpServerTestAccess {
    static int start(RobotHttpServer& server) {
        const int port = server.server_->bind_to_any_port("127.0.0.1");
        REQUIRE(port > 0);
        server.thread_ = std::make_unique<std::thread>([&server]() { server.server_->listen_after_bind(); });
        server.server_->wait_until_ready();
        return port;
    }

    static auto age(RobotHttpServer& server, int seconds) {
        std::lock_guard<std::mutex> lock(server.control_mutex_);
        server.last_seen_at_ = std::chrono::steady_clock::now() - std::chrono::seconds(seconds);
        return server.last_seen_at_;
    }

    static auto lastSeen(RobotHttpServer& server) {
        std::lock_guard<std::mutex> lock(server.control_mutex_);
        return server.last_seen_at_;
    }
};

}  // namespace rocos

namespace {

using Json = nlohmann::json;
using Access = rocos::RobotHttpServerTestAccess;

struct ControlFixture {
    rocos::RobotHttpServer server{nullptr, nullptr, 9};
    httplib::Client client{"127.0.0.1", Access::start(server)};
    httplib::Headers credentials;
    Json owner;

    ControlFixture() {
        client.set_read_timeout(2);
        owner = post("/api/control/acquire", {}, R"({"client_id":"tab-a"})")["data"];
        credentials = {{"X-Rocos-Control-Token", owner["token"].get<std::string>()}, {"X-Rocos-Client-Id", "tab-a"}};
    }

    Json post(const std::string& path, const httplib::Headers& headers = {}, const std::string& body = "{}") {
        auto response = client.Post(path, headers, body, "application/json");
        REQUIRE(response);
        CHECK(response->status == 200);
        return Json::parse(response->body);
    }

    Json get(const std::string& path) {
        auto response = client.Get(path, credentials);
        REQUIRE(response);
        CHECK(response->status == 200);
        return Json::parse(response->body);
    }
};

}  // namespace

TEST_CASE_FIXTURE(ControlFixture, "Robot info baseline is public and never renews control") {
    const auto before = Access::age(server, 3);
    auto response = client.Get("/api/robot/info");
    REQUIRE(response);
    CHECK(response->status == 200);
    const auto reply = Json::parse(response->body);
    CHECK(reply["success"] == true);
    const auto& data = reply["data"];
    CHECK(data["info_schema_version"] == 1);
    CHECK(data["controller"]["app_version"] == ROCOS_APP_VERSION);
    CHECK(data["controller"]["shm_abi_version"] == rocos::SHM_ABI_VERSION);
    CHECK(data["controller"]["capabilities"] == Json::array({"robot_info_v1"}));
    CHECK(data["robot"]["source"] == "placeholder");
    CHECK(data["robot"]["serial_number"].is_null());
    CHECK(data["master"]["source"] == "placeholder");
    CHECK(data["master"]["backend"] == "unknown");
    CHECK(data["master"]["version"].is_null());
    CHECK(data["slaves_complete"] == false);
    CHECK(data["slaves"] == Json::array());
    CHECK(data["joint_infos"] == Json::array());
    CHECK(Access::lastSeen(server) == before);
}

TEST_CASE("Robot info retains legacy joint values and marks configured slave identities unknown") {
    rocos::Robot::JointInfo joint;
    joint.id = 4;
    joint.name = "joint_2";
    joint.ratio = -100;
    joint.cnt_per_unit = 123.5;
    joint.torque_per_unit = 0.2;
    joint.zero_offset = -42;
    joint.unit_name = "rad";
    const auto info = rocos::robotInfoJson({joint});
    CHECK(info["joint_infos"][0] == Json{{"id", 4}, {"name", "joint_2"}, {"ratio", -100.0},
        {"cnt_per_unit", 123.5}, {"torque_per_unit", 0.2}, {"zero_offset", -42}, {"unit_name", "rad"}});
    const auto& slave = info["slaves"][0];
    CHECK(slave["id"] == 4);
    CHECK(slave["joint_name"] == "joint_2");
    CHECK(slave["source"] == "configuration");
    CHECK(slave["identity_source"] == "unknown");
    for (const auto* field : {"vendor_id", "product_code", "revision_number", "serial_number",
                              "firmware_version", "input_pdo_count", "output_pdo_count"}) {
        CHECK(slave.at(field).is_null());
    }
    CHECK_FALSE(slave.contains("online"));
}

TEST_CASE_FIXTURE(ControlFixture, "Control heartbeat renews owner and returns configured timing") {
    CHECK(owner["client_id_auto"] == false);
    CHECK(owner["ttl_seconds"] == 9);
    CHECK(owner["heartbeat_interval_seconds"] == 3.0);
    const auto before = Access::age(server, 6);
    auto reply = post("/api/control/heartbeat", credentials, "");
    CHECK(reply["success"] == true);
    CHECK(reply["code"] == 0);
    CHECK(reply["data"]["has_owner"] == true);
    CHECK(reply["data"]["ttl_seconds"] == 9);
    CHECK(reply["data"]["heartbeat_interval_seconds"] == 3.0);
    CHECK(reply["data"]["expires_in_seconds"] == 9);
    CHECK_FALSE(reply["data"].contains("token"));
    CHECK(Access::lastSeen(server) > before);
    CHECK(post("/api/control/acquire")["code"] == 3008);
}

TEST_CASE_FIXTURE(ControlFixture, "Control requests require both headers without renewing on rejection") {
    for (const auto* path : {"/api/control/heartbeat", "/api/control/release", "/api/script/run"}) {
        CAPTURE(path);
        const auto before = Access::age(server, 3);
        CHECK(post(path)["code"] == 3006);
        auto headers = credentials;
        headers.erase("X-Rocos-Client-Id");
        CHECK(post(path, headers, R"({"client_id":"tab-a"})")["code"] == 3007);
        headers.emplace("X-Rocos-Client-Id", "tab-b");
        CHECK(post(path, headers)["code"] == 3007);
        headers = credentials;
        headers.erase("X-Rocos-Control-Token");
        headers.emplace("X-Rocos-Control-Token", "invalid-token");
        CHECK(post(path, headers)["code"] == 3007);
        CHECK(Access::lastSeen(server) == before);
    }
    const httplib::Headers debug{{"X-Rocos-Control-Token", "debug-token"}, {"X-Rocos-Client-Id", "debug-client"}};
    const auto before = Access::lastSeen(server);
    CHECK(post("/api/control/heartbeat", debug)["code"] == 3007);
    CHECK(post("/api/control/release", debug)["code"] == 3007);
    CHECK(Access::lastSeen(server) == before);
    CHECK(post("/api/control/release", credentials)["success"] == true);
    CHECK(get("/api/control/status")["data"]["has_owner"] == false);
}

TEST_CASE_FIXTURE(ControlFixture, "GET does not renew but authenticated writes do") {
    const auto before = Access::age(server, 3);
    CHECK(get("/api/control/status")["data"]["has_owner"] == true);
    CHECK_FALSE(get("/api/control/status")["data"].contains("token"));
    get("/api/script/status");
    CHECK(Access::lastSeen(server) == before);
    // 无 Lua 引擎时业务失败，但已通过鉴权的写请求仍续期。
    CHECK(post("/api/script/run", credentials)["code"] == static_cast<int>(rocos::Result::FunctionNotSupported));
    CHECK(Access::lastSeen(server) > before);
}

TEST_CASE_FIXTURE(ControlFixture, "Expired control cannot be revived or released") {
    SUBCASE("heartbeat") {
        Access::age(server, 9);
        CHECK(post("/api/control/heartbeat", credentials)["code"] == 3007);
    }
    SUBCASE("release") {
        Access::age(server, 9);
        CHECK(post("/api/control/release", credentials)["code"] == 3007);
    }
    SUBCASE("write") {
        Access::age(server, 9);
        CHECK(post("/api/script/run", credentials)["code"] == 3007);
    }
    SUBCASE("status lazily expires owner") { Access::age(server, 9); }
    CHECK(get("/api/control/status")["data"]["has_owner"] == false);
    CHECK(post("/api/control/heartbeat", credentials)["code"] == 3007);
    CHECK(post("/api/control/acquire")["success"] == true);
}

TEST_CASE_FIXTURE(ControlFixture, "Takeover or acquisition after expiry invalidates old credentials") {
    Json replacement;
    SUBCASE("explicit takeover") { replacement = post("/api/control/takeover"); }
    SUBCASE("acquire after expiry") {
        Access::age(server, 9);
        replacement = post("/api/control/acquire");
    }
    REQUIRE(replacement["success"] == true);
    CHECK(replacement["data"]["token"] != owner["token"]);
    CHECK(replacement["data"]["client_id_auto"] == true);
    CHECK_FALSE(replacement["data"]["client_id"].get<std::string>().empty());
    CHECK(post("/api/control/heartbeat", credentials)["code"] == 3007);
    CHECK(post("/api/control/release", credentials)["code"] == 3007);
    credentials = {{"X-Rocos-Control-Token", replacement["data"]["token"].get<std::string>()},
                   {"X-Rocos-Client-Id", replacement["data"]["client_id"].get<std::string>()}};
    CHECK(post("/api/control/heartbeat", credentials)["success"] == true);
}

TEST_CASE("Control TTL defaults and short TTL heartbeat interval") {
    int ttl = 300;
    SUBCASE("default") {}
    SUBCASE("one second") { ttl = 1; }
    auto server = ttl == 300 ? std::make_unique<rocos::RobotHttpServer>(nullptr)
                             : std::make_unique<rocos::RobotHttpServer>(nullptr, nullptr, ttl);
    httplib::Client client("127.0.0.1", Access::start(*server));
    client.set_read_timeout(2);
    const auto response = client.Post("/api/control/acquire", "{}", "application/json");
    REQUIRE(response);
    const auto data = Json::parse(response->body)["data"];
    CHECK(data["ttl_seconds"] == ttl);
    const double interval = data["heartbeat_interval_seconds"];
    CHECK(interval > 0);
    CHECK(interval <= ttl / 3.0);
    if (ttl == 300) CHECK(interval == 5.0);
}

TEST_CASE("Nonpositive control TTL is rejected before starting workers") {
    CHECK_THROWS_AS(rocos::RobotHttpServer(nullptr, nullptr, 0), std::invalid_argument);
    CHECK_THROWS_AS(rocos::RobotHttpServer(nullptr, nullptr, -1), std::invalid_argument);
}
