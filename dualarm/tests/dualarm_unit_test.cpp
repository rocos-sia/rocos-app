#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <limits>

#include "api_values.hpp"
#include "arm_api_context.hpp"
#include "doctest.h"
#include "model_bundle.hpp"
using rocos::dualarm::ControlLease;
namespace {
using Json = nlohmann::json;
Json Command(const std::string& id, uint64_t epoch = 1) {
  return {{"command_id", id},
          {"instance_id", "app-A"},
          {"motion_epoch", epoch},
          {"joints", {0, 0, 0, 0, 0, 0, 0}}};
}
Json Feedback(double timestamp, bool busy = false) {
  return {{"timestamp", timestamp},
          {"motion_busy", busy},
          {"robot_state", busy ? "RUNNING" : "STOPPED"},
          {"joint_states", {{{"velocity", 0.0}}}}};
}
}  // namespace

TEST_CASE("stop fences delayed motion and each arm has its own epoch") {
  using rocos::dualarm::MotionCommands;
  MotionCommands left, right;
  const auto delayed = Command("delayed");
  CHECK(left.Validate(delayed, "app-A", true, true));
  left.Stop();
  CHECK_THROWS(left.Validate(delayed, "app-A", true, true));
  CHECK(right.Validate(delayed, "app-A", true, true));
  CHECK(left.Validate(Command("next", 2), "app-A", true, true));
  CHECK_THROWS(left.Validate(Command("old-app", 2), "app-B", true, true));
  CHECK_THROWS(left.Validate(Json::object(), "app-A", true, true));
  CHECK_FALSE(left.Validate(Json::object(), "app-A", false, true));
}

TEST_CASE("command IDs deduplicate dispatch and enforce owner and body") {
  rocos::dualarm::MotionCommands commands;
  const auto body = Command("once");
  REQUIRE(commands.Begin(body, "movej", "left-token"));
  commands.Finish("once", true, 0, "accepted");
  CHECK_FALSE(commands.Begin(body, "movej", "left-token"));
  auto changed = body;
  changed["joints"][0] = 0.1;
  CHECK_THROWS(commands.Begin(changed, "movej", "left-token"));
  CHECK_THROWS(commands.Begin(body, "movel_fk", "left-token"));
  CHECK_THROWS(commands.Read("once", "right-token"));
  CHECK_THROWS(commands.Read("missing", "left-token"));
  CHECK(commands.Read("once", "left-token")["status"] == "accepted");
}

TEST_CASE("fresh sequences finish commands across wall-clock rollback") {
  rocos::dualarm::MotionCommands commands;
  REQUIRE(commands.Begin(Command("motion"), "movej", "owner"));
  commands.Observe(Feedback(1000));  // Idle while planning cannot finish it.
  CHECK(commands.Read("motion", "owner")["status"] == "planning");
  commands.Finish("motion", true, 0, "accepted");
  commands.Observe(Feedback(1001, true));
  CHECK(commands.Read("motion", "owner")["status"] == "running");
  for (double timestamp : {10., 11., 12.})
    commands.Observe(Feedback(timestamp));
  CHECK(commands.Read("motion", "owner")["status"] == "finished");
  CHECK(commands.Metadata("app-A")["state_seq"] == 5);
  REQUIRE(commands.Begin(Command("stopped"), "movej", "owner"));
  commands.Finish("stopped", true, 0, "accepted");
  commands.Stop();
  CHECK(commands.Read("stopped", "owner")["status"] == "stopping");
  for (int sample = 0; sample < 3; ++sample) commands.Observe(Feedback(1));
  CHECK(commands.Read("stopped", "owner")["status"] == "stopped");
}

TEST_CASE("bounded command history never permits replay of an evicted ID") {
  rocos::dualarm::MotionCommands commands;
  for (int index = 0; index < 256; ++index) {
    const auto id = "command-" + std::to_string(index);
    REQUIRE(commands.Begin(Command(id), "movej", "owner"));
    commands.Finish(id, false, 1001, "invalid target");
  }
  CHECK_THROWS(commands.Begin(Command("overflow"), "movej", "owner"));
  CHECK_FALSE(commands.Begin(Command("command-0"), "movej", "owner"));
  commands.Stop();
  CHECK_THROWS(commands.Validate(Command("command-0"), "app-A", true, true));
  CHECK(commands.Validate(Command("new", 2), "app-A", true, true));
  CHECK(commands.Begin(Command("new", 2), "movej", "owner"));
}
TEST_CASE("model snapshot serves only referenced, confined resources") {
  namespace fs = std::filesystem;
  char pattern[] = "/tmp/rocos-dualarm-model-XXXXXX";
  const char* directory = mkdtemp(pattern);
  REQUIRE(directory != nullptr);
  const fs::path root(directory);
  struct Cleanup {
    fs::path path;
    ~Cleanup() { fs::remove_all(path); }
  } cleanup{root};
  fs::create_directories(root / "package/urdf");
  fs::create_directories(root / "package/meshes");
  const auto file = root / "package/urdf/model.urdf";
  auto write = [](const fs::path& file, const std::string& value) {
    std::ofstream(file) << value;
  };
  const std::string xml =
      "<robot name='test'><link name='base'><visual><geometry>"
      "<mesh filename='../meshes/a.stl'/></geometry></visual></link></robot>";
  write(file, xml);
  write(root / "package/meshes/a.stl", "mesh");
  write(root / "package/secret", "not a model resource");
  rocos::dualarm::ModelBundle model(file, root / "package");
  CHECK(model.Xml() == xml);
  CHECK(model.RootLink() == "base");
  CHECK(model.Asset("../meshes/a.stl") == "mesh");
  CHECK_THROWS(model.Asset("../secret"));
  CHECK_THROWS(model.Asset("/etc/passwd"));
  write(root / "package/meshes/a.stl", "changed");
  CHECK(model.Asset("../meshes/a.stl") == "mesh");
  CHECK(rocos::dualarm::ModelBundle(file, root / "package").Revision() !=
        model.Revision());
  fs::remove(root / "package/meshes/a.stl");
  write(root / "outside", "secret");
  fs::create_symlink(root / "outside", root / "package/meshes/a.stl");
  CHECK_THROWS(rocos::dualarm::ModelBundle(file, root / "package"));
  fs::remove(root / "package/meshes/a.stl");
  CHECK_THROWS(rocos::dualarm::ModelBundle(file, root / "package"));
}
TEST_CASE("motion input rejects invalid vectors and zero quaternions") {
  using namespace rocos::dualarm;
  CHECK_THROWS(Values(Json{{"joints", {0, 1}}}, "joints", 7));
  CHECK_THROWS(Number(Json(true)));
  CHECK_THROWS(Number(Json(std::numeric_limits<double>::infinity())));
  CHECK_THROWS(Positive(Json{{"timeout", 0}}, "timeout", 0.1));
  Json pose{{"position", {{"x", 0}, {"y", 0}, {"z", 0}}},
            {"orientation", {{"x", 0}, {"y", 0}, {"z", 0}, {"w", 0}}}};
  CHECK_THROWS(ParsePose(pose));
  pose["orientation"]["w"] = 2;
  CHECK(PoseJson(ParsePose(pose))["orientation"]["w"] == 1);
}
TEST_CASE("arm leases are independent and cannot cross-authorize") {
  ControlLease left(3), right(3);
  nlohmann::json l, r;
  const auto now = ControlLease::Clock::now();
  REQUIRE(left.Acquire("client", "test", "127.0.0.1", 1234, "test", l, now));
  REQUIRE(right.Acquire("client", "test", "127.0.0.1", 1234, "test", r, now));
  CHECK(l["token"] != r["token"]);
  CHECK(right.Authorize(l["token"], "client", now) == 3007);
  CHECK(left.Authorize("", "client", now) == 3006);
  CHECK(left.Authorize(l["token"], "wrong", now) == 3007);
  CHECK(left.Authorize("debug-token", "debug-client", now) == 3007);
  CHECK(left.Authorize(l["token"], "client", now) == 0);
  nlohmann::json busy;
  CHECK_FALSE(left.Acquire("other", "test", "", 0, "", busy, now));
  CHECK_FALSE(busy.contains("token"));
  left.Release();
  CHECK(left.Authorize(l["token"], "client", now) == 3007);
  CHECK(right.Authorize(r["token"], "client", now) == 0);
}
TEST_CASE("expiry cannot be revived; reads and rejected tokens do not renew") {
  ControlLease lease(3);
  nlohmann::json first, next;
  const auto now = ControlLease::Clock::now();
  REQUIRE(lease.Acquire("", "test", "", 0, "", first, now));
  CHECK(first["client_id_auto"] == true);
  CHECK_FALSE(lease.Status(now).contains("token"));
  CHECK(lease.Status(now + std::chrono::seconds(2))["has_owner"] == true);
  CHECK(lease.Authorize("bad", first["client_id"],
                        now + std::chrono::seconds(2)) == 3007);
  CHECK(lease.Authorize(first["token"], first["client_id"],
                        now + std::chrono::seconds(3)) == 3007);
  REQUIRE(lease.Acquire("new", "test", "", 0, "", next,
                        now + std::chrono::seconds(3)));
  CHECK(lease.Authorize(first["token"], first["client_id"],
                        now + std::chrono::seconds(3)) == 3007);
  CHECK(lease.Authorize(next["token"], "new", now + std::chrono::seconds(5)) ==
        0);
  CHECK(lease.Status(now + std::chrono::seconds(7))["has_owner"] == true);
  CHECK(lease.Status(now + std::chrono::seconds(8))["has_owner"] == false);
}
