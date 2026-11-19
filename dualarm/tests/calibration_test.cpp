#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "calibration_sessions.hpp"
#include "doctest.h"

using namespace rocos;
using namespace rocos::dualarm;
namespace {
std::array<Frame, 6> Samples(const Frame& tool) {
  const Vector pivot(0.5, -0.2, 0.4);
  std::array<Frame, 6> points;
  points[0].M = Rotation::RPY(0.6, -0.3, 0.2);
  points[1].M = Rotation::RPY(-0.4, 0.7, -0.5);
  points[2].M = Rotation::RPY(0.3, 0.4, 0.8);
  points[3].M = Rotation::RPY(-0.2, -0.3, 0.4);
  for (int i = 0; i < 4; ++i) points[i].p = pivot - points[i].M * tool.p;
  points[4] = points[5] = points[3];
  points[4].p = points[4].p + points[3].M * tool.M * Vector(0.07, 0, 0);
  points[5].p = points[5].p + points[3].M * tool.M * Vector(0, 0, 0.08);
  return points;
}
CalibrationSessions::Capture Capture(const Frame& frame) {
  return [frame](ToolCalibrationSample& sample) {
    sample = {frame, {0.1, 0.2, 0.3}, 12345.0};
    return Result::NoError;
  };
}
std::string Fill(CalibrationSessions& sessions, const Frame& tool,
                 const std::string& owner = "owner",
                 const CalibrationSessions::Lookup& lookup = {}) {
  auto samples = Samples(tool);
  std::string id;
  for (int i = 0; i < 6; ++i) {
    const auto value = sessions.CapturePoint(
        {{"name", "tcp"}, {"index", i + 1}, {"session_id", id}}, owner,
        Capture(samples[i]), lookup);
    id = value.at("session_id").get<std::string>();
  }
  return id;
}
template <class F>
void Error(int code, F function) {
  try {
    function();
    FAIL("Expected ApiError");
  } catch (const ApiError& error) {
    CHECK(error.code == code);
  }
}
}  // namespace

namespace {
struct PackageFixture {
  std::filesystem::path root, source;
  std::unique_ptr<FrameRegistry> left, right;
  CalibrationSessions sessions;
  Frame mount{Rotation::RPY(0.3, 0.2, -0.4), Vector(0.02, 0.04, 0.05)};
  Frame expected{Rotation::RPY(0.4, -0.5, 1.2), Vector(0.03, -0.02, 0.18)};
  PackageFixture() {
    char pattern[] = "/tmp/rocos-dualarm-package-cal-XXXXXX";
    const auto directory = mkdtemp(pattern);
    REQUIRE(directory != nullptr);
    root = directory;
    source = root / "source";
    std::filesystem::create_directories(source);
    std::ofstream(source / "tool.yaml")
        << "schema_version: 1\nname: tcp\npackage_version: 1.0.0\n"
           "tcp: {position: [0.1,0,0], orientation: [0,0,0,1]}\n";
    left = std::make_unique<FrameRegistry>(
        (root / "left.yaml").string(), (root / "left-tools").string(), "left");
    right = std::make_unique<FrameRegistry>((root / "right.yaml").string(),
                                            (root / "right-tools").string(),
                                            "right");
    auto tool = ToolPackageFiles::read(source.string());
    tool.mount = mount;
    left->initializePackage(tool);
    right->initializePackage(tool);
    REQUIRE(left->save() == Result::NoError);
    REQUIRE(right->save() == Result::NoError);
  }
  ~PackageFixture() { std::filesystem::remove_all(root); }
  CalibrationSessions::Lookup Lookup() {
    return [&](const std::string& name) -> std::optional<ToolPackage> {
      const auto state = left->snapshot();
      const auto found = state->packages.find(name);
      return found == state->packages.end()
                 ? std::nullopt
                 : std::optional<ToolPackage>(found->second);
    };
  }
  CalibrationSessions::Persist Persist() {
    return [&](const auto& name, const auto& solution, const auto& digest,
               uint64_t revision, uint64_t& saved_revision) {
      return left->saveCalibration(name, solution, digest, revision,
                                   saved_revision);
    };
  }
  std::string Solved() {
    const auto id = Fill(sessions, expected, "owner", Lookup());
    sessions.Calculate({{"session_id", id}}, "owner", Lookup());
    return id;
  }
};
}  // namespace

TEST_CASE_FIXTURE(
    PackageFixture,
    "package calibration converts local TCP and persists only this arm") {
  REQUIRE(left->activate("", true, true) == Result::NoError);
  const auto original = left->snapshot()->packages.at("tcp");
  const auto right_original = right->exportInstallation("tcp");
  const auto original_bytes = ToolPackageFiles::exportZip(original, "", false);
  const auto id = Solved();
  CHECK(sessions.Read(id)["package_digest"] == original.digest);
  CHECK(sessions.Read(id)["package_revision"] == original.revision);
  sessions.Save({{"session_id", id}}, "owner", Persist(), Lookup());
  const auto saved = left->snapshot()->packages.at("tcp");
  CHECK(Equal(saved.mount, mount, 1e-12));
  CHECK(Equal(saved.tcp, mount.Inverse() * expected, 1e-9));
  CHECK(Equal(saved.mount * saved.tcp, expected, 1e-9));
  CHECK(saved.digest == original.digest);
  CHECK(saved.version == original.version);
  CHECK(saved.revision > original.revision);
  CHECK(sessions.Read(id)["package_revision"] == saved.revision);
  CHECK(left->snapshot()->active_tool.empty());
  CHECK(right->exportInstallation("tcp") == right_original);
  CHECK(ToolPackageFiles::exportZip(saved, "", false) == original_bytes);
  const auto metadata = YAML::Load(saved.calibration_yaml);
  CHECK(metadata["robot_id"].as<std::string>() == "left");
  CHECK(metadata["package_digest"].as<std::string>() == original.digest);
  CHECK(metadata["timestamp"].as<double>() > 0);
  CHECK(Equal(ToolPackageFiles::pose(metadata["flange_tcp"]), expected, 1e-9));
  CHECK(metadata["max_error"].as<double>() < 1e-12);
  FrameRegistry restored((root / "left.yaml").string(),
                         (root / "left-tools").string(), "left");
  REQUIRE(restored.load() == Result::NoError);
  CHECK(Equal(restored.snapshot()->packages.at("tcp").tcp, saved.tcp, 1e-9));
  CHECK(restored.snapshot()->packages.at("tcp").calibration_yaml ==
        saved.calibration_yaml);
  sessions.Save({{"session_id", id}}, "owner", Persist(), Lookup());
  CHECK(sessions.Read(id)["package_revision"].get<uint64_t>() > saved.revision);
}

TEST_CASE_FIXTURE(
    PackageFixture,
    "changed package installation invalidates capture calculate and save") {
  const auto id = Solved();
  const auto package = left->snapshot()->packages.at("tcp");
  left->updatePackage("tcp", Frame(Vector(0, 0, 0.1)), package.tcp,
                      package.digest, package.revision);
  const auto updated = left->exportInstallation("tcp");
  Error(4004, [&] {
    sessions.Save({{"session_id", id}}, "owner", Persist(), Lookup());
  });
  CHECK(sessions.Read(id)["result"].is_null());
  Error(4004, [&] {
    sessions.Calculate({{"session_id", id}}, "owner", Lookup());
  });
  Error(4004, [&] {
    sessions.CapturePoint({{"name", "tcp"}, {"index", 1}, {"session_id", id}},
                          "owner", Capture(Frame::Identity()), Lookup());
  });
  CHECK(left->exportInstallation("tcp") == updated);
  const auto new_id = Solved();
  REQUIRE(left->remove("tcp", true, true) == Result::NoError);
  Error(4004, [&] {
    sessions.Save({{"session_id", new_id}}, "owner", Persist(), Lookup());
  });
  CHECK(left->snapshot()->tools.count("tcp") == 0);
}

TEST_CASE_FIXTURE(PackageFixture,
                  "atomic package save rejects stale identities and rolls back "
                  "file failure") {
  const auto id = Solved();
  const auto original = left->snapshot()->packages.at("tcp");
  ToolCalibrationSolution solution;
  solution.frame = expected;
  uint64_t revision = 999;
  CHECK(left->saveCalibration("tcp", solution, "", 0, revision) ==
        Result::CallingConflictError);
  CHECK(left->saveCalibration("tcp", solution, std::string(64, 'f'),
                              original.revision,
                              revision) == Result::CallingConflictError);
  CHECK(left->saveCalibration("tcp", solution, original.digest,
                              original.revision + 1,
                              revision) == Result::CallingConflictError);
  CHECK(revision == 999);
  // Force rename failure regardless of root/non-root permissions.
  std::filesystem::remove(root / "left.yaml");
  std::filesystem::create_directory(root / "left.yaml");
  std::ofstream(root / "left.yaml" / "block") << "keep";
  Error(-3001, [&] {
    sessions.Save({{"session_id", id}}, "owner", Persist(), Lookup());
  });
  CHECK(left->snapshot()->packages.at("tcp").revision == original.revision);
  CHECK(Equal(left->snapshot()->packages.at("tcp").tcp, original.tcp));
  CHECK(sessions.Read(id)["package_revision"] == original.revision);
  CHECK_FALSE(sessions.Read(id)["result"].is_null());
  std::filesystem::remove_all(root / "left.yaml");
  sessions.Save({{"session_id", id}}, "owner", Persist(), Lookup());
  CHECK(Equal(left->snapshot()->packages.at("tcp").tcp,
              mount.Inverse() * expected, 1e-9));
}

TEST_CASE("six points solve and persist only the selected ordinary tool") {
  char pattern[] = "/tmp/rocos-dualarm-calibration-XXXXXX";
  const char* directory = mkdtemp(pattern);
  REQUIRE(directory != nullptr);
  const std::filesystem::path root(directory);
  struct Cleanup {
    std::filesystem::path path;
    ~Cleanup() { std::filesystem::remove_all(path); }
  } cleanup{root};
  FrameRegistry left_frames((root / "left.yaml").string());
  FrameRegistry right_frames((root / "right.yaml").string());
  const Frame previous(Vector(0, 0, 0.02));
  REQUIRE(left_frames.set("existing", previous, true, false, true) ==
          Result::NoError);
  REQUIRE(left_frames.activate("existing", true, true) == Result::NoError);
  REQUIRE(right_frames.set("tcp", previous, true, false, true) ==
          Result::NoError);
  const Frame expected(Rotation::RPY(0.4, -0.5, 1.2),
                       Vector(0.03, -0.02, 0.18));
  CalibrationSessions left, right;
  const auto id = Fill(left, expected);
  Error(4001, [&] {
    left.Save({{"session_id", id}}, "owner",
              [](const auto&...) { return Result::NoError; });
  });
  const auto result = left.Calculate({{"session_id", id}}, "owner");
  CHECK(result["captured"] == 6);
  CHECK(result["saved"] == false);
  CHECK(result["max_error"] == doctest::Approx(0.001));
  CHECK(Equal(ParsePose(result["result"]["pose"]), expected, 1e-9));
  CHECK(result["result"]["metrics"]["max_error"].get<double>() < 1e-12);
  CHECK(left_frames.snapshot()->tools.count("tcp") == 0);
  left.Save({{"session_id", id}}, "owner",
            [&](const auto& name, const auto& solution, const auto& digest,
                uint64_t revision, uint64_t& saved_revision) {
              return left_frames.saveCalibration(name, solution, digest,
                                                 revision, saved_revision);
            });
  FrameRegistry reloaded((root / "left.yaml").string());
  CHECK(left.Read(id)["saved"] == true);
  REQUIRE(reloaded.load() == Result::NoError);
  Frame actual;
  REQUIRE(reloaded.get("tcp", true, actual) == Result::NoError);
  CHECK(Equal(actual, expected, 1e-9));
  CHECK(reloaded.snapshot()->active_tool == "existing");
  REQUIRE(right_frames.get("tcp", true, actual) == Result::NoError);
  CHECK(Equal(actual, previous));
  Error(4004, [&] { right.Read(id); });
  Error(4004, [&] { right.Calculate({{"session_id", id}}, "owner"); });
  Error(4004, [&] { left.Calculate({{"session_id", id}}, "new-owner"); });
  bool saved = false;
  Error(4004, [&] {
    left.Save({{"session_id", id}}, "new-owner", [&](const auto&...) {
      saved = true;
      return Result::NoError;
    });
  });
  CHECK_FALSE(saved);
  Error(1001, [&] {
    left.Save({{"session_id", id}, {"path", "/tmp/wrong.yaml"}}, "owner",
              [&](const auto&...) {
                saved = true;
                return Result::NoError;
              });
  });
  CHECK_FALSE(saved);
  Error(-3001, [&] {
    left.Save({{"session_id", id}}, "owner",
              [](const auto&...) { return Result::ResourceUnavailable; });
  });
  CHECK_FALSE(left.Read(id)["result"].is_null());
}

TEST_CASE("recapture and failed recalculation cannot save an old result") {
  CalibrationSessions sessions;
  const Frame tool(Vector(0, 0, 0.2));
  const auto id = Fill(sessions, tool);
  sessions.Calculate({{"session_id", id}}, "owner");
  auto points = Samples(tool);
  points[0].p = points[0].p + Vector(0.0001, 0, 0);
  sessions.CapturePoint({{"name", "tcp"}, {"index", 1}, {"session_id", id}},
                        "owner", Capture(points[0]));
  CHECK(sessions.Read(id)["result"].is_null());
  CHECK(sessions.Read(id)["saved"] == false);
  sessions.Calculate({{"session_id", id}}, "owner");
  Error(4002, [&] {
    sessions.Calculate({{"session_id", id}, {"max_error", 1e-8}}, "owner");
  });
  CHECK(sessions.Read(id)["result"].is_null());
  Error(4001, [&] {
    sessions.Save({{"session_id", id}}, "owner",
                  [](const auto&...) { return Result::NoError; });
  });
  sessions.Clear();
  Error(4004, [&] { sessions.Read(id); });
}

TEST_CASE("invalid captures and bounded arm-local session lifetime") {
  CalibrationSessions left, right;
  const auto capture = Capture(Frame::Identity());
  Error(4001, [&] {
    left.CapturePoint({{"name", "tcp"}, {"index", 2}}, "owner", capture);
  });
  for (const Json& invalid :
       {Json{{"name", "../bad"}, {"index", 1}},
        Json{{"name", "tcp"}, {"index", 7}},
        Json{{"name", "tcp"}, {"index", true}},
        Json{{"name", "tcp"}, {"index", 1}, {"pose", Json::object()}}})
    Error(1001, [&] { left.CapturePoint(invalid, "owner", capture); });
  Error(4005, [&] {
    left.CapturePoint({{"name", "tcp"}, {"index", 1}}, "owner",
                      [](auto&) { return Result::CallingConflictError; });
  });
  const auto first =
      left.CapturePoint({{"name", "tcp"}, {"index", 1}}, "owner", capture);
  const std::string id = first["session_id"];
  const auto other =
      right.CapturePoint({{"name", "tcp"}, {"index", 1}}, "owner", capture);
  CHECK(id != other["session_id"]);
  Error(4004, [&] {
    left.CapturePoint({{"name", "tcp"}, {"index", 2}, {"session_id", id}},
                      "other", capture);
  });
  Error(4001, [&] { left.Calculate({{"session_id", id}}, "owner"); });
  Error(1001, [&] {
    left.Calculate({{"session_id", id}, {"max_error", 0}}, "owner");
  });
  for (int i = 0; i < 16; ++i)
    left.CapturePoint({{"name", "tcp"}, {"index", 1}}, "owner", capture);
  Error(4004, [&] { left.Read(id); });
  CHECK(right.Read(other["session_id"])["captured"] == 1);
  CalibrationSessions restarted;
  Error(4004, [&] { restarted.Read(other["session_id"]); });
}
