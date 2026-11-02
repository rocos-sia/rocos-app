#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "test/doctest.h"
#include "src/frame_registry.hpp"
#include <filesystem>
#include <fstream>
#include <archive.h>
#include <archive_entry.h>

namespace {
namespace fs=std::filesystem;
using rocos::ToolPackageFiles;
using rocos::Result;
struct Fixture {
    fs::path root, library, source;
    Fixture() {
        char pattern[]="/tmp/rocos-packages-XXXXXX"; auto* made=mkdtemp(pattern); REQUIRE(made);
        root=made; library=root/"tools"; source=root/"source"/"gripper"; fs::create_directories(source/"meshes");
        std::ofstream(source/"tool.yaml") << "schema_version: 1\nname: gripper\npackage_version: 1.0.0\nmodel: {urdf: tool.urdf}\ntcp: {position: [0.1,0,0], orientation: [0,0,0,1]}\n";
        std::ofstream(source/"tool.urdf") << "<robot name='gripper'><link name='base'><visual><geometry><mesh filename='package://gripper/meshes/base.stl'/></geometry></visual></link></robot>";
        std::ofstream(source/"meshes/base.stl") << "solid base\nendsolid base\n";
    }
    ~Fixture() { fs::remove_all(root); }
    std::string zip() { return ToolPackageFiles::exportZip(ToolPackageFiles::read(source.string()),"",false); }
};
std::string maliciousZip(const char* name, bool symlink=false, int repetitions=1, bool numbered=false, size_t bytes=1) {
    std::string buffer(1024*1024,'\0'); size_t size=0; auto* archive=archive_write_new(); archive_write_set_format_zip(archive);
    REQUIRE(archive_write_open_memory(archive,buffer.data(),buffer.size(),&size)==ARCHIVE_OK);
    for (int i=0;i<repetitions;++i) {
        auto* entry=archive_entry_new(); const auto path=std::string(name)+(numbered?std::to_string(i):"");
        archive_entry_set_pathname(entry,path.c_str());
        archive_entry_set_filetype(entry,symlink?AE_IFLNK:AE_IFREG); archive_entry_set_perm(entry,0644);
        if (symlink) archive_entry_set_symlink(entry,"/tmp/escape"); else archive_entry_set_size(entry,bytes);
        REQUIRE(archive_write_header(archive,entry)==ARCHIVE_OK);
        if (!symlink) {
            const std::string block(65536,'x'); size_t remaining=bytes;
            while (remaining) { const auto count=std::min(remaining,block.size()); REQUIRE(archive_write_data(archive,block.data(),count)==static_cast<la_ssize_t>(count)); remaining-=count; }
        }
        archive_entry_free(entry);
    }
    archive_write_close(archive); archive_write_free(archive); buffer.resize(size); return buffer;
}
}

TEST_CASE_FIXTURE(Fixture,"Package transforms compose once, overrides restart and original exports stay original") {
    rocos::FrameRegistry registry((root/"frames.yaml").string(),library.string(),"robot_A");
    REQUIRE(registry.set("plain",rocos::Frame(rocos::Vector(0,0,0.2)),true,false,true)==Result::NoError);
    REQUIRE_NOTHROW(registry.importPackage(zip()));
    CHECK(registry.snapshot()->active_tool.empty());
    auto tool=registry.snapshot()->packages.at("gripper");
    const rocos::Frame mount(rocos::Rotation::RotZ(M_PI/2),rocos::Vector(0,0,0.05));
    REQUIRE_NOTHROW(registry.updatePackage(tool.name,mount,tool.tcp,tool.digest,tool.revision));
    REQUIRE(registry.activate(tool.name,true,true)==Result::NoError);
    const rocos::Frame flange(rocos::Rotation::RotX(0.4),rocos::Vector(1,2,3)); rocos::Frame tcp;
    REQUIRE(rocos::FrameRegistry::resolveTcp(*registry.snapshot(),flange,tcp)==Result::NoError);
    CHECK(rocos::Equal(tcp,flange*mount*tool.tcp,1e-12));
    CHECK_FALSE(rocos::Equal(tcp,flange*tool.tcp*mount,1e-6));
    CHECK(registry.set(tool.name,rocos::Frame(),true,false,true)==Result::CallingConflictError);
    rocos::FrameRegistry restarted((root/"frames.yaml").string(),library.string(),"robot_A");
    REQUIRE(restarted.load()==Result::NoError);
    rocos::FrameRegistry wrong_robot((root/"frames.yaml").string(),library.string(),"robot_B");
    CHECK(wrong_robot.load()==Result::IllegalParameter); CHECK(wrong_robot.snapshot()->packages.empty());
    CHECK(restarted.snapshot()->active_tool=="gripper"); CHECK(restarted.snapshot()->tools.count("plain")==1);
    CHECK(rocos::Equal(restarted.snapshot()->tools.at("gripper"),mount*tool.tcp,1e-12));
    tool=restarted.snapshot()->packages.at("gripper");
    const auto original=ToolPackageFiles::installZip(ToolPackageFiles::exportZip(tool,"",false),(root/"copy").string());
    CHECK(rocos::Equal(original.mount,rocos::Frame(),1e-12)); CHECK(original.digest==tool.digest);
    const rocos::Frame flange_tcp(rocos::Rotation::RotY(0.3),rocos::Vector(0.2,0.3,0.4));
    YAML::Node calibration; calibration["rms_error"]=0.0001;
    REQUIRE_NOTHROW(restarted.updatePackage(tool.name,mount,mount.Inverse()*flange_tcp,tool.digest,tool.revision,calibration));
    tool=restarted.snapshot()->packages.at("gripper");
    CHECK(rocos::Equal(restarted.snapshot()->tools.at("gripper"),flange_tcp,1e-12));
    const auto exported=ToolPackageFiles::installZip(ToolPackageFiles::exportZip(tool,"1.0.1",true),(root/"new").string());
    CHECK(exported.version=="1.0.1"); CHECK(rocos::Equal(exported.mount,rocos::Frame(),1e-12));
    CHECK(rocos::Equal(exported.tcp,mount.Inverse()*flange_tcp,1e-12));
    auto installation=restarted.exportInstallation(tool.name);
    REQUIRE_NOTHROW(restarted.importInstallation(installation));
    rocos::FrameRegistry other((root/"other.yaml").string(),library.string(),"robot_B");
    other.importPackage(zip()); CHECK_THROWS(other.importInstallation(installation));
    const auto before=other.snapshot();
    CHECK_THROWS(other.updatePackage(tool.name,mount,tool.tcp,"wrong",1)); CHECK(other.snapshot()==before);
    CHECK_THROWS(restarted.updatePackage(tool.name,mount,tool.tcp,tool.digest,1));
    REQUIRE(restarted.remove(tool.name,true,true)==Result::NoError);
    CHECK(restarted.snapshot()->active_tool.empty()); CHECK(restarted.snapshot()->packages.empty());
    CHECK(fs::exists(tool.directory));
}

TEST_CASE_FIXTURE(Fixture,"Invalid imports and failed persistence leave the registry and plain tools unchanged") {
    rocos::FrameRegistry registry((root/"frames.yaml").string(),library.string());
    registry.set("plain",rocos::Frame(),true); const auto before=registry.snapshot();
    for (const auto& bytes : {std::string("not zip"),maliciousZip("../escape"),maliciousZip("gripper/../../escape"),maliciousZip("gripper/link",true)}) {
        CHECK_THROWS(registry.importPackage(bytes)); CHECK(registry.snapshot()==before);
    }
    fs::remove(source/"meshes/base.stl"); CHECK_THROWS(ToolPackageFiles::read(source.string()));
    std::ofstream(source/"meshes/base.stl")<<"solid base\nendsolid base\n";
    rocos::FrameRegistry failed((root/"missing/frames.yaml").string(),library.string());
    CHECK_THROWS(failed.importPackage(zip())); CHECK(failed.snapshot()->tools.empty());
    registry.set("gripper",rocos::Frame(),true); CHECK_THROWS(registry.importPackage(zip())); CHECK(registry.snapshot()->packages.empty());
}

TEST_CASE_FIXTURE(Fixture,"Directory scans freeze source assets and repeated imports preserve local calibration") {
    fs::create_directories(library); fs::copy(source,library/"gripper",fs::copy_options::recursive);
    rocos::FrameRegistry registry((root/"frames.yaml").string(),library.string());
    REQUIRE_NOTHROW(registry.reloadPackages()); auto tool=registry.snapshot()->packages.at("gripper");
    CHECK(fs::path(tool.directory).parent_path().filename()=="gripper");
    const auto content=ToolPackageFiles::asset(tool,"meshes/base.stl");
    std::ofstream(library/"gripper/meshes/base.stl")<<"changed source";
    CHECK(ToolPackageFiles::asset(tool,"meshes/base.stl")==content);
    CHECK_THROWS(registry.reloadPackages());
    REQUIRE_NOTHROW(registry.reloadPackages(false));
    const auto mount=rocos::Frame(rocos::Vector(0,0,0.03)); YAML::Node metadata; metadata["test"]=1;
    registry.updatePackage(tool.name,mount,tool.tcp,tool.digest,tool.revision,metadata);
    registry.importPackage(zip()); CHECK(rocos::Equal(registry.snapshot()->packages.at(tool.name).mount,mount,1e-12));
    CHECK(YAML::Load(registry.snapshot()->packages.at(tool.name).calibration_yaml)["test"].as<int>()==1);
    CHECK_THROWS(ToolPackageFiles::asset(tool,"../../frames.yaml"));
    fs::create_symlink(root/"frames.yaml",fs::path(tool.directory)/"bad.stl");
    CHECK_THROWS(ToolPackageFiles::asset(tool,"bad.stl"));
}

TEST_CASE_FIXTURE(Fixture,"ZIP updates require the old digest and preserve active selection while clearing old overrides") {
    rocos::FrameRegistry registry((root/"frames.yaml").string(),library.string());
    registry.importPackage(zip()); auto tool=registry.snapshot()->packages.at("gripper");
    registry.updatePackage(tool.name,rocos::Frame(rocos::Vector(0,0,0.04)),tool.tcp,tool.digest,tool.revision);
    registry.activate(tool.name,true,true);
    std::ofstream(source/"tool.yaml") << "schema_version: 1\nname: gripper\npackage_version: 1.0.1\nmodel: {urdf: tool.urdf}\ntcp: {position: [0,0,0.3], orientation: [0,0,0,1]}\n";
    const auto before=registry.snapshot();
    CHECK_THROWS(registry.importPackage(zip())); CHECK(registry.snapshot()==before);
    REQUIRE_NOTHROW(registry.importPackage(zip(),tool.digest));
    const auto updated=registry.snapshot()->packages.at(tool.name);
    CHECK(updated.version=="1.0.1"); CHECK(updated.revision==3);
    CHECK(rocos::Equal(updated.mount,rocos::Frame(),1e-12)); CHECK(updated.tcp.p.z()==doctest::Approx(0.3));
    CHECK(registry.snapshot()->active_tool==tool.name);
}

TEST_CASE_FIXTURE(Fixture,"ZIP and resource paths preserve UTF8 spaces and percent characters") {
    const std::string name="夹 爪%.stl";
    std::filesystem::rename(source/"meshes/base.stl",source/"meshes"/name);
    std::ofstream(source/"tool.urdf") << "<robot name='gripper'><link name='base'><visual><geometry><mesh filename='meshes/" << name << "'/></geometry></visual></link></robot>";
    const auto original=ToolPackageFiles::read(source.string());
    const auto imported=ToolPackageFiles::installZip(zip(),library.string());
    CHECK(imported.digest==original.digest);
    CHECK(ToolPackageFiles::asset(imported,"meshes/"+name)==ToolPackageFiles::asset(original,"meshes/"+name));
}

TEST_CASE_FIXTURE(Fixture,"Published calibration records stay immutable after override edits and failed persistence") {
    const auto parent=root/"settings"; fs::create_directory(parent);
    rocos::FrameRegistry registry((parent/"frames.yaml").string(),library.string()); registry.importPackage(zip());
    auto tool=registry.snapshot()->packages.at("gripper"); YAML::Node record; record["rms_error"]=0.0001;
    registry.updatePackage(tool.name,tool.mount,tool.tcp,tool.digest,tool.revision,record);
    const auto calibrated=registry.snapshot(); tool=calibrated->packages.at("gripper");
    registry.updatePackage(tool.name,tool.mount,tool.tcp,tool.digest,tool.revision);
    CHECK(YAML::Load(calibrated->packages.at(tool.name).calibration_yaml)["rms_error"].as<double>(-1)==doctest::Approx(0.0001));
    CHECK(registry.snapshot()->packages.at(tool.name).calibration_yaml.empty());
    tool=registry.snapshot()->packages.at("gripper"); registry.updatePackage(tool.name,tool.mount,tool.tcp,tool.digest,tool.revision,record);
    const auto before=registry.snapshot(); tool=before->packages.at("gripper");
    fs::rename(parent,root/"saved-settings"); std::ofstream(parent)<<"not a directory";
    CHECK_THROWS(registry.updatePackage(tool.name,tool.mount,tool.tcp,tool.digest,tool.revision));
    CHECK(registry.snapshot()==before);
    CHECK(YAML::Load(before->packages.at(tool.name).calibration_yaml)["rms_error"].as<double>(-1)==doctest::Approx(0.0001));
}

TEST_CASE_FIXTURE(Fixture,"Tool package XML rejects textures and external mesh URIs") {
    for (const auto* xml : {
        "<robot name='g'><link name='base'><visual><geometry><box size='1 1 1'/></geometry><material name='m'><texture filename='http://example.invalid/a.png'/></material></visual></link></robot>",
        "<robot name='g'><link name='base'><visual><geometry><mesh filename='https://example.invalid/base.stl'/></geometry></visual></link></robot>"}) {
        std::ofstream(source/"tool.urdf")<<xml;
        CHECK_THROWS(ToolPackageFiles::read(source.string()));
    }
}

TEST_CASE_FIXTURE(Fixture,"ZIP duplicate entries expanded size and file quotas fail before publishing") {
    rocos::FrameRegistry registry((root/"frames.yaml").string(),library.string()); const auto before=registry.snapshot();
    CHECK_THROWS_WITH(registry.importPackage(maliciousZip("gripper/file",false,2)),"Invalid or duplicate ZIP entry");
    CHECK_THROWS_WITH(registry.importPackage(maliciousZip("gripper/file",false,513,true)),"Invalid or duplicate ZIP entry");
    CHECK_THROWS_WITH(registry.importPackage(maliciousZip("gripper/file",false,1,false,64*1024*1024+1)),"Expanded tool package exceeds size limit");
    CHECK(registry.snapshot()==before);
}
