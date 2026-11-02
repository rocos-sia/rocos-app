#include "robot_http_server.hpp"
#include "tool_package_json.hpp"
#include <filesystem>
#include <ctime>
#include <algorithm>

namespace rocos {
namespace {
rocos::Frame jsonPose(const nlohmann::json& value) {
    YAML::Node node;
    for (const auto* key : {"x","y","z"}) node["position"].push_back(value.at("position").at(key).get<double>());
    for (const auto* key : {"x","y","z","w"}) node["orientation"].push_back(value.at("orientation").at(key).get<double>());
    return ToolPackageFiles::pose(node);
}
}
void RobotHttpServer::handleToolPackages(const httplib::Request& req, httplib::Response& res) {
    std::unique_lock<std::recursive_mutex> state_lock;
    if (req.method!="GET" && !prepareToolMutation(req,res,state_lock)) return;
    if (!frame_registry_) { sendJson(res,false,-3001,"Tool registry is unavailable"); return; }
    try {
        const auto state=frame_registry_->snapshot();
        const auto name=req.get_param_value("name");
        if (req.method=="GET") {
            if (req.path=="/api/robot/tool_packages") {
                auto tools=nlohmann::json::array(); for (const auto& item : state->packages) tools.push_back(toolPackageJson(item.second));
                sendJson(res,true,0,"ok",{{"tools",tools},{"library_path",frame_registry_->library()},{"robot_id",frame_registry_->robotId()}}); return;
            }
            const auto found=state->packages.find(name);
            if (req.path=="/api/robot/tool_package/asset") {
                const auto digest=req.get_param_value("digest");
                if (!FrameRegistry::validName(name) || name=="." || name==".." || digest.size()!=64 || digest.find_first_not_of("0123456789abcdef")!=std::string::npos) throw std::invalid_argument("Invalid tool asset identity");
                ToolPackage tool; tool.directory=(std::filesystem::path(frame_registry_->library())/".objects"/name/digest).string();
                const auto path=req.get_param_value("path");
                const auto bytes=ToolPackageFiles::asset(tool,path);
                auto extension=std::filesystem::path(path).extension().string();
                std::transform(extension.begin(),extension.end(),extension.begin(),::tolower);
                res.set_content(bytes,extension==".urdf" || extension==".xml" ? "application/xml" : "application/octet-stream");
                res.set_header("Cache-Control","private, max-age=31536000, immutable"); return;
            }
            if (found==state->packages.end()) throw std::invalid_argument("Package tool not found");
            if (req.path=="/api/robot/tool_package/installation") {
                res.set_content(frame_registry_->exportInstallation(name),"application/yaml");
                res.set_header("Content-Disposition","attachment; filename=\""+name+"-installation.yaml\""); return;
            }
            const auto calibrated=req.get_param_value("calibrated");
            if (!calibrated.empty() && calibrated!="true" && calibrated!="false") throw std::invalid_argument("calibrated must be true or false");
            res.set_content(ToolPackageFiles::exportZip(found->second,req.get_param_value("package_version"),calibrated=="true"),"application/zip");
            res.set_header("Content-Disposition","attachment; filename=\""+name+".zip\""); return;
        }
        if (req.path=="/api/robot/tool_packages/import") {
            frame_registry_->importPackage(req.body,req.get_param_value("expected_digest"));
        } else if (req.path=="/api/robot/tool_packages/reload") {
            frame_registry_->reloadPackages();
        } else if (req.path=="/api/robot/tool_package/installation") {
            frame_registry_->importInstallation(req.body);
        } else {
            const auto body=nlohmann::json::parse(req.body);
            if (req.path=="/api/robot/tool_package/overrides") {
                if (!body.at("revision").is_number_integer() || body.at("revision").get<double>()<1 || body.at("revision").get<double>()>=9007199254740991.0) throw std::invalid_argument("revision must be a positive safe integer");
                frame_registry_->updatePackage(body.at("name").get<std::string>(),jsonPose(body.at("mount")),jsonPose(body.at("tcp")),body.at("digest").get<std::string>(),body.at("revision").get<uint64_t>());
            } else {
                std::lock_guard<std::mutex> lock(calibration_mutex_);
                auto found=calibration_sessions_.find(body.at("session_id").get<std::string>());
                if (found==calibration_sessions_.end() || found->second.owner_token!=req.get_header_value(CONTROL_TOKEN_HEADER) || !calibrationContextCurrent(found->second)) {
                    sendJson(res,false,4004,"Calibration session or tool installation changed; restart calibration"); return;
                }
                auto& session=found->second;
                if (!session.result) { sendJson(res,false,4001,"Calculate calibration before saving"); return; }
                const auto current=frame_registry_->snapshot(); const auto package=current->packages.find(session.name);
                if (package!=current->packages.end()) {
                    const auto& tool=package->second; const auto& solution=*session.result;
                    YAML::Node metadata; metadata["timestamp"]=static_cast<double>(std::time(nullptr)); metadata["robot_id"]=frame_registry_->robotId();
                    metadata["package_digest"]=tool.digest; metadata["mount"]=ToolPackageFiles::pose(tool.mount);
                    metadata["flange_tcp"]=ToolPackageFiles::pose(solution.frame); metadata["rms_error"]=solution.rms_error;
                    metadata["max_error"]=solution.max_error; metadata["condition_number"]=solution.condition_number;
                    frame_registry_->updatePackage(session.name,tool.mount,tool.mount.Inverse()*solution.frame,tool.digest,tool.revision,metadata);
                    // 允许回读和再次明确保存；采集上下文跟随已保存的同一安装。
                    session.package_revision=frame_registry_->snapshot()->packages.at(session.name).revision;
                } else if (frame_registry_->set(session.name,session.result->frame,true,false,true)!=Result::NoError) throw std::runtime_error("Cannot persist calibrated tool");
            }
        }
        sendJson(res,true,0,"Tool configuration saved");
    } catch (const std::invalid_argument& error) { sendJson(res,false,1001,error.what()); }
      catch (const YAML::Exception& error) { sendJson(res,false,1001,error.what()); }
      catch (const nlohmann::json::exception& error) { sendJson(res,false,1001,error.what()); }
      catch (const std::exception& error) { sendJson(res,false,-3001,error.what()); }
}
} // namespace rocos
