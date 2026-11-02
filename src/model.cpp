#include "model.hpp"

#include <kdl/chainfdsolver_recursive_newton_euler.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>

#include <limits>
#include <string>
#include <unordered_map>

namespace rocos {

namespace {

struct ParsedJointLimit {
  bool has_lower{false};
  bool has_upper{false};
  bool has_velocity{false};
  bool has_effort{false};
  double lower{0.0};
  double upper{0.0};
  double velocity{0.0};
  double effort{0.0};
};

std::unordered_map<std::string, ParsedJointLimit> ParseHardwareLimits(
    const std::string& urdf_string) {
  std::unordered_map<std::string, ParsedJointLimit> limits;

  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_string.c_str()) != tinyxml2::XML_SUCCESS) {
    return limits;
  }

  const auto* robot = doc.FirstChildElement("robot");
  if (robot == nullptr) {
    return limits;
  }

  for (const auto* joint = robot->FirstChildElement("joint");
       joint != nullptr; joint = joint->NextSiblingElement("joint")) {
    const char* joint_name = joint->Attribute("name");
    if (joint_name == nullptr) {
      continue;
    }

    const auto* hardware = joint->FirstChildElement("hardware");
    const auto* limit = hardware != nullptr ? hardware->FirstChildElement("limit") : nullptr;
    if (limit == nullptr) {
      continue;
    }

    ParsedJointLimit parsed;
    parsed.has_lower = limit->QueryDoubleAttribute("lower", &parsed.lower) == tinyxml2::XML_SUCCESS;
    parsed.has_upper = limit->QueryDoubleAttribute("upper", &parsed.upper) == tinyxml2::XML_SUCCESS;
    parsed.has_velocity =
        limit->QueryDoubleAttribute("velocity", &parsed.velocity) == tinyxml2::XML_SUCCESS ||
        limit->QueryDoubleAttribute("vel", &parsed.velocity) == tinyxml2::XML_SUCCESS;
    parsed.has_effort = limit->QueryDoubleAttribute("effort", &parsed.effort) == tinyxml2::XML_SUCCESS;

    limits[joint_name] = parsed;
  }

  return limits;
}

}  // namespace

Model::Model(const std::string& urdf_file_path,
             const std::string& base_link,
             const std::string& tip) {
  log_ptr_ = Logger::getInstance("Model");

  //先根据文件路径把内容加载到urdf_string_
  std::ifstream urdf_file(urdf_file_path);
  if (!urdf_file) {
    log_ptr_->error("Could not open URDF file: {}", urdf_file_path);
    throw std::runtime_error("Could not open urdf file");
  }
  urdf_string_ = std::string((std::istreambuf_iterator<char>(urdf_file)),
                             std::istreambuf_iterator<char>());

  if (!ParseUrdf(urdf_string_, base_link, tip)) {
    log_ptr_->error("Could not extract  urdf!");
    throw std::runtime_error("Could not extract urdf");
  }

  UpdateSolvers();
}

std::unique_ptr<Model> Model::CloneForPreview() const {
  auto copy = std::unique_ptr<Model>(new Model());
  copy->chain_ = chain_;
  copy->joint_names_ = joint_names_;
  copy->q_min_ = q_min_;
  copy->q_max_ = q_max_;
  copy->q_vel_ = q_vel_;
  copy->q_effort_ = q_effort_;
  copy->gravity_ = gravity_;
  copy->log_ptr_ = log_ptr_;
  copy->UpdateSolvers();
  return copy;
}

Result Model::ForwardKinematics(const JntArray& q_in, Frame& p_out) {
  if (fk_solver_->JntToCart(q_in, p_out) < 0) {
    emitLogEvent(LogEventType::Result, LogModule::Model, static_cast<int>(Result::FkCalcFail));
    return Result::FkCalcFail;
  }
  return Result::NoError;
}

Result Model::InverseKinematics(const JntArray& q_in, const Frame& p_in,
                                JntArray& q_out) {
  if (ik_solver_->CartToJnt(q_in, p_in, q_out) < 0) {
    emitLogEvent(LogEventType::Result, LogModule::Model, static_cast<int>(Result::IkCalcFail));
    return Result::IkCalcFail;
  }
  return Result::NoError;
}

Result Model::ForwardDynamics(const JntArray& q, const JntArray& q_dot,
                              const JntArray& torques, const Wrenches& f_ext,
                              JntArray& q_dotdot) {
  // KDL 要求 f_ext.size() == ns（段数），空 vector 会导致 E_SIZE_MISMATCH
  // 调用方传空 vector 表示"无外力"，此处自动补齐为零旋量
  const Wrenches* f_ext_ptr = &f_ext;
  Wrenches f_ext_zero;
  if (f_ext.empty()) {
    f_ext_zero.resize(chain_.getNrOfSegments(), KDL::Wrench::Zero());
    f_ext_ptr = &f_ext_zero;
  }
  if (fd_solver_->CartToJnt(q, q_dot, torques, *f_ext_ptr, q_dotdot) < 0) {
    emitLogEvent(LogEventType::Result, LogModule::Model, static_cast<int>(Result::FdCalcFail));
    return Result::FdCalcFail;
  }
  return Result::NoError;
}

Result Model::InverseDynamics(const JntArray& q, const JntArray& q_dot,
                              const JntArray& q_dotdot, const Wrenches& f_ext,
                              JntArray& torques) {
  // KDL 要求 f_ext.size() == ns（段数），空 vector 会导致 E_SIZE_MISMATCH
  // 调用方传空 vector 表示"无外力"，此处自动补齐为零旋量
  const Wrenches* f_ext_ptr = &f_ext;
  Wrenches f_ext_zero;
  if (f_ext.empty()) {
    f_ext_zero.resize(chain_.getNrOfSegments(), KDL::Wrench::Zero());
    f_ext_ptr = &f_ext_zero;
  }
  if (id_solver_->CartToJnt(q, q_dot, q_dotdot, *f_ext_ptr, torques) < 0) {
    emitLogEvent(LogEventType::Result, LogModule::Model, static_cast<int>(Result::IdCalcFail));
    return Result::IdCalcFail;
  }
  return Result::NoError;
}

Result Model::GetJacobian(const JntArray& q, Jacobian& J_out) {
  KDL::ChainJntToJacSolver jac_solver(chain_);
  J_out.resize(chain_.getNrOfJoints());
  if (jac_solver.JntToJac(q, J_out) < 0) {
    emitLogEvent(LogEventType::Result, LogModule::Model, static_cast<int>(Result::JacobianCalcFail));
    return Result::JacobianCalcFail;
  }

  return Result::NoError;
}

bool Model::SetChain(const Tree& tree, const std::string& base_link,
                     const std::string& tip) {
  if (!tree.getChain(base_link, tip, chain_)) {
    // 从KDL::Tree获取运动链失败
    log_ptr_->error("Could not get chain from kdl tree!");
    return false;
  }

  q_min_.resize(chain_.getNrOfJoints());
  q_max_.resize(chain_.getNrOfJoints());
  q_vel_.resize(chain_.getNrOfJoints());
  q_effort_.resize(chain_.getNrOfJoints());

  return true;
}

void Model::SetGravity(const Vector& gravity) {
  gravity_ = gravity;

  UpdateSolvers();  // 更新动力学求解器以使用新的重力向量
}

void Model::SetPosLimits(const JntArray& q_min, const JntArray& q_max) {
  q_min_ = q_min;
  q_max_ = q_max;

  UpdateSolvers();
}

void Model::UpdateSolvers() {
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  ik_solver_ = std::make_unique<TRAC_IK::TRAC_IK>(chain_, q_min_, q_max_);

  fd_solver_ = std::make_unique<KDL::ChainFdSolver_RNE>(chain_, gravity_);
  id_solver_ = std::make_unique<KDL::ChainIdSolver_RNE>(chain_, gravity_);

}

bool Model::ParseUrdf(const std::string& urdf_string,
                      const std::string& base_link, const std::string& tip) {
  static const double kDefaultLowerLimit = -KDL::PI;  // -180 deg
  static const double kDefaultUpperLimit = KDL::PI;   //  180 deg
  static const double kDefaultVelocityLimit = std::numeric_limits<double>::infinity();
  static const double kDefaultEffortLimit = std::numeric_limits<double>::infinity();

  const auto model = urdf::parseURDF(urdf_string);
  if (model == nullptr) {
    log_ptr_->error("Could not parse urdf string!");
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*model, tree)) {
    log_ptr_->error("Could not extract urdf to Tree!");
    return false;
  }
  tree_ = tree;

  if (!SetChain(tree_, base_link, tip)) {
    log_ptr_->error("Could not set kinematic chain!");
    return false;
  }

  const auto hardware_limits = ParseHardwareLimits(urdf_string);

  // 按 Chain 中的关节顺序逐段解析 URDF limit 约束
  unsigned int joint_index = 0;
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const auto& kdl_joint = chain_.getSegment(i).getJoint();
    if (kdl_joint.getType() == KDL::Joint::None) {
      continue;  // 固定关节不占 JntArray 位置，跳过
    }

    // 先写默认值，再尝试从 URDF 覆盖
    q_min_(joint_index) = kDefaultLowerLimit;
    q_max_(joint_index) = kDefaultUpperLimit;
    q_vel_(joint_index) = kDefaultVelocityLimit;
    q_effort_(joint_index) = kDefaultEffortLimit;

    const auto urdf_joint = model->getJoint(kdl_joint.getName());
    if (urdf_joint != nullptr && urdf_joint->limits != nullptr) {
      q_min_(joint_index) = urdf_joint->limits->lower;
      q_max_(joint_index) = urdf_joint->limits->upper;
      q_vel_(joint_index) = urdf_joint->limits->velocity;
      q_effort_(joint_index) = urdf_joint->limits->effort;
    }

    const auto hardware_limit = hardware_limits.find(kdl_joint.getName());
    if (hardware_limit != hardware_limits.end()) {
      const auto& limit = hardware_limit->second;
      if (limit.has_lower) q_min_(joint_index) = limit.lower;
      if (limit.has_upper) q_max_(joint_index) = limit.upper;
      if (limit.has_velocity) q_vel_(joint_index) = limit.velocity;
      if (limit.has_effort) q_effort_(joint_index) = limit.effort;
    }

    ++joint_index;
  }

  // 刷新可动关节名称列表，顺序与 KDL JntArray 一致
  joint_names_.clear();
  for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
    const auto& kdl_joint = chain_.getSegment(i).getJoint();
    if (kdl_joint.getType() != KDL::Joint::None) {
      joint_names_.push_back(kdl_joint.getName());
    }
  }

  if (joint_names_.size() != chain_.getNrOfJoints()) {
    log_ptr_->error("joint_names_ count ({}) != chain joints ({})",
                    joint_names_.size(), chain_.getNrOfJoints());
    return false;
  }

  return true;
}

int Model::GetJointNum() const {
  return chain_.getNrOfJoints();
}

std::vector<std::string> Model::GetJointNames() const {
  return joint_names_;
}

}  // namespace rocos
