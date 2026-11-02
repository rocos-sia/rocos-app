// pinocchio/fwd.hpp 必须先于其他 Boost/Eigen 头文件。
// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on
#include "model.hpp"

#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <fstream>
#include <limits>
#include <mutex>
#include <pin_ik/pin_ik.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <stdexcept>
#include <unordered_set>

namespace rocos {
namespace {
Result failure(Result result) {
  emitLogEvent(LogEventType::Result, LogModule::Model,
               static_cast<int>(result));
  return result;
}
bool valid(const JntArray& q, int size) {
  return q.data.size() == size && q.data.allFinite();
}
bool valid(const Frame& f) {
  return f.p.data.allFinite() && f.M.data.allFinite() &&
         (f.M.data.transpose() * f.M.data)
             .isApprox(Eigen::Matrix3d::Identity(), 1e-8) &&
         std::abs(f.M.data.determinant() - 1.0) < 1e-8;
}

// 仅裁剪 base→tip 以外的拓扑；保留链内固定连杆和完整惯量。
std::string chainXml(const std::string& xml,
                     const std::unordered_set<std::string>& links,
                     const std::unordered_set<std::string>& joints) {
  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::invalid_argument("Invalid URDF XML");
  auto* root = doc.FirstChildElement("robot");
  if (!root) throw std::invalid_argument("URDF has no robot element");
  for (auto* element = root->FirstChildElement(); element;) {
    auto* next = element->NextSiblingElement();
    const std::string kind = element->Name();
    const char* name = element->Attribute("name");
    if ((kind == "link" && (!name || !links.count(name))) ||
        (kind == "joint" && (!name || !joints.count(name))))
      root->DeleteChild(element);
    element = next;
  }
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  return printer.CStr();
}
}  // namespace

struct Model::Impl {
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  std::unique_ptr<PIN_IK::PIN_IK> ik;
  pinocchio::FrameIndex tip{};
  std::vector<pinocchio::FrameIndex> links;
  Eigen::VectorXd configuration;
  pinocchio::Data::Matrix6x jacobian;
  pinocchio::container::aligned_vector<pinocchio::Force> external;
  // IK 自带工作区；独立锁避免阻塞 HTTP FK 和周期动力学计算。
  std::mutex compute_mutex, ik_mutex;

  void setConfiguration(const JntArray& q) {
    for (pinocchio::JointIndex i = 1; i < model.joints.size(); ++i) {
      const auto& joint = model.joints[i];
      const double value = q(joint.idx_v());
      if (joint.nq() == 2) {
        configuration(joint.idx_q()) = std::cos(value);
        configuration(joint.idx_q() + 1) = std::sin(value);
      } else
        configuration(joint.idx_q()) = value;
    }
  }
  bool setExternal(const Wrenches& forces) {
    if (!forces.empty() && forces.size() != links.size()) return false;
    for (auto& force : external) force.setZero();
    for (size_t i = 0; i < forces.size(); ++i) {
      if (!forces[i].force.data.allFinite() ||
          !forces[i].torque.data.allFinite())
        return false;
      const auto& frame = model.frames[links[i]];
      const pinocchio::Force force(forces[i].force.data, forces[i].torque.data);
      external[frame.parentJoint] += frame.placement.act(force);
    }
    return true;
  }
};

Model::~Model() = default;
Model::Model(const std::string& path, const std::string& base,
             const std::string& tip) {
  log_ptr_ = Logger::getInstance("Model");
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Could not open URDF: " + path);
  const std::string xml((std::istreambuf_iterator<char>(file)), {});
  if (!ParseUrdf(xml, base, tip))
    throw std::runtime_error("Could not extract supported URDF chain");
}

Result Model::ForwardKinematics(const JntArray& q, Frame& out) {
  if (!valid(q, GetJointNum())) return failure(Result::FkCalcFail);
  std::lock_guard<std::mutex> lock(impl_->compute_mutex);
  impl_->setConfiguration(q);
  pinocchio::forwardKinematics(impl_->model, *impl_->data,
                               impl_->configuration);
  pinocchio::updateFramePlacements(impl_->model, *impl_->data);
  const auto& pose = impl_->data->oMf[impl_->tip];
  const Frame result(Rotation(pose.rotation()), Vector(pose.translation()));
  if (!valid(result)) return failure(Result::FkCalcFail);
  out = result;
  return Result::NoError;
}
Result Model::GetJacobian(const JntArray& q, Jacobian& out) {
  if (!valid(q, GetJointNum())) return failure(Result::JacobianCalcFail);
  std::lock_guard<std::mutex> lock(impl_->compute_mutex);
  impl_->setConfiguration(q);
  impl_->jacobian.setZero();
  pinocchio::computeFrameJacobian(
      impl_->model, *impl_->data, impl_->configuration, impl_->tip,
      pinocchio::LOCAL_WORLD_ALIGNED, impl_->jacobian);
  if (!impl_->jacobian.allFinite()) return failure(Result::JacobianCalcFail);
  out.data = impl_->jacobian;
  return Result::NoError;
}
Result Model::InverseDynamics(const JntArray& q, const JntArray& v,
                              const JntArray& a, const Wrenches& forces,
                              JntArray& out) {
  const int n = GetJointNum();
  if (!valid(q, n) || !valid(v, n) || !valid(a, n))
    return failure(Result::IdCalcFail);
  std::lock_guard<std::mutex> lock(impl_->compute_mutex);
  if (!impl_->setExternal(forces)) return failure(Result::IdCalcFail);
  impl_->setConfiguration(q);
  const auto& tau =
      pinocchio::rnea(impl_->model, *impl_->data, impl_->configuration, v.data,
                      a.data, impl_->external);
  if (!tau.allFinite()) return failure(Result::IdCalcFail);
  out.data = tau;
  return Result::NoError;
}
Result Model::ForwardDynamics(const JntArray& q, const JntArray& v,
                              const JntArray& tau, const Wrenches& forces,
                              JntArray& out) {
  const int n = GetJointNum();
  if (!valid(q, n) || !valid(v, n) || !valid(tau, n))
    return failure(Result::FdCalcFail);
  std::lock_guard<std::mutex> lock(impl_->compute_mutex);
  if (!impl_->setExternal(forces)) return failure(Result::FdCalcFail);
  impl_->setConfiguration(q);
  const auto& acceleration =
      pinocchio::aba(impl_->model, *impl_->data, impl_->configuration, v.data,
                     tau.data, impl_->external);
  if (!acceleration.allFinite()) return failure(Result::FdCalcFail);
  out.data = acceleration;
  return Result::NoError;
}
Result Model::InverseKinematics(const JntArray& seed, const Frame& target,
                                JntArray& out) {
  if (!valid(seed, GetJointNum()) || !valid(target))
    return failure(Result::IkCalcFail);
  std::lock_guard<std::mutex> lock(impl_->ik_mutex);
  Eigen::VectorXd solution;
  const pinocchio::SE3 pose(target.M.data, target.p.data);
  if (impl_->ik->CartToJnt(seed.data, pose, solution) <= 0 ||
      solution.size() != GetJointNum() || !solution.allFinite())
    return failure(Result::IkCalcFail);
  for (int i = 0; i < solution.size(); ++i)
    if (solution(i) < q_min_(i) || solution(i) > q_max_(i))
      return failure(Result::IkCalcFail);
  JntArray candidate;
  candidate.data = solution;
  Frame actual;
  if (ForwardKinematics(candidate, actual) != Result::NoError ||
      (actual.p - target.p).Norm() > 2e-5 ||
      (actual.M.Inverse() * target.M).GetRot().Norm() > 2e-5)
    return failure(Result::IkCalcFail);
  out = candidate;
  return Result::NoError;
}

void Model::SetGravity(const Vector& gravity) {
  if (!gravity.data.allFinite())
    throw std::invalid_argument("Gravity must be finite");
  std::scoped_lock lock(impl_->ik_mutex, impl_->compute_mutex);
  gravity_ = gravity;
  impl_->model.gravity.linear() = gravity.data;
}
void Model::SetPosLimits(const JntArray& lower, const JntArray& upper) {
  std::lock_guard<std::mutex> lock(impl_->ik_mutex);
  auto next = std::make_unique<PIN_IK::PIN_IK>(impl_->model, lower.data,
                                               upper.data, impl_->tip);
  q_min_ = lower;
  q_max_ = upper;
  impl_->ik = std::move(next);
}
void Model::UpdateSolvers() {
  std::scoped_lock lock(impl_->ik_mutex, impl_->compute_mutex);
  impl_->ik = std::make_unique<PIN_IK::PIN_IK>(impl_->model, q_min_.data,
                                               q_max_.data, impl_->tip);
}

bool Model::ParseUrdf(const std::string& xml, const std::string& base,
                      const std::string& tip) {
  // 与原接口一致，仅允许初始化/停止后重新加载，不支持运动中换模型。
  try {
    const auto source = urdf::parseURDF(xml);
    if (!source || !source->getLink(base) || !source->getLink(tip))
      return false;
    std::vector<urdf::JointConstSharedPtr> chain;
    auto link = source->getLink(tip);
    while (link->name != base) {
      const auto joint = link->parent_joint;
      if (!joint || joint->mimic) return false;
      if (joint->type != urdf::Joint::FIXED &&
          joint->type != urdf::Joint::REVOLUTE &&
          joint->type != urdf::Joint::CONTINUOUS &&
          joint->type != urdf::Joint::PRISMATIC)
        return false;
      chain.push_back(joint);
      link = source->getLink(joint->parent_link_name);
      if (!link) return false;
    }
    std::reverse(chain.begin(), chain.end());
    std::unordered_set<std::string> links{base}, joints;
    std::vector<std::string> names;
    for (const auto& joint : chain) {
      links.insert(joint->child_link_name);
      joints.insert(joint->name);
      if (joint->type != urdf::Joint::FIXED) names.push_back(joint->name);
    }
    if (names.empty()) return false;
    auto next = std::make_unique<Impl>();
    pinocchio::urdf::buildModelFromXML(chainXml(xml, links, joints),
                                       next->model);
    next->model.gravity.linear() = gravity_.data;
    next->data = std::make_unique<pinocchio::Data>(next->model);
    next->tip = next->model.getFrameId(tip, pinocchio::BODY);
    if (next->tip >= next->model.frames.size() ||
        next->model.nv != static_cast<int>(names.size()))
      return false;
    JntArray lower(names.size()), upper(names.size()), velocity(names.size()),
        effort(names.size());
    size_t i = 0;
    for (const auto& joint : chain) {
      next->links.push_back(
          next->model.getFrameId(joint->child_link_name, pinocchio::BODY));
      if (joint->type == urdf::Joint::FIXED) continue;
      const auto id = next->model.getJointId(joint->name);
      if (id >= next->model.joints.size() ||
          next->model.joints[id].idx_v() != static_cast<int>(i))
        return false;
      const double inf = std::numeric_limits<double>::infinity();
      lower(i) = -inf;
      upper(i) = inf;
      velocity(i) = inf;
      effort(i) = inf;
      if (joint->limits) {
        if (joint->type != urdf::Joint::CONTINUOUS) {
          lower(i) = joint->limits->lower;
          upper(i) = joint->limits->upper;
        }
        velocity(i) = joint->limits->velocity;
        effort(i) = joint->limits->effort;
      }
      ++i;
    }
    next->ik = std::make_unique<PIN_IK::PIN_IK>(next->model, lower.data,
                                                upper.data, next->tip);
    next->configuration.setZero(next->model.nq);
    next->jacobian.setZero(6, next->model.nv);
    next->external.resize(next->model.njoints, pinocchio::Force::Zero());
    impl_ = std::move(next);
    joint_names_ = std::move(names);
    q_min_ = lower;
    q_max_ = upper;
    q_vel_ = velocity;
    q_effort_ = effort;
    urdf_string_ = xml;
    return true;
  } catch (const std::exception& error) {
    log_ptr_->error("Could not load robot model: {}", error.what());
    return false;
  }
}
int Model::GetJointNum() const { return static_cast<int>(joint_names_.size()); }
std::vector<std::string> Model::GetJointNames() const { return joint_names_; }
}  // namespace rocos
