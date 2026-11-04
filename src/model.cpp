// pinocchio/fwd.hpp 必须先于其他 Boost/Eigen 头文件。
// clang-format off
#include <pinocchio/fwd.hpp>
// clang-format on
#include "model.hpp"

#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <limits>
#include <mutex>
#include <pin_ik/pin_ik.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
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

std::vector<MimicJoint> readMimicJoints(const urdf::ModelInterface& model) {
  std::vector<MimicJoint> result;
  for (const auto& entry : model.joints_) {
    if (!entry.second->mimic) continue;
    MimicJoint relation{entry.first, entry.first, 1, 0};
    auto joint = entry.second;
    std::unordered_set<std::string> visited;
    while (joint->mimic) {
      if (!visited.insert(joint->name).second)
        throw std::invalid_argument("Cyclic mimic relation: " + entry.first);
      const auto& mimic = *joint->mimic;
      relation.offset += relation.multiplier * mimic.offset;
      relation.multiplier *= mimic.multiplier;
      relation.source = mimic.joint_name;
      const auto source = model.joints_.find(relation.source);
      if (source == model.joints_.end())
        throw std::invalid_argument("Unknown mimic source: " + relation.source);
      joint = source->second;
    }
    if (!std::isfinite(relation.multiplier) || !std::isfinite(relation.offset))
      throw std::invalid_argument("Non-finite mimic relation: " + entry.first);
    result.push_back(std::move(relation));
  }
  return result;
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
    else if (kind == "joint") {
      // 后端保留完整树自由度，耦合在统一的坐标映射中处理。
      if (auto* mimic = element->FirstChildElement("mimic"))
        element->DeleteChild(mimic);
    }
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
  std::unique_ptr<pinocchio::Data> coupled_ik_data;
  pinocchio::FrameIndex tip{};
  std::vector<pinocchio::FrameIndex> links;
  Eigen::VectorXd configuration;
  pinocchio::Data::Matrix6x jacobian;
  pinocchio::container::aligned_vector<pinocchio::Force> external;
  bool coupled{false};
  // q_full = expansion * reduction * q_active + offset。
  // active 保留有驱动 mimic 槽位；独立坐标仅包含非 mimic 关节。
  Eigen::MatrixXd expansion, reduction, actuation, force_lift;
  Eigen::VectorXd offset, active_offset, full_lower, full_upper;
  std::vector<int> full_to_active, source_columns, root_slots;
  // IK 自带工作区；独立锁避免阻塞 HTTP FK 和周期动力学计算。
  std::mutex compute_mutex, ik_mutex;

  Eigen::VectorXd toConfiguration(const Eigen::VectorXd& independent) const {
    const Eigen::VectorXd full = expansion * independent + offset;
    Eigen::VectorXd result(model.nq);
    for (pinocchio::JointIndex i = 1; i < model.joints.size(); ++i) {
      const auto& joint = model.joints[i];
      const double value = full(joint.idx_v());
      if (joint.nq() == 2) {
        result(joint.idx_q()) = std::cos(value);
        result(joint.idx_q() + 1) = std::sin(value);
      } else
        result(joint.idx_q()) = value;
    }
    return result;
  }
  void setConfiguration(const JntArray& q) {
    if (coupled) {
      configuration = toConfiguration(reduction * q.data);
      return;
    }
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
  void constrainLimits(JntArray& lower, JntArray& upper) const {
    if (lower.rows() != actuation.rows() || upper.rows() != lower.rows())
      throw std::invalid_argument("Wrong joint limit dimensions");
    for (int i = 0; i < lower.data.size(); ++i)
      if (std::isnan(lower(i)) || std::isnan(upper(i)) || lower(i) > upper(i))
        throw std::invalid_argument("Invalid joint limits");
    for (int i = 0; i < model.nv; ++i) {
      const int active = full_to_active[i], column = source_columns[i];
      const double lo = active < 0 ? full_lower(i) : lower(active);
      const double hi = active < 0 ? full_upper(i) : upper(active);
      const double scale = expansion(i, column);
      if (scale == 0) {
        if (offset(i) < lo || offset(i) > hi)
          throw std::invalid_argument(
              "Constant mimic position is outside limits");
        continue;
      }
      double a = (lo - offset(i)) / scale, b = (hi - offset(i)) / scale;
      if (scale < 0) std::swap(a, b);
      const int root = root_slots[column];
      lower(root) = std::max(lower(root), a);
      upper(root) = std::min(upper(root), b);
      if (lower(root) > upper(root))
        throw std::invalid_argument(
            "Mimic joint limits have no feasible intersection");
    }
  }
  void initialize(const JntArray& lower, const JntArray& upper) {
    data = std::make_unique<pinocchio::Data>(model);
    if (coupled)
      coupled_ik_data = std::make_unique<pinocchio::Data>(model);
    else
      ik = std::make_unique<PIN_IK::PIN_IK>(model, lower.data, upper.data, tip);
    configuration.setZero(model.nq);
    jacobian.setZero(6, model.nv);
    external.resize(model.njoints, pinocchio::Force::Zero());
  }
  bool solveCoupled(const JntArray& seed, const pinocchio::SE3& target,
                    const JntArray& lower, const JntArray& upper,
                    Eigen::VectorXd& solution) {
    // 不用矩阵乘法选取无穷限位，避免 0 * inf 产生 NaN。
    Eigen::VectorXd minimum(root_slots.size()), maximum(root_slots.size());
    for (size_t i = 0; i < root_slots.size(); ++i) {
      minimum(i) = lower(root_slots[i]);
      maximum(i) = upper(root_slots[i]);
    }
    const Eigen::VectorXd initial = reduction * seed.data;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(5);
    auto residual = [&](const Eigen::VectorXd& q) {
      pinocchio::forwardKinematics(model, *coupled_ik_data, toConfiguration(q));
      pinocchio::updateFramePlacements(model, *coupled_ik_data);
      return pinocchio::log6(coupled_ik_data->oMf[tip].actInv(target))
          .toVector()
          .eval();
    };
    for (int restart = 0; restart < 4; ++restart) {
      Eigen::VectorXd q = initial;
      for (int i = 0; i < q.size(); ++i) {
        if (restart && std::isfinite(minimum(i)) && std::isfinite(maximum(i)))
          q(i) = minimum(i) + (maximum(i) - minimum(i)) *
                                  (0.5 + 0.35 * std::sin(restart * (i + 1.0)));
        q(i) = std::clamp(q(i), minimum(i), maximum(i));
      }
      for (int iteration = 0; iteration < 150; ++iteration) {
        const Eigen::Matrix<double, 6, 1> error = residual(q);
        if (error.head<3>().norm() <= 1e-5 && error.tail<3>().norm() <= 1e-5) {
          solution = actuation * q + active_offset;
          return solution.allFinite();
        }
        if (std::chrono::steady_clock::now() >= deadline) return false;
        pinocchio::Data::Matrix6x full_jacobian(6, model.nv);
        full_jacobian.setZero();
        pinocchio::computeFrameJacobian(model, *coupled_ik_data,
                                        toConfiguration(q), tip,
                                        pinocchio::LOCAL, full_jacobian);
        const Eigen::MatrixXd jac = full_jacobian * expansion;
        Eigen::Matrix<double, 6, 6> normal = jac * jac.transpose();
        normal.diagonal().array() += 1e-8;
        Eigen::VectorXd step = jac.transpose() * normal.ldlt().solve(error);
        if (!step.allFinite()) return false;
        const double largest = step.cwiseAbs().maxCoeff();
        if (largest > 0.5) step *= 0.5 / largest;
        bool improved = false;
        for (int line = 0; line < 10; ++line) {
          Eigen::VectorXd candidate =
              (q + step).cwiseMax(minimum).cwiseMin(maximum);
          if (residual(candidate).squaredNorm() < error.squaredNorm()) {
            q = std::move(candidate);
            improved = true;
            break;
          }
          step *= 0.5;
        }
        if (!improved) break;
      }
    }
    return false;
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
Model::Model() = default;
Model::Model(const std::string& path, const std::string& base,
             const std::string& tip,
             const std::vector<std::string>& driven_joint_names) {
  log_ptr_ = Logger::getInstance("Model");
  std::ifstream file(path);
  if (!file) throw std::runtime_error("Could not open URDF: " + path);
  const std::string xml((std::istreambuf_iterator<char>(file)), {});
  if (!ParseUrdf(xml, base, tip, driven_joint_names))
    throw std::runtime_error("Could not extract supported URDF chain");
}

std::unique_ptr<Model> Model::CloneForPreview() const {
  auto copy = std::unique_ptr<Model>(new Model());
  copy->impl_ = std::make_unique<Impl>();
  {
    // 只在复制配置时持锁；求解器和工作区在释放源模型锁后独立创建。
    std::scoped_lock lock(impl_->ik_mutex, impl_->compute_mutex);
    copy->impl_->model = impl_->model;
    copy->impl_->tip = impl_->tip;
    copy->impl_->links = impl_->links;
    copy->impl_->coupled = impl_->coupled;
    copy->impl_->expansion = impl_->expansion;
    copy->impl_->reduction = impl_->reduction;
    copy->impl_->actuation = impl_->actuation;
    copy->impl_->force_lift = impl_->force_lift;
    copy->impl_->offset = impl_->offset;
    copy->impl_->active_offset = impl_->active_offset;
    copy->impl_->full_lower = impl_->full_lower;
    copy->impl_->full_upper = impl_->full_upper;
    copy->impl_->full_to_active = impl_->full_to_active;
    copy->impl_->source_columns = impl_->source_columns;
    copy->impl_->root_slots = impl_->root_slots;
    copy->joint_names_ = joint_names_;
    copy->mimic_joints_ = mimic_joints_;
    copy->urdf_string_ = urdf_string_;
    copy->q_min_ = q_min_;
    copy->q_max_ = q_max_;
    copy->q_vel_ = q_vel_;
    copy->q_effort_ = q_effort_;
    copy->gravity_ = gravity_;
    copy->log_ptr_ = log_ptr_;
  }
  copy->impl_->initialize(copy->q_min_, copy->q_max_);
  return copy;
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
  if (impl_->coupled)
    out.data = impl_->jacobian * impl_->expansion * impl_->reduction;
  else
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
  if (!impl_->coupled) {
    const auto& tau =
        pinocchio::rnea(impl_->model, *impl_->data, impl_->configuration,
                        v.data, a.data, impl_->external);
    if (!tau.allFinite()) return failure(Result::IdCalcFail);
    out.data = tau;
    return Result::NoError;
  }
  const Eigen::VectorXd full_v = impl_->expansion * (impl_->reduction * v.data);
  const Eigen::VectorXd full_a = impl_->expansion * (impl_->reduction * a.data);
  const auto& tau =
      pinocchio::rnea(impl_->model, *impl_->data, impl_->configuration, full_v,
                      full_a, impl_->external);
  if (!tau.allFinite()) return failure(Result::IdCalcFail);
  // 按虚功映射到独立坐标，再以最小范数分配给实际驱动器。
  out.data = impl_->force_lift * (impl_->expansion.transpose() * tau);
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
  if (impl_->coupled) {
    const Eigen::VectorXd full_v =
        impl_->expansion * (impl_->reduction * v.data);
    const Eigen::VectorXd bias = pinocchio::rnea(
        impl_->model, *impl_->data, impl_->configuration, full_v,
        Eigen::VectorXd::Zero(impl_->model.nv), impl_->external);
    pinocchio::crba(impl_->model, *impl_->data, impl_->configuration);
    const Eigen::MatrixXd mass = impl_->data->M.selfadjointView<Eigen::Upper>();
    const Eigen::MatrixXd reduced_mass =
        impl_->expansion.transpose() * mass * impl_->expansion;
    const Eigen::LDLT<Eigen::MatrixXd> solver(reduced_mass);
    if (solver.info() != Eigen::Success || !solver.isPositive())
      return failure(Result::FdCalcFail);
    const Eigen::VectorXd acceleration =
        impl_->actuation *
        solver.solve(impl_->actuation.transpose() * tau.data -
                     impl_->expansion.transpose() * bias);
    if (!acceleration.allFinite()) return failure(Result::FdCalcFail);
    out.data = acceleration;
    return Result::NoError;
  }
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
  const bool solved =
      impl_->coupled ? impl_->solveCoupled(seed, pose, q_min_, q_max_, solution)
                     : impl_->ik->CartToJnt(seed.data, pose, solution) > 0;
  if (!solved || solution.size() != GetJointNum() || !solution.allFinite())
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
  JntArray lo = lower, hi = upper;
  impl_->constrainLimits(lo, hi);
  std::unique_ptr<PIN_IK::PIN_IK> next;
  if (!impl_->coupled)
    next = std::make_unique<PIN_IK::PIN_IK>(impl_->model, lo.data, hi.data,
                                            impl_->tip);
  q_min_ = lo;
  q_max_ = hi;
  impl_->ik = std::move(next);
}
void Model::UpdateSolvers() {
  std::scoped_lock lock(impl_->ik_mutex, impl_->compute_mutex);
  if (!impl_->coupled)
    impl_->ik = std::make_unique<PIN_IK::PIN_IK>(impl_->model, q_min_.data,
                                               q_max_.data, impl_->tip);
}

Result Model::ApplyPositionCoupling(JntArray& q) const {
  if (q.rows() != static_cast<unsigned int>(GetJointNum()))
    return Result::UnmatchedJointsNumber;
  if (!q.data.allFinite()) return Result::ParameterNanOrInf;
  if (impl_->coupled)
    q.data =
        (impl_->actuation * (impl_->reduction * q.data) + impl_->active_offset)
            .eval();
  return q.data.allFinite() ? Result::NoError : Result::ParameterNanOrInf;
}

bool Model::ParseUrdf(const std::string& xml, const std::string& base,
                      const std::string& tip,
                      const std::vector<std::string>& driven_joint_names) {
  // 与原接口一致，仅允许初始化/停止后重新加载，不支持运动中换模型。
  try {
    const auto source = urdf::parseURDF(xml);
    if (!source || !source->getLink(base) || !source->getLink(tip))
      return false;
    auto mimics = readMimicJoints(*source);
    std::vector<urdf::JointConstSharedPtr> chain;
    auto link = source->getLink(tip);
    while (link->name != base) {
      const auto joint = link->parent_joint;
      if (!joint) return false;
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
    for (const auto& joint : chain) {
      links.insert(joint->child_link_name);
      joints.insert(joint->name);
    }
    // 纳入有驱动 mimic 的旁支及主链 mimic 所依赖的源关节路径。
    auto includePath = [&](const std::string& name) {
      const auto joint = source->getJoint(name);
      if (!joint) throw std::invalid_argument("Unknown driven joint: " + name);
      auto child = source->getLink(joint->child_link_name);
      std::vector<urdf::JointConstSharedPtr> path;
      while (child->name != base) {
        if (!child->parent_joint)
          throw std::invalid_argument(
              "Mimic source/drive is outside the selected base: " + name);
        path.push_back(child->parent_joint);
        child = source->getLink(child->parent_joint->parent_link_name);
      }
      for (auto it = path.rbegin(); it != path.rend(); ++it)
        if (joints.insert((*it)->name).second) {
          chain.push_back(*it);
          links.insert((*it)->child_link_name);
        }
    };
    const std::unordered_set<std::string> driven(driven_joint_names.begin(),
                                                 driven_joint_names.end());
    for (const auto& mimic : mimics)
      if (driven.count(mimic.name)) includePath(mimic.name);
    for (size_t i = 0; i < chain.size(); ++i)
      if (chain[i]->mimic) {
        const auto it = std::find_if(mimics.begin(), mimics.end(),
                                     [&](const auto& relation) {
                                       return relation.name == chain[i]->name;
                                     });
        includePath(it->source);
      }
    std::vector<std::string> names;
    for (const auto& joint : chain) {
      if (joint->type != urdf::Joint::FIXED &&
          joint->type != urdf::Joint::REVOLUTE &&
          joint->type != urdf::Joint::CONTINUOUS &&
          joint->type != urdf::Joint::PRISMATIC)
        throw std::invalid_argument("Unsupported joint type: " + joint->name);
      if (joint->type != urdf::Joint::FIXED &&
          (!joint->mimic || driven.count(joint->name)))
        names.push_back(joint->name);
    }
    if (names.empty()) return false;
    auto next = std::make_unique<Impl>();
    pinocchio::urdf::buildModelFromXML(chainXml(xml, links, joints),
                                       next->model);
    next->model.gravity.linear() = gravity_.data;
    next->tip = next->model.getFrameId(tip, pinocchio::BODY);
    if (next->tip >= next->model.frames.size()) return false;
    for (size_t i = 0; i < names.size(); ++i)
      if (!source->getJoint(names[i])->mimic) next->root_slots.push_back(i);
    if (next->root_slots.empty()) return false;
    const int full_count = next->model.nv,
              independent_count = next->root_slots.size();
    next->reduction = Eigen::MatrixXd::Zero(independent_count, names.size());
    for (int i = 0; i < independent_count; ++i)
      next->reduction(i, next->root_slots[i]) = 1;
    next->expansion = Eigen::MatrixXd::Zero(full_count, independent_count);
    next->actuation = Eigen::MatrixXd::Zero(names.size(), independent_count);
    next->offset = Eigen::VectorXd::Zero(full_count);
    next->active_offset = Eigen::VectorXd::Zero(names.size());
    next->full_lower.resize(full_count);
    next->full_upper.resize(full_count);
    next->full_to_active.assign(full_count, -1);
    next->source_columns.resize(full_count);
    JntArray lower(names.size()), upper(names.size()), velocity(names.size()),
        effort(names.size());
    Eigen::VectorXd full_velocity(full_count);
    for (const auto& joint : chain) {
      next->links.push_back(
          next->model.getFrameId(joint->child_link_name, pinocchio::BODY));
      if (joint->type == urdf::Joint::FIXED) continue;
      const auto id = next->model.getJointId(joint->name);
      if (id >= next->model.joints.size()) return false;
      const int full = next->model.joints[id].idx_v();
      const auto active_it = std::find(names.begin(), names.end(), joint->name);
      const int active =
          active_it == names.end() ? -1 : active_it - names.begin();
      MimicJoint relation{joint->name, joint->name, 1, 0};
      if (joint->mimic) {
        next->coupled = true;
        relation = *std::find_if(
            mimics.begin(), mimics.end(),
            [&](const auto& value) { return value.name == joint->name; });
      }
      const auto source_it =
          std::find(names.begin(), names.end(), relation.source);
      if (source_it == names.end())
        throw std::invalid_argument(
            "Mimic source is not a movable controlled joint: " +
            relation.source);
      const int root = source_it - names.begin();
      const auto column_it =
          std::find(next->root_slots.begin(), next->root_slots.end(), root);
      const int column = column_it - next->root_slots.begin();
      if (column == independent_count) return false;
      next->expansion(full, column) = relation.multiplier;
      next->offset(full) = relation.offset;
      next->source_columns[full] = column;
      next->full_to_active[full] = active;
      const double inf = std::numeric_limits<double>::infinity();
      double lo = -inf, hi = inf, vel = inf, tau = inf;
      if (joint->limits) {
        if (joint->type != urdf::Joint::CONTINUOUS) {
          lo = joint->limits->lower;
          hi = joint->limits->upper;
        }
        vel = joint->limits->velocity;
        tau = joint->limits->effort;
      }
      next->full_lower(full) = lo;
      next->full_upper(full) = hi;
      full_velocity(full) = vel;
      if (active >= 0) {
        next->actuation(active, column) = relation.multiplier;
        next->active_offset(active) = relation.offset;
        lower(active) = lo;
        upper(active) = hi;
        velocity(active) = vel;
        effort(active) = tau;
      }
    }
    next->constrainLimits(lower, upper);
    for (int i = 0; i < full_count; ++i) {
      const int column = next->source_columns[i],
                root = next->root_slots[column];
      const double scale = std::abs(next->expansion(i, column));
      if (scale > 0)
        velocity(root) = std::min(velocity(root), full_velocity(i) / scale);
    }
    next->force_lift = next->actuation;
    for (int i = 0; i < independent_count; ++i)
      next->force_lift.col(i) /= next->actuation.col(i).squaredNorm();
    next->initialize(lower, upper);
    impl_ = std::move(next);
    joint_names_ = std::move(names);
    mimic_joints_ = std::move(mimics);
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
