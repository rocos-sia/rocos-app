#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <ostream>

namespace rocos {

// SI 单位；保留现有调用形式，几何类型不依赖运动学后端。
class Vector {
   public:
    Eigen::Vector3d data{Eigen::Vector3d::Zero()};
    Vector() = default;
    Vector(double x, double y, double z) : data(x, y, z) {}
    explicit Vector(const Eigen::Vector3d& value) : data(value) {}
    static Vector Zero() { return {}; }
    double x() const { return data.x(); }
    double y() const { return data.y(); }
    double z() const { return data.z(); }
    double& x() { return data.x(); }
    double& y() { return data.y(); }
    double& z() { return data.z(); }
    void x(double value) { data.x() = value; }
    void y(double value) { data.y() = value; }
    void z(double value) { data.z() = value; }
    double operator()(int i) const { return data(i); }
    double& operator()(int i) { return data(i); }
    double operator[](int i) const { return data(i); }
    double& operator[](int i) { return data(i); }
    double Norm() const { return data.norm(); }
    double Normalize(double eps = 1e-6) {
        const double norm = Norm();
        if (norm < eps) {
            data = Eigen::Vector3d::UnitX();
            return 0;
        } else
            data /= norm;
        return norm;
    }
    Vector operator-() const { return Vector(-data); }
    Vector& operator+=(const Vector& rhs) {
        data += rhs.data;
        return *this;
    }
    Vector& operator-=(const Vector& rhs) {
        data -= rhs.data;
        return *this;
    }
    Vector& operator*=(double rhs) {
        data *= rhs;
        return *this;
    }
    Vector& operator/=(double rhs) {
        data /= rhs;
        return *this;
    }
};
inline Vector operator+(Vector a, const Vector& b) { return a += b; }
inline Vector operator-(Vector a, const Vector& b) { return a -= b; }
inline Vector operator*(Vector a, double b) { return a *= b; }
inline Vector operator*(double a, Vector b) { return b *= a; }
inline Vector operator/(Vector a, double b) { return a /= b; }
inline Vector operator*(const Vector& a, const Vector& b) {
    return Vector(a.data.cross(b.data));
}
inline double dot(const Vector& a, const Vector& b) {
    return a.data.dot(b.data);
}
inline bool Equal(const Vector& a, const Vector& b, double eps = 1e-6) {
    return (a.data - b.data).cwiseAbs().maxCoeff() <= eps;
}
inline std::ostream& operator<<(std::ostream& out, const Vector& v) {
    return out << v.data.transpose();
}

class Rotation {
   public:
    Eigen::Matrix3d data{Eigen::Matrix3d::Identity()};
    Rotation() = default;
    explicit Rotation(const Eigen::Matrix3d& value) : data(value) {}
    Rotation(double xx, double yx, double zx, double xy, double yy, double zy,
             double xz, double yz, double zz) {
        data << xx, yx, zx, xy, yy, zy, xz, yz, zz;
    }
    Rotation(const Vector& x, const Vector& y, const Vector& z) {
        data.col(0) = x.data;
        data.col(1) = y.data;
        data.col(2) = z.data;
    }
    static Rotation Identity() { return {}; }
    static Rotation Quaternion(double x, double y, double z, double w) {
        return Rotation(Eigen::Quaterniond(w, x, y, z).toRotationMatrix());
    }
    static Rotation Rot(const Vector& axis, double angle) {
        Vector unit = axis;
        const double norm = unit.Norm();
        if (norm == 0)
            unit = Vector(1, 0, 0);
        else
            unit /= norm;
        return Rotation(Eigen::AngleAxisd(angle, unit.data).toRotationMatrix());
    }
    static Rotation RotX(double angle) { return Rot(Vector(1, 0, 0), angle); }
    static Rotation RotY(double angle) { return Rot(Vector(0, 1, 0), angle); }
    static Rotation RotZ(double angle) { return Rot(Vector(0, 0, 1), angle); }
    static Rotation RPY(double roll, double pitch, double yaw) {
        return Rotation((Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                            .toRotationMatrix());
    }
    double operator()(int r, int c) const { return data(r, c); }
    double& operator()(int r, int c) { return data(r, c); }
    Vector UnitX() const { return Vector(data.col(0)); }
    Vector UnitY() const { return Vector(data.col(1)); }
    Vector UnitZ() const { return Vector(data.col(2)); }
    Rotation Inverse() const { return Rotation(data.transpose()); }
    Vector Inverse(const Vector& v) const {
        return Vector(data.transpose() * v.data);
    }
    Vector operator*(const Vector& v) const { return Vector(data * v.data); }
    Rotation operator*(const Rotation& r) const {
        return Rotation(data * r.data);
    }
    void GetQuaternion(double& x, double& y, double& z, double& w) const {
        const Eigen::Quaterniond q(data);
        x = q.x();
        y = q.y();
        z = q.z();
        w = q.w();
    }
    Vector GetRot() const {
        const Eigen::AngleAxisd aa(data);
        return Vector(aa.axis() * aa.angle());
    }
};
inline bool Equal(const Rotation& a, const Rotation& b, double eps = 1e-6) {
    return (a.data - b.data).cwiseAbs().maxCoeff() <= eps;
}

// T_A_B 把 B 中坐标变换到 A；M 为旋转，p 为平移。
struct Frame {
    Rotation M;
    Vector p;
    Frame() = default;
    Frame(const Rotation& rotation, const Vector& position)
        : M(rotation), p(position) {}
    explicit Frame(const Rotation& rotation) : M(rotation) {}
    explicit Frame(const Vector& position) : p(position) {}
    static Frame Identity() { return {}; }
    Frame Inverse() const {
        const auto r = M.Inverse();
        return Frame(r, r * -p);
    }
    Vector operator*(const Vector& point) const { return M * point + p; }
    Frame operator*(const Frame& other) const {
        return Frame(M * other.M, *this * other.p);
    }
};
inline bool Equal(const Frame& a, const Frame& b, double eps = 1e-6) {
    return Equal(a.M, b.M, eps) && Equal(a.p, b.p, eps);
}

// 线性分量在前，角分量在后；参考点和表达系由调用接口约定。
struct Twist {
    Vector vel, rot;
    Twist() = default;
    Twist(const Vector& linear, const Vector& angular)
        : vel(linear), rot(angular) {}
    static Twist Zero() { return {}; }
    double operator()(int i) const { return i < 3 ? vel(i) : rot(i - 3); }
    double& operator()(int i) { return i < 3 ? vel(i) : rot(i - 3); }
};
inline Twist operator*(const Twist& t, double s) {
    return {t.vel * s, t.rot * s};
}
inline Twist operator*(double s, const Twist& t) { return t * s; }
inline Twist operator/(const Twist& t, double s) {
    return {t.vel / s, t.rot / s};
}
inline Twist operator+(const Twist& a, const Twist& b) {
    return {a.vel + b.vel, a.rot + b.rot};
}
inline Twist operator-(const Twist& a, const Twist& b) {
    return {a.vel - b.vel, a.rot - b.rot};
}
inline Twist diff(const Frame& a, const Frame& b, double dt = 1.0) {
    return {(b.p - a.p) / dt, a.M * (a.M.Inverse() * b.M).GetRot() / dt};
}

struct Wrench {
    Vector force, torque;
    Wrench() = default;
    Wrench(const Vector& f, const Vector& t) : force(f), torque(t) {}
    static Wrench Zero() { return {}; }
    double operator()(int i) const { return i < 3 ? force(i) : torque(i - 3); }
    double& operator()(int i) { return i < 3 ? force(i) : torque(i - 3); }
};

// 关节顺序为模型 base→tip；保留 Eigen 存储以免改写现有控制器矩阵计算。
struct JntArray {
    Eigen::VectorXd data;
    JntArray() = default;
    explicit JntArray(unsigned int size) : data(Eigen::VectorXd::Zero(size)) {}
    void resize(unsigned int size) {
        data.conservativeResizeLike(Eigen::VectorXd::Zero(size));
    }
    unsigned int rows() const { return static_cast<unsigned int>(data.size()); }
    unsigned int columns() const { return 1; }
    double operator()(unsigned int i) const { return data(i); }
    double& operator()(unsigned int i) { return data(i); }
};
inline void SetToZero(JntArray& q) { q.data.setZero(); }

struct Jacobian {
    Eigen::Matrix<double, 6, Eigen::Dynamic> data;
    Jacobian() : data(6, 0) {}
    explicit Jacobian(unsigned int n) : data(6, n) { data.setZero(); }
    void resize(unsigned int n) { data.setZero(6, n); }
    unsigned int rows() const { return 6; }
    unsigned int columns() const {
        return static_cast<unsigned int>(data.cols());
    }
    double operator()(unsigned int r, unsigned int c) const {
        return data(r, c);
    }
    double& operator()(unsigned int r, unsigned int c) { return data(r, c); }
    void setColumn(unsigned int i, const Twist& t) {
        data.col(i).head<3>() = t.vel.data;
        data.col(i).tail<3>() = t.rot.data;
    }
};
}  // namespace rocos
