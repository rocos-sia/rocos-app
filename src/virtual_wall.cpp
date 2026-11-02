// Copyright 2026, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#include "virtual_wall.hpp"

#include <algorithm>
#include <cmath>

namespace {

/// @brief 计算向量的 Euclid 范数
inline double vecNorm(const rocos::Vector& v) {
    return std::sqrt(rocos::dot(v, v));
}

/// @brief 返回单位向量，零向量时返回 fallback
inline rocos::Vector normalizeSafe(const rocos::Vector& v,
                                    const rocos::Vector& fallback) {
    double len = vecNorm(v);
    return (len > 1e-12) ? v / len : fallback;
}

}  // namespace

namespace rocos {

// ============================================================================
// signedDistance
// ============================================================================

double signedDistance(const PlaneWall& wall, const Vector& point) {
    const Vector diff = point - wall.point;
    return rocos::dot(diff, wall.normal);
}

double signedDistance(const CylinderWall& wall, const Vector& point) {
    const Vector to_point = point - wall.center;
    const double proj_len = rocos::dot(to_point, wall.axis);
    const Vector proj = proj_len * wall.axis;
    const Vector radial = to_point - proj;
    const double radial_dist = vecNorm(radial);

    if (wall.mode == WallMode::InsideOnly) {
        return radial_dist - wall.radius;
    } else {
        return wall.radius - radial_dist;
    }
}

double signedDistance(const SphereWall& wall, const Vector& point) {
    const Vector to_point = point - wall.center;
    const double dist = vecNorm(to_point);

    if (wall.mode == WallMode::InsideOnly) {
        return dist - wall.radius;
    } else {
        return wall.radius - dist;
    }
}

double signedDistance(const BoxWall& wall, const Vector& point) {
    const double dx = std::max(0.0, std::max(wall.min_corner.x() - point.x(),
                                              point.x() - wall.max_corner.x()));
    const double dy = std::max(0.0, std::max(wall.min_corner.y() - point.y(),
                                              point.y() - wall.max_corner.y()));
    const double dz = std::max(0.0, std::max(wall.min_corner.z() - point.z(),
                                              point.z() - wall.max_corner.z()));
    const double d_out = std::max({dx, dy, dz});

    if (d_out > 0.0) {
        return (wall.mode == WallMode::InsideOnly) ? d_out : -d_out;
    }

    const double pen_x = std::min(point.x() - wall.min_corner.x(),
                                   wall.max_corner.x() - point.x());
    const double pen_y = std::min(point.y() - wall.min_corner.y(),
                                   wall.max_corner.y() - point.y());
    const double pen_z = std::min(point.z() - wall.min_corner.z(),
                                   wall.max_corner.z() - point.z());
    const double d_in = -std::min({pen_x, pen_y, pen_z});

    return (wall.mode == WallMode::InsideOnly) ? d_in : -d_in;
}

// ============================================================================
// wallNormal — 指向禁止侧的单位法向量（已内化 WallMode）
// ============================================================================

Vector wallNormal(const PlaneWall& wall, const Vector& /*point*/) {
    // 平面墙法向量为常量，构造时已指向禁止侧
    return wall.normal;
}

Vector wallNormal(const CylinderWall& wall, const Vector& point) {
    const Vector to_point = point - wall.center;
    const double proj_len = rocos::dot(to_point, wall.axis);
    const Vector proj = proj_len * wall.axis;
    const Vector radial = to_point - proj;

    // 几何上的径向朝外方向
    const Vector outward = normalizeSafe(radial, wall.axis);

    return (wall.mode == WallMode::InsideOnly)
        ? outward       // 禁止侧 = 圆柱外部
        : -outward;     // 禁止侧 = 圆柱内部
}

Vector wallNormal(const SphereWall& wall, const Vector& point) {
    const Vector to_center = point - wall.center;
    const Vector outward = normalizeSafe(to_center, Vector(1, 0, 0));

    return (wall.mode == WallMode::InsideOnly)
        ? outward       // 禁止侧 = 球外部
        : -outward;     // 禁止侧 = 球内部
}

Vector wallNormal(const BoxWall& wall, const Vector& point) {
    // 找点距离最近的面的朝外法向量
    const double dx_min = std::abs(point.x() - wall.min_corner.x());
    const double dx_max = std::abs(point.x() - wall.max_corner.x());
    const double dy_min = std::abs(point.y() - wall.min_corner.y());
    const double dy_max = std::abs(point.y() - wall.max_corner.y());
    const double dz_min = std::abs(point.z() - wall.min_corner.z());
    const double dz_max = std::abs(point.z() - wall.max_corner.z());

    const double d_x = std::min(dx_min, dx_max);
    const double d_y = std::min(dy_min, dy_max);
    const double d_z = std::min(dz_min, dz_max);

    Vector outward;
    if (d_x <= d_y && d_x <= d_z) {
        outward = (dx_min < dx_max) ? Vector(-1, 0, 0) : Vector(1, 0, 0);
    } else if (d_y <= d_x && d_y <= d_z) {
        outward = (dy_min < dy_max) ? Vector(0, -1, 0) : Vector(0, 1, 0);
    } else {
        outward = (dz_min < dz_max) ? Vector(0, 0, -1) : Vector(0, 0, 1);
    }

    return (wall.mode == WallMode::InsideOnly)
        ? outward       // 禁止侧 = 立方体外部
        : -outward;     // 禁止侧 = 立方体内部
}

// ============================================================================
// projectOntoBoundary — 纯几何投影到墙面（不受 WallMode 影响）
// ============================================================================

Vector projectOntoBoundary(const PlaneWall& wall, const Vector& point) {
    const double d = signedDistance(wall, point);
    return point - d * wall.normal;   // 沿法向量将点推回到平面上
}

Vector projectOntoBoundary(const CylinderWall& wall, const Vector& point) {
    const Vector to_point = point - wall.center;
    const double proj_len = rocos::dot(to_point, wall.axis);
    const Vector axial = proj_len * wall.axis;
    const Vector radial = to_point - axial;
    const double radial_len = vecNorm(radial);

    if (radial_len < 1e-12) {
        // 恰好在中心轴上，选任意径向方向
        const Vector arbitrary{1, 0, 0};
        return wall.center + wall.radius * arbitrary;
    }

    // 轴向分量不变，径向分量截断到 radius
    return wall.center + axial + (wall.radius / radial_len) * radial;
}

Vector projectOntoBoundary(const SphereWall& wall, const Vector& point) {
    const Vector to_center = point - wall.center;
    const double dist = vecNorm(to_center);

    if (dist < 1e-12) {
        return wall.center + Vector(wall.radius, 0, 0);
    }

    return wall.center + (wall.radius / dist) * to_center;
}

Vector projectOntoBoundary(const BoxWall& wall, const Vector& point) {
    // 先在各轴 clamp 到 [min, max] 区间
    const double cx = std::max(wall.min_corner.x(),
                                std::min(wall.max_corner.x(), point.x()));
    const double cy = std::max(wall.min_corner.y(),
                                std::min(wall.max_corner.y(), point.y()));
    const double cz = std::max(wall.min_corner.z(),
                                std::min(wall.max_corner.z(), point.z()));

    // 如果在某个轴上点已在区间外，clamp 后的点就在边界面上 → 直接返回
    bool outside_x = (point.x() < wall.min_corner.x() ||
                       point.x() > wall.max_corner.x());
    bool outside_y = (point.y() < wall.min_corner.y() ||
                       point.y() > wall.max_corner.y());
    bool outside_z = (point.z() < wall.min_corner.z() ||
                       point.z() > wall.max_corner.z());

    if (outside_x || outside_y || outside_z) {
        return Vector(cx, cy, cz);  // 已在某个面上（或棱/角上）
    }

    // 点在立方体内部 → 推到最近的面
    const double dx = std::min(point.x() - wall.min_corner.x(),
                                wall.max_corner.x() - point.x());
    const double dy = std::min(point.y() - wall.min_corner.y(),
                                wall.max_corner.y() - point.y());
    const double dz = std::min(point.z() - wall.min_corner.z(),
                                wall.max_corner.z() - point.z());

    double px = cx, py = cy, pz = cz;

    if (dx <= dy && dx <= dz) {
        px = (point.x() - wall.min_corner.x() < wall.max_corner.x() - point.x())
             ? wall.min_corner.x() : wall.max_corner.x();
    } else if (dy <= dx && dy <= dz) {
        py = (point.y() - wall.min_corner.y() < wall.max_corner.y() - point.y())
             ? wall.min_corner.y() : wall.max_corner.y();
    } else {
        pz = (point.z() - wall.min_corner.z() < wall.max_corner.z() - point.z())
             ? wall.min_corner.z() : wall.max_corner.z();
    }

    return Vector(px, py, pz);
}

}  // namespace rocos
