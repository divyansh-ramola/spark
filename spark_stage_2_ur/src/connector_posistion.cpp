// bilinear_hole_pose.cpp
// Piecewise bilinear interpolation for a 3x16 grid of connector holes.
// Returns geometry_msgs::msg::Pose for any (row, col) (1-based).
//
// How to use:
// - Edit the control_nodes vector below with measured poses.
//   Each entry is: { row, col, x, y, z, qx, qy, qz, qw }
//   Example line you can paste: {1,1, 0.729, -0.050, 0.580, 0.997, 0.027, 0.016, -0.069},
//
// Recommended control columns: {1, 4, 8, 12, 16} -> total 15 poses (3 rows × 5 cols).
// The function will return a stored pose directly if (row,col) is present in control_nodes.
// For other (row,col) it bilinearly interpolates x,y between the nearest control columns
// and linearly between rows (1↔2 or 2↔3). z and orientation are taken from base (1,1)
// unless an exact measured pose is available for the requested (row,col).
//
// NOTE: indices are 1-based: row in {1,2,3}, col in {1..16}.

#include <vector>
#include <tuple>
#include <stdexcept>
#include <cmath>
#include <optional>
#include <algorithm>
#include "geometry_msgs/msg/pose.hpp"

struct MeasuredNode {
    int row; int col;
    double x, y, z;
    double qx, qy, qz, qw;
};

// --------------------------
// EDITABLE: measured/control nodes (fill these with your measured values)
// Recommended: control columns = {1, 4, 8, 12, 16}  -> total 15 entries.
// Example formatting: {1,1, 0.241, -0.669, 0.288, -0.697, 0.717, -0.008, 0.011},
static std::vector<MeasuredNode> control_nodes = {
    // column 1
    {1,1, 0.241, -0.669, 0.288, -0.697, 0.717, -0.008, 0.011}, // (1,1)
    {2,1, 0.243, -0.672, 0.283, -0.697, 0.717, -0.008, 0.011}, // (2,1)
    {3,1, 0.246, -0.674, 0.285, -0.697, 0.717, -0.008, 0.011}, // (3,1)

    // column 4  <-- replace these example numbers with your measured (1,4),(2,4),(3,4)
    {1,4, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},
    {2,4, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},
    {3,4, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},

    // column 8  <-- fill (1,8),(2,8),(3,8)
    {1,8, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},
    {2,8, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},
    {3,8, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},

    // column 12  <-- fill (1,12),(2,12),(3,12)
    {1,12, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},
    {2,12, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},
    {3,12, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},

    // column 16  <-- fill (1,16),(2,16),(3,16)
    {1,16, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},
    {2,16, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},
    {3,16, 0.???, -0.???, 0.???, 0.???, 0.???, 0.???, 0.???},
};
// --------------------------

// Recommended control column indices (must match the columns used in control_nodes)
static const std::vector<int> control_cols = {1, 4, 8, 12, 16};

// Helper: find measured node (exact match) -> returns Pose if found.
static std::optional<geometry_msgs::msg::Pose> findMeasuredPoseExact(int row, int col)
{
    for (const auto &n : control_nodes) {
        if (n.row == row && n.col == col) {
            geometry_msgs::msg::Pose p;
            p.position.x = n.x; p.position.y = n.y; p.position.z = n.z;
            p.orientation.x = n.qx; p.orientation.y = n.qy;
            p.orientation.z = n.qz; p.orientation.w = n.qw;
            return p;
        }
    }
    return std::nullopt;
}

// Helper: fetch MeasuredNode* pointer for given (row,col) - nullptr if not found
static const MeasuredNode* getMeasuredNode(int row, int col)
{
    for (const auto &n : control_nodes) {
        if (n.row == row && n.col == col) return &n;
    }
    return nullptr;
}

// Main function: returns Pose for requested (row,col)
// - row in {1,2,3}, col in {1..16} (1-based)
// - If exact measured pose exists, returns it directly.
// - Otherwise bilinear-interpolates x & y and uses base z/orientation from (1,1).
geometry_msgs::msg::Pose getConnectorPose_bilinear(int row, int col)
{
    // -- bounds check --
    if (row < 1 || row > 3) throw std::out_of_range("row must be 1..3");
    if (col < 1 || col > 16) throw std::out_of_range("col must be 1..16");

    // -- 1) if we have an exact measured pose, return it directly (no math) --
    auto exact = findMeasuredPoseExact(row, col);
    if (exact.has_value()) return exact.value();

    // -- 2) find which control column interval this column belongs to: find j with c_j <= col <= c_{j+1}
    int j_idx = -1;
    for (size_t k = 0; k + 1 < control_cols.size(); ++k) {
        if (control_cols[k] <= col && col <= control_cols[k+1]) { j_idx = static_cast<int>(k); break; }
    }
    if (j_idx == -1) {
        // shouldn't happen if control_cols cover 1..16; guard anyway
        throw std::runtime_error("Requested column outside control column coverage. Update control_cols.");
    }
    int c_j = control_cols[j_idx];
    int c_j1 = control_cols[j_idx + 1];

    // -- 3) determine which two rows to interpolate between
    // rows only 1..3: if row in (1..2] use r0=1; if row in (2..3] use r0=2
    int r0;
    double v; // row fraction between r0 and r0+1
    if (row <= 2) { r0 = 1; v = static_cast<double>(row) - 1.0; } // row in [1,2)
    else           { r0 = 2; v = static_cast<double>(row) - 2.0; } // row in [2,3]

    // -- 4) load the four corner measured nodes:
    const MeasuredNode* n00 = getMeasuredNode(r0,   c_j);   // (r0, c_j)
    const MeasuredNode* n10 = getMeasuredNode(r0,   c_j1);  // (r0, c_j1)
    const MeasuredNode* n01 = getMeasuredNode(r0+1, c_j);   // (r0+1, c_j)
    const MeasuredNode* n11 = getMeasuredNode(r0+1, c_j1);  // (r0+1, c_j1)

    // If any corner is missing, we cannot do bilinear interpolation reliably.
    if (!(n00 && n10 && n01 && n11)) {
        throw std::runtime_error("Missing measured control nodes for bilinear interpolation. Fill control_nodes for required (row,col) control positions.");
    }

    // -- 5) compute u (column fraction between c_j and c_j1)
    double u;
    if (c_j1 == c_j) u = 0.0; // guard (shouldn't happen)
    else u = (static_cast<double>(col) - static_cast<double>(c_j)) / static_cast<double>(c_j1 - c_j);

    // -- 6) bilinear interpolation for x and y
    // x(u,v) = (1-u)(1-v)x00 + u(1-v)x10 + (1-u)v x01 + u v x11
    double x00 = n00->x, x10 = n10->x, x01 = n01->x, x11 = n11->x;
    double y00 = n00->y, y10 = n10->y, y01 = n01->y, y11 = n11->y;

    double x_interp = (1-u)*(1-v)*x00 + u*(1-v)*x10 + (1-u)*v*x01 + u*v*x11;
    double y_interp = (1-u)*(1-v)*y00 + u*(1-v)*y10 + (1-u)*v*y01 + u*v*y11;

    // -- 7) For z and orientation, use the base (1,1) measured node if it exists.
    const MeasuredNode* base = getMeasuredNode(1, 1);
    if (!base) throw std::runtime_error("Base pose (1,1) missing in control_nodes -- fill it.");

    geometry_msgs::msg::Pose out;
    out.position.x = x_interp;
    out.position.y = y_interp;
    // keep z and orientation same as base (unless exact measured pose exists which was handled above)
    out.position.z = base->z;
    out.orientation.x = base->qx;
    out.orientation.y = base->qy;
    out.orientation.z = base->qz;
    out.orientation.w = base->qw;

    return out;
}

// ------------------------------
// Optional helper: set/update a measured control node at runtime.
// You can call this from code to populate control_nodes programmatically.
void setMeasuredNode(int row, int col, double x, double y, double z,
                     double qx, double qy, double qz, double qw)
{
    // try to find existing
    for (auto &n : control_nodes) {
        if (n.row == row && n.col == col) {
            n.x = x; n.y = y; n.z = z;
            n.qx = qx; n.qy = qy; n.qz = qz; n.qw = qw;
            return;
        }
    }
    // else append new
    control_nodes.push_back({row,col,x,y,z,qx,qy,qz,qw});
}
