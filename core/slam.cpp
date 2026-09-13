#include "slam.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

struct ResidualLinearization {
    double e[3];
    double a[3][3];
    double b[3][3];
};

ResidualLinearization linearize(const Pose2D& pi, const Pose2D& pj,
                                const Pose2D& measurement) {
    const double c = std::cos(pi.yaw), s = std::sin(pi.yaw);
    const double dx = pj.x - pi.x, dy = pj.y - pi.y;
    const double predicted_x = c * dx + s * dy;
    const double predicted_y = -s * dx + c * dy;

    ResidualLinearization out{};
    out.e[0] = predicted_x - measurement.x;
    out.e[1] = predicted_y - measurement.y;
    out.e[2] = wrap_angle(pj.yaw - pi.yaw - measurement.yaw);

    out.a[0][0] = -c; out.a[0][1] = -s; out.a[0][2] = predicted_y;
    out.a[1][0] =  s; out.a[1][1] = -c; out.a[1][2] = -predicted_x;
    out.a[2][2] = -1.0;

    out.b[0][0] = c;  out.b[0][1] = s;
    out.b[1][0] = -s; out.b[1][1] = c;
    out.b[2][2] = 1.0;
    return out;
}

bool solve_dense(std::vector<double> matrix, std::vector<double> rhs,
                 std::vector<double>& solution) {
    const int n = static_cast<int>(rhs.size());
    for (int column = 0; column < n; ++column) {
        int pivot = column;
        for (int row = column + 1; row < n; ++row)
            if (std::fabs(matrix[row * n + column]) > std::fabs(matrix[pivot * n + column]))
                pivot = row;
        if (std::fabs(matrix[pivot * n + column]) < 1e-14) return false;
        if (pivot != column) {
            for (int k = column; k < n; ++k)
                std::swap(matrix[column * n + k], matrix[pivot * n + k]);
            std::swap(rhs[column], rhs[pivot]);
        }
        const double diagonal = matrix[column * n + column];
        for (int row = column + 1; row < n; ++row) {
            const double factor = matrix[row * n + column] / diagonal;
            if (factor == 0.0) continue;
            for (int k = column; k < n; ++k)
                matrix[row * n + k] -= factor * matrix[column * n + k];
            rhs[row] -= factor * rhs[column];
        }
    }

    solution.assign(n, 0.0);
    for (int row = n - 1; row >= 0; --row) {
        double value = rhs[row];
        for (int column = row + 1; column < n; ++column)
            value -= matrix[row * n + column] * solution[column];
        solution[row] = value / matrix[row * n + row];
    }
    return true;
}

void add_block(std::vector<double>& hessian, int dimension, int row_node, int col_node,
               const double left[3][3], const double right[3][3], const double weight[3]) {
    if (row_node == 0 || col_node == 0) return;
    const int row0 = 3 * (row_node - 1), col0 = 3 * (col_node - 1);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) {
            double value = 0.0;
            for (int k = 0; k < 3; ++k) value += left[k][r] * weight[k] * right[k][c];
            hessian[(row0 + r) * dimension + col0 + c] += value;
        }
}

}  // namespace

double wrap_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

Pose2D compose_pose(const Pose2D& base, const Pose2D& local_motion) {
    const double c = std::cos(base.yaw), s = std::sin(base.yaw);
    return {base.x + c * local_motion.x - s * local_motion.y,
            base.y + s * local_motion.x + c * local_motion.y,
            wrap_angle(base.yaw + local_motion.yaw)};
}

Pose2D relative_pose(const Pose2D& from, const Pose2D& to) {
    const double c = std::cos(from.yaw), s = std::sin(from.yaw);
    const double dx = to.x - from.x, dy = to.y - from.y;
    return {c * dx + s * dy, -s * dx + c * dy, wrap_angle(to.yaw - from.yaw)};
}

PoseGraphSLAM::PoseGraphSLAM(const Pose2D& initial) : poses_{initial} {}

int PoseGraphSLAM::add_odometry(const Pose2D& measurement, double sigma_translation,
                                double sigma_yaw) {
    const int from = static_cast<int>(poses_.size()) - 1;
    poses_.push_back(compose_pose(poses_.back(), measurement));
    constraints_.push_back({from, from + 1, measurement, sigma_translation, sigma_yaw, false});
    return from + 1;
}

void PoseGraphSLAM::add_loop_closure(int from, int to, const Pose2D& measurement,
                                    double sigma_translation, double sigma_yaw) {
    if (from < 0 || to < 0 || from >= static_cast<int>(poses_.size()) ||
        to >= static_cast<int>(poses_.size()) || from == to)
        throw std::out_of_range("loop-closure pose index");
    constraints_.push_back({from, to, measurement, sigma_translation, sigma_yaw, true});
}

std::vector<double> PoseGraphSLAM::information_matrix(double huber_delta) const {
    const int dimension = 3 * (static_cast<int>(poses_.size()) - 1);
    std::vector<double> hessian(static_cast<size_t>(dimension) * dimension, 0.0);
    for (const PoseConstraint2D& edge : constraints_) {
        const ResidualLinearization q = linearize(poses_[edge.from], poses_[edge.to], edge.measurement);
        const double base_weight[3] = {
            1.0 / (edge.sigma_translation * edge.sigma_translation),
            1.0 / (edge.sigma_translation * edge.sigma_translation),
            1.0 / (edge.sigma_yaw * edge.sigma_yaw)};
        const double normalized = std::sqrt(q.e[0] * q.e[0] * base_weight[0] +
                                            q.e[1] * q.e[1] * base_weight[1] +
                                            q.e[2] * q.e[2] * base_weight[2]);
        const double robust = normalized > huber_delta ? huber_delta / normalized : 1.0;
        const double weight[3] = {base_weight[0] * robust, base_weight[1] * robust,
                                  base_weight[2] * robust};
        add_block(hessian, dimension, edge.from, edge.from, q.a, q.a, weight);
        add_block(hessian, dimension, edge.from, edge.to, q.a, q.b, weight);
        add_block(hessian, dimension, edge.to, edge.from, q.b, q.a, weight);
        add_block(hessian, dimension, edge.to, edge.to, q.b, q.b, weight);
    }
    return hessian;
}

double PoseGraphSLAM::optimize(int iterations, double huber_delta) {
    const int dimension = 3 * (static_cast<int>(poses_.size()) - 1);
    if (dimension == 0) return 0.0;
    double cost = 0.0;

    for (int iteration = 0; iteration < iterations; ++iteration) {
        std::vector<double> hessian(static_cast<size_t>(dimension) * dimension, 0.0);
        std::vector<double> gradient(dimension, 0.0);
        cost = 0.0;

        for (const PoseConstraint2D& edge : constraints_) {
            const ResidualLinearization q = linearize(poses_[edge.from], poses_[edge.to], edge.measurement);
            const double base_weight[3] = {
                1.0 / (edge.sigma_translation * edge.sigma_translation),
                1.0 / (edge.sigma_translation * edge.sigma_translation),
                1.0 / (edge.sigma_yaw * edge.sigma_yaw)};
            const double normalized = std::sqrt(q.e[0] * q.e[0] * base_weight[0] +
                                                q.e[1] * q.e[1] * base_weight[1] +
                                                q.e[2] * q.e[2] * base_weight[2]);
            const double robust = normalized > huber_delta ? huber_delta / normalized : 1.0;
            const double weight[3] = {base_weight[0] * robust, base_weight[1] * robust,
                                      base_weight[2] * robust};
            cost += normalized <= huber_delta ? 0.5 * normalized * normalized
                                              : huber_delta * (normalized - 0.5 * huber_delta);

            add_block(hessian, dimension, edge.from, edge.from, q.a, q.a, weight);
            add_block(hessian, dimension, edge.from, edge.to, q.a, q.b, weight);
            add_block(hessian, dimension, edge.to, edge.from, q.b, q.a, weight);
            add_block(hessian, dimension, edge.to, edge.to, q.b, q.b, weight);

            for (int node_index : {edge.from, edge.to}) {
                if (node_index == 0) continue;
                const double (*jacobian)[3] = node_index == edge.from ? q.a : q.b;
                const int offset = 3 * (node_index - 1);
                for (int column = 0; column < 3; ++column)
                    for (int row = 0; row < 3; ++row)
                        gradient[offset + column] += jacobian[row][column] * weight[row] * q.e[row];
            }
        }

        for (int i = 0; i < dimension; ++i) hessian[i * dimension + i] += 1e-9;
        for (double& value : gradient) value = -value;
        std::vector<double> delta;
        if (!solve_dense(hessian, gradient, delta)) throw std::runtime_error("singular pose graph");

        double largest = 0.0;
        for (int node = 1; node < static_cast<int>(poses_.size()); ++node) {
            const int offset = 3 * (node - 1);
            poses_[node].x += delta[offset];
            poses_[node].y += delta[offset + 1];
            poses_[node].yaw = wrap_angle(poses_[node].yaw + delta[offset + 2]);
            largest = std::max(largest, std::max(std::fabs(delta[offset]),
                                      std::max(std::fabs(delta[offset + 1]), std::fabs(delta[offset + 2]))));
        }
        if (largest < 1e-9) break;
    }
    return cost;
}

const std::vector<Pose2D>& PoseGraphSLAM::poses() const { return poses_; }
const std::vector<PoseConstraint2D>& PoseGraphSLAM::constraints() const { return constraints_; }

std::vector<PoseUncertainty2D> PoseGraphSLAM::uncertainties() const {
    std::vector<PoseUncertainty2D> result(poses_.size());
    const int dimension = 3 * (static_cast<int>(poses_.size()) - 1);
    if (dimension == 0) return result;
    std::vector<double> hessian = information_matrix(1e12);
    for (int i = 0; i < dimension; ++i) hessian[i * dimension + i] += 1e-12;

    for (int column = 0; column < dimension; ++column) {
        std::vector<double> rhs(dimension, 0.0), inverse_column;
        rhs[column] = 1.0;
        if (!solve_dense(hessian, rhs, inverse_column)) throw std::runtime_error("singular covariance");
        const double sigma = std::sqrt(std::max(0.0, inverse_column[column]));
        PoseUncertainty2D& uncertainty = result[1 + column / 3];
        if (column % 3 == 0) uncertainty.sigma_x = sigma;
        else if (column % 3 == 1) uncertainty.sigma_y = sigma;
        else uncertainty.sigma_yaw = sigma;
    }
    return result;
}

PlanarImuIntegrator::PlanarImuIntegrator(const Pose2D& initial, double velocity_x,
                                         double velocity_y) {
    state_.pose = initial;
    state_.velocity_x = velocity_x;
    state_.velocity_y = velocity_y;
}

ImuState2D PlanarImuIntegrator::step(double acceleration_body_x, double acceleration_body_y,
                                     double yaw_rate, double dt) {
    const double yaw_mid = state_.pose.yaw + 0.5 * yaw_rate * dt;
    const double c = std::cos(yaw_mid), s = std::sin(yaw_mid);
    const double acceleration_world_x = c * acceleration_body_x - s * acceleration_body_y;
    const double acceleration_world_y = s * acceleration_body_x + c * acceleration_body_y;
    state_.pose.x += state_.velocity_x * dt + 0.5 * acceleration_world_x * dt * dt;
    state_.pose.y += state_.velocity_y * dt + 0.5 * acceleration_world_y * dt * dt;
    state_.velocity_x += acceleration_world_x * dt;
    state_.velocity_y += acceleration_world_y * dt;
    state_.pose.yaw = wrap_angle(state_.pose.yaw + yaw_rate * dt);
    return state_;
}

const ImuState2D& PlanarImuIntegrator::state() const { return state_; }
