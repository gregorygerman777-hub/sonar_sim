#pragma once

#include <vector>

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct PoseUncertainty2D {
    double sigma_x = 0.0;
    double sigma_y = 0.0;
    double sigma_yaw = 0.0;
};

struct PoseConstraint2D {
    int from = 0;
    int to = 0;
    Pose2D measurement;
    double sigma_translation = 0.1;
    double sigma_yaw = 0.05;
    bool loop_closure = false;
};

double wrap_angle(double angle);
Pose2D compose_pose(const Pose2D& base, const Pose2D& local_motion);
Pose2D relative_pose(const Pose2D& from, const Pose2D& to);

// A small SE(2) pose graph. The first pose is fixed to remove the global
// translation/yaw gauge freedom; odometry creates nodes and loop constraints
// pull revisited places back into agreement.
class PoseGraphSLAM {
public:
    explicit PoseGraphSLAM(const Pose2D& initial);

    int add_odometry(const Pose2D& measurement, double sigma_translation, double sigma_yaw);
    void add_loop_closure(int from, int to, const Pose2D& measurement,
                          double sigma_translation, double sigma_yaw);
    double optimize(int iterations, double huber_delta);

    const std::vector<Pose2D>& poses() const;
    const std::vector<PoseConstraint2D>& constraints() const;
    std::vector<PoseUncertainty2D> uncertainties() const;

private:
    std::vector<Pose2D> poses_;
    std::vector<PoseConstraint2D> constraints_;
    std::vector<double> information_matrix(double huber_delta) const;
};

struct ImuState2D {
    Pose2D pose;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
};

// Planar strapdown integration. Acceleration is measured in the moving body
// frame and yaw rate in rad/s; biases and noise are intentionally supplied by
// the caller so experiments can make them explicit.
class PlanarImuIntegrator {
public:
    PlanarImuIntegrator(const Pose2D& initial, double velocity_x, double velocity_y);
    ImuState2D step(double acceleration_body_x, double acceleration_body_y,
                    double yaw_rate, double dt);
    const ImuState2D& state() const;

private:
    ImuState2D state_;
};
