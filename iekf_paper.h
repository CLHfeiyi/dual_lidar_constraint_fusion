#pragma once
#ifndef _IEKF_PAPER_H_
#define _IEKF_PAPER_H_

#include "utility.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * IEKF (Iterated Extended Kalman Filter) for LiDAR-IMU Tightly-Coupled Fusion
 *
 * Based on paper: "煤矿巷道环境下基于激光雷达与IMU的鲁棒三维重建方法研究"
 *
 * Implements equations (3)-(17) from the paper:
 *   - State definition: x = [position, velocity, orientation, acc_bias, gyro_bias, gravity]^T  (eq.3)
 *   - Error state definition: delta_x (eq.4)
 *   - Posterior state: x_post = x_prior ⊕ delta_x (eq.5)
 *   - IMU prediction model: continuous-time linearization (eq.6-8)
 *   - IMU discrete propagation: F_k, Q_k (eq.9-10)
 *   - LiDAR observation model: point-line/point-plane residual (eq.11-12)
 *   - Iterative Kalman update: cost function (eq.13)
 *   - Kalman gain: K (eq.14)
 *   - Posterior state update: x_hat (eq.15-16)
 *   - Global pose transformation (eq.17)
 */

// State dimension: [orientation(3), position(3), velocity(3), gyro_bias(3), acc_bias(3), gravity(3)]
#define IEKF_STATE_DIM 18

// State vector indices (matching paper eq.3)
#define IEKF_IDX_ROT    0   // orientation (3)
#define IEKF_IDX_POS    3   // position (3)
#define IEKF_IDX_VEL    6   // velocity (3)
#define IEKF_IDX_BA     9   // accelerometer bias (3)
#define IEKF_IDX_BG    12   // gyroscope bias (3)
#define IEKF_IDX_GRV   15   // gravity vector (3)

class IEKFStatePaper
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Full state (paper eq.3)
    Eigen::Quaterniond rotation;   // orientation q
    Eigen::Vector3d position;      // position p
    Eigen::Vector3d velocity;      // velocity v
    Eigen::Vector3d acc_bias;      // accelerometer bias b_a
    Eigen::Vector3d gyro_bias;     // gyroscope bias b_g
    Eigen::Vector3d gravity;       // gravity vector g
    double g_mag_init;             // initial gravity magnitude (fixed)

    // Error-state covariance (18x18)
    Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM> P;

    IEKFStatePaper()
    {
        rotation = Eigen::Quaterniond::Identity();
        position.setZero();
        velocity.setZero();
        acc_bias.setZero();
        gyro_bias.setZero();
        gravity << 0.0, 0.0, -9.80511;
        g_mag_init = 9.80511;
        P.setZero();
        // Initial covariance (paper section 2.2.1)
        P.diagonal() << 1e-2, 1e-2, 1e-2,     // rotation
                         1e-1, 1e-1, 1e-1,     // position
                         1e-1, 1e-1, 1e-1,     // velocity
                         1e-3, 1e-3, 1e-3,     // acc bias
                         1e-3, 1e-3, 1e-3,     // gyro bias
                         1e-2, 1e-2, 1e-2;    // gravity
    }

    // Error state ⊕ operation (paper eq.5)
    void applyErrorState(const Eigen::Matrix<double, IEKF_STATE_DIM, 1>& dx)
    {
        // Rotation update via quaternion multiplication
        Eigen::Vector3d dtheta = dx.segment<3>(IEKF_IDX_ROT);
        double angle = dtheta.norm();
        if (angle > 1e-10) {
            Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, dtheta / angle));
            rotation = dq * rotation;
            rotation.normalize();
        }
        // Additive updates for other states
        position += dx.segment<3>(IEKF_IDX_POS);
        velocity += dx.segment<3>(IEKF_IDX_VEL);
        acc_bias += dx.segment<3>(IEKF_IDX_BA);
        gyro_bias += dx.segment<3>(IEKF_IDX_BG);
        gravity  += dx.segment<3>(IEKF_IDX_GRV);
        // Soft gravity normalization: maintain magnitude, allow direction to evolve.
        // The gravitational direction in world frame is determined by the IMU
        // initialization and then refined by the IEKF update. Forcing sign can
        // cause 180° orientation flips when gravity.z() crosses zero from
        // numerical noise.
        double g_norm = gravity.norm();
        if (g_norm > 1e-6) {
            gravity *= g_mag_init / g_norm;
        } else {
            gravity << 0.0, 0.0, -g_mag_init;
        }
    }

    // Convert to transformation matrix (paper eq.17)
    Eigen::Isometry3d toIsometry() const
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = rotation.toRotationMatrix();
        T.translation() = position;
        return T;
    }

    // Get rotation matrix
    Eigen::Matrix3d R() const { return rotation.toRotationMatrix(); }

    static Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& q)
    {
        Eigen::Vector3d e = q.toRotationMatrix().eulerAngles(2, 1, 0);
        if (fabs(e.y()) > M_PI / 2.0)
        {
            e.y() = (e.y() > 0 ? M_PI : -M_PI) - e.y();
            e.x() += M_PI;
            e.z() += M_PI;
        }
        auto wrap = [](double& a) { a = fmod(a + M_PI, 2*M_PI); if (a < 0) a += 2*M_PI; a -= M_PI; };
        wrap(e.x()); wrap(e.z());
        return e;
    }
};

class IEKFPaper
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // IMU noise parameters (paper eq.10)
    double noise_acc;       // accelerometer noise
    double noise_gyro;      // gyroscope noise
    double noise_acc_bias;  // accelerometer bias random walk
    double noise_gyro_bias; // gyroscope bias random walk
    double gravity_mag;

    // Iterative update parameters (paper eq.13-16)
    double convergence_threshold;
    int max_iterations;

    // Current state
    IEKFStatePaper state;
    bool initialized;

    // Process noise covariance Q (18x18) (paper eq.10)
    Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM> Q;

    IEKFPaper()
    {
        noise_acc = 0.01;
        noise_gyro = 0.001;
        noise_acc_bias = 0.0002;
        noise_gyro_bias = 0.0003;
        gravity_mag = 9.80511;

        convergence_threshold = 5e-3;
        max_iterations = 5;

        initialized = false;
        initProcessNoise();
    }

    void initProcessNoise()
    {
        Q.setZero();
        Q.block<3,3>(IEKF_IDX_ROT, IEKF_IDX_ROT) = Eigen::Matrix3d::Identity() * noise_gyro * noise_gyro;
        Q.block<3,3>(IEKF_IDX_POS, IEKF_IDX_POS) = Eigen::Matrix3d::Identity() * noise_acc * noise_acc;
        Q.block<3,3>(IEKF_IDX_VEL, IEKF_IDX_VEL) = Eigen::Matrix3d::Identity() * noise_acc * noise_acc;
        Q.block<3,3>(IEKF_IDX_BA,  IEKF_IDX_BA)  = Eigen::Matrix3d::Identity() * noise_acc_bias * noise_acc_bias;
        Q.block<3,3>(IEKF_IDX_BG,  IEKF_IDX_BG)  = Eigen::Matrix3d::Identity() * noise_gyro_bias * noise_gyro_bias;
        Q.block<3,3>(IEKF_IDX_GRV, IEKF_IDX_GRV) = Eigen::Matrix3d::Identity() * 1e-6;
    }

    /**
     * Initialize state from IMU measurement
     * Aligns initial orientation with gravity direction
     */
    void initialize(const Eigen::Vector3d& acc0, const Eigen::Vector3d& gyr0)
    {
        Eigen::Vector3d g_init = acc0.normalized();
        Eigen::Vector3d g_ref(0, 0, 1.0);

        Eigen::Vector3d cross = g_ref.cross(g_init);
        double dot = g_ref.dot(g_init);

        if (cross.norm() > 1e-6) {
            double angle = std::acos(std::max(-1.0, std::min(1.0, dot)));
            Eigen::Vector3d axis = cross.normalized();
            state.rotation = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
        }

        state.position.setZero();
        state.velocity.setZero();
        state.acc_bias.setZero();
        state.gyro_bias.setZero();
        state.gravity << 0.0, 0.0, -gravity_mag;
        state.g_mag_init = gravity_mag;

        state.P.setZero();
        state.P.diagonal() << 1e-2, 1e-2, 1e-2,
                               1e-1, 1e-1, 1e-1,
                               1e-1, 1e-1, 1e-1,
                               1e-4, 1e-4, 1e-4,
                               1e-4, 1e-4, 1e-4,
                               1e-2, 1e-2, 1e-2;

        Eigen::Vector3d euler_init = IEKFStatePaper::quaternionToEuler(state.rotation);
        ROS_INFO("IEKF INIT: rpy=(%.2f,%.2f,%.2f) deg acc_norm=%.3f",
                 euler_init.x()*180/M_PI, euler_init.y()*180/M_PI, euler_init.z()*180/M_PI, acc0.norm());
        initialized = true;
    }

    /**
     * IMU forward propagation (prediction step)
     * Implements paper equations (6)-(10)
     *
     * Continuous-time model: dx/dt = F_t * x + G_t * n  (eq.6)
     * First-order Taylor expansion of F_t and G_t  (eq.7-8)
     * Discrete propagation:  (eq.9-10)
     *   P_k|k-1 = F_k * P_{k-1|k-1} * F_k^T + Q_k
     */
    void predict(const Eigen::Vector3d& acc_raw, const Eigen::Vector3d& gyr_raw, double dt)
    {
        if (!initialized || dt <= 0) return;

        // Remove bias (paper section 2.2.2)
        Eigen::Vector3d acc = acc_raw - state.acc_bias;
        Eigen::Vector3d gyr = gyr_raw - state.gyro_bias;

        Eigen::Matrix3d R_before = state.R();

        // --- State propagation ---
        // Rotation update via mid-point integration
        Eigen::Vector3d half_gyr = 0.5 * gyr * dt;
        Eigen::Quaterniond dq(1.0, half_gyr.x(), half_gyr.y(), half_gyr.z());
        dq.normalize();
        state.rotation = state.rotation * dq;
        state.rotation.normalize();

        // Mid-point rotation for dynamics (between old and new orientation)
        Eigen::Vector3d quarter_gyr = 0.25 * gyr * dt;
        Eigen::Quaterniond dq_half(1.0, quarter_gyr.x(), quarter_gyr.y(), quarter_gyr.z());
        dq_half.normalize();
        Eigen::Quaterniond q_mid = Eigen::Quaterniond(R_before) * dq_half;
        q_mid.normalize();
        Eigen::Matrix3d R_mid = q_mid.toRotationMatrix();

        // Acceleration in world frame (using mid-point rotation)
        Eigen::Vector3d acc_world = R_mid * acc + state.gravity;

        // Position and velocity update (mid-point)
        state.position += state.velocity * dt + 0.5 * acc_world * dt * dt;
        state.velocity += acc_world * dt;

        // --- Error-state transition matrix F_t (paper eq.7) ---
        // First-order Taylor expansion of state transition Jacobian
        Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM> F =
            Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM>::Identity();

        Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d O3 = Eigen::Matrix3d::Zero();

        // df/d(rotation) - attitude dynamics
        F.block<3,3>(IEKF_IDX_ROT, IEKF_IDX_ROT) = I3;

        // df/d(gyro_bias) - attitude depends on gyro bias
        F.block<3,3>(IEKF_IDX_ROT, IEKF_IDX_BG) = -R_mid * dt;

        // df/d(position)
        F.block<3,3>(IEKF_IDX_POS, IEKF_IDX_POS) = I3;
        F.block<3,3>(IEKF_IDX_POS, IEKF_IDX_VEL) = I3 * dt;

        // df/d(velocity) - velocity depends on rotation and acc_bias
        Eigen::Matrix3d acc_skew = skewSymmetric(R_mid * acc);
        F.block<3,3>(IEKF_IDX_VEL, IEKF_IDX_ROT) = -acc_skew * dt;
        F.block<3,3>(IEKF_IDX_VEL, IEKF_IDX_VEL) = I3;
        F.block<3,3>(IEKF_IDX_VEL, IEKF_IDX_BA)  = -R_mid * dt;

        // --- Noise Jacobian G_t (paper eq.8) ---
        // Noise input matrix for the process noise propagation
        Eigen::Matrix<double, IEKF_STATE_DIM, 12> G;
        G.setZero();
        G.block<3,3>(IEKF_IDX_ROT, 0) = -I3;
        G.block<3,3>(IEKF_IDX_VEL, 3) = -R_mid;
        G.block<3,3>(IEKF_IDX_BA,  6) = I3;
        G.block<3,3>(IEKF_IDX_BG,  9) = I3;

        // --- Discrete propagation (paper eq.9-10) ---
        // F already contains dt in off-diagonal blocks; use it directly as F_k
        // P_{k|k-1} = F_k * P_{k-1|k-1} * F_k^T + Q_k
        Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM> Fk = F;

        // Process noise covariance (paper eq.10)
        Eigen::Matrix<double, 12, 12> Q_noise;
        Q_noise.setZero();
        Q_noise.block<3,3>(0,0) = I3 * noise_gyro * noise_gyro;
        Q_noise.block<3,3>(3,3) = I3 * noise_acc * noise_acc;
        Q_noise.block<3,3>(6,6) = I3 * noise_acc_bias * noise_acc_bias;
        Q_noise.block<3,3>(9,9) = I3 * noise_gyro_bias * noise_gyro_bias;

        Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM> Qk = G * Q_noise * G.transpose() * dt;

        // Covariance propagation
        state.P = Fk * state.P * Fk.transpose() + Qk;
    }

    /**
     * LiDAR observation model: point-line and point-plane residuals
     * Implements paper equations (11)-(12)
     *
     * For edge feature points: point-to-line distance (eq.12 top)
     * For plane feature points: point-to-plane distance (eq.12 bottom)
     */
    struct ObservationResidual
    {
        Eigen::Vector3d residual;                        // residual vector
        Eigen::Matrix<double, 3, IEKF_STATE_DIM> Jacobian; // Jacobian H
    };

    /**
     * Compute observation residual for edge feature (point-to-line)
     * Paper eq.12 (top): distance from point to line defined by two nearest points
     */
    ObservationResidual computeEdgeResidual(
        const Eigen::Vector3d& point_lidar,       // point in LiDAR frame
        const Eigen::Vector3d& point_line_a,       // line point A in map
        const Eigen::Vector3d& point_line_b,       // line point B in map
        const Eigen::Isometry3d& T_imu_lidar)      // extrinsic: LiDAR to IMU
    {
        ObservationResidual obs;

        // Transform LiDAR point to IMU frame
        Eigen::Vector3d p_imu = T_imu_lidar.linear() * point_lidar + T_imu_lidar.translation();

        // Transform to world frame using current state (paper eq.12)
        Eigen::Matrix3d R = state.R();
        Eigen::Vector3d p_world = R * p_imu + state.position;

        // Line direction
        Eigen::Vector3d line_dir = (point_line_b - point_line_a).normalized();
        Eigen::Vector3d pa = p_world - point_line_a;

        // Point-to-line distance: ||pa - (pa·l)*l|| (paper eq.12)
        Eigen::Vector3d residual_vec = pa - (pa.dot(line_dir)) * line_dir;
        double dist = residual_vec.norm();

        if (dist < 1e-6) {
            obs.residual.setZero();
            obs.Jacobian.setZero();
            return obs;
        }

        // Normalize residual
        obs.residual = residual_vec;

        // Jacobian: d(residual)/d(state)
        // d(p_world)/d(rotation) = -R * skew(p_imu)
        // d(p_world)/d(position) = I
        obs.Jacobian.setZero();
        obs.Jacobian.block<3,3>(0, IEKF_IDX_ROT) = -skewSymmetric(R * p_imu);
        obs.Jacobian.block<3,3>(0, IEKF_IDX_POS) = Eigen::Matrix3d::Identity();

        return obs;
    }

    /**
     * Compute observation residual for plane feature (point-to-plane)
     * Paper eq.12 (bottom): distance from point to plane
     */
    ObservationResidual computePlaneResidual(
        const Eigen::Vector3d& point_lidar,       // point in LiDAR frame
        const Eigen::Vector3d& point_plane,       // plane point in map
        const Eigen::Vector3d& plane_normal,      // plane normal
        const Eigen::Isometry3d& T_imu_lidar)     // extrinsic
    {
        ObservationResidual obs;

        Eigen::Vector3d p_imu = T_imu_lidar.linear() * point_lidar + T_imu_lidar.translation();
        Eigen::Matrix3d R = state.R();
        Eigen::Vector3d p_world = R * p_imu + state.position;

        // Point-to-plane distance (paper eq.12 bottom)
        // Scalar residual: signed distance from point to plane
        double dist = (p_world - point_plane).dot(plane_normal);

        // Store scalar residual and 1×18 Jacobian
        // The distance is the projection of the residual vector onto the plane normal
        obs.residual = plane_normal * dist;  // 3D vector residual
        obs.Jacobian.setZero();
        // d(dist)/d(theta) = n^T * (-skew(R * p_imu)), d(dist)/d(t) = n^T
        Eigen::Matrix<double, 1, 3> nT = plane_normal.transpose();
        obs.Jacobian.block<1,3>(0, IEKF_IDX_ROT) = nT * (-skewSymmetric(R * p_imu));
        obs.Jacobian.block<1,3>(0, IEKF_IDX_POS) = nT;

        return obs;
    }

    /**
     * Iterative Kalman filter update (paper eq.13-16)
     *
     * Nonlinear cost function (eq.13):
     *   J(dx) = ||dx||_{P^{-1}} + ||z - h(x⊕dx)||_{R^{-1}}
     *
     * Kalman gain (eq.14):
     *   K = P * H^T * (H * P * H^T + R)^{-1}
     *
     * Posterior update (eq.15):
     *   dx_hat = K * (z - h(x_prior))
     *
     * Convergence check (eq.16):
     *   ||dx_hat^{k+1} - dx_hat^k|| < threshold
     *
     * Global pose (eq.17):
     *   T_world_imu = [R, p; 0, 1]
     */
    struct UpdateResult
    {
        bool converged;
        int iterations;
        double cost;
    };

    UpdateResult updateWithFeatures(
        const std::vector<ObservationResidual>& observations,
        const Eigen::MatrixXd& R_obs)
    {
        UpdateResult result;
        result.converged = false;
        result.iterations = 0;
        result.cost = 0;

        if (observations.empty() || !initialized)
            return result;

        int N = observations.size();
        int obs_dim = 3; // 3D residual per observation

        // Save prior state
        IEKFStatePaper state_prior = state;

        // Prior information matrix
        Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM> P_inv =
            state.P.inverse();

        Eigen::Matrix<double, IEKF_STATE_DIM, 1> dx_prev =
            Eigen::Matrix<double, IEKF_STATE_DIM, 1>::Zero();

        for (int iter = 0; iter < max_iterations; ++iter)
        {
            result.iterations = iter + 1;

            // Stack all observation Jacobians and residuals
            Eigen::MatrixXd H_total(obs_dim * N, IEKF_STATE_DIM);
            Eigen::VectorXd r_total(obs_dim * N);

            for (int i = 0; i < N; ++i)
            {
                H_total.block<3, IEKF_STATE_DIM>(3*i, 0) = observations[i].Jacobian;
                r_total.segment<3>(3*i) = observations[i].residual;
            }

            // Kalman gain (paper eq.14)
            // K = P * H^T * (H * P * H^T + R)^{-1}
            Eigen::MatrixXd S = H_total * state.P * H_total.transpose() + R_obs;
            Eigen::MatrixXd K = state.P * H_total.transpose() * S.inverse();

            // Posterior state update (paper eq.15)
            Eigen::Matrix<double, IEKF_STATE_DIM, 1> dx = K * r_total;

            // Convergence check (paper eq.16)
            double delta = (dx - dx_prev).norm();
            if (delta < convergence_threshold && iter > 0)
            {
                result.converged = true;
                // Apply error state to prior (paper eq.5)
                state.applyErrorState(dx);
                // Update covariance
                Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM> I_KH =
                    Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM>::Identity() - (K * H_total).eval();
                state.P = (I_KH * state.P).eval();
                result.cost = 0.5 * (dx.transpose() * P_inv * dx).value() +
                              0.5 * (r_total.transpose() * R_obs.inverse() * r_total).value();
                return result;
            }

            dx_prev = dx;

            // Apply update
            state.applyErrorState(dx);

            // Update covariance (Joseph form for numerical stability)
            Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM> I_KH =
                Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM>::Identity() - (K * H_total).eval();
            state.P = (I_KH * state.P * I_KH.transpose() + K * R_obs * K.transpose()).eval();

            // Recompute residuals with updated state for next iteration
            // (residuals are recomputed externally by caller)
            result.cost = 0.5 * (r_total.transpose() * R_obs.inverse() * r_total).value();
        }

        return result;
    }

    /**
     * Get global pose transformation (paper eq.17)
     * T_world_imu = [R(q), p; 0, 1]
     */
    Eigen::Isometry3d getGlobalPose() const
    {
        return state.toIsometry();
    }

    /**
     * Get 6x6 pose covariance (rotation + position)
     */
    Eigen::Matrix<double, 6, 6> getPoseCovariance() const
    {
        Eigen::Matrix<double, 6, 6> cov;
        cov.setZero();
        cov.block<3,3>(0,0) = state.P.block<3,3>(IEKF_IDX_ROT, IEKF_IDX_ROT);
        cov.block<3,3>(3,3) = state.P.block<3,3>(IEKF_IDX_POS, IEKF_IDX_POS);
        return cov;
    }

    // Skew-symmetric matrix utility
    static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d S;
        S <<     0, -v.z(),  v.y(),
             v.z(),     0, -v.x(),
            -v.y(),  v.x(),     0;
        return S;
    }

    void reset()
    {
        initialized = false;
        state = IEKFStatePaper();
    }
};

#endif // _IEKF_PAPER_H_
