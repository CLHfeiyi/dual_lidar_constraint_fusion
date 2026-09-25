#pragma once
#ifndef _VGICP_LOOP_H_
#define _VGICP_LOOP_H_

#include "utility.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

/**
 * VGICP (Voxelized Generalized ICP) Loop Closure Detection
 *
 * Based on paper section 3: "基于VGICP回环检测的巷道模型累计误差校正方法"
 *
 * Implements equations (19)-(28):
 *   - Gaussian distribution assumption for points (eq.19-20)
 *   - Neighborhood distance sum (eq.21)
 *   - Distribution of transformed points (eq.23-25)
 *   - Likelihood maximization for transform estimation (eq.26-27)
 *   - Loop candidate selection criteria (eq.28)
 */

// Voxel structure for VGICP
struct VGICPVoxel
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d mean;                          // Point mean
    Eigen::Matrix3d covariance;                    // Point covariance
    int num_points;                                // Number of points in voxel

    VGICPVoxel() : mean(Eigen::Vector3d::Zero()),
                   covariance(Eigen::Matrix3d::Identity()),
                   num_points(0) {}
};

// Type alias for voxel grid
using VGICPVoxelMap = std::unordered_map<Eigen::Vector3i, VGICPVoxel,
    std::function<size_t(const Eigen::Vector3i&)>>;

class VGICPLoopDetector
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // VGICP parameters
    double voxel_resolution;         // Voxel size for spatial hashing
    double neighbor_search_radius;   // Radius for nearest neighbor search (paper eq.22)
    int max_iterations;              // Maximum VGICP iterations
    double convergence_threshold;    // Convergence tolerance
    double search_radius;            // Loop search radius for keyframes (paper eq.28)
    double distance_threshold;       // Euclidean distance threshold (paper eq.28)

    // Keyframe parameters
    double keyframe_distance;        // Minimum distance between keyframes
    double keyframe_angle;           // Minimum rotation between keyframes
    double time_diff_threshold;      // Minimum time difference for loop candidates

    VGICPLoopDetector()
    {
        voxel_resolution = 1.0;
        neighbor_search_radius = 1.5;
        max_iterations = 30;
        convergence_threshold = 1e-3;
        search_radius = 15.0;
        distance_threshold = 5.0;
        keyframe_distance = 1.0;
        keyframe_angle = 0.2;
        time_diff_threshold = 30.0;
    }

    /**
     * Voxel hash function for 3D grid
     */
    static inline Eigen::Vector3i voxelIndex(const Eigen::Vector3d& point, double resolution)
    {
        return Eigen::Vector3i(
            static_cast<int>(std::floor(point.x() / resolution)),
            static_cast<int>(std::floor(point.y() / resolution)),
            static_cast<int>(std::floor(point.z() / resolution)));
    }

    static size_t voxelHash(const Eigen::Vector3i& idx)
    {
        // Large prime numbers for hashing
        const size_t p1 = 73856093;
        const size_t p2 = 19349669;
        const size_t p3 = 83492791;
        return static_cast<size_t>(idx.x()) * p1 ^
               static_cast<size_t>(idx.y()) * p2 ^
               static_cast<size_t>(idx.z()) * p3;
    }

    /**
     * Build voxel map from point cloud
     * Each voxel stores a Gaussian distribution (mean + covariance) (paper eq.19-20)
     */
    VGICPVoxelMap buildVoxelMap(const pcl::PointCloud<PointType>::Ptr& cloud)
    {
        VGICPVoxelMap vmap(1000, [](const Eigen::Vector3i& k) -> size_t {
            return voxelHash(k);
        });

        // First pass: accumulate points into voxels
        for (const auto& pt : cloud->points)
        {
            Eigen::Vector3d p(pt.x, pt.y, pt.z);
            Eigen::Vector3i idx = voxelIndex(p, voxel_resolution);

            auto it = vmap.find(idx);
            if (it == vmap.end())
            {
                VGICPVoxel voxel;
                voxel.mean = p;
                voxel.num_points = 1;
                voxel.covariance.setZero();
                vmap[idx] = voxel;
            }
            else
            {
                // Online mean update
                int n = it->second.num_points;
                it->second.num_points++;
                it->second.mean += (p - it->second.mean) / it->second.num_points;
            }
        }

        // Second pass: compute covariance for each voxel using neighborhood
        pcl::PointCloud<PointType>::Ptr cloud_downsampled(new pcl::PointCloud<PointType>());
        cloud_downsampled->reserve(vmap.size());
        for (const auto& kv : vmap)
        {
            PointType pt;
            pt.x = kv.second.mean.x();
            pt.y = kv.second.mean.y();
            pt.z = kv.second.mean.z();
            cloud_downsampled->push_back(pt);
        }

        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(cloud_downsampled);

        for (size_t i = 0; i < cloud_downsampled->size(); ++i)
        {
            std::vector<int> indices;
            std::vector<float> distances;
            PointType search_pt = cloud_downsampled->points[i];

            if (kdtree.radiusSearch(search_pt, neighbor_search_radius, indices, distances) > 2)
            {
                // Compute covariance from neighboring voxel means (paper eq.20)
                Eigen::Vector3d mean(Eigen::Vector3d::Zero());
                for (int idx : indices)
                {
                    mean += Eigen::Vector3d(cloud_downsampled->points[idx].x,
                                            cloud_downsampled->points[idx].y,
                                            cloud_downsampled->points[idx].z);
                }
                mean /= indices.size();

                Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
                for (int idx : indices)
                {
                    Eigen::Vector3d diff(cloud_downsampled->points[idx].x - mean.x(),
                                         cloud_downsampled->points[idx].y - mean.y(),
                                         cloud_downsampled->points[idx].z - mean.z());
                    cov += diff * diff.transpose();
                }
                cov /= (indices.size() - 1);

                // Find the voxel for this point and update covariance
                Eigen::Vector3i vidx = voxelIndex(Eigen::Vector3d(search_pt.x, search_pt.y, search_pt.z), voxel_resolution);
                auto it = vmap.find(vidx);
                if (it != vmap.end())
                {
                    it->second.covariance = cov;
                }
            }
        }

        return vmap;
    }

    /**
     * VGICP registration between source and target point clouds
     * Implements paper equations (21)-(27)
     *
     * Paper eq.21: S(T) = sum of neighborhood distances
     * Paper eq.23-25: Distribution of transformed source point
     * Paper eq.26: E[S(T)] = 0 when transform is correct
     * Paper eq.27: Maximize likelihood to find T
     */
    struct VGICPResult
    {
        bool converged;
        int iterations;
        Eigen::Isometry3d transformation;
        double fitness_score;
    };

    VGICPResult align(
        const pcl::PointCloud<PointType>::Ptr& source,
        const pcl::PointCloud<PointType>::Ptr& target,
        const Eigen::Isometry3d& initial_guess = Eigen::Isometry3d::Identity())
    {
        VGICPResult result;
        result.converged = false;
        result.iterations = 0;
        result.transformation = initial_guess;
        result.fitness_score = std::numeric_limits<double>::max();

        if (source->empty() || target->empty())
            return result;

        // Build voxel map for target (paper eq.19-20)
        VGICPVoxelMap target_vmap = buildVoxelMap(target);

        // Build KD-tree for target for nearest neighbor search
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(target);

        Eigen::Isometry3d T = initial_guess;

        for (int iter = 0; iter < max_iterations; ++iter)
        {
            result.iterations = iter + 1;

            // Accumulate for Gauss-Newton optimization
            Eigen::Matrix<double, 6, 6> Hessian = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
            double total_cost = 0;
            int valid_pairs = 0;

            for (const auto& pt : source->points)
            {
                // Transform source point (paper eq.23)
                Eigen::Vector3d p_src(pt.x, pt.y, pt.z);
                Eigen::Vector3d p_transformed = T * p_src;

                // Find nearest neighbor in target (paper eq.22)
                PointType search_pt;
                search_pt.x = p_transformed.x();
                search_pt.y = p_transformed.y();
                search_pt.z = p_transformed.z();

                std::vector<int> indices;
                std::vector<float> distances;
                if (kdtree.nearestKSearch(search_pt, 1, indices, distances) == 0)
                    continue;
                if (distances[0] > search_radius * search_radius)
                    continue;

                // Target point and its voxel distribution (paper eq.20)
                Eigen::Vector3d p_target(target->points[indices[0]].x,
                                         target->points[indices[0]].y,
                                         target->points[indices[0]].z);

                Eigen::Vector3i vidx = voxelIndex(p_target, voxel_resolution);
                auto vmap_it = target_vmap.find(vidx);

                // Source point covariance (paper eq.19): assume small isotropic
                Eigen::Matrix3d C_src = Eigen::Matrix3d::Identity() * 0.01;

                // Target point covariance from voxel map (paper eq.20)
                Eigen::Matrix3d C_target;
                if (vmap_it != target_vmap.end() && vmap_it->second.num_points > 2)
                {
                    C_target = vmap_it->second.covariance;
                }
                else
                {
                    C_target = Eigen::Matrix3d::Identity() * 0.01;
                }

                // Combined distribution (paper eq.25)
                // T(p_src) ~ N(T*p_src, R*C_src*R^T + C_target)
                Eigen::Matrix3d R = T.linear();
                Eigen::Matrix3d C_combined = R * C_src * R.transpose() + C_target;

                // Residual (paper eq.21)
                Eigen::Vector3d residual = p_transformed - p_target;

                // Mahalanobis distance (paper eq.23-26)
                Eigen::Matrix3d C_inv = C_combined.inverse();
                double mahal_dist = residual.transpose() * C_inv * residual;
                total_cost += mahal_dist;

                // Jacobian of transformed point w.r.t. [rotation, translation]
                Eigen::Matrix<double, 3, 6> J;
                J.block<3,3>(0,0) = -skewSymmetric(R * p_src); // d(R*p)/d(theta)
                J.block<3,3>(0,3) = Eigen::Matrix3d::Identity(); // d(R*p+t)/d(t)

                // Gauss-Newton accumulation
                // H = J^T * C^{-1} * J,  g = J^T * C^{-1} * r
                Eigen::Matrix<double, 3, 6> J_weighted = C_inv * J;
                Hessian += J.transpose() * J_weighted;
                gradient += J.transpose() * (C_inv * residual);

                valid_pairs++;
            }

            if (valid_pairs < 10)
                break;

            result.fitness_score = total_cost / valid_pairs;

            // Solve for update (paper eq.27): maximize likelihood
            Eigen::Matrix<double, 6, 1> dx = Hessian.ldlt().solve(-gradient);

            // Apply update to transformation
            Eigen::Vector3d dtheta = dx.head<3>();
            double angle = dtheta.norm();
            if (angle > 1e-10)
            {
                Eigen::Matrix3d dR(Eigen::AngleAxisd(angle, dtheta / angle));
                T.linear() = dR * T.linear();
                T.translation() += dx.tail<3>();
            }
            else
            {
                T.translation() += dx.tail<3>();
            }

            // Convergence check
            if (dx.norm() < convergence_threshold)
            {
                result.converged = true;
                result.transformation = T;
                return result;
            }
        }

        result.transformation = T;
        return result;
    }

    /**
     * Loop candidate frame selection (paper section 3, step 1)
     *
     * Criteria:
     * 1. Current frame is a keyframe (distance/angle threshold)
     * 2. Search historical keyframes within spatial radius using KD-tree
     * 3. Time interval exceeds threshold
     */
    struct LoopCandidate
    {
        int current_keyframe_id;
        int history_keyframe_id;
        double time_diff;
        double spatial_distance;
    };

    LoopCandidate findLoopCandidate(
        const Eigen::Vector3d& current_position,
        double current_time,
        const pcl::PointCloud<PointType>::Ptr& keyframe_positions,
        const std::vector<double>& keyframe_times,
        int current_id)
    {
        LoopCandidate candidate;
        candidate.current_keyframe_id = -1;
        candidate.history_keyframe_id = -1;
        candidate.time_diff = 0;
        candidate.spatial_distance = 0;

        if (keyframe_positions->empty() || current_id < 0)
            return candidate;

        // Build KD-tree for historical keyframes
        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(keyframe_positions);

        PointType search_pt;
        search_pt.x = current_position.x();
        search_pt.y = current_position.y();
        search_pt.z = current_position.z();

        std::vector<int> indices;
        std::vector<float> distances;
        if (kdtree.radiusSearch(search_pt, search_radius, indices, distances) == 0)
            return candidate;

        // Find the best candidate: nearest in space, furthest in time
        double max_time_diff = 0;
        int best_idx = -1;

        for (size_t i = 0; i < indices.size(); ++i)
        {
            int idx = indices[i];
            // Skip if too recent (paper: time interval > threshold)
            double time_diff = std::abs(current_time - keyframe_times[idx]);
            if (time_diff < time_diff_threshold)
                continue;

            if (time_diff > max_time_diff)
            {
                max_time_diff = time_diff;
                best_idx = idx;
            }
        }

        if (best_idx < 0)
            return candidate;

        candidate.current_keyframe_id = current_id;
        candidate.history_keyframe_id = best_idx;
        candidate.time_diff = max_time_diff;
        candidate.spatial_distance = std::sqrt(distances[0]);

        return candidate;
    }

    /**
     * Validate loop closure using Euclidean distance threshold (paper eq.28)
     *
     * eq.28: d = sqrt(dx^2 + dy^2 + dz^2)
     * Loop accepted if d < distance_threshold
     */
    bool validateLoopClosure(
        const Eigen::Isometry3d& transform,
        double threshold = -1)
    {
        if (threshold < 0)
            threshold = distance_threshold;

        // Paper eq.28: Euclidean distance between frames
        Eigen::Vector3d translation = transform.translation();
        double dist = translation.norm();

        return dist < threshold && dist > 0.1; // Also reject if too close (same frame)
    }

    // Utility: skew-symmetric matrix
    static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d S;
        S <<     0, -v.z(),  v.y(),
             v.z(),     0, -v.x(),
            -v.y(),  v.x(),     0;
        return S;
    }
};

#endif // _VGICP_LOOP_H_
