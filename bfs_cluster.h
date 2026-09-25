#pragma once
#ifndef _BFS_CLUSTER_H_
#define _BFS_CLUSTER_H_

#include "utility.h"

/**
 * BFS-based Point Cloud Clustering and Dynamic Feature Elimination
 *
 * Reference: "面向煤矿巷道的LiDAR/IMU紧耦合三维地图构建方法"
 *
 * Key concepts from paper:
 *   - Project LiDAR image to 16 x 1800 matrix
 *   - BFS clustering based on angular threshold (alpha_th = 30 degrees)
 *   - Point threshold: >5 points considered same object
 *   - Dynamic features: points belonging to clusters with small size
 *     or inconsistent geometric properties are eliminated
 */

// Grid cell value: -1 = unvisited, 0 = no return, >0 = cluster ID
const int CLUSTER_UNVISITED = -1;
const int CLUSTER_NO_RETURN = 0;

// Angular threshold for BFS clustering (from paper: alpha_th = 30 degrees)
const float BFS_ANGLE_THRESHOLD_DEG = 30.0f;
// Minimum points to form a valid cluster
const int BFS_MIN_CLUSTER_SIZE = 5;
// Maximum cluster size (larger clusters are ground/walls, keep them)
const int BFS_MAX_CLUSTER_SIZE = 50000;

class BFSCluster
{
public:
    // Cluster information
    struct ClusterInfo
    {
        int id;
        int size;
        Eigen::Vector3f centroid;
        Eigen::Vector3f min_bound;
        Eigen::Vector3f max_bound;
        float variance;  // geometric variance of points in cluster
        bool is_dynamic; // flagged as dynamic (likely moving object)
        std::vector<int> point_indices;
    };

private:
    // Grid dimensions (C16 LiDAR: 16 lines, 1800 horizontal points)
    int n_scan_lines_;
    int horiz_points_;
    float vert_resolution_;   // vertical resolution in radians (approx 2 degrees for C16)
    float horiz_resolution_;   // horizontal resolution in radians (approx 0.2 degrees)

    // Grid data
    std::vector<std::vector<int>> grid_; // [row][col] -> cluster ID
    std::vector<ClusterInfo> clusters_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_clouds_;

    // Parameters from paper
    float angle_threshold_;
    int min_cluster_size_;
    int max_cluster_size_;

public:
    BFSCluster()
    {
        // Default: C16 LiDAR
        n_scan_lines_ = 16;
        horiz_points_ = 1800;
        vert_resolution_ = 2.0f * M_PI / 180.0f;  // ~2 degrees
        horiz_resolution_ = 0.2f * M_PI / 180.0f;  // ~0.2 degrees

        angle_threshold_ = BFS_ANGLE_THRESHOLD_DEG * M_PI / 180.0f;
        min_cluster_size_ = BFS_MIN_CLUSTER_SIZE;
        max_cluster_size_ = BFS_MAX_CLUSTER_SIZE;

        initGrid();
    }

    BFSCluster(int n_scan_lines, int horiz_points, float angle_thresh_deg = 30.0f)
    {
        n_scan_lines_ = n_scan_lines;
        horiz_points_ = horiz_points;
        vert_resolution_ = 2.0f * M_PI / 180.0f;
        horiz_resolution_ = 360.0f / horiz_points_ * M_PI / 180.0f;

        angle_threshold_ = angle_thresh_deg * M_PI / 180.0f;
        min_cluster_size_ = BFS_MIN_CLUSTER_SIZE;
        max_cluster_size_ = BFS_MAX_CLUSTER_SIZE;

        initGrid();
    }

    void initGrid()
    {
        grid_.resize(n_scan_lines_);
        for (int i = 0; i < n_scan_lines_; ++i)
        {
            grid_[i].resize(horiz_points_, CLUSTER_UNVISITED);
        }
    }

    void resetGrid()
    {
        for (int i = 0; i < n_scan_lines_; ++i)
        {
            std::fill(grid_[i].begin(), grid_[i].end(), CLUSTER_UNVISITED);
        }
        clusters_.clear();
        cluster_clouds_.clear();
    }

    /**
     * Main clustering function - performs BFS clustering on range image
     * Returns: vector of cluster IDs that are classified as dynamic (should be removed)
     */
    std::vector<int> clusterAndDetectDynamic(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        const cv::Mat& range_image,
        std::vector<int>& ring_indices,
        std::vector<int>& col_indices)
    {
        resetGrid();

        if (input_cloud->empty())
            return {};

        int cloud_size = input_cloud->size();

        // Project cloud to grid
        std::vector<int> point_to_grid(cloud_size, -1);
        for (int i = 0; i < cloud_size; ++i)
        {
            const auto& pt = input_cloud->points[i];
            int row = ring_indices[i];
            int col = col_indices[i];

            if (row < 0 || row >= n_scan_lines_ || col < 0 || col >= horiz_points_)
                continue;

            grid_[row][col] = CLUSTER_NO_RETURN; // mark as occupied
            point_to_grid[i] = row * horiz_points_ + col;
        }

        // BFS clustering (from paper description and Fig.3)
        int current_cluster_id = 1;
        std::queue<std::pair<int, int>> bfs_queue;

        for (int row = 0; row < n_scan_lines_; ++row)
        {
            for (int col = 0; col < horiz_points_; ++col)
            {
                if (grid_[row][col] != CLUSTER_UNVISITED)
                    continue; // already visited

                // Start BFS from this cell
                std::vector<int> cluster_point_indices;
                bfs_queue.push({row, col});
                grid_[row][col] = current_cluster_id;

                while (!bfs_queue.empty())
                {
                    int r = bfs_queue.front().first;
                    int c = bfs_queue.front().second;
                    bfs_queue.pop();

                    // Find corresponding point index
                    int idx = r * horiz_points_ + c;
                    // Search through original cloud for this grid cell
                    for (int i = 0; i < cloud_size; ++i)
                    {
                        int rr = ring_indices[i];
                        int cc = col_indices[i];
                        if (rr == r && cc == c)
                        {
                            cluster_point_indices.push_back(i);
                            break;
                        }
                    }

                    // Explore 4-connected neighbors
                    for (int dr = -1; dr <= 1; ++dr)
                    {
                        for (int dc = -1; dc <= 1; ++dc)
                        {
                            if (dr == 0 && dc == 0) continue;

                            int nr = r + dr;
                            int nc = c + dc;

                            if (nr < 0 || nr >= n_scan_lines_ || nc < 0 || nc >= horiz_points_)
                                continue;

                            if (grid_[nr][nc] != CLUSTER_UNVISITED)
                                continue;

                            // Check angular threshold (from paper: alpha_th = 30 degrees)
                            if (isWithinAngleThreshold(r, c, nr, nc))
                            {
                                grid_[nr][nc] = current_cluster_id;
                                bfs_queue.push({nr, nc});
                            }
                        }
                    }
                }

                // Process completed cluster
                if ((int)cluster_point_indices.size() >= min_cluster_size_)
                {
                    ClusterInfo info;
                    info.id = current_cluster_id;
                    info.size = cluster_point_indices.size();
                    info.point_indices = cluster_point_indices;
                    info.is_dynamic = false;
                    computeClusterStats(input_cloud, info);
                    clusters_.push_back(info);
                    current_cluster_id++;
                }
            }
        }

        // Detect dynamic clusters
        return detectDynamicClusters();
    }

    /**
     * Check if two grid cells are within angular threshold
     * From paper: adjacent laser rays angle alpha < alpha_th = 30 degrees
     */
    bool isWithinAngleThreshold(int r1, int c1, int r2, int c2)
    {
        // Calculate angular difference between two rays
        // Vertical angle difference
        float vert_diff = std::abs(r1 - r2) * vert_resolution_;

        // Horizontal angle difference (handle wrap-around)
        int dc = std::abs(c1 - c2);
        if (dc > horiz_points_ / 2)
            dc = horiz_points_ - dc;
        float horiz_diff = dc * horiz_resolution_;

        // Total angular difference
        float alpha = std::sqrt(vert_diff * vert_diff + horiz_diff * horiz_diff);

        return alpha < angle_threshold_;
    }

    /**
     * Compute cluster statistics for dynamic detection
     */
    void computeClusterStats(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, ClusterInfo& info)
    {
        if (info.point_indices.empty()) return;

        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        Eigen::Vector3f min_pt = Eigen::Vector3f::Zero();
        Eigen::Vector3f max_pt = Eigen::Vector3f::Zero();

        bool first = true;
        float total_dist = 0;

        for (int idx : info.point_indices)
        {
            const auto& pt = cloud->points[idx];
            Eigen::Vector3f p(pt.x, pt.y, pt.z);

            sum += p;
            total_dist += std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);

            if (first)
            {
                min_pt = p;
                max_pt = p;
                first = false;
            }
            else
            {
                min_pt = min_pt.cwiseMin(p);
                max_pt = max_pt.cwiseMax(p);
            }
        }

        info.centroid = sum / (float)info.point_indices.size();
        info.min_bound = min_pt;
        info.max_bound = max_pt;

        // Compute variance
        float mean_dist = total_dist / (float)info.point_indices.size();
        float variance = 0;
        for (int idx : info.point_indices)
        {
            const auto& pt = cloud->points[idx];
            float d = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
            variance += (d - mean_dist) * (d - mean_dist);
        }
        info.variance = variance / (float)info.point_indices.size();
    }

    /**
     * Detect dynamic clusters based on paper criteria:
     *   - Small clusters (likely pedestrians/vehicles)
     *   - High geometric variance (unstable features)
     *   - Clusters at unusual heights
     */
    std::vector<int> detectDynamicClusters()
    {
        std::vector<int> dynamic_cluster_ids;

        for (auto& info : clusters_)
        {
            bool is_dynamic = false;

            // Criterion 1: Small clusters (typical of moving objects)
            // Pedestrians: 5-100 points, Vehicles: 100-1000 points
            if (info.size < min_cluster_size_ || info.size > max_cluster_size_)
            {
                is_dynamic = true;
            }

            // Criterion 2: Small bounding box (typical of moving objects)
            float bbox_volume = (info.max_bound.x() - info.min_bound.x()) *
                                (info.max_bound.y() - info.min_bound.y()) *
                                (info.max_bound.z() - info.min_bound.z());

            if (bbox_volume < 1.0f && info.size < 500) // small volume + small cluster
            {
                is_dynamic = true;
            }

            // Criterion 3: High variance relative to size (unstable features)
            if (info.size > 0 && info.variance > 100.0f && info.size < 200)
            {
                is_dynamic = true;
            }

            // Criterion 4: Height analysis - dynamic objects usually at human/vehicle height
            float z_range = info.max_bound.z() - info.min_bound.z();
            float z_center = info.centroid.z();
            if (z_range < 2.0f && z_center > 0.3f && z_center < 2.5f && info.size < 300)
            {
                // Likely a pedestrian or object at human height
                is_dynamic = true;
            }

            if (is_dynamic)
            {
                info.is_dynamic = true;
                dynamic_cluster_ids.push_back(info.id);
            }
        }

        return dynamic_cluster_ids;
    }

    /**
     * Get static point cloud (dynamic features removed)
     * From paper: preserve complete drivable area map
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getStaticCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        const std::vector<int>& dynamic_cluster_ids)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // Build set of dynamic cluster IDs
        std::set<int> dynamic_set(dynamic_cluster_ids.begin(), dynamic_cluster_ids.end());

        for (const auto& info : clusters_)
        {
            if (info.is_dynamic)
                continue; // skip dynamic clusters

            for (int idx : info.point_indices)
            {
                if (idx >= 0 && idx < (int)input_cloud->size())
                {
                    static_cloud->push_back(input_cloud->points[idx]);
                }
            }
        }

        return static_cloud;
    }

    /**
     * Get all cluster clouds
     */
    const std::vector<ClusterInfo>& getClusters() const { return clusters_; }

    /**
     * Get dynamic point cloud
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr getDynamicCloud(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        for (const auto& info : clusters_)
        {
            if (!info.is_dynamic)
                continue;

            for (int idx : info.point_indices)
            {
                if (idx >= 0 && idx < (int)input_cloud->size())
                {
                    dynamic_cloud->push_back(input_cloud->points[idx]);
                }
            }
        }

        return dynamic_cloud;
    }

    // Get cluster count statistics
    int getStaticClusterCount() const
    {
        int count = 0;
        for (const auto& info : clusters_)
        {
            if (!info.is_dynamic)
                count++;
        }
        return count;
    }

    int getDynamicClusterCount() const
    {
        int count = 0;
        for (const auto& info : clusters_)
        {
            if (info.is_dynamic)
                count++;
        }
        return count;
    }
};

#endif
