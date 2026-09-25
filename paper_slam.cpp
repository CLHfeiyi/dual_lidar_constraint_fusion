/**
 * paper_slam.cpp
 *
 * 基于论文"煤矿巷道环境下基于激光雷达与IMU的鲁棒三维重建方法研究"的完整算法实现
 *
 * 算法流程对应论文各节:
 *   - 公式(2):  基于曲率的特征提取（边缘点/平面点）
 *   - 公式(3-5): 状态定义与误差状态
 *   - 公式(6-10): IMU预测模型（连续/离散化）
 *   - 公式(11-12): LiDAR观测模型（点线/点面残差）
 *   - 公式(13-16): 迭代卡尔曼滤波更新
 *   - 公式(17):  全局位姿转换
 *   - 公式(18):  巷道三维模型更新
 *   - 公式(19-28): VGICP回环检测与累计误差校正
 */

#include "utility.h"
#include "lio_sam/cloud_info.h"
#include "lio_sam/save_map.h"
#include "iekf_paper.h"
#include "vgicp_loop.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

using namespace gtsam;
using symbol_shorthand::X; // Pose3
using symbol_shorthand::V; // Vel
using symbol_shorthand::B; // Bias

// ========== 自定义点类型 ==========
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time))

struct OusterPointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range))

using PointXYZIRT = VelodynePointXYZIRT;

// 带位姿信息的点类型（用于关键帧位姿存储）
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
    (double, time, time))

typedef PointXYZIRPYT PointTypePose;

const int queueLength = 2000;

// ========== 平滑度结构体（论文公式2：曲率计算） ==========
struct smoothness_t
{
    float value;  // 曲率值
    size_t ind;   // 点索引
};

struct by_value
{
    bool operator()(smoothness_t const &left, smoothness_t const &right)
    {
        return left.value < right.value;
    }
};

// ========== 主类: 论文算法完整实现 ==========
class PaperSLAM : public ParamServer
{
public:
    // ---- GTSAM 因子图 ----
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    // ---- 论文核心模块 ----
    IEKFPaper iekf;                    // 迭代卡尔曼滤波器（论文公式3-17）
    VGICPLoopDetector vgicp;           // VGICP回环检测器（论文公式19-28）

    // ---- ROS 接口 ----
    ros::Subscriber subLaserCloud;
    ros::Subscriber subImu;
    ros::Subscriber subOdom;
    ros::Subscriber subGPS;
    ros::Subscriber subWheelOdom;

    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;
    ros::Publisher pubCornerPoints;
    ros::Publisher pubSurfacePoints;
    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLoopConstraintEdge;

    ros::ServiceServer srvSaveMap;

    // ---- 数据队列 ----
    std::deque<sensor_msgs::Imu> imuQueue;
    std::deque<nav_msgs::Odometry> odomQueue;
    std::deque<nav_msgs::Odometry> gpsQueue;
    std::deque<sensor_msgs::PointCloud2> cloudQueue;

    // ---- IMU 缓存 ----
    double *imuTime;
    double *imuRotX;
    double *imuRotY;
    double *imuRotZ;
    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    // ---- 点云数据 ----
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;
    pcl::PointCloud<PointType>::Ptr cornerCloud;      // 边缘特征点（论文公式2）
    pcl::PointCloud<PointType>::Ptr surfaceCloud;      // 平面特征点（论文公式2）

    // ---- 特征提取相关（论文公式2） ----
    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    // ---- 关键帧相关 ----
    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    // ---- 当前帧特征点 ----
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;

    // ---- scan-to-map 匹配中间变量 ----
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;
    std::vector<PointType> laserCloudOriCornerVec;
    std::vector<PointType> coeffSelCornerVec;
    std::vector<PointType> laserCloudOriSurfVec;
    std::vector<PointType> coeffSelSurfVec;

    // ---- 局部地图 ----
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;

    // ---- KD-Tree ----
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    // ---- 下采样滤波器 ----
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses;

    // ---- 状态变量 ----
    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;
    lio_sam::cloud_info cloudInfo;

    float transformTobeMapped[6];
    bool odomDeskewFlag;
    float odomIncreX, odomIncreY, odomIncreZ;
    int deskewFlag;
    cv::Mat rangeMat;

    std::mutex imuLock;
    std::mutex odoLock;
    std::mutex wheelOdomLock;
    Eigen::Vector3d wheelOdomPos;
    Eigen::Quaterniond wheelOdomQuat;
    double wheelOdomTime;
    std::mutex mtx;
    std::mutex mtxLoopInfo;
    bool aLoopIsClosed;
    map<int, int> loopIndexContainer;

    // 回环检测队列
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    deque<std_msgs::Float64MultiArray> loopInfoVec;

    nav_msgs::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;

    int laserCloudCornerFromMapDSNum;
    int laserCloudSurfFromMapDSNum;
    int laserCloudCornerLastDSNum;
    int laserCloudSurfLastDSNum;

    vector<int> columnIdnCountVec;

    // ========== 构造函数 ==========
    PaperSLAM() : deskewFlag(0)
    {
        // 初始化ISAM2
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        // 分配IMU缓存
        imuTime = new double[queueLength];
        imuRotX = new double[queueLength];
        imuRotY = new double[queueLength];
        imuRotZ = new double[queueLength];

        // 订阅话题
        subImu = nh.subscribe<sensor_msgs::Imu>(
            imuTopic, 2000, &PaperSLAM::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>(
            odomTopic + "_incremental", 2000, &PaperSLAM::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
            pointCloudTopic, 5, &PaperSLAM::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS = nh.subscribe<nav_msgs::Odometry>(
            gpsTopic, 200, &PaperSLAM::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subWheelOdom = nh.subscribe<nav_msgs::Odometry>(
            "/odom", 200, &PaperSLAM::wheelOdomHandler, this, ros::TransportHints().tcpNoDelay());

        // 发布话题
        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2>(
            "lio_sam/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info>(
            "lio_sam/deskew/cloud_info", 1);
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>(
            "lio_sam/feature/cloud_corner", 1);
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>(
            "lio_sam/feature/cloud_surface", 1);
        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>(
            "lio_sam/mapping/trajectory", 1);
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>(
            "lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>(
            "lio_sam/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry>(
            "lio_sam/mapping/odometry_incremental", 1);
        pubPath = nh.advertise<nav_msgs::Path>(
            "lio_sam/mapping/path", 1);
        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>(
            "lio_sam/mapping/map_local", 1);
        pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>(
            "lio_sam/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>(
            "lio_sam/mapping/cloud_registered_raw", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>(
            "/lio_sam/mapping/loop_closure_constraints", 1);

        srvSaveMap = nh.advertiseService("lio_sam/save_map", &PaperSLAM::saveMapService, this);

        // 初始化下采样滤波器
        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity);

        // 分配内存
        allocateMemory();
        resetParameters();

        // 加载VGICP回环检测参数（论文公式19-28相关）
        nh.param<double>("lio_sam/vgicpVoxelResolution", vgicp.voxel_resolution, 1.0);
        nh.param<double>("lio_sam/vgicpNeighborRadius", vgicp.neighbor_search_radius, 1.5);
        nh.param<int>("lio_sam/vgicpMaxIterations", vgicp.max_iterations, 30);
        nh.param<double>("lio_sam/vgicpConvergenceThreshold", vgicp.convergence_threshold, 0.001);
        nh.param<double>("lio_sam/vgicpDistanceThreshold", vgicp.distance_threshold, 5.0);

        // 加载IEKF参数
        nh.param<double>("lio_sam/imuAccNoise", iekf.noise_acc, imuAccNoise);
        nh.param<double>("lio_sam/imuGyrNoise", iekf.noise_gyro, imuGyrNoise);
        nh.param<double>("lio_sam/imuAccBiasN", iekf.noise_acc_bias, imuAccBiasN);
        nh.param<double>("lio_sam/imuGyrBiasN", iekf.noise_gyro_bias, imuGyrBiasN);
        nh.param<double>("lio_sam/imuGravity", iekf.gravity_mag, imuGravity);
        iekf.initProcessNoise();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

        ROS_INFO("\033[1;32m----> Paper SLAM Algorithm Started.\033[0m");
        ROS_INFO("  IEKF + VGICP Loop Closure (论文算法)");
        ROS_INFO("  VGICP: voxel=%.2f radius=%.2f dist_th=%.2f",
                 vgicp.voxel_resolution, vgicp.neighbor_search_radius, vgicp.distance_threshold);
    }

    ~PaperSLAM()
    {
        delete[] imuTime;
        delete[] imuRotX;
        delete[] imuRotY;
        delete[] imuRotZ;
        delete isam;
    }

    // ========== 内存分配 ==========
    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCAN * Horizon_SCAN);

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        cloudSmoothness.resize(N_SCAN * Horizon_SCAN);
        cloudCurvature = new float[N_SCAN * Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
        cloudLabel = new int[N_SCAN * Horizon_SCAN];

        for (int i = 0; i < 6; ++i)
            transformTobeMapped[i] = 0;
    }

    // ========== 参数重置（每帧处理前调用） ==========
    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        cornerCloud->clear();
        surfaceCloud->clear();

        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;
        deskewFlag = 0;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

        columnIdnCountVec.assign(N_SCAN, 0);
        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);
        cloudInfo.pointColInd.assign(N_SCAN * Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN * Horizon_SCAN, 0);
    }

    // ======================================================
    // Part2: 回调函数 + 点云投影与去畸变
    // ======================================================

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void wheelOdomHandler(const nav_msgs::Odometry::ConstPtr& wheelOdomMsg)
    {
        std::lock_guard<std::mutex> lock(wheelOdomLock);
        wheelOdomPos.x() = wheelOdomMsg->pose.pose.position.x;
        wheelOdomPos.y() = wheelOdomMsg->pose.pose.position.y;
        wheelOdomPos.z() = wheelOdomMsg->pose.pose.position.z;
        wheelOdomTime = wheelOdomMsg->header.stamp.toSec();
        wheelOdomQuat.x() = wheelOdomMsg->pose.pose.orientation.x;
        wheelOdomQuat.y() = wheelOdomMsg->pose.pose.orientation.y;
        wheelOdomQuat.z() = wheelOdomMsg->pose.pose.orientation.z;
        wheelOdomQuat.w() = wheelOdomMsg->pose.pose.orientation.w;
        static int dbgCount = 0;
        if (++dbgCount <= 3)
            ROS_WARN("WHEELODOM: t=%.3f pos=(%.2f,%.2f,%.2f)",
                     wheelOdomTime, wheelOdomPos.x(), wheelOdomPos.y(), wheelOdomPos.z());
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg)) return;
        if (!deskewInfo()) return;
        projectPointCloud();
        cloudExtraction();

        // 提取特征并发布（Part3）
        calculateSmoothness();
        markOccludedPoints();
        extractFeatures();
        publishFeatureCloud();

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2) return false;
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();

        if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
            pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);
        else if (sensor == SensorType::OUSTER)
        {
            pcl::PointCloud<OusterPointXYZIRT> tmpOusterCloud;
            pcl::fromROSMsg(currentCloudMsg, tmpOusterCloud);
            laserCloudIn->clear();
            for (auto& pt : tmpOusterCloud.points)
            {
                VelodynePointXYZIRT vpt;
                vpt.x = pt.x; vpt.y = pt.y; vpt.z = pt.z;
                vpt.intensity = pt.intensity;
                vpt.ring = pt.ring;
                vpt.time = pt.t * 1e-9f;
                laserCloudIn->points.push_back(vpt);
            }
        }
        timeScanCur = currentCloudMsg.header.stamp.toSec();
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;

        if (laserCloudIn->points.empty()) return false;

        cloudHeader = currentCloudMsg.header;
        cloudInfo.header = cloudHeader;
        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur)
            return false;
        imuDeskewInfo();
        odomDeskewInfo();
        return true;
    }

    void imuDeskewInfo()
    {
        if (imuQueue.empty()) return;
        cloudInfo.imuAvailable = false;
        while (!imuQueue.empty() && imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
        {
            imuQueue.pop_front();
            if (imuQueue.empty()) return;
        }
        if (imuQueue.empty()) return;
        imuPointerCur = 0;
        for (auto& thisImuMsg : imuQueue)
        {
            double imuTime_cur = thisImuMsg.header.stamp.toSec();
            if (imuTime_cur < timeScanCur - 0.01) continue;
            if (imuTime_cur > timeScanEnd + 0.01) break;
            imuRotX[imuPointerCur] = thisImuMsg.angular_velocity.x;
            imuRotY[imuPointerCur] = thisImuMsg.angular_velocity.y;
            imuRotZ[imuPointerCur] = thisImuMsg.angular_velocity.z;
            imuTime[imuPointerCur] = imuTime_cur;
            if (imuPointerCur > 0)
            {
                // 初始化IEKF（使用首次IMU测量）
                if (!iekf.initialized)
                {
                    Eigen::Vector3d acc(thisImuMsg.linear_acceleration.x, thisImuMsg.linear_acceleration.y, thisImuMsg.linear_acceleration.z);
                    Eigen::Vector3d gyr(thisImuMsg.angular_velocity.x, thisImuMsg.angular_velocity.y, thisImuMsg.angular_velocity.z);
                    iekf.initialize(acc, gyr);
                }
            }
            imuPointerCur++;
            if (imuPointerCur >= queueLength) break;
        }
        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;
        while (!odomQueue.empty() && odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
        {
            odomQueue.pop_front();
            if (odomQueue.empty()) return;
        }
        if (odomQueue.empty()) return;
        if (odomQueue.front().header.stamp.toSec() > timeScanCur) return;

        nav_msgs::Odometry startOdomMsg = odomQueue.front();
        nav_msgs::Odometry endOdomMsg = odomQueue.back();
        double startOdomTime = startOdomMsg.header.stamp.toSec();
        double endOdomTime = endOdomMsg.header.stamp.toSec();

        // 计算帧间平均速度，用于平移去畸变
        if (endOdomTime > startOdomTime)
        {
            double dt = endOdomTime - startOdomTime;
            odomIncreX = (endOdomMsg.pose.pose.position.x - startOdomMsg.pose.pose.position.x) / dt;
            odomIncreY = (endOdomMsg.pose.pose.position.y - startOdomMsg.pose.pose.position.y) / dt;
            odomIncreZ = (endOdomMsg.pose.pose.position.z - startOdomMsg.pose.pose.position.z) / dt;
        }
        else
        {
            odomIncreX = 0; odomIncreY = 0; odomIncreZ = 0;
        }

        Eigen::Affine3f transStart = pcl::getTransformation(
            startOdomMsg.pose.pose.position.x,
            startOdomMsg.pose.pose.position.y,
            startOdomMsg.pose.pose.position.z, 0, 0, 0);
        transStartInverse = transStart.inverse();
        cloudInfo.odomAvailable = true;

        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        tf::Quaternion q(startOdomMsg.pose.pose.orientation.x, startOdomMsg.pose.pose.orientation.y,
                         startOdomMsg.pose.pose.orientation.z, startOdomMsg.pose.pose.orientation.w);
        tf::Matrix3x3(q).getRPY(cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;
        int imuIdx = imuPointerCur;
        if (imuIdx <= 0) return;
        for (int i = 0; i < imuIdx; i++)
        {
            if (pointTime < imuTime[i]) { imuIdx = i; break; }
        }
        if (imuIdx <= 0 || imuIdx >= imuPointerCur) return;
        if (pointTime > imuTime[imuIdx]) return;
        float ratio = (pointTime - imuTime[imuIdx - 1]) / (imuTime[imuIdx] - imuTime[imuIdx - 1]);
        *rotXCur = imuRotX[imuIdx - 1] + ratio * (imuRotX[imuIdx] - imuRotX[imuIdx - 1]);
        *rotYCur = imuRotY[imuIdx - 1] + ratio * (imuRotY[imuIdx] - imuRotY[imuIdx - 1]);
        *rotZCur = imuRotZ[imuIdx - 1] + ratio * (imuRotZ[imuIdx] - imuRotZ[imuIdx - 1]);
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;
        if (cloudInfo.odomAvailable)
        {
            *posXCur = odomIncreX * relTime;
            *posYCur = odomIncreY * relTime;
            *posZCur = odomIncreZ * relTime;
        }
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        for (int i = 0; i < cloudSize; ++i)
        {
            auto& pt = laserCloudIn->points[i];
            float range = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (range < lidarMinRange || range > lidarMaxRange) continue;

            int rowIdn = pt.ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN) continue;
            if (rowIdn % downsampleRate != 0) continue;

            float horizonAngle = atan2(pt.x, pt.y) * 180 / M_PI;
            int columnIdn = -round((horizonAngle - 90.0) / 360.0 * Horizon_SCAN) + Horizon_SCAN / 2;
            if (columnIdn >= Horizon_SCAN) columnIdn -= Horizon_SCAN;
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN) continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX) continue;
            rangeMat.at<float>(rowIdn, columnIdn) = range;

            // 运动畸变补偿
            if (deskewFlag >= 0)
            {
                float rotXCur, rotYCur, rotZCur;
                findRotation(pt.time, &rotXCur, &rotYCur, &rotZCur);
                float posXCur, posYCur, posZCur;
                findPosition(pt.time - laserCloudIn->points.front().time, &posXCur, &posYCur, &posZCur);
                float x1 = pt.x - posXCur;
                float y1 = pt.y - posYCur;
                float z1 = pt.z - posZCur;
                float rx = rotXCur, ry = rotYCur, rz = rotZCur;
                float rx2 = rx * rx, ry2 = ry * ry, rz2 = rz * rz;
                float s = rx2 + ry2 + rz2;
                if (s > 0)
                {
                    float r = sqrt(s);
                    float cx = cos(-r); float sx = sin(-r);
                    float a = 1 - cx;
                    float x2 = (a * rx2 + cx) * x1 + (a * rx * ry - rz * sx) * y1 + (a * rx * rz + ry * sx) * z1;
                    float y2 = (a * rx * ry + rz * sx) * x1 + (a * ry2 + cx) * y1 + (a * ry * rz - rx * sx) * z1;
                    float z2 = (a * rx * rz - ry * sx) * x1 + (a * ry * rz + rx * sx) * y1 + (a * rz2 + cx) * z1;
                    pt.x = x2; pt.y = y2; pt.z = z2;
                }
            }

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = pt;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;
            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i, j) != FLT_MAX)
                {
                    cloudInfo.pointColInd[count] = j;
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count - 1 - 5;
        }
    }

    // ======================================================
    // Part3: 特征提取 - 基于曲率（论文公式2）
    // ======================================================

    /**
     * 论文公式(2): 曲率计算
     * c = 1/|S| * || (sum_{j∈S, j≠i} (p_i - p_j)) ||
     * 取当前点及其左右各5个点进行曲率计算
     */
    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {
            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];
            cloudCurvature[i] = diffRange * diffRange;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));
            if (columnDiff < 10)
            {
                if (depth1 - depth2 > 0.3)
                {
                    for (int l = -5; l <= 0; ++l) cloudNeighborPicked[i + l] = 1;
                }
                else if (depth2 - depth1 > 0.3)
                {
                    for (int l = 1; l <= 6; ++l) cloudNeighborPicked[i + l] = 1;
                }
            }
            float diff1 = std::abs(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]);
            float diff2 = std::abs(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]);
            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    /**
     * 论文公式(2): 根据曲率大小区分边缘点和平面点
     * - 曲率最大的N_edge个点为边缘特征点(edge features)
     * - 曲率最小的N_plane个点为平面特征点(plane features)
     */
    void extractFeatures()
    {
        cornerCloud->clear();
        surfaceCloud->clear();
        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());

        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();
            for (int j = 0; j < 6; j++)
            {
                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;
                if (sp >= ep) continue;

                std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

                // 取曲率最大的点为边缘特征点
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20)
                        {
                            cloudLabel[ind] = 1;
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        }
                        else break;
                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            if (std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1])) > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            if (std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1])) > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 取曲率最小的点为平面特征点
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {
                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            if (std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1])) > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            if (std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1])) > 10) break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0)
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                }
            }

            // 下采样平面特征点
            pcl::PointCloud<PointType> surfaceCloudScanDS;
            pcl::VoxelGrid<PointType> surfFilter;
            surfFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);
            surfFilter.setInputCloud(surfaceCloudScan);
            surfFilter.filter(surfaceCloudScanDS);
            *surfaceCloud += surfaceCloudScanDS;
        }
    }

    void publishFeatureCloud()
    {
        cloudInfo.cloud_corner = publishCloud(pubCornerPoints, cornerCloud, cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_deskewed = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo);

        // 保存特征点用于后续匹配
        *laserCloudCornerLast = *cornerCloud;
        *laserCloudSurfLast = *surfaceCloud;

        // 触发建图流程
        static double timeLastProcessing = -1;
        if (timeScanCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeScanCur;
            timeLaserInfoStamp = cloudHeader.stamp;
            timeLaserInfoCur = timeScanCur;

            updateInitialGuess();
            extractSurroundingKeyFrames();
            downsampleCurrentScan();
            scan2MapOptimization();
            saveKeyFramesAndFactor();
            correctPoses();
            publishOdometry();
            publishFrames();
        }
    }

    // ======================================================
    // Part4: IMU预积分与IEKF预测（论文公式3-10）
    // ======================================================

    /**
     * 初始位姿猜测（使用IMU预积分或里程计）
     * 对应论文公式(3)-(5): 状态定义与预测传播
     */
    void updateInitialGuess()
    {
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        if (cloudKeyPoses3D->points.empty())
        {
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;
            transformTobeMapped[2] = cloudInfo.imuYawInit;
            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;
            return;
        }

        // 使用IMU预积分进行位姿预测 (论文公式6-10) — 仅用于旋转和状态传播
        static double lastImuTime = -1;
        std::lock_guard<std::mutex> lock(imuLock);
        for (const auto& imuMsg : imuQueue)
        {
            double imuTime_cur = imuMsg.header.stamp.toSec();
            if (imuTime_cur <= lastImuTime) continue;
            double dt = (lastImuTime < 0) ? 0.0 : imuTime_cur - lastImuTime;
            lastImuTime = imuTime_cur;

            if (dt > 0 && dt < 0.1)
            {
                Eigen::Vector3d acc(imuMsg.linear_acceleration.x, imuMsg.linear_acceleration.y, imuMsg.linear_acceleration.z);
                Eigen::Vector3d gyr(imuMsg.angular_velocity.x, imuMsg.angular_velocity.y, imuMsg.angular_velocity.z);
                iekf.predict(acc, gyr, dt);
            }
        }

        // 从IEKF获取预测旋转
        Eigen::Quaterniond q_pred = iekf.state.rotation;
        Eigen::Vector3d euler = IEKFStatePaper::quaternionToEuler(q_pred);
        transformTobeMapped[0] = euler.x();
        transformTobeMapped[1] = euler.y();
        transformTobeMapped[2] = euler.z();

        // 平移预测: 使用轮式里程计 (比纯IMU积分更可靠)
        static Eigen::Vector3d lastWheelPos(0, 0, 0);
        static bool wheelOdomFirst = true;
        static int initDbgCount = 0;
        {
            std::lock_guard<std::mutex> lock2(wheelOdomLock);
            if (wheelOdomTime > 0)
            {
                if (!wheelOdomFirst)
                {
                    Eigen::Vector3d dPos = wheelOdomPos - lastWheelPos;
                    transformTobeMapped[3] += dPos.x();
                    transformTobeMapped[4] += dPos.y();
                    transformTobeMapped[5] += dPos.z();
                    if (++initDbgCount <= 5)
                        ROS_WARN("INITGUESS: wheel=(%.2f,%.2f,%.2f) dPos=(%.3f,%.3f,%.3f) -> trans=(%.2f,%.2f,%.2f)",
                                 wheelOdomPos.x(), wheelOdomPos.y(), wheelOdomPos.z(),
                                 dPos.x(), dPos.y(), dPos.z(),
                                 transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
                }
                else
                {
                    wheelOdomFirst = false;
                    ROS_WARN("INITGUESS: first wheel odom at (%.2f,%.2f,%.2f)",
                             wheelOdomPos.x(), wheelOdomPos.y(), wheelOdomPos.z());
                }
                lastWheelPos = wheelOdomPos;
            }
        }
    }

    // ======================================================
    // Part5: 局部地图提取、scan-to-map优化、IEKF更新（论文公式11-17）
    // ======================================================

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty()) return;
        // 回环关闭模式下也不启用 for_loop 版本，保持一致性
        extractNearby();
    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (size_t i = 0; i < pointSearchInd.size(); ++i)
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchInd[i]]);

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for (auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else break;
        }
        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (size_t i = 0; i < cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius) continue;
            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end())
            {
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
            }
            else
            {
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp   = *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();

        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
    }

    void downsampleCurrentScan()
    {
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty()) return;
        if (laserCloudCornerFromMapDSNum < 10 || laserCloudSurfFromMapDSNum < 100) return;

        // 保存初始位姿
        float backupTransform[6];
        for (int i = 0; i < 6; i++) backupTransform[i] = transformTobeMapped[i];

        // 收敛判断参数
        float deltaR = 0.05;  // 旋转收敛阈值 rad
        float deltaT = 0.05;  // 平移收敛阈值 m

        for (int iterCount = 0; iterCount < 30; iterCount++)
        {
            // 构建KD-Tree
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            // 清空上一轮约束
            laserCloudOriCornerVec.clear();
            coeffSelCornerVec.clear();
            laserCloudOriSurfVec.clear();
            coeffSelSurfVec.clear();

            // 查找对应关系
            cornerOptimization();
            surfOptimization();

            // 构建法方程并求解（LM迭代）
            if (combineOptimization(iterCount))
            {
                // 收敛检查
                float rotChange = sqrt(
                    (transformTobeMapped[0] - backupTransform[0]) * (transformTobeMapped[0] - backupTransform[0]) +
                    (transformTobeMapped[1] - backupTransform[1]) * (transformTobeMapped[1] - backupTransform[1]) +
                    (transformTobeMapped[2] - backupTransform[2]) * (transformTobeMapped[2] - backupTransform[2]));
                float transChange = sqrt(
                    (transformTobeMapped[3] - backupTransform[3]) * (transformTobeMapped[3] - backupTransform[3]) +
                    (transformTobeMapped[4] - backupTransform[4]) * (transformTobeMapped[4] - backupTransform[4]) +
                    (transformTobeMapped[5] - backupTransform[5]) * (transformTobeMapped[5] - backupTransform[5]));

                if (iterCount > 2 && rotChange < deltaR && transChange < deltaT)
                {
                    // 收敛，停止迭代
                    break;
                }

                // 保存当前位姿作下一轮对比
                for (int i = 0; i < 6; i++) backupTransform[i] = transformTobeMapped[i];
            }
        }

        // 检查位姿是否有效
        float deltaR2 = (transformTobeMapped[0] - backupTransform[0]) * (transformTobeMapped[0] - backupTransform[0]) +
                        (transformTobeMapped[1] - backupTransform[1]) * (transformTobeMapped[1] - backupTransform[1]) +
                        (transformTobeMapped[2] - backupTransform[2]) * (transformTobeMapped[2] - backupTransform[2]);
        float deltaT2 = (transformTobeMapped[3] - backupTransform[3]) * (transformTobeMapped[3] - backupTransform[3]) +
                        (transformTobeMapped[4] - backupTransform[4]) * (transformTobeMapped[4] - backupTransform[4]) +
                        (transformTobeMapped[5] - backupTransform[5]) * (transformTobeMapped[5] - backupTransform[5]);
        if (deltaR2 > 1.0 || deltaT2 > 10.0)
        {
            // 位姿跳变太大，回退
            for (int i = 0; i < 6; i++) transformTobeMapped[i] = backupTransform[i];
        }

        // IMU Bias/Velocity估计: 用IEKF观测更新修正bias
        // 但最终位姿始终来自LM优化（避免IEKF不稳定导致发散）
        bool ekfConverged = updateIEKFFromScanMatching();

        // 始终同步IEKF的位姿到LM结果（用于下一帧IMU预测）
        // IEKF内部保留velocity/bias估计
        Eigen::Vector3d lmEuler(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Vector3d lmPos(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
        iekf.state.rotation = Eigen::AngleAxisd(lmEuler.z(), Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(lmEuler.y(), Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(lmEuler.x(), Eigen::Vector3d::UnitX());
        iekf.state.position = lmPos;
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri = laserCloudCornerLastDS->points[i];
            PointType pointSel;
            pointAssociateToMap(&pointOri, &pointSel);
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0)
            {
                // 计算5个最近邻点的中心
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++)
                {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5; cz /= 5;

                // 协方差矩阵
                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++)
                {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;
                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az; a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                // 特征分解
                cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
                cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
                cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
                matA1.at<float>(0,0) = a11; matA1.at<float>(0,1) = a12; matA1.at<float>(0,2) = a13;
                matA1.at<float>(1,0) = a12; matA1.at<float>(1,1) = a22; matA1.at<float>(1,2) = a23;
                matA1.at<float>(2,0) = a13; matA1.at<float>(2,1) = a23; matA1.at<float>(2,2) = a33;
                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0,0) > 3 * matD1.at<float>(0,1))
                {
                    float x0 = pointSel.x, y0 = pointSel.y, z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0,0);
                    float y1 = cy + 0.1 * matV1.at<float>(0,1);
                    float z1 = cz + 0.1 * matV1.at<float>(0,2);
                    float x2 = cx - 0.1 * matV1.at<float>(0,0);
                    float y2 = cy - 0.1 * matV1.at<float>(0,1);
                    float z2 = cz - 0.1 * matV1.at<float>(0,2);

                    // 点到直线距离
                    float a012 = sqrt(((x0-x1)*(y0-y2) - (x0-x2)*(y0-y1)) * ((x0-x1)*(y0-y2) - (x0-x2)*(y0-y1))
                                    + ((x0-x1)*(z0-z2) - (x0-x2)*(z0-z1)) * ((x0-x1)*(z0-z2) - (x0-x2)*(z0-z1))
                                    + ((y0-y1)*(z0-z2) - (y0-y2)*(z0-z1)) * ((y0-y1)*(z0-z2) - (y0-y2)*(z0-z1)));
                    float l12 = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
                    float la = ((y1-y2)*((x0-x1)*(y0-y2) - (x0-x2)*(y0-y1))
                              + (z1-z2)*((x0-x1)*(z0-z2) - (x0-x2)*(z0-z1))) / a012 / l12;
                    float lb = -((x1-x2)*((x0-x1)*(y0-y2) - (x0-x2)*(y0-y1))
                               - (z1-z2)*((y0-y1)*(z0-z2) - (y0-y2)*(z0-z1))) / a012 / l12;
                    float lc = -((x1-x2)*((x0-x1)*(z0-z2) - (x0-x2)*(z0-z1))
                               + (y1-y2)*((y0-y1)*(z0-z2) - (y0-y2)*(z0-z1))) / a012 / l12;

                    // 点到直线的距离残差
                    float ld2 = a012 / l12;
                    float s = 1 - 0.9 * fabs(ld2);
                    float coeff_x = s * la;
                    float coeff_y = s * lb;
                    float coeff_z = s * lc;
                    if (s > 0.1)
                    {
                        laserCloudOriCornerVec.push_back(pointOri);
                        PointType coeff_pt;
                        coeff_pt.x = coeff_x; coeff_pt.y = coeff_y; coeff_pt.z = coeff_z;
                        coeff_pt.intensity = ld2;
                        coeffSelCornerVec.push_back(coeff_pt);
                    }
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri = laserCloudSurfLastDS->points[i];
            PointType pointSel;
            pointAssociateToMap(&pointOri, &pointSel);
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0)
            {
                // 平面拟合
                cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
                cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
                cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));
                for (int j = 0; j < 5; j++)
                {
                    matA0.at<float>(j,0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0.at<float>(j,1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0.at<float>(j,2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }
                cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

                float pa = matX0.at<float>(0,0);
                float pb = matX0.at<float>(1,0);
                float pc = matX0.at<float>(2,0);
                float pd = 1;
                float ps = sqrt(pa*pa + pb*pb + pc*pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid)
                {
                    // 点到平面距离
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x*pointSel.x + pointSel.y*pointSel.y + pointSel.z*pointSel.z));
                    if (s > 0.1)
                    {
                        laserCloudOriSurfVec.push_back(pointOri);
                        PointType coeff_pt;
                        coeff_pt.x = s * pa; coeff_pt.y = s * pb; coeff_pt.z = s * pc;
                        coeff_pt.intensity = s * pd2;
                        coeffSelSurfVec.push_back(coeff_pt);
                    }
                }
            }
        }
    }

    bool combineOptimization(int iterCount)
    {
        // 构建6-DOF法方程: H * dx = -b
        // dx = [roll, pitch, yaw, x, y, z]^T
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();

        int totalConstraints = 0;

        // 从当前transformTobeMapped构建变换矩阵
        Eigen::Affine3f T_cur = trans2Affine3f(transformTobeMapped);
        Eigen::Matrix3f R_cur = T_cur.rotation();
        Eigen::Vector3f t_cur = T_cur.translation();

        // ---- 边缘特征约束（点到直线距离） ----
        for (size_t i = 0; i < laserCloudOriCornerVec.size(); i++)
        {
            // 原始点（LiDAR系）
            Eigen::Vector3f p_lidar(laserCloudOriCornerVec[i].x,
                                     laserCloudOriCornerVec[i].y,
                                     laserCloudOriCornerVec[i].z);
            // 变换到世界系: p_world = R_cur * p_lidar + t_cur
            Eigen::Vector3f p_world = R_cur * p_lidar + t_cur;

            // 系数（方向余弦）
            float la = coeffSelCornerVec[i].x;
            float lb = coeffSelCornerVec[i].y;
            float lc = coeffSelCornerVec[i].z;
            float ld2 = coeffSelCornerVec[i].intensity;
            Eigen::Vector3f l_dir(la, lb, lc);

            // 残差: r = l_dir · p_world - ld2
            float r = l_dir.dot(p_world) - ld2;

            // 如果残差过大，跳过（外点剔除）
            if (fabs(r) > 2.0) continue;

            // 雅可比: dr/d(theta) = l_dir^T * (-skew(R*p_lidar))
            //          dr/d(t)     = l_dir^T
            Eigen::Vector3f Rp = R_cur * p_lidar;
            Eigen::Matrix<float, 3, 3> skewRp;
            skewRp <<     0, -Rp.z(),  Rp.y(),
                      Rp.z(),      0, -Rp.x(),
                     -Rp.y(),  Rp.x(),      0;
            Eigen::Matrix<float, 1, 3> J_rot = l_dir.transpose() * (-skewRp);
            Eigen::Matrix<float, 1, 3> J_trans = l_dir.transpose();

            // 权重：基于残差
            float weight = 1.0 / (1.0 + fabs(r));

            // 累加到法方程
            Eigen::Matrix<double, 6, 1> J_row;
            J_row << J_rot(0), J_rot(1), J_rot(2), J_trans(0), J_trans(1), J_trans(2);
            H += weight * J_row * J_row.transpose();
            b += weight * J_row * r;
            totalConstraints++;
        }

        // ---- 平面特征约束（点到平面距离） ----
        for (size_t i = 0; i < laserCloudOriSurfVec.size(); i++)
        {
            Eigen::Vector3f p_lidar(laserCloudOriSurfVec[i].x,
                                     laserCloudOriSurfVec[i].y,
                                     laserCloudOriSurfVec[i].z);
            Eigen::Vector3f p_world = R_cur * p_lidar + t_cur;

            // 平面法向量和偏移
            float pa = coeffSelSurfVec[i].x;
            float pb = coeffSelSurfVec[i].y;
            float pc = coeffSelSurfVec[i].z;
            float pd2 = coeffSelSurfVec[i].intensity;
            Eigen::Vector3f normal(pa, pb, pc);

            // 残差: r = normal · p_world + pd2
            float r = normal.dot(p_world) + pd2;

            if (fabs(r) > 2.0) continue;

            // 雅可比
            Eigen::Vector3f Rp = R_cur * p_lidar;
            Eigen::Matrix<float, 3, 3> skewRp;
            skewRp <<     0, -Rp.z(),  Rp.y(),
                      Rp.z(),      0, -Rp.x(),
                     -Rp.y(),  Rp.x(),      0;
            Eigen::Matrix<float, 1, 3> J_rot = normal.transpose() * (-skewRp);
            Eigen::Matrix<float, 1, 3> J_trans = normal.transpose();

            float weight = 1.0 / (1.0 + fabs(r));

            Eigen::Matrix<double, 6, 1> J_row;
            J_row << J_rot(0), J_rot(1), J_rot(2), J_trans(0), J_trans(1), J_trans(2);
            H += weight * J_row * J_row.transpose();
            b += weight * J_row * r;
            totalConstraints++;
        }

        if (totalConstraints < 50)
            return false;

        // LM阻尼因子（随迭代递减）
        float lambda = (iterCount < 5) ? 1e-2 : 1e-4;
        Eigen::Matrix<double, 6, 6> H_lm = H + lambda * Eigen::Matrix<double, 6, 6>::Identity();

        // 求解: H * dx = -b
        Eigen::Matrix<double, 6, 1> dx = H_lm.ldlt().solve(-b);

        // 检查解的有效性
        if (!dx.array().isFinite().all())
            return false;

        // 应用更新
        float roll  = transformTobeMapped[0] + dx(0);
        float pitch = transformTobeMapped[1] + dx(1);
        float yaw   = transformTobeMapped[2] + dx(2);

        // 角度归一化
        if (yaw > M_PI) yaw -= 2 * M_PI;
        if (yaw < -M_PI) yaw += 2 * M_PI;

        transformTobeMapped[0] = roll;
        transformTobeMapped[1] = pitch;
        transformTobeMapped[2] = yaw;
        transformTobeMapped[3] += dx(3);
        transformTobeMapped[4] += dx(4);
        transformTobeMapped[5] += dx(5);

        return true;
    }

    /**
     * IEKF状态更新 对应论文公式(11)-(17)
     *
     * 严格实现迭代卡尔曼滤波：
     *   - 公式(3-5):  状态定义，先验状态来自IMU预测
     *   - 公式(11-12): LiDAR观测模型（点线/点面残差）
     *   - 公式(13):    非线性目标函数 J(dx) = ||dx||_P^{-1} + ||f(x+dx)||_R^{-1}
     *   - 公式(14):    卡尔曼增益 K = P * H^T * (H * P * H^T + R)^{-1}
     *   - 公式(15-16): 后验状态更新与收敛判断
     *   - 公式(17):    全局位姿转换
     *
     * 关键：每轮IEKF迭代重新线性化观测模型（在更新后的状态处）
     */
    bool updateIEKFFromScanMatching()
    {
        if (!iekf.initialized) return false;
        if (laserCloudOriCornerVec.empty() && laserCloudOriSurfVec.empty()) return false;

        // 保存先验（IMU预测）状态
        IEKFStatePaper priorState = iekf.state;
        Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM> P_prior = iekf.state.P;

        // 外参（LiDAR到IMU）
        Eigen::Matrix3d R_li = extRot;
        Eigen::Vector3d t_li = extTrans;

        // 工作状态（迭代中更新）
        IEKFStatePaper state_k = priorState;

        // 观测噪声协方差（标量，公式13中的R）
        double R_obs = 0.01;

        // 保存最后迭代的 K, H_k, total_rows 用于非收敛回退
        Eigen::MatrixXd K_last, H_k_last;
        int total_rows_last = 0;

        // IEKF迭代循环（论文公式14-16）
        for (int iter = 0; iter < iekf.max_iterations; iter++)
        {
            // ===== 步骤1: 在当前状态处重新线性化观测模型 =====
            // 计算观测量 H_k 和残差 r_k（论文公式11-12）
            Eigen::Matrix3d Rk = state_k.rotation.toRotationMatrix();
            Eigen::Vector3d tk = state_k.position;

            int total_rows = laserCloudOriCornerVec.size() + laserCloudOriSurfVec.size();
            Eigen::MatrixXd H_k(total_rows, IEKF_STATE_DIM);
            Eigen::VectorXd r_k(total_rows);
            H_k.setZero();
            int row_idx = 0;

            // 边缘特征约束（点线距离，论文公式12上半部）
            for (size_t i = 0; i < laserCloudOriCornerVec.size(); i++)
            {
                Eigen::Vector3d p_L(laserCloudOriCornerVec[i].x,
                                     laserCloudOriCornerVec[i].y,
                                     laserCloudOriCornerVec[i].z);
                Eigen::Vector3d p_I = R_li * p_L + t_li;
                Eigen::Vector3d p_W = Rk * p_I + tk;

                Eigen::Vector3d l(coeffSelCornerVec[i].x,
                                  coeffSelCornerVec[i].y,
                                  coeffSelCornerVec[i].z);
                double ld2 = coeffSelCornerVec[i].intensity;

                double dist = l.dot(p_W) - ld2;
                r_k(row_idx) = dist;

                Eigen::Matrix3d skew_RpI = iekf.skewSymmetric(Rk * p_I);
                H_k.block<1,3>(row_idx, IEKF_IDX_ROT) = -l.transpose() * skew_RpI;
                H_k.block<1,3>(row_idx, IEKF_IDX_POS) = l.transpose();

                row_idx++;
            }

            // 平面特征约束（点面距离，论文公式12下半部）
            for (size_t i = 0; i < laserCloudOriSurfVec.size(); i++)
            {
                Eigen::Vector3d p_L(laserCloudOriSurfVec[i].x,
                                     laserCloudOriSurfVec[i].y,
                                     laserCloudOriSurfVec[i].z);
                Eigen::Vector3d p_I = R_li * p_L + t_li;
                Eigen::Vector3d p_W = Rk * p_I + tk;

                Eigen::Vector3d n(coeffSelSurfVec[i].x,
                                  coeffSelSurfVec[i].y,
                                  coeffSelSurfVec[i].z);
                double pd2 = coeffSelSurfVec[i].intensity;

                double dist = n.dot(p_W) + pd2;
                r_k(row_idx) = dist;

                Eigen::Matrix3d skew_RpI = iekf.skewSymmetric(Rk * p_I);
                H_k.block<1,3>(row_idx, IEKF_IDX_ROT) = -n.transpose() * skew_RpI;
                H_k.block<1,3>(row_idx, IEKF_IDX_POS) = n.transpose();

                row_idx++;
            }

            // ===== 步骤2: 卡尔曼增益（论文公式14） =====
            Eigen::MatrixXd HPHt = H_k * P_prior * H_k.transpose();
            Eigen::MatrixXd S = HPHt + R_obs * Eigen::MatrixXd::Identity(total_rows, total_rows);
            Eigen::MatrixXd K = P_prior * H_k.transpose() * S.inverse();

            // ===== 步骤3: 后验状态更新（论文公式15） =====
            Eigen::Matrix<double, IEKF_STATE_DIM, 1> dx = -K * r_k;

            // ===== 步骤4: 对先验状态应用误差状态（论文公式5） =====
            state_k = priorState;
            state_k.applyErrorState(dx);

            // 保存最后迭代的矩阵（非收敛回退使用）
            K_last = K;
            H_k_last = H_k;
            total_rows_last = total_rows;

            // ===== 步骤5: 收敛判断（论文公式16） =====
            if (dx.norm() < iekf.convergence_threshold)
            {
                iekf.state = state_k;
                Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM> I_KH =
                    Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM>::Identity() - K * H_k;
                iekf.state.P = I_KH * P_prior * I_KH.transpose() +
                                K * (R_obs * Eigen::MatrixXd::Identity(total_rows, total_rows)) * K.transpose();
                return true;
            }
        }

        // 达到最大迭代次数但未收敛：仍使用最后的迭代结果，但也要更新协方差
        iekf.state = state_k;
        if (total_rows_last > 0)
        {
            Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM> I_KH =
                Eigen::Matrix<double, IEKF_STATE_DIM, IEKF_STATE_DIM>::Identity() - K_last * H_k_last;
            iekf.state.P = I_KH * P_prior * I_KH.transpose() +
                            K_last * (R_obs * Eigen::MatrixXd::Identity(total_rows_last, total_rows_last)) * K_last.transpose();
        }
        return false;
    }

    // ======================================================
    // Part6: 关键帧与因子图
    // ======================================================

    void saveKeyFramesAndFactor()
    {
        // 保存当前位姿到关键帧列表
        PointType thisPose3D;
        PointTypePose thisPose6D;
        gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                  gtsam::Point3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));

        thisPose3D.x = transformTobeMapped[3];
        thisPose3D.y = transformTobeMapped[4];
        thisPose3D.z = transformTobeMapped[5];
        thisPose3D.intensity = cloudKeyPoses3D->size();
        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.roll = transformTobeMapped[0];
        thisPose6D.pitch = transformTobeMapped[1];
        thisPose6D.yaw = transformTobeMapped[2];
        thisPose6D.time = timeLaserInfoCur;
        thisPose6D.intensity = thisPose3D.intensity;

        cloudKeyPoses3D->push_back(thisPose3D);
        cloudKeyPoses6D->push_back(thisPose6D);

        // 保存当前帧特征点云
        cornerCloudKeyFrames.push_back(pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>(*laserCloudCornerLast)));
        surfCloudKeyFrames.push_back(pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>(*laserCloudSurfLast)));

        // 构建GTSAM因子图
        if (cloudKeyPoses3D->size() == 1)
        {
            // 第一帧：添加先验因子
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-6).finished());
            gtSAMgraph.add(PriorFactor<Pose3>(0, poseFrom, priorNoise));
            initialEstimate.insert(0, poseFrom);
        }
        else
        {
            // 添加里程计因子
            gtsam::Pose3 poseFromPrior = gtsam::Pose3(gtsam::Rot3::RzRyRx(
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].roll,
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].pitch,
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].yaw),
                gtsam::Point3(
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].x,
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].y,
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].z));

            noiseModel::Diagonal::shared_ptr odomNoise = noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished());
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-2, cloudKeyPoses3D->size()-1,
                                                poseFromPrior.between(poseFrom), odomNoise));
            initialEstimate.insert(cloudKeyPoses3D->size()-1, poseFrom);
        }

        // 添加回环约束
        if (!loopIndexQueue.empty())
        {
            for (size_t i = 0; i < loopIndexQueue.size(); ++i)
            {
                gtSAMgraph.add(BetweenFactor<Pose3>(loopIndexQueue[i].second, loopIndexQueue[i].first,
                                                     loopPoseQueue[i], loopNoiseQueue[i]));
            }
            loopIndexQueue.clear();
            loopPoseQueue.clear();
            loopNoiseQueue.clear();
            aLoopIsClosed = true;
        }

        // 执行ISAM2优化
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            aLoopIsClosed = false;
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        isamCurrentEstimate = isam->calculateEstimate();
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);
    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty()) return;
        if (aLoopIsClosed)
        {
            laserCloudMapContainer.clear();
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                if (isamCurrentEstimate.exists(i))
                {
                    cloudKeyPoses6D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                    cloudKeyPoses6D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                    cloudKeyPoses6D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();
                    cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                    cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                    cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                    cloudKeyPoses3D->points[i].x = cloudKeyPoses6D->points[i].x;
                    cloudKeyPoses3D->points[i].y = cloudKeyPoses6D->points[i].y;
                    cloudKeyPoses3D->points[i].z = cloudKeyPoses6D->points[i].z;
                }
            }
            // Sync IEKF with optimized pose
            int lastIdx = numPoses - 1;
            if (lastIdx >= 0 && isamCurrentEstimate.exists(lastIdx))
            {
                gtsam::Pose3 lastPose = isamCurrentEstimate.at<Pose3>(lastIdx);
                iekf.state.rotation = lastPose.rotation().toQuaternion();
                iekf.state.position = lastPose.translation();
            }
        }

        // 更新当前位姿
        int numPoses = cloudKeyPoses3D->size();
        if (numPoses < 2) return;
        transformTobeMapped[0] = cloudKeyPoses6D->points[numPoses-1].roll;
        transformTobeMapped[1] = cloudKeyPoses6D->points[numPoses-1].pitch;
        transformTobeMapped[2] = cloudKeyPoses6D->points[numPoses-1].yaw;
        transformTobeMapped[3] = cloudKeyPoses6D->points[numPoses-1].x;
        transformTobeMapped[4] = cloudKeyPoses6D->points[numPoses-1].y;
        transformTobeMapped[5] = cloudKeyPoses6D->points[numPoses-1].z;

        updatePointAssociateToMap();
    }

    // ======================================================
    // Part7: VGICP回环检测（论文公式19-28）
    // ======================================================

    void loopClosureThread()
    {
        if (!loopClosureEnableFlag) return;
        ros::Rate rate(loopClosureFrequency);
        while (ros::ok())
        {
            rate.sleep();
            performLoopClosure();
            visualizeLoopClosure();
        }
    }

    /**
     * 回环检测主流程（论文第3节）
     *
     * 步骤:
     *   1. 回环候选帧选取：基于空间距离和时间间隔 (论文第3节步骤1)
     *   2. VGICP配准验证 (论文公式19-27)
     *   3. 距离阈值验证 (论文公式28)
     *   4. 添加回环约束到因子图
     */
    void performLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty()) return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        int loopKeyCur, loopKeyPre;
        if (!detectLoopClosureDistance(&loopKeyCur, &loopKeyPre)) return;

        // 提取回环候选帧与历史关键帧点云
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);

        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000) return;

        // ===== 论文公式(19)-(27): VGICP配准 =====
        // 使用VGICP替代传统ICP进行回环帧匹配
        Eigen::Affine3f currentPose = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        Eigen::Affine3f prevPose = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyPre]);
        Eigen::Isometry3d initial_guess;
        initial_guess.linear() = (currentPose.inverse() * prevPose).rotation().cast<double>();
        initial_guess.translation() = (currentPose.inverse() * prevPose).translation().cast<double>();

        VGICPLoopDetector::VGICPResult vgicp_result = vgicp.align(cureKeyframeCloud, prevKeyframeCloud, initial_guess);

        // ===== 论文公式(28): 距离阈值验证 =====
        double dist = vgicp_result.transformation.translation().norm();
        if (!vgicp_result.converged || dist > historyKeyframeFitnessScore * 10)
        {
            // VGICP失败时尝试传统ICP作为fallback
            static pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setInputSource(cureKeyframeCloud);
            icp.setInputTarget(prevKeyframeCloud);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            icp.align(*unused_result);
            if (!icp.hasConverged() || icp.getFitnessScore() > historyKeyframeFitnessScore) return;

            // 构建回环约束
            Eigen::Affine3f correctionFrame = icp.getFinalTransformation();
            Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
            Eigen::Affine3f tCorrect = correctionFrame * tWrong;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

            gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
            gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
            gtsam::Vector Vector6(6);
            float noiseScore = icp.getFitnessScore();
            Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
            noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

            mtx.lock();
            loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
            loopPoseQueue.push_back(poseFrom.between(poseTo));
            loopNoiseQueue.push_back(constraintNoise);
            mtx.unlock();
            loopIndexContainer[loopKeyCur] = loopKeyPre;
        }
        else
        {
            // VGICP配准成功（论文公式27），验证通过后构建回环约束
            if (dist > vgicp.distance_threshold || dist < 0.1) return; // 论文公式(28)验证

            Eigen::Matrix3d R_vgicp = vgicp_result.transformation.linear();
            Eigen::Vector3d t_vgicp = vgicp_result.transformation.translation();

            // 从VGICP变换计算校正位姿
            Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
            Eigen::Affine3f correctionFrame = Eigen::Affine3f::Identity();
            correctionFrame.linear() = R_vgicp.cast<float>();
            correctionFrame.translation() = t_vgicp.cast<float>();
            Eigen::Affine3f tCorrect = correctionFrame * tWrong;

            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

            gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
            gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
            gtsam::Vector Vector6(6);
            float noiseScore = vgicp_result.fitness_score;
            Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
            noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

            mtx.lock();
            loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
            loopPoseQueue.push_back(poseFrom.between(poseTo));
            loopNoiseQueue.push_back(constraintNoise);
            mtx.unlock();
            loopIndexContainer[loopKeyCur] = loopKeyPre;
        }
    }

    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end()) return false;

        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        for (size_t i = 0; i < pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre) return false;
        *latestID = loopKeyCur;
        *closestID = loopKeyPre;
        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize) continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
        }
        if (nearKeyframes->empty()) return;
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure()
    {
        if (loopIndexContainer.empty() || pubLoopConstraintEdge.getNumSubscribers() == 0) return;
        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3;
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;

        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto& kv : loopIndexContainer)
        {
            int key_cur = kv.first;
            int key_pre = kv.second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }
        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge.publish(markerArray);
    }

    // ======================================================
    // Part8: 发布与可视化
    // ======================================================

    void publishOdometry()
    {
        nav_msgs::Odometry laserOdometry;
        laserOdometry.header.frame_id = odometryFrame;
        laserOdometry.child_frame_id = "odom";
        laserOdometry.header.stamp = timeLaserInfoStamp;
        laserOdometry.pose.pose.position.x = transformTobeMapped[3];
        laserOdometry.pose.pose.position.y = transformTobeMapped[4];
        laserOdometry.pose.pose.position.z = transformTobeMapped[5];
        tf::Quaternion q = tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        laserOdometry.pose.pose.orientation.x = q.x();
        laserOdometry.pose.pose.orientation.y = q.y();
        laserOdometry.pose.pose.orientation.z = q.z();
        laserOdometry.pose.pose.orientation.w = q.w();
        pubLaserOdometryGlobal.publish(laserOdometry);

        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);

        // Path
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header = laserOdometry.header;
        poseStamped.pose = laserOdometry.pose.pose;
        globalPath.header = laserOdometry.header;
        globalPath.header.frame_id = odometryFrame;
        globalPath.poses.push_back(poseStamped);
        pubPath.publish(globalPath);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty()) return;

        // 发布当前帧配准点云
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        *cloudOut += *transformPointCloud(laserCloudCornerLast, &thisPose6D);
        *cloudOut += *transformPointCloud(laserCloudSurfLast, &thisPose6D);
        publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);

        // 发布局部地图
        pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
        for (size_t i = 0; i < laserCloudCornerFromMapDS->size(); ++i)
            laserCloudSurround->push_back(laserCloudCornerFromMapDS->points[i]);
        for (size_t i = 0; i < laserCloudSurfFromMapDS->size(); ++i)
            laserCloudSurround->push_back(laserCloudSurfFromMapDS->points[i]);
        publishCloud(pubRecentKeyFrames, laserCloudSurround, timeLaserInfoStamp, odometryFrame);
    }

    void publishGlobalMap()
    {
        if (cloudKeyPoses3D->points.empty() || pubLaserCloudSurround.getNumSubscribers() == 0) return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());

        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);

        for (size_t i = 0; i < pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);

        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity);
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        for (auto& pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
            int thisKeyInd = (int)pt.intensity;
            *globalMapCloud += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapCloud += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        }

        pcl::PointCloud<PointType>::Ptr globalMapCloudDS(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMap;
        downSizeFilterGlobalMap.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize);
        downSizeFilterGlobalMap.setInputCloud(globalMapCloud);
        downSizeFilterGlobalMap.filter(*globalMapCloudDS);

        publishCloud(pubLaserCloudSurround, globalMapCloudDS, timeLaserInfoStamp, odometryFrame);
    }

    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        while (ros::ok())
        {
            rate.sleep();
            publishGlobalMap();
        }
    }

    // ======================================================
    // Part9: 工具函数
    // ======================================================

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);
        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto& pt = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pt.x + transCur(0,1) * pt.y + transCur(0,2) * pt.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pt.x + transCur(1,1) * pt.y + transCur(1,2) * pt.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pt.x + transCur(2,1) * pt.y + transCur(2,2) * pt.z + transCur(2,3);
            cloudOut->points[i].intensity = pt.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                            gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                            gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    bool saveMapService(lio_sam::save_mapRequest& req, lio_sam::save_mapResponse& res)
    {
        string saveMapDirectory = req.destination.empty() ? (string(getenv("HOME")) + savePCDDirectory) : (string(getenv("HOME")) + req.destination);
        int unused = system((string("exec rm -r ") + saveMapDirectory).c_str());
        unused = system((string("mkdir -p ") + saveMapDirectory).c_str());

        pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);

        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

        for (size_t i = 0; i < cloudKeyPoses3D->size(); i++)
        {
            *globalCornerCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *globalSurfCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            cout << "\r" << flush << i << " / " << cloudKeyPoses6D->size() << " ...";
        }
        *globalMapCloud += *globalCornerCloud;
        *globalMapCloud += *globalSurfCloud;

        int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
        res.success = ret == 0;
        cout << "\nSaving map completed.\n" << endl;
        return true;
    }
};

// ========== ROS 主节点入口 ==========
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam_paper");
    PaperSLAM paperSLAM;

    // 启动回环检测线程
    std::thread loopthread(&PaperSLAM::loopClosureThread, &paperSLAM);
    // 启动全局地图可视化线程
    std::thread vizThread(&PaperSLAM::visualizeGlobalMapThread, &paperSLAM);

    ROS_INFO("\033[1;32m----> Paper SLAM: IEKF + VGICP Loop Detection Ready.\033[0m");

    ros::spin();

    loopthread.join();
    vizThread.join();
    return 0;
}
