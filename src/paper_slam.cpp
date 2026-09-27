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
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
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
    ros::Subscriber subLaserCloudFront;
    ros::Subscriber subLaserCloudBack;
    ros::Subscriber subImu;
    ros::Subscriber subOdom;
    ros::Subscriber subGPS;

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
    std::deque<sensor_msgs::PointCloud2> cloudQueue[2];

    // ---- 当前点云缓存 ----
    sensor_msgs::PointCloud2 currentCloudMsg[2];

    // ---- IMU 缓存 ----
    double *imuTime;
    double *imuRotX;
    double *imuRotY;
    double *imuRotZ;
    double *imuAccX;
    double *imuAccY;
    double *imuAccZ;
    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;
    double lastScanPosX, lastScanPosY, lastScanPosZ;
    double lastScanTime;
    double lastIEKFPredictTime;          // last IMU time used for prediction (dedup)
    double totalImuYawAccum;          // running total of integrated IMU yaw (rad)
    double totalImuYawAtLastKf;       // totalImuYawAccum value at last saved keyframe
    double prevScanYaw;               // previous scan's optimized yaw (set even when kf skipped)
    double prevScanTime;              // time of previous scan
    int prevScanKfIdx;                // keyframe index used by previous scan's initial guess
    double iekfRefYaw;                // IEKF yaw at last LM update (for relative prior)
    double iekfRefVx, iekfRefVy;      // IEKF velocity at last LM update (for relative prior)
    bool headingInitialized;          // whether initial heading has been extracted from GPS/GT
    double imuWeightAdaptive;         // 在线自适应 IMU 权重（regime 切换，初始=imuWeight）
    double transDegBelief;            // 平移退化置信度 EMA（0..1，>0.5 判退化 regime → 信任 IMU）
    double condT[2];                  // 每雷达平移子空间条件数（Schur 补后）
    bool degX[2];                     // 每雷达是否沿 x 退化（weakIdx==3，上一迭代 adjustLidarHessian 计算）

    // ---- 点云数据 ----
    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn[2];
    pcl::PointCloud<PointType>::Ptr fullCloud[2];
    pcl::PointCloud<PointType>::Ptr extractedCloud[2];
    pcl::PointCloud<PointType>::Ptr cornerCloud[2];      // 边缘特征点（论文公式2）
    pcl::PointCloud<PointType>::Ptr surfaceCloud[2];      // 平面特征点（论文公式2）

    // ---- 特征提取相关（论文公式2） ----
    std::vector<smoothness_t> cloudSmoothness[2];
    float *cloudCurvature[2];
    int *cloudNeighborPicked[2];
    int *cloudLabel[2];

    // ---- 关键帧相关 ----
    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> denseCloudKeyFrames;   // full deskewed scans (dense map)
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    // ---- 当前帧特征点 ----
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast[2];
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast[2];
    pcl::PointCloud<PointType>::Ptr laserCloudDenseLast[2];   // 全量去畸变点（持久化，跨 resetParameters 保留）
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS[2];
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS[2];

    // ---- scan-to-map 匹配中间变量 ----
    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;
    std::vector<PointType> laserCloudOriCornerVec[2];
    std::vector<PointType> coeffSelCornerVec[2];
    std::vector<PointType> laserCloudOriSurfVec[2];
    std::vector<PointType> coeffSelSurfVec[2];
    // GenZ-ICP: target (closest) point in map for each edge correspondence
    std::vector<pcl::PointXYZI> laserCloudOriCornerTargetVec[2];

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
    double timeScanCur[2];
    double timeScanEnd[2];
    std_msgs::Header cloudHeader[2];
    lio_sam::cloud_info cloudInfo[2];

    float transformTobeMapped[6];
    bool odomDeskewFlag;
    float odomIncreX, odomIncreY, odomIncreZ;
    int deskewFlag;
    cv::Mat rangeMat[2];

    std::mutex imuLock;
    std::mutex odoLock;
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

    Eigen::Affine3f transPointAssociateToMap[2];
    Eigen::Affine3f incrementalOdometryAffineFront;

    double maxDistFromOrigin_;

    int laserCloudCornerFromMapDSNum;
    int laserCloudSurfFromMapDSNum;
    int laserCloudCornerLastDSNum[2];
    int laserCloudSurfLastDSNum[2];

    vector<int> columnIdnCountVec;

    // ---- 退化检测与方向性自适应加权状态（双雷达） ----
    Eigen::Matrix<double,6,6> Hraw[2];       // raw information matrix per lidar
    Eigen::Matrix<double,6,1> braw[2];       // raw gradient per lidar
    Eigen::Matrix<double,6,6> eigVecs[2];    // eigenvectors (ascending eigenvalue columns)
    Eigen::Matrix<double,6,1> eigVals[2];    // ascending eigenvalues
    Eigen::Matrix<double,6,1> dW[2];         // directional weights d_k
    Eigen::Matrix<double,6,1> vWeak[2];      // weakest eigenvector
    bool isDeg[2];                           // whether weakest direction is degenerate
    int weakIdx[2];                          // index of weakest direction (0..5)
    Eigen::Matrix<double,6,6> projMat[2];    // V·diag(d_k)·Vᵀ 方向投影矩阵（用于更新量投影）

    // ---- per-LiDAR config accessors (single mode: lid=0 maps to main sensor/geometry) ----
    SensorType sensorOf(int lid) { return lid == 0 ? sensorFront : sensorBack; }
    int nScanOf(int lid) { return lid == 0 ? N_SCANFront : N_SCANBack; }
    int horizonOf(int lid) { return lid == 0 ? Horizon_SCANFront : Horizon_SCANBack; }
    const Eigen::Affine3d& extAffineOf(int lid) { return lid == 0 ? extAffineFront : extAffineBack; }

    // ========== 构造函数 ==========
    PaperSLAM() : deskewFlag(0), totalImuYawAccum(0), totalImuYawAtLastKf(0),
                  prevScanYaw(0), prevScanTime(0), prevScanKfIdx(-1),
                  iekfRefYaw(0), iekfRefVx(0), iekfRefVy(0),
                  headingInitialized(false),
                  lastScanPosX(0), lastScanPosY(0), lastScanPosZ(0), lastScanTime(0),
                  lastIEKFPredictTime(-1),
                  maxDistFromOrigin_(0.0)
    {
        imuWeightAdaptive = adaptiveImuWeight ? imuWeightMin : imuWeight;
        transDegBelief = 0.0;
        condT[0] = condT[1] = 1.0;
        degX[0] = degX[1] = false;

        // 初始化ISAM2 (performance-tuned: reduce relinearization frequency)
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        int relinearizeSkip;
        nh.param<int>("lio_sam/relinearizeSkip", relinearizeSkip, 10);
        parameters.relinearizeSkip = relinearizeSkip;
        isam = new ISAM2(parameters);

        // 分配IMU缓存
        imuTime = new double[queueLength];
        imuRotX = new double[queueLength];
        imuRotY = new double[queueLength];
        imuRotZ = new double[queueLength];
        imuAccX = new double[queueLength];
        imuAccY = new double[queueLength];
        imuAccZ = new double[queueLength];

        // 订阅话题
        subImu = nh.subscribe<sensor_msgs::Imu>(
            imuTopic, 2000, &PaperSLAM::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom = nh.subscribe<nav_msgs::Odometry>(
            odomTopic, 2000, &PaperSLAM::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        if (useDualLidar)
        {
            subLaserCloudFront = nh.subscribe<sensor_msgs::PointCloud2>(
                pointCloudTopicFront, 2000, &PaperSLAM::cloudHandlerFront, this, ros::TransportHints().tcpNoDelay());
            subLaserCloudBack = nh.subscribe<sensor_msgs::PointCloud2>(
                pointCloudTopicBack, 2000, &PaperSLAM::cloudHandlerBack, this, ros::TransportHints().tcpNoDelay());
            ROS_INFO("Dual-LiDAR mode: front=%s back=%s", pointCloudTopicFront.c_str(), pointCloudTopicBack.c_str());
        }
        else
        {
            subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(
                pointCloudTopic, 2000, &PaperSLAM::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        }
        subGPS = nh.subscribe<nav_msgs::Odometry>(
            gpsTopic, 200, &PaperSLAM::gpsHandler, this, ros::TransportHints().tcpNoDelay());

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
        resetParameters(0);
        resetParameters(1);

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

        ROS_INFO("\033[1;32m----> OURS SLAM Algorithm Started.\033[0m");
        ROS_INFO("  IEKF + VGICP Loop Closure (OURS)");
        ROS_INFO("  imuWeight=%.2f (IMU velocity seed + yaw prior scale)", (double)imuWeight);
        ROS_INFO("  VGICP: voxel=%.2f radius=%.2f dist_th=%.2f",
                 vgicp.voxel_resolution, vgicp.neighbor_search_radius, vgicp.distance_threshold);
    }

    ~PaperSLAM()
    {
        delete[] imuTime;
        delete[] imuRotX;
        delete[] imuRotY;
        delete[] imuRotZ;
        delete[] imuAccX;
        delete[] imuAccY;
        delete[] imuAccZ;
        for (int lid = 0; lid < 2; ++lid)
        {
            delete[] cloudCurvature[lid];
            delete[] cloudNeighborPicked[lid];
            delete[] cloudLabel[lid];
        }
        delete isam;
    }

    // ========== 内存分配 ==========
    void allocateMemory()
    {
        for (int lid = 0; lid < 2; ++lid)
        {
            laserCloudIn[lid].reset(new pcl::PointCloud<PointXYZIRT>());
            fullCloud[lid].reset(new pcl::PointCloud<PointType>());
            extractedCloud[lid].reset(new pcl::PointCloud<PointType>());
            cornerCloud[lid].reset(new pcl::PointCloud<PointType>());
            surfaceCloud[lid].reset(new pcl::PointCloud<PointType>());

            fullCloud[lid]->points.resize(nScanOf(lid) * horizonOf(lid));

            laserCloudCornerLast[lid].reset(new pcl::PointCloud<PointType>());
            laserCloudSurfLast[lid].reset(new pcl::PointCloud<PointType>());
            laserCloudDenseLast[lid].reset(new pcl::PointCloud<PointType>());
            laserCloudCornerLastDS[lid].reset(new pcl::PointCloud<PointType>());
            laserCloudSurfLastDS[lid].reset(new pcl::PointCloud<PointType>());

            cloudSmoothness[lid].resize(nScanOf(lid) * horizonOf(lid));
            cloudCurvature[lid] = new float[nScanOf(lid) * horizonOf(lid)];
            cloudNeighborPicked[lid] = new int[nScanOf(lid) * horizonOf(lid)];
            cloudLabel[lid] = new int[nScanOf(lid) * horizonOf(lid)];
        }

        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

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

        for (int i = 0; i < 6; ++i)
            transformTobeMapped[i] = 0;
    }

    // ========== 参数重置（每帧处理前调用） ==========
    void resetParameters(int lid)
    {
        laserCloudIn[lid]->clear();
        extractedCloud[lid]->clear();
        cornerCloud[lid]->clear();
        surfaceCloud[lid]->clear();

        rangeMat[lid] = cv::Mat(nScanOf(lid), horizonOf(lid), CV_32F, cv::Scalar::all(FLT_MAX));

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
            imuAccX[i] = 0;
            imuAccY[i] = 0;
            imuAccZ[i] = 0;
        }
        columnIdnCountVec.assign(nScanOf(lid), 0);
        cloudInfo[lid].startRingIndex.assign(nScanOf(lid), 0);
        cloudInfo[lid].endRingIndex.assign(nScanOf(lid), 0);
        cloudInfo[lid].pointColInd.assign(nScanOf(lid) * horizonOf(lid), 0);
        cloudInfo[lid].pointRange.assign(nScanOf(lid) * horizonOf(lid), 0);
    }

    // ======================================================
    // Part2: 回调函数 + 点云投影与去畸变
    // ======================================================

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        static int imuCount = 0;
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);
        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
        imuCount++;
        if (imuCount == 1) ROS_INFO("DEBUG: First IMU received, stamp=%.3f", imuMsg->header.stamp.toSec());
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
        static int odomCount = 0;
        if (++odomCount == 1) ROS_INFO("DEBUG: First Odom received, stamp=%.3f x=%.3f y=%.3f",
            odometryMsg->header.stamp.toSec(),
            odometryMsg->pose.pose.position.x,
            odometryMsg->pose.pose.position.y);
    }

    void gpsHandler(const nav_msgs::Odometry::ConstPtr& gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
        // Initialize heading from first GPS/GT message
        if (!headingInitialized)
        {
            tf::Quaternion q(gpsMsg->pose.pose.orientation.x,
                             gpsMsg->pose.pose.orientation.y,
                             gpsMsg->pose.pose.orientation.z,
                             gpsMsg->pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            cloudInfo[0].imuRollInit = roll;
            cloudInfo[0].imuPitchInit = pitch;
            cloudInfo[0].imuYawInit = yaw;
            cloudInfo[1].imuRollInit = roll;
            cloudInfo[1].imuPitchInit = pitch;
            cloudInfo[1].imuYawInit = yaw;
            headingInitialized = true;
            ROS_INFO("HEADING: initialized from GPS rpy=(%.2f,%.2f,%.2f) deg",
                     roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
        }
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        processCloud(laserCloudMsg, 0);
    }

    void cloudHandlerFront(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        processCloud(laserCloudMsg, 0);
    }

    void cloudHandlerBack(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        processCloud(laserCloudMsg, 1);
    }

    void processCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg, int lid)
    {
        static int cloudCount[2] = {0, 0};
        static int cacheFail[2] = {0, 0};
        static int deskewFail[2] = {0, 0};
        cloudCount[lid]++;
        if (!cachePointCloud(laserCloudMsg, lid)) { cacheFail[lid]++; return; }
        if (!deskewInfo(lid)) { deskewFail[lid]++; return; }
        projectPointCloud(lid);
        cloudExtraction(lid);
        calculateSmoothness(lid);
        markOccludedPoints(lid);
        extractFeatures(lid);
        *laserCloudDenseLast[lid] = *extractedCloud[lid];
        publishFeatureCloud(lid);
        maybeRunMapping(lid);

        resetParameters(lid);
        if (cloudCount[lid] % 100 == 0)
            ROS_INFO("clouds=%d kf=%zu deskewF=%d (lid=%d)",
                     cloudCount[lid], cloudKeyPoses3D->size(), deskewFail[lid], lid);

        publishOdometry();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg, int lid)
    {
        cloudQueue[lid].push_back(*laserCloudMsg);
        if (cloudQueue[lid].size() <= 2) return false;
        currentCloudMsg[lid] = std::move(cloudQueue[lid].front());
        cloudQueue[lid].pop_front();

        SensorType sen = sensorOf(lid);
        if (sen == SensorType::VELODYNE || sen == SensorType::LIVOX)
            pcl::fromROSMsg(currentCloudMsg[lid], *laserCloudIn[lid]);
        else if (sen == SensorType::OUSTER)
        {
            pcl::PointCloud<OusterPointXYZIRT> tmpOusterCloud;
            pcl::fromROSMsg(currentCloudMsg[lid], tmpOusterCloud);
            laserCloudIn[lid]->clear();
            for (auto& pt : tmpOusterCloud.points)
            {
                VelodynePointXYZIRT vpt;
                vpt.x = pt.x; vpt.y = pt.y; vpt.z = pt.z;
                vpt.intensity = pt.intensity;
                vpt.ring = pt.ring;
                vpt.time = pt.t * 1e-9f;
                laserCloudIn[lid]->points.push_back(vpt);
            }
        }
        timeScanCur[lid] = currentCloudMsg[lid].header.stamp.toSec();
        timeScanEnd[lid] = timeScanCur[lid] + laserCloudIn[lid]->points.back().time;

        if (laserCloudIn[lid]->points.empty()) return false;

        cloudHeader[lid] = currentCloudMsg[lid].header;
        cloudInfo[lid].header = cloudHeader[lid];
        return true;
    }

    bool deskewInfo(int lid)
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur[lid])
            return false;
        imuDeskewInfo(lid);
        if (lid == 0 || !useDualLidar)
            odomDeskewInfo();
        return true;
    }

    void imuDeskewInfo(int lid)
    {
        if (imuQueue.empty()) return;
        cloudInfo[lid].imuAvailable = false;
        while (!imuQueue.empty() && imuQueue.front().header.stamp.toSec() < timeScanCur[lid] - 0.1)
        {
            imuQueue.pop_front();
            if (imuQueue.empty()) return;
        }
        if (imuQueue.empty()) return;
        imuPointerCur = 0;
        for (auto& thisImuMsg : imuQueue)
        {
            double imuTime_cur = thisImuMsg.header.stamp.toSec();
            if (imuTime_cur < timeScanCur[lid] - 0.1) continue;
            if (imuTime_cur > timeScanEnd[lid] + 0.1) break;
            imuRotX[imuPointerCur] = thisImuMsg.angular_velocity.x;
            imuRotY[imuPointerCur] = thisImuMsg.angular_velocity.y;
            imuRotZ[imuPointerCur] = thisImuMsg.angular_velocity.z;
            imuTime[imuPointerCur] = imuTime_cur;
            if (imuPointerCur > 0)
            {
                Eigen::Vector3d acc(thisImuMsg.linear_acceleration.x, thisImuMsg.linear_acceleration.y, thisImuMsg.linear_acceleration.z);
                Eigen::Vector3d gyr(thisImuMsg.angular_velocity.x, thisImuMsg.angular_velocity.y, thisImuMsg.angular_velocity.z);
                if (!iekf.initialized)
                {
                    iekf.initialize(acc, gyr);
                    lastIEKFPredictTime = imuTime_cur;
                }
                else if (imuTime_cur > lastIEKFPredictTime)
                {
                    double dt = imuTime_cur - lastIEKFPredictTime;
                    if (dt > 0 && dt < 0.05)
                        iekf.predict(acc, gyr, dt);
                    lastIEKFPredictTime = imuTime_cur;
                }
            }
            imuPointerCur++;
            if (imuPointerCur >= queueLength) break;
        }
        cloudInfo[lid].imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo[0].odomAvailable = false;
        cloudInfo[1].odomAvailable = false;
        // Keep messages from last 0.2s + ensure at least 2 messages for velocity
        while (!odomQueue.empty() && odomQueue.front().header.stamp.toSec() < timeScanCur[0] - 0.2
               && odomQueue.size() > 2)
        {
            odomQueue.pop_front();
            if (odomQueue.empty()) return;
        }
        if (odomQueue.empty()) return;
        if (odomQueue.front().header.stamp.toSec() > timeScanCur[0]) return;

        // Use last TWO messages for instantaneous velocity
        nav_msgs::Odometry endOdomMsg = odomQueue.back();
        double endOdomTime = endOdomMsg.header.stamp.toSec();

        nav_msgs::Odometry prevOdomMsg;
        double prevOdomTime;
        if (odomQueue.size() >= 2)
        {
            prevOdomMsg = odomQueue[odomQueue.size() - 2];
            prevOdomTime = prevOdomMsg.header.stamp.toSec();
        }
        else
        {
            prevOdomMsg = odomQueue.front();
            prevOdomTime = prevOdomMsg.header.stamp.toSec();
        }

        // Compute instantaneous velocity from last two consecutive messages
        if (endOdomTime > prevOdomTime)
        {
            double dt = endOdomTime - prevOdomTime;
            odomIncreX = (endOdomMsg.pose.pose.position.x - prevOdomMsg.pose.pose.position.x) / dt;
            odomIncreY = (endOdomMsg.pose.pose.position.y - prevOdomMsg.pose.pose.position.y) / dt;
            odomIncreZ = (endOdomMsg.pose.pose.position.z - prevOdomMsg.pose.pose.position.z) / dt;
        }
        else
        {
            odomIncreX = 0; odomIncreY = 0; odomIncreZ = 0;
        }

        // Use latest odom message for initial guess
        Eigen::Affine3f transStart = pcl::getTransformation(
            endOdomMsg.pose.pose.position.x,
            endOdomMsg.pose.pose.position.y,
            endOdomMsg.pose.pose.position.z, 0, 0, 0);
        transStartInverse = transStart.inverse();

        tf::Quaternion q(endOdomMsg.pose.pose.orientation.x, endOdomMsg.pose.pose.orientation.y,
                         endOdomMsg.pose.pose.orientation.z, endOdomMsg.pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        for (int lid = 0; lid < 2; ++lid)
        {
            cloudInfo[lid].odomAvailable = true;
            cloudInfo[lid].initialGuessX = endOdomMsg.pose.pose.position.x;
            cloudInfo[lid].initialGuessY = endOdomMsg.pose.pose.position.y;
            cloudInfo[lid].initialGuessZ = endOdomMsg.pose.pose.position.z;
            cloudInfo[lid].initialGuessRoll = roll;
            cloudInfo[lid].initialGuessPitch = pitch;
            cloudInfo[lid].initialGuessYaw = yaw;
        }
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

    void findPosition(int lid, double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;
        if (cloudInfo[lid].odomAvailable)
        {
            // KITTI Velodyne HDL-64E: X=forward, Y=left, Z=up
            // odom X = forward speed → LiDAR X
            *posXCur = odomIncreX * relTime;
            *posYCur = odomIncreY * relTime;
            *posZCur = odomIncreZ * relTime;
        }
    }

    void projectPointCloud(int lid)
    {
        int N = nScanOf(lid);
        int H = horizonOf(lid);
        int cloudSize = laserCloudIn[lid]->points.size();
        for (int i = 0; i < cloudSize; ++i)
        {
            auto& pt = laserCloudIn[lid]->points[i];
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            float range = sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (range < lidarMinRange || range > lidarMaxRange) continue;

            int rowIdn = pt.ring;
            if (rowIdn < 0 || rowIdn >= N) continue;
            if (rowIdn % downsampleRate != 0) continue;

            float horizonAngle = atan2(pt.x, pt.y) * 180 / M_PI;
            int columnIdn = -round((horizonAngle - 90.0) / 360.0 * H) + H / 2;
            if (columnIdn >= H) columnIdn -= H;
            if (columnIdn < 0 || columnIdn >= H) continue;

            if (rangeMat[lid].at<float>(rowIdn, columnIdn) != FLT_MAX) continue;
            rangeMat[lid].at<float>(rowIdn, columnIdn) = range;

            // 运动畸变补偿
            if (deskewFlag >= 0)
            {
                float rotXCur, rotYCur, rotZCur;
                findRotation(pt.time, &rotXCur, &rotYCur, &rotZCur);
                float posXCur, posYCur, posZCur;
                findPosition(lid, pt.time - laserCloudIn[lid]->points.front().time, &posXCur, &posYCur, &posZCur);
                float x1 = pt.x - posXCur;
                float y1 = pt.y - posYCur;
                float z1 = pt.z - posZCur;
                float rx = imuWeightAdaptive * rotXCur, ry = imuWeightAdaptive * rotYCur, rz = imuWeightAdaptive * rotZCur;
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

            int index = columnIdn + rowIdn * H;
            fullCloud[lid]->points[index].x = pt.x;
            fullCloud[lid]->points[index].y = pt.y;
            fullCloud[lid]->points[index].z = pt.z;
            fullCloud[lid]->points[index].intensity = pt.intensity;
        }
    }

    void cloudExtraction(int lid)
    {
        int N = nScanOf(lid);
        int H = horizonOf(lid);
        int count = 0;
        for (int i = 0; i < N; ++i)
        {
            cloudInfo[lid].startRingIndex[i] = count - 1 + 5;
            for (int j = 0; j < H; ++j)
            {
                if (rangeMat[lid].at<float>(i, j) != FLT_MAX)
                {
                    cloudInfo[lid].pointColInd[count] = j;
                    cloudInfo[lid].pointRange[count] = rangeMat[lid].at<float>(i, j);
                    extractedCloud[lid]->push_back(fullCloud[lid]->points[j + i * H]);
                    ++count;
                }
            }
            cloudInfo[lid].endRingIndex[i] = count - 1 - 5;
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
    void calculateSmoothness(int lid)
    {
        int cloudSize = extractedCloud[lid]->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {
            float diffRange = cloudInfo[lid].pointRange[i-5] + cloudInfo[lid].pointRange[i-4]
                            + cloudInfo[lid].pointRange[i-3] + cloudInfo[lid].pointRange[i-2]
                            + cloudInfo[lid].pointRange[i-1] - cloudInfo[lid].pointRange[i] * 10
                            + cloudInfo[lid].pointRange[i+1] + cloudInfo[lid].pointRange[i+2]
                            + cloudInfo[lid].pointRange[i+3] + cloudInfo[lid].pointRange[i+4]
                            + cloudInfo[lid].pointRange[i+5];
            cloudCurvature[lid][i] = diffRange * diffRange;
            cloudNeighborPicked[lid][i] = 0;
            cloudLabel[lid][i] = 0;
            cloudSmoothness[lid][i].value = cloudCurvature[lid][i];
            cloudSmoothness[lid][i].ind = i;
        }
    }

    void markOccludedPoints(int lid)
    {
        int cloudSize = extractedCloud[lid]->points.size();
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            float depth1 = cloudInfo[lid].pointRange[i];
            float depth2 = cloudInfo[lid].pointRange[i+1];
            int columnDiff = std::abs(int(cloudInfo[lid].pointColInd[i+1] - cloudInfo[lid].pointColInd[i]));
            if (columnDiff < 10)
            {
                if (depth1 - depth2 > 0.3)
                {
                    for (int l = -5; l <= 0; ++l) cloudNeighborPicked[lid][i + l] = 1;
                }
                else if (depth2 - depth1 > 0.3)
                {
                    for (int l = 1; l <= 6; ++l) cloudNeighborPicked[lid][i + l] = 1;
                }
            }
            float diff1 = std::abs(cloudInfo[lid].pointRange[i-1] - cloudInfo[lid].pointRange[i]);
            float diff2 = std::abs(cloudInfo[lid].pointRange[i+1] - cloudInfo[lid].pointRange[i]);
            if (diff1 > 0.02 * cloudInfo[lid].pointRange[i] && diff2 > 0.02 * cloudInfo[lid].pointRange[i])
                cloudNeighborPicked[lid][i] = 1;
        }
    }

    /**
     * 论文公式(2): 根据曲率大小区分边缘点和平面点
     * - 曲率最大的N_edge个点为边缘特征点(edge features)
     * - 曲率最小的N_plane个点为平面特征点(plane features)
     */
    void extractFeatures(int lid)
    {
        cornerCloud[lid]->clear();
        surfaceCloud[lid]->clear();
        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());

        int N = nScanOf(lid);
        for (int i = 0; i < N; i++)
        {
            surfaceCloudScan->clear();
            for (int j = 0; j < 6; j++)
            {
                int sp = (cloudInfo[lid].startRingIndex[i] * (6 - j) + cloudInfo[lid].endRingIndex[i] * j) / 6;
                int ep = (cloudInfo[lid].startRingIndex[i] * (5 - j) + cloudInfo[lid].endRingIndex[i] * (j + 1)) / 6 - 1;
                if (sp >= ep) continue;

                std::sort(cloudSmoothness[lid].begin() + sp, cloudSmoothness[lid].begin() + ep, by_value());

                // 取曲率最大的点为边缘特征点
                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[lid][k].ind;
                    if (cloudNeighborPicked[lid][ind] == 0 && cloudCurvature[lid][ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20)
                        {
                            cloudLabel[lid][ind] = 1;
                            cornerCloud[lid]->push_back(extractedCloud[lid]->points[ind]);
                        }
                        else break;
                        cloudNeighborPicked[lid][ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            if (std::abs(int(cloudInfo[lid].pointColInd[ind + l] - cloudInfo[lid].pointColInd[ind + l - 1])) > 10) break;
                            cloudNeighborPicked[lid][ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            if (std::abs(int(cloudInfo[lid].pointColInd[ind + l] - cloudInfo[lid].pointColInd[ind + l + 1])) > 10) break;
                            cloudNeighborPicked[lid][ind + l] = 1;
                        }
                    }
                }

                // 取曲率最小的点为平面特征点
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[lid][k].ind;
                    if (cloudNeighborPicked[lid][ind] == 0 && cloudCurvature[lid][ind] < surfThreshold)
                    {
                        cloudLabel[lid][ind] = -1;
                        cloudNeighborPicked[lid][ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            if (std::abs(int(cloudInfo[lid].pointColInd[ind + l] - cloudInfo[lid].pointColInd[ind + l - 1])) > 10) break;
                            cloudNeighborPicked[lid][ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            if (std::abs(int(cloudInfo[lid].pointColInd[ind + l] - cloudInfo[lid].pointColInd[ind + l + 1])) > 10) break;
                            cloudNeighborPicked[lid][ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[lid][k] <= 0)
                        surfaceCloudScan->push_back(extractedCloud[lid]->points[k]);
                }
            }

            // 下采样平面特征点
            pcl::PointCloud<PointType> surfaceCloudScanDS;
            pcl::VoxelGrid<PointType> surfFilter;
            surfFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);
            surfFilter.setInputCloud(surfaceCloudScan);
            surfFilter.filter(surfaceCloudScanDS);
            *surfaceCloud[lid] += surfaceCloudScanDS;
        }
    }

    void publishFeatureCloud(int lid)
    {
        cloudInfo[lid].cloud_corner = publishCloud(pubCornerPoints, cornerCloud[lid], cloudHeader[lid].stamp, lidarFrame);
        cloudInfo[lid].cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud[lid], cloudHeader[lid].stamp, lidarFrame);
        cloudInfo[lid].cloud_deskewed = publishCloud(pubExtractedCloud, extractedCloud[lid], cloudHeader[lid].stamp, lidarFrame);
        pubLaserCloudInfo.publish(cloudInfo[lid]);

        // 保存特征点用于后续匹配
        *laserCloudCornerLast[lid] = *cornerCloud[lid];
        *laserCloudSurfLast[lid] = *surfaceCloud[lid];
    }

    // 点云拼接基线：把后雷达特征点转到机体系（前雷达系），合并进前雷达，走单雷达 pipeline。
    void mergeBackToFront()
    {
        Eigen::Affine3f T = extAffineBack.cast<float>();  // T_body_backlidar (lidar -> body/front)
        for (const auto& p : laserCloudCornerLast[1]->points)
        {
            Eigen::Vector3f pb = T * Eigen::Vector3f(p.x, p.y, p.z);
            PointType q = p; q.x = pb.x(); q.y = pb.y(); q.z = pb.z();
            laserCloudCornerLast[0]->push_back(q);
        }
        for (const auto& p : laserCloudSurfLast[1]->points)
        {
            Eigen::Vector3f pb = T * Eigen::Vector3f(p.x, p.y, p.z);
            PointType q = p; q.x = pb.x(); q.y = pb.y(); q.z = pb.z();
            laserCloudSurfLast[0]->push_back(q);
        }
        for (const auto& p : extractedCloud[1]->points)
        {
            Eigen::Vector3f pb = T * Eigen::Vector3f(p.x, p.y, p.z);
            PointType q = p; q.x = pb.x(); q.y = pb.y(); q.z = pb.z();
            extractedCloud[0]->push_back(q);
        }
        laserCloudCornerLast[1]->clear();
        laserCloudSurfLast[1]->clear();
        extractedCloud[1]->clear();
    }

    void maybeRunMapping(int lid)
    {
        // In dual mode, mapping is triggered once per cycle (on the back lidar).
        if (useDualLidar && lid != 1) return;

        static double timeLastProcessing = -1;
        if (timeScanCur[lid] - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeScanCur[lid];
            timeLaserInfoStamp = cloudHeader[lid].stamp;
            timeLaserInfoCur = timeScanCur[lid];

            updateInitialGuess();
            extractSurroundingKeyFrames();
            if (useDualLidar)
            {
                if (mergeClouds)
                {
                    mergeBackToFront();
                    downsampleCurrentScan(0);
                    scan2MapOptimizationDual();
                }
                else
                {
                    downsampleCurrentScan(0);
                    downsampleCurrentScan(1);
                    scan2MapOptimizationDual();
                }
            }
            else
            {
                downsampleCurrentScan(0);
                scan2MapOptimization();
            }
            prevScanYaw = transformTobeMapped[2];
            prevScanTime = timeLaserInfoCur;
            lastScanPosX = transformTobeMapped[3];
            lastScanPosY = transformTobeMapped[4];
            lastScanPosZ = transformTobeMapped[5];
            lastScanTime = timeLaserInfoCur;
            saveKeyFramesAndFactor();
            correctPoses();
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
            if (iekf.initialized)
            {
                Eigen::Vector3d euler = IEKFStatePaper::quaternionToEuler(iekf.state.rotation);
                transformTobeMapped[0] = euler.z();
                transformTobeMapped[1] = euler.y();
            }
            else
            {
                transformTobeMapped[0] = cloudInfo[0].imuRollInit;
                transformTobeMapped[1] = cloudInfo[0].imuPitchInit;
            }
            transformTobeMapped[2] = cloudInfo[0].imuYawInit;
            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;
            return;
        }

        // Odometry + IEKF velocity hybrid initial guess.
        // Position: odometry (when available) or IEKF velocity integration.
        // Yaw: odometry (when available) or IEKF gyro integration.
        if (cloudKeyPoses3D->size() >= 1)
        {
            int lastIdx = cloudKeyPoses3D->size() - 1;
            double dt = (lastScanTime > 0) ? (timeLaserInfoCur - lastScanTime) : 0.0;
            if (dt < 0 || dt > 2.0) dt = 0;

            if (cloudInfo[0].odomAvailable)
            {
                transformTobeMapped[3] = cloudInfo[0].initialGuessX;
                transformTobeMapped[4] = cloudInfo[0].initialGuessY;
                transformTobeMapped[0] = cloudInfo[0].initialGuessRoll;
                transformTobeMapped[1] = cloudInfo[0].initialGuessPitch;
                transformTobeMapped[2] = cloudInfo[0].initialGuessYaw;
            }
            else if (iekf.initialized)
            {
                if (dt > mappingProcessInterval * 2.0) dt = mappingProcessInterval;
                transformTobeMapped[0] = cloudKeyPoses6D->points[lastIdx].roll;
                transformTobeMapped[1] = cloudKeyPoses6D->points[lastIdx].pitch;
                transformTobeMapped[2] = cloudKeyPoses6D->points[lastIdx].yaw;
                transformTobeMapped[3] = lastScanPosX + imuWeightAdaptive * iekf.state.velocity.x() * dt;
                transformTobeMapped[4] = lastScanPosY + imuWeightAdaptive * iekf.state.velocity.y() * dt;
            }
            else
            {
                // Fallback: odometry incremental velocity
                if (dt > mappingProcessInterval * 2.0) dt = mappingProcessInterval;
                transformTobeMapped[3] = lastScanPosX + odomIncreX * dt;
                transformTobeMapped[4] = lastScanPosY + odomIncreY * dt;
                transformTobeMapped[0] = cloudKeyPoses6D->points[lastIdx].roll;
                transformTobeMapped[1] = cloudKeyPoses6D->points[lastIdx].pitch;
                transformTobeMapped[2] = cloudKeyPoses6D->points[lastIdx].yaw;
            }
            transformTobeMapped[5] = cloudKeyPoses6D->points[lastIdx].z;
        }
        static int dbgInit = 0;
        if (++dbgInit <= 5) ROS_INFO("INIT_GUESS #%d: ttm=(%.2f,%.2f,%.2f,%.2f,%.2f,%.2f) kf=(%.2f,%.2f,%.2f) odom=%d",
                 dbgInit, transformTobeMapped[0]*180/M_PI, transformTobeMapped[1]*180/M_PI,
                 transformTobeMapped[2]*180/M_PI, transformTobeMapped[3], transformTobeMapped[4],
                 transformTobeMapped[5], cloudKeyPoses6D->back().roll*180/M_PI,
                 cloudKeyPoses6D->back().pitch*180/M_PI, cloudKeyPoses6D->back().yaw*180/M_PI,
                 (int)cloudInfo[0].odomAvailable);
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

    void downsampleCurrentScan(int lid)
    {
        laserCloudCornerLastDS[lid]->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast[lid]);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS[lid]);
        laserCloudCornerLastDSNum[lid] = laserCloudCornerLastDS[lid]->size();

        laserCloudSurfLastDS[lid]->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast[lid]);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS[lid]);
        laserCloudSurfLastDSNum[lid] = laserCloudSurfLastDS[lid]->size();
    }

    void updatePointAssociateToMap(int lid)
    {
        transPointAssociateToMap[lid] = trans2Affine3f(transformTobeMapped) * extAffineOf(lid).cast<float>();
    }

    // Simplified LM: plane constraints + roll/pitch/yaw priors.
    // Provides yaw estimate that LiDAR cannot observe in corridors.

    bool combineOptimization(int iterCount)
    {
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
        int totalConstraints = 0;

        Eigen::Affine3f T_cur = trans2Affine3f(transformTobeMapped);
        Eigen::Matrix3f R_cur = T_cur.rotation();
        Eigen::Vector3f t_cur = T_cur.translation();

        // GenZ-ICP adaptive weighting
        // Edge features -> non-planar (3D point-to-point, isotropic)
        // Surface + directPlane features -> planar (1D point-to-plane, accurate)
        size_t N_non_planar = laserCloudOriCornerVec[0].size();
        size_t N_planar = laserCloudOriSurfVec[0].size();
        double alpha_raw = (N_planar + N_non_planar > 0)
            ? static_cast<double>(N_planar) / static_cast<double>(N_planar + N_non_planar)
            : 0.5;
        double alpha = std::min(alpha_raw, 0.7);

        // Edge constraints (non-planar): 3D point-to-point residual
        for (size_t i = 0; i < laserCloudOriCornerVec[0].size(); i++)
        {
            Eigen::Vector3f p_lidar(laserCloudOriCornerVec[0][i].x, laserCloudOriCornerVec[0][i].y, laserCloudOriCornerVec[0][i].z);
            Eigen::Vector3f p_world = R_cur * p_lidar + t_cur;
            Eigen::Vector3f q(laserCloudOriCornerTargetVec[0][i].x,
                              laserCloudOriCornerTargetVec[0][i].y,
                              laserCloudOriCornerTargetVec[0][i].z);
            Eigen::Vector3d r_po = (p_world - q).cast<double>();
            double rn = r_po.norm();
            if (rn > 2.0) continue;

            Eigen::Matrix3d skewPw;
            skewPw << 0, -p_world.z(), p_world.y(),
                      p_world.z(), 0, -p_world.x(),
                      -p_world.y(), p_world.x(), 0;
            Eigen::Matrix<double, 3, 6> J_po;
            J_po.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
            J_po.block<3,3>(0,3) = -skewPw;

            double w;
            if (dynamicOutlierRemoval) {
                double s = rn / dynamicOutlierSigma;
                w = 1.0 / (1.0 + s * s);
                if (w < dynamicOutlierMinWeight) continue;
            } else {
                w = 1.0 / (1.0 + rn);
            }
            H += (1.0 - alpha) * w * J_po.transpose() * J_po;
            b += (1.0 - alpha) * w * J_po.transpose() * r_po;
            totalConstraints++;
        }

        // Plane constraints (planar): 1D point-to-plane residual
        for (size_t i = 0; i < laserCloudOriSurfVec[0].size(); i++)
        {
            Eigen::Vector3f p_lidar(laserCloudOriSurfVec[0][i].x, laserCloudOriSurfVec[0][i].y, laserCloudOriSurfVec[0][i].z);
            Eigen::Vector3f p_world = R_cur * p_lidar + t_cur;
            float pa = coeffSelSurfVec[0][i].x, pb = coeffSelSurfVec[0][i].y, pc = coeffSelSurfVec[0][i].z;
            float pd = coeffSelSurfVec[0][i].intensity;
            Eigen::Vector3f normal(pa, pb, pc);
            float r = normal.dot(p_world) + pd;
            if (fabs(r) > 2.0) continue;
            Eigen::Vector3f Rp = R_cur * p_lidar;
            Eigen::Matrix3f skewRp;
            skewRp << 0, -Rp.z(), Rp.y(), Rp.z(), 0, -Rp.x(), -Rp.y(), Rp.x(), 0;
            Eigen::Matrix<float, 1, 3> J_rot = normal.transpose() * (-skewRp);
            Eigen::Matrix<float, 1, 3> J_trans = normal.transpose();
            float weight;
            if (dynamicOutlierRemoval) {
                float s = fabs(r) / dynamicOutlierSigma;
                weight = 1.0f / (1.0f + s * s);
                if (weight < dynamicOutlierMinWeight) continue;
            } else {
                weight = 1.0f / (1.0f + fabs(r));
            }
            Eigen::Matrix<double, 6, 1> J_row;
            J_row << J_rot(0), J_rot(1), J_rot(2), J_trans(0), J_trans(1), J_trans(2);
            H += alpha * weight * J_row * J_row.transpose();
            b += alpha * weight * J_row * r;
            totalConstraints++;
        }

        if (totalConstraints < 50) return false;

        // Roll/pitch: gravity-aligned weak constraint.
        // Yaw: odometry absolute prior with degeneracy detection.
        // Position: no prior -- let LiDAR scan matching determine translation.
        {
            double rpW = 10.0;
            H(0,0) += rpW; b(0) += rpW * transformTobeMapped[0];
            H(1,1) += rpW; b(1) += rpW * transformTobeMapped[1];

            Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            double cond = svd.singularValues()(0) / std::max(svd.singularValues()(5), 1e-10);
            double yawW;
            if (cond > 500) yawW = 10000.0; else if (cond > 100) yawW = 5000.0;
            else if (cond > 30) yawW = 2000.0; else yawW = 1000.0;

            if (cloudInfo[0].odomAvailable)
            {
                double r_yaw = transformTobeMapped[2] - cloudInfo[0].initialGuessYaw;
                if (r_yaw > M_PI) r_yaw -= 2*M_PI; else if (r_yaw < -M_PI) r_yaw += 2*M_PI;
                H(2,2) += yawW; b(2) += yawW * r_yaw;
            }
            else if (iekf.initialized)
            {
                Eigen::Vector3d euler = IEKFStatePaper::quaternionToEuler(iekf.state.rotation);
                double r_yaw = transformTobeMapped[2] - euler.x();
                if (r_yaw > M_PI) r_yaw -= 2*M_PI; else if (r_yaw < -M_PI) r_yaw += 2*M_PI;
                H(2,2) += imuWeightAdaptive * yawW; b(2) += imuWeightAdaptive * yawW * r_yaw;
            }
        }

        // Decreasing lambda schedule
        float lambda;
        if (iterCount < 2) lambda = 1e-2;
        else if (iterCount < 4) lambda = 1e-3;
        else lambda = 1e-4;

        Eigen::Matrix<double, 6, 6> H_lm = H + lambda * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 1> dx = H_lm.completeOrthogonalDecomposition().solve(-b);
        if (!dx.array().isFinite().all()) return false;

        // Yaw clamp: prevent unwinding on outdoor data with weak yaw observability
        double maxYawCorr = (iterCount <= 1) ? 0.05 : 0.02;
        if (fabs(dx(2)) > maxYawCorr) dx(2) = (dx(2) > 0 ? 1.0 : -1.0) * maxYawCorr;

        transformTobeMapped[0] += dx(0);
        transformTobeMapped[1] += dx(1);
        transformTobeMapped[2] += dx(2);
        // Keep yaw in [-PI, PI]
        transformTobeMapped[2] = fmod(transformTobeMapped[2] + M_PI, 2*M_PI);
        if (transformTobeMapped[2] < 0) transformTobeMapped[2] += 2*M_PI;
        transformTobeMapped[2] -= M_PI;
        transformTobeMapped[3] += dx(3);
        transformTobeMapped[4] += dx(4);
        transformTobeMapped[5] += dx(5);

        return true;
    }



    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty()) return;
        if (laserCloudCornerFromMapDSNum < 10 || laserCloudSurfFromMapDSNum < 100) return;

        float backupTransform[6];
        for (int i = 0; i < 6; i++) backupTransform[i] = transformTobeMapped[i];

        static int scanDbg = 0;
        bool dbg = (++scanDbg <= 15);
        if (dbg) ROS_INFO("SCANOPT #%d init: pos=(%.2f,%.2f,%.2f) rpy=(%.2f,%.2f,%.2f) nCorner=%lu nSurf=%lu",
                 scanDbg, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                 transformTobeMapped[0]*180/M_PI, transformTobeMapped[1]*180/M_PI, transformTobeMapped[2]*180/M_PI,
                 laserCloudCornerFromMapDSNum, laserCloudSurfFromMapDSNum);

        for (int iterCount = 0; iterCount < 10; iterCount++)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
            laserCloudOriCornerVec[0].clear();
            coeffSelCornerVec[0].clear();
            laserCloudOriSurfVec[0].clear();
            coeffSelSurfVec[0].clear();
            laserCloudOriCornerTargetVec[0].clear();
            cornerOptimization(0);
            surfOptimization(0);
            directPlaneOptimization(0);

            // LM optimization with GenZ-ICP adaptive weighting
            if (combineOptimization(iterCount))
            {
                float rotChange = sqrt(pow(transformTobeMapped[0]-backupTransform[0],2)+
                                       pow(transformTobeMapped[1]-backupTransform[1],2)+
                                       pow(transformTobeMapped[2]-backupTransform[2],2));
                float transChange = sqrt(pow(transformTobeMapped[3]-backupTransform[3],2)+
                                         pow(transformTobeMapped[4]-backupTransform[4],2)+
                                         pow(transformTobeMapped[5]-backupTransform[5],2));
                if (iterCount > 1 && rotChange < 0.02 && transChange < 0.02) break;
                for (int i = 0; i < 6; i++) backupTransform[i] = transformTobeMapped[i];
            }
            else break;
        }

        // Sync IEKF state from LM-optimized result for next IMU prediction cycle
        if (iekf.initialized)
        {
            Eigen::Vector3d newPos(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
            if (prevScanTime > 0)
            {
                double dt = timeLaserInfoCur - prevScanTime;
                if (dt > 0 && dt < 1.0)
                    iekf.state.velocity = (newPos - Eigen::Vector3d(lastScanPosX, lastScanPosY, lastScanPosZ)) / dt;
            }
            Eigen::Quaterniond q(Eigen::AngleAxisd(transformTobeMapped[2], Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(transformTobeMapped[1], Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(transformTobeMapped[0], Eigen::Vector3d::UnitX()));
            iekf.state.rotation = q;
            iekf.state.position = newPos;
        }

        if (dbg) ROS_INFO("SCANOPT #%d final: pos=(%.2f,%.2f,%.2f) rpy=(%.2f,%.2f,%.2f) nCorner=%lu nSurf=%lu",
                 scanDbg, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                 transformTobeMapped[0]*180/M_PI, transformTobeMapped[1]*180/M_PI, transformTobeMapped[2]*180/M_PI,
                 laserCloudOriCornerVec[0].size(), laserCloudOriSurfVec[0].size());
    }



    // ========== 双雷达约束级融合（核心创新点1-4） ==========

    // 累积单路 LiDAR 的信息矩阵与梯度（H_i = Σ JᵀWJ, b_i = Σ JᵀWr）
    void accumulateLidarHessian(int lid)
    {
        Eigen::Matrix<double,6,6>& H = Hraw[lid];
        Eigen::Matrix<double,6,1>& b = braw[lid];
        H.setZero(); b.setZero();

        Eigen::Affine3f T_cur = trans2Affine3f(transformTobeMapped);
        Eigen::Matrix3f R_cur = T_cur.rotation();
        Eigen::Vector3f t_cur = T_cur.translation();
        Eigen::Affine3f T_ext = extAffineOf(lid).cast<float>();  // T_body_lidar

        size_t N_non_planar = laserCloudOriCornerVec[lid].size();
        size_t N_planar = laserCloudOriSurfVec[lid].size();
        double alpha_raw = (N_planar + N_non_planar > 0)
            ? static_cast<double>(N_planar) / static_cast<double>(N_planar + N_non_planar) : 0.5;
        double alpha = std::min(alpha_raw, 0.7);

        // 边缘点: 点到点残差, 雅可比 [rot | trans] = [-skew(Rp) | I]
        // 注：点到线（F-LOAM 标准）在退化隧道里反而更差（AT1 8.00→12.13m）——隧道内边线
        // 大多平行于行进方向，点到线允许沿线（=沿行进方向）自由滑动，丢失唯一的弱 x 锚定；
        // 点到点（最近邻匹配）提供 3D 各向同性约束，恰好锚住退化方向 x。故保留点到点。
        for (size_t i = 0; i < laserCloudOriCornerVec[lid].size(); i++)
        {
            Eigen::Vector3f p_lidar(laserCloudOriCornerVec[lid][i].x, laserCloudOriCornerVec[lid][i].y, laserCloudOriCornerVec[lid][i].z);
            Eigen::Vector3f p_body = T_ext * p_lidar;
            Eigen::Vector3f p_world = R_cur * p_body + t_cur;
            Eigen::Vector3f q(laserCloudOriCornerTargetVec[lid][i].x,
                              laserCloudOriCornerTargetVec[lid][i].y,
                              laserCloudOriCornerTargetVec[lid][i].z);
            Eigen::Vector3d r_po = (p_world - q).cast<double>();
            double rn = r_po.norm();
            if (rn > 2.0) continue;
            Eigen::Vector3f Rp = R_cur * p_body;
            Eigen::Matrix3f skewRp;
            skewRp << 0, -Rp.z(), Rp.y(), Rp.z(), 0, -Rp.x(), -Rp.y(), Rp.x(), 0;
            Eigen::Matrix<double,3,6> J_po;
            J_po.block<3,3>(0,0) = -skewRp.cast<double>();
            J_po.block<3,3>(0,3) = Eigen::Matrix3d::Identity();
            double w;
            if (dynamicOutlierRemoval) {
                double s = rn / dynamicOutlierSigma;
                w = 1.0 / (1.0 + s * s);                 // Cauchy 鲁棒核：大幅压制动态/外点大残差
                if (w < dynamicOutlierMinWeight) continue;
            } else {
                w = 1.0 / (1.0 + rn);
            }
            if (degeneracyAwareFeatures && degX[lid])
                w *= degAwareXGain;                      // 沿 x 退化：放大各向同性 corner 的 x 约束
            H += (1.0 - alpha) * w * J_po.transpose() * J_po;
            b += (1.0 - alpha) * w * J_po.transpose() * r_po;
        }

        // 平面点: 点到面残差, 雅可比 [rot | trans] = [-nᵀskew(Rp) | nᵀ]
        for (size_t i = 0; i < laserCloudOriSurfVec[lid].size(); i++)
        {
            Eigen::Vector3f p_lidar(laserCloudOriSurfVec[lid][i].x, laserCloudOriSurfVec[lid][i].y, laserCloudOriSurfVec[lid][i].z);
            Eigen::Vector3f p_body = T_ext * p_lidar;
            Eigen::Vector3f p_world = R_cur * p_body + t_cur;
            float pa = coeffSelSurfVec[lid][i].x, pb = coeffSelSurfVec[lid][i].y, pc = coeffSelSurfVec[lid][i].z;
            float pd = coeffSelSurfVec[lid][i].intensity;
            Eigen::Vector3f normal(pa, pb, pc);
            float r = normal.dot(p_world) + pd;
            if (fabs(r) > 2.0) continue;
            Eigen::Vector3f Rp = R_cur * p_body;
            Eigen::Matrix3f skewRp;
            skewRp << 0, -Rp.z(), Rp.y(), Rp.z(), 0, -Rp.x(), -Rp.y(), Rp.x(), 0;
            Eigen::Matrix<float,1,3> J_rot = normal.transpose() * (-skewRp);
            Eigen::Matrix<float,1,3> J_trans = normal.transpose();
            float weight;
            if (dynamicOutlierRemoval) {
                float s = fabs(r) / dynamicOutlierSigma;
                weight = 1.0f / (1.0f + s * s);        // Cauchy 鲁棒核
                if (weight < dynamicOutlierMinWeight) continue;
            } else {
                weight = 1.0f / (1.0f + fabs(r));
            }
            if (degeneracyAwareFeatures && degX[lid]) {
                float nx = normal.dot(R_cur.col(0));     // 表面法向在机体系 x（沿行进方向）分量
                weight *= 1.0f + (degAwareXGain - 1.0f) * fabs(nx);  // 横截面（x-normal）面放大
            }
            Eigen::Matrix<double,6,1> J_row;
            J_row << J_rot(0), J_rot(1), J_rot(2), J_trans(0), J_trans(1), J_trans(2);
            H += alpha * weight * J_row * J_row.transpose();
            b += alpha * weight * J_row * r;
        }
    }

    // 方向性自适应加权（创新点2/3，DCReg 改造）：Schur 补解耦旋转/平移 + 特征值钳制。
    // S_R = H_RR − H_Rt·(H_tt+εI)⁻¹·H_tR,  S_t = H_tt − H_tR·(H_RR+εI)⁻¹·H_Rt
    // 对各自 3×3 EVD，d_k = min(1, λ_k·κ_tg/λ_max)，得到退化方向权重与投影矩阵。
    void adjustLidarHessian(int lid, Eigen::Matrix<double,6,6>& Hout, Eigen::Matrix<double,6,1>& bout)
    {
        Hout = Hraw[lid];   // 原样传递，保证 LM 求解稳定
        bout = braw[lid];

        // 退化处理开关：关闭时纯 Hf+Hb 融合（等价 ε=1e-8 基线），不做方向降权。
        // adaptiveImuWeight 需要 condT/weakIdx 作退化信号，故该开关下也走 EVD 计算（但不投影）。
        if (!degeneracyEnable && !degeneracyLogOnly && !adaptiveImuWeight && !degeneracyAwareFeatures)
        {
            isDeg[lid] = false;
            dW[lid].setOnes();
            projMat[lid].setIdentity();
            weakIdx[lid] = 0;
            vWeak[lid].setZero();
            condT[lid] = 1.0;
            return;
        }

        const Eigen::Matrix<double,6,6>& H = Hraw[lid];
        Eigen::Matrix3d HRR = H.block<3,3>(0,0);
        Eigen::Matrix3d Htt = H.block<3,3>(3,3);
        Eigen::Matrix3d HRt = H.block<3,3>(0,3);
        Eigen::Matrix3d HtR = H.block<3,3>(3,0);

        const double reg = 1e-9;
        Eigen::Matrix3d SR = HRR - HRt * (Htt + reg * Eigen::Matrix3d::Identity()).inverse() * HtR;
        Eigen::Matrix3d St = Htt - HtR * (HRR + reg * Eigen::Matrix3d::Identity()).inverse() * HRt;
        SR = 0.5 * (SR + SR.transpose());   // 对称化，保证 EVD 稳定
        St = 0.5 * (St + St.transpose());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> esR(SR);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> esT(St);
        Eigen::Vector3d lamR = esR.eigenvalues();   // 升序
        Eigen::Vector3d lamT = esT.eigenvalues();

        eigVecs[lid].setZero();
        eigVecs[lid].block<3,3>(0,0) = esR.eigenvectors();
        eigVecs[lid].block<3,3>(3,3) = esT.eigenvectors();
        eigVals[lid].setZero();
        eigVals[lid].segment<3>(0) = lamR;
        eigVals[lid].segment<3>(3) = lamT;

        // 空 Hessian（该雷达本帧无有效特征）：不施加退化投影
        if (H.cwiseAbs().maxCoeff() <= 1e-12)
        {
            isDeg[lid] = false;
            dW[lid].setOnes();
            projMat[lid].setIdentity();
            weakIdx[lid] = 0;
            vWeak[lid].setZero();
            return;
        }

        double lamRmax = std::max(lamR(2), 1e-12);
        double lamTmax = std::max(lamT(2), 1e-12);
        double condR = lamRmax / std::max(lamR(0), 1e-12);
        double condT = lamTmax / std::max(lamT(0), 1e-12);
        this->condT[lid] = condT;

        // DCReg 检测：任一解耦子空间条件数超过阈值即判退化
        isDeg[lid] = (condR > degeneracyCondThresh) || (condT > degeneracyCondThresh);

        // 特征值钳制：d_k = min(1, λ_k·κ_tg/λ_max)，再施加 floor 保留弱锚点
        dW[lid].setOnes();
        for (int k = 0; k < 3; k++)
        {
            dW[lid](k)     = std::min(1.0, lamR(k) * degeneracyKappaTg / lamRmax);
            dW[lid](k + 3) = std::min(1.0, lamT(k) * degeneracyKappaTg / lamTmax);
        }
        for (int k = 0; k < 6; k++)
            if (dW[lid](k) < degeneracyFloor) dW[lid](k) = degeneracyFloor;

        Eigen::Index mi;
        dW[lid].minCoeff(&mi);
        weakIdx[lid] = (int)mi;
        vWeak[lid] = eigVecs[lid].col(mi);
        degX[lid] = (weakIdx[lid] == 3);   // 沿 x（平移 idx=3）退化

        if (degeneracyEnable && !degeneracyLogOnly)
            projMat[lid] = eigVecs[lid] * dW[lid].asDiagonal() * eigVecs[lid].transpose();
        else
            projMat[lid].setIdentity();   // 诊断/自适应模式：只记录退化信息，不投影更新量

        static int degDbg = 0;
        if (++degDbg <= 5 || degDbg % 100 == 0)
            ROS_INFO("DEGEN lid=%d condR=%.3e condT=%.3e isDeg=%d weakIdx=%d dMin=%.4f",
                     lid, condR, condT, (int)isDeg[lid], weakIdx[lid], dW[lid](weakIdx[lid]));
    }

    // 双雷达退化一致性抑制（创新点4）：同向退化时进一步削弱叠加，避免过度自信
    void consistencySuppression(Eigen::Matrix<double,6,6>& HbAdj, Eigen::Matrix<double,6,1>& bbAdj)
    {
        if (!degeneracyEnable || degeneracyLogOnly) return;
        double c = fabs(vWeak[0].dot(vWeak[1]));
        if (isDeg[0] && isDeg[1] && c > consistencyThreshold)
        {
            double s = 1.0 - c;
            int wi = weakIdx[1];
            dW[1](wi) *= s;
            if (dW[1](wi) < degeneracyFloor) dW[1](wi) = degeneracyFloor;
            projMat[1] = eigVecs[1] * dW[1].asDiagonal() * eigVecs[1].transpose();

            static int suppressDbg = 0;
            if (++suppressDbg <= 5 || suppressDbg % 100 == 0)
                ROS_INFO("DEGEN-CONSIST c=%.3f s=%.3f weakIdx_b=%d",
                         c, s, wi);
        }
    }

    // 融合两路加权信息并求解（先验 + LM）
    bool fuseAndSolve(int iterCount,
                      const Eigen::Matrix<double,6,6>& Hf, const Eigen::Matrix<double,6,1>& bf,
                      const Eigen::Matrix<double,6,6>& Hb, const Eigen::Matrix<double,6,1>& bb)
    {
        // 后雷达旋转降权：后 M1 稀疏、yaw 噪声大（condR 高达 30 vs 前雷达 ~5），
        // 将其旋转 DOF（roll/pitch/yaw = idx 0,1,2）按 backRotWeight 缩放，
        // 等效 H_b' = W·H_b·W, b_b' = W·b_b（W=diag(wr,wr,wr,1,1,1)），平移约束保留。
        Eigen::Matrix<double,6,6> HbW = Hb;
        Eigen::Matrix<double,6,1> bbW = bb;
        double wr = backRotWeight;
        if (wr < 1.0 - 1e-9)
        {
            for (int i = 0; i < 3; i++)
            {
                HbW.row(i) *= wr;
                HbW.col(i) *= wr;
                bbW(i) *= wr;
            }
        }

        Eigen::Matrix<double,6,6> H = Hf + HbW;
        Eigen::Matrix<double,6,1> b = bf + bbW;

        double rpW = 10.0;
        H(0,0) += rpW; b(0) += rpW * transformTobeMapped[0];
        H(1,1) += rpW; b(1) += rpW * transformTobeMapped[1];

        // 自适应 yaw 先验（抗摆动）：先验权重按 LIDAR 的 yaw 信息 H(2,2) 放大，
        // 使先验主导 yaw 求解，压制退化隧道里的横向蛇形摆动（yaw 振荡）。
        // 先验目标 = IEKF 陀螺积分姿态（≈ 上一帧 yaw + 本帧陀螺角增量）。
        double H_yaw_lidar = H(2,2);
        double yawW = 2000.0;
        if (H_yaw_lidar > 1e-6)
            yawW = std::max(yawW, yawPriorGain * H_yaw_lidar);

        if (cloudInfo[0].odomAvailable)
        {
            double r_yaw = transformTobeMapped[2] - cloudInfo[0].initialGuessYaw;
            if (r_yaw > M_PI) r_yaw -= 2*M_PI; else if (r_yaw < -M_PI) r_yaw += 2*M_PI;
            H(2,2) += yawW; b(2) += yawW * r_yaw;
        }
        else if (iekf.initialized)
        {
            Eigen::Vector3d euler = IEKFStatePaper::quaternionToEuler(iekf.state.rotation);
            double r_yaw = transformTobeMapped[2] - euler.x();
            if (r_yaw > M_PI) r_yaw -= 2*M_PI; else if (r_yaw < -M_PI) r_yaw += 2*M_PI;
            if (adaptiveImuWeight && iterCount == 0)
            {
                double degFlag = (condT[0] > imuAdaptDegThresh) ? 1.0 : 0.0;
                transDegBelief += imuAdaptEmaAlpha * (degFlag - transDegBelief);
                imuWeightAdaptive = (transDegBelief >= 0.5) ? imuWeight : imuWeightMin;
            }
            H(2,2) += imuWeightAdaptive * yawW; b(2) += imuWeightAdaptive * yawW * r_yaw;
        }

        static int yawDbg = 0;
        if (++yawDbg <= 5 || yawDbg % 200 == 0)
            ROS_INFO("YAW_PRIOR H_yaw=%.2e condT0=%.2e degBelief=%.2f imuW_adapt=%.3f",
                     H_yaw_lidar, condT[0], transDegBelief, imuWeightAdaptive);

        float lambda;
        if (iterCount < 2) lambda = 1e-2;
        else if (iterCount < 4) lambda = 1e-3;
        else lambda = 1e-4;

        Eigen::Matrix<double,6,6> H_lm = H + lambda * Eigen::Matrix<double,6,6>::Identity();

        Eigen::Matrix<double,6,1> dx = H_lm.completeOrthogonalDecomposition().solve(-b);
        if (!dx.array().isFinite().all()) return false;

        // 方向性自适应加权：用两路雷达的退化方向投影矩阵削弱退化方向上的更新量
        dx = projMat[1] * projMat[0] * dx;

        double maxYawCorr = (iterCount <= 1) ? 0.05 : 0.02;
        if (fabs(dx(2)) > maxYawCorr) dx(2) = (dx(2) > 0 ? 1.0 : -1.0) * maxYawCorr;

        transformTobeMapped[0] += dx(0);
        transformTobeMapped[1] += dx(1);
        transformTobeMapped[2] += dx(2);
        transformTobeMapped[2] = fmod(transformTobeMapped[2] + M_PI, 2*M_PI);
        if (transformTobeMapped[2] < 0) transformTobeMapped[2] += 2*M_PI;
        transformTobeMapped[2] -= M_PI;
        transformTobeMapped[3] += dx(3);
        transformTobeMapped[4] += dx(4);
        transformTobeMapped[5] += dx(5);

        return true;
    }

    void scan2MapOptimizationDual()
    {
        if (cloudKeyPoses3D->points.empty()) return;
        if (laserCloudCornerFromMapDSNum < 10 || laserCloudSurfFromMapDSNum < 100) return;

        float backupTransform[6];
        for (int i = 0; i < 6; i++) backupTransform[i] = transformTobeMapped[i];

        Eigen::Matrix<double,6,6> HfAdj, HbAdj;
        Eigen::Matrix<double,6,1> bfAdj, bbAdj;

        for (int iterCount = 0; iterCount < 10; iterCount++)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            int nLid = (useDualLidar && mergeClouds) ? 1 : 2;
            for (int lid = 0; lid < nLid; lid++)
            {
                laserCloudOriCornerVec[lid].clear();
                coeffSelCornerVec[lid].clear();
                laserCloudOriSurfVec[lid].clear();
                coeffSelSurfVec[lid].clear();
                laserCloudOriCornerTargetVec[lid].clear();
                cornerOptimization(lid);
                surfOptimization(lid);
                directPlaneOptimization(lid);
                accumulateLidarHessian(lid);
            }

            if (useDualLidar && mergeClouds)
            {
                Hraw[1].setZero();
                braw[1].setZero();
            }
            adjustLidarHessian(0, HfAdj, bfAdj);
            adjustLidarHessian(1, HbAdj, bbAdj);
            consistencySuppression(HbAdj, bbAdj);

            if (!fuseAndSolve(iterCount, HfAdj, bfAdj, HbAdj, bbAdj))
                break;

            float rotChange = sqrt(pow(transformTobeMapped[0]-backupTransform[0],2)+
                                   pow(transformTobeMapped[1]-backupTransform[1],2)+
                                   pow(transformTobeMapped[2]-backupTransform[2],2));
            float transChange = sqrt(pow(transformTobeMapped[3]-backupTransform[3],2)+
                                     pow(transformTobeMapped[4]-backupTransform[4],2)+
                                     pow(transformTobeMapped[5]-backupTransform[5],2));
            if (iterCount > 1 && rotChange < 0.02 && transChange < 0.02) break;
            for (int i = 0; i < 6; i++) backupTransform[i] = transformTobeMapped[i];
        }

        // Sync IEKF state (same as single mode)
        if (iekf.initialized)
        {
            Eigen::Vector3d newPos(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]);
            if (prevScanTime > 0)
            {
                double dt = timeLaserInfoCur - prevScanTime;
                if (dt > 0 && dt < 1.0)
                    iekf.state.velocity = (newPos - Eigen::Vector3d(lastScanPosX, lastScanPosY, lastScanPosZ)) / dt;
            }
            Eigen::Quaterniond q(Eigen::AngleAxisd(transformTobeMapped[2], Eigen::Vector3d::UnitZ()) *
                                 Eigen::AngleAxisd(transformTobeMapped[1], Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(transformTobeMapped[0], Eigen::Vector3d::UnitX()));
            iekf.state.rotation = q;
            iekf.state.position = newPos;
        }
    }

    void cornerOptimization(int lid)
    {
        updatePointAssociateToMap(lid);
        for (int i = 0; i < laserCloudCornerLastDSNum[lid]; i++)
        {
            PointType pointOri = laserCloudCornerLastDS[lid]->points[i];
            PointType pointSel;
            pointAssociateToMap(&pointOri, &pointSel, lid);
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
                    if (s > 0.1)
                    {
                        // V18: Store unscaled unit normal (la,lb,lc) and line offset constant d0
                        // Line equation: la*x + lb*y + lc*z = d0 (for points on the line)
                        // Distance from any point p: |la*p.x + lb*p.y + lc*p.z - d0|
                        // At pointSel: distance = la*pointSel.x + lb*pointSel.y + lc*pointSel.z - d0 = ld2
                        // So: d0 = la*pointSel.x + lb*pointSel.y + lc*pointSel.z - ld2
                        float d0 = la*pointSel.x + lb*pointSel.y + lc*pointSel.z - ld2;
                        laserCloudOriCornerVec[lid].push_back(pointOri);
                        PointType coeff_pt;
                        coeff_pt.x = la; coeff_pt.y = lb; coeff_pt.z = lc;
                        coeff_pt.intensity = d0;
                        coeffSelCornerVec[lid].push_back(coeff_pt);
                        // GenZ-ICP: store closest neighbor as target for point-to-point residual
                        pcl::PointXYZI targetPt;
                        targetPt.x = laserCloudCornerFromMapDS->points[pointSearchInd[0]].x;
                        targetPt.y = laserCloudCornerFromMapDS->points[pointSearchInd[0]].y;
                        targetPt.z = laserCloudCornerFromMapDS->points[pointSearchInd[0]].z;
                        laserCloudOriCornerTargetVec[lid].push_back(targetPt);
                    }
                }
            }
        }
    }

    void surfOptimization(int lid)
    {
        updatePointAssociateToMap(lid);
        for (int i = 0; i < laserCloudSurfLastDSNum[lid]; i++)
        {
            PointType pointOri = laserCloudSurfLastDS[lid]->points[i];
            PointType pointSel;
            pointAssociateToMap(&pointOri, &pointSel, lid);
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
                    // 点到平面距离: pd2 = pa*pointSel.x + pb*pointSel.y + pc*pointSel.z + pd
                    // Plane equation: pa*x + pb*y + pc*z + pd = 0
                    // Store unscaled normal and plane constant pd
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x*pointSel.x + pointSel.y*pointSel.y + pointSel.z*pointSel.z));
                    if (s > 0.1)
                    {
                        laserCloudOriSurfVec[lid].push_back(pointOri);
                        PointType coeff_pt;
                        coeff_pt.x = pa; coeff_pt.y = pb; coeff_pt.z = pc;
                        coeff_pt.intensity = pd;
                        coeffSelSurfVec[lid].push_back(coeff_pt);
                    }
                }
            }
        }
    }

    /**
     * Direct plane registration (FAST-LIO2 style): use all downsampled points
     * for point-to-plane constraints, not just curvature-selected features.
     * This provides 3-5x more constraints in corridor environments where
     * walls are large flat surfaces with low curvature.
     */
    void directPlaneOptimization(int lid)
    {
        updatePointAssociateToMap(lid);

        // Downsample the full extracted cloud for efficiency
        pcl::PointCloud<PointType>::Ptr scanDS(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> ds;
        ds.setLeafSize(0.8, 0.8, 0.8);
        ds.setInputCloud(extractedCloud[lid]);
        ds.filter(*scanDS);

        for (auto& pointOri : scanDS->points)
        {
            PointType pointSel;
            pointAssociateToMap(&pointOri, &pointSel, lid);

            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[4] > 1.0) continue;

            // Fit plane to the 5 neighbors via PCA
            float cx = 0, cy = 0, cz = 0;
            for (int j = 0; j < 5; j++)
            {
                cx += laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                cy += laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                cz += laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
            }
            cx /= 5; cy /= 5; cz /= 5;

            float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
            for (int j = 0; j < 5; j++)
            {
                float ax = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x - cx;
                float ay = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y - cy;
                float az = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z - cz;
                a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                a22 += ay * ay; a23 += ay * az; a33 += az * az;
            }
            a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

            // Eigen decomposition of covariance matrix
            cv::Mat matA(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
            matA.at<float>(0,0) = a11; matA.at<float>(0,1) = a12; matA.at<float>(0,2) = a13;
            matA.at<float>(1,0) = a12; matA.at<float>(1,1) = a22; matA.at<float>(1,2) = a23;
            matA.at<float>(2,0) = a13; matA.at<float>(2,1) = a23; matA.at<float>(2,2) = a33;
            cv::eigen(matA, matD, matV);

            // Plane validity: smallest eigenvalue << second smallest
            // (planar: one small eigenvalue, two large)
            float lambda_min = matD.at<float>(0,0);
            float lambda_mid = matD.at<float>(0,1);

            if (lambda_mid < 1e-6 || lambda_min / lambda_mid > 0.05) continue;

            // Plane normal is the eigenvector of the smallest eigenvalue
            float pa = matV.at<float>(0,0);
            float pb = matV.at<float>(1,0);
            float pc = matV.at<float>(2,0);
            float ps = sqrt(pa*pa + pb*pb + pc*pc);
            pa /= ps; pb /= ps; pc /= ps;

            // Plane equation: pa*x + pb*y + pc*z + pd = 0
            float pd = -(pa * cx + pb * cy + pc * cz);

            // Validate: all 5 neighbors within 0.2m of plane
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
            if (!planeValid) continue;

            // Point-to-plane distance
            float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
            float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x*pointSel.x + pointSel.y*pointSel.y + pointSel.z*pointSel.z));
            if (s <= 0.1) continue;

            laserCloudOriSurfVec[lid].push_back(pointOri);
            PointType coeff_pt;
            coeff_pt.x = pa; coeff_pt.y = pb; coeff_pt.z = pc;
            coeff_pt.intensity = pd;
            coeffSelSurfVec[lid].push_back(coeff_pt);
        }

        static int directDbg = 0;
        if (++directDbg <= 5 || directDbg % 100 == 0)
            ROS_INFO("DIRECTPLANE: scanDS=%zu totalSurf=%zu", scanDS->size(), laserCloudOriSurfVec[lid].size());
    }

    // ======================================================
    // Part6: 关键帧与因子图
    // ======================================================

    void saveKeyFramesAndFactor()
    {
        // Process pending loop closures FIRST — must happen even when no new
        // keyframe is added, otherwise loop constraints accumulate in the queue
        // and are never fed into the factor graph after the robot stops moving.
        bool hasLoop = false;
        if (!loopIndexQueue.empty())
        {
            for (size_t i = 0; i < loopIndexQueue.size(); ++i)
            {
                gtSAMgraph.add(BetweenFactor<Pose3>(loopIndexQueue[i].second, loopIndexQueue[i].first,
                                                     loopPoseQueue[i], loopNoiseQueue[i]));
            }
            size_t nLoops = loopIndexQueue.size();
            loopIndexQueue.clear();
            loopPoseQueue.clear();
            loopNoiseQueue.clear();
            hasLoop = true;
            ROS_INFO("FGRAPH: added %zu loop constraints to factor graph", nLoops);
        }

        // Keyframe selection: only add when sufficient movement
        if (cloudKeyPoses3D->points.size() > 0)
        {
            auto& lastPose = cloudKeyPoses6D->points.back();
            float deltaDist = sqrt(
                pow(transformTobeMapped[3] - lastPose.x, 2) +
                pow(transformTobeMapped[4] - lastPose.y, 2) +
                pow(transformTobeMapped[5] - lastPose.z, 2));
            float deltaAngle = fabs(transformTobeMapped[2] - lastPose.yaw);
            if (deltaAngle > M_PI) deltaAngle = 2 * M_PI - deltaAngle;
            static int kfSkipCount = 0;
            if (deltaDist < surroundingkeyframeAddingDistThreshold &&
                deltaAngle < surroundingkeyframeAddingAngleThreshold &&
                !hasLoop)  // force keyframe when loop closure is pending
            {
                if (++kfSkipCount % 100 == 0)
                    ROS_INFO("KFDBG: skip #%d dDist=%.2f dAng=%.2f curPos=(%.1f,%.1f,%.1f) lastKf=(%.1f,%.1f,%.1f)",
                             kfSkipCount, deltaDist, deltaAngle,
                             transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                             lastPose.x, lastPose.y, lastPose.z);
                return;
            }
        }

        // 保存当前位姿到关键帧列表
        PointType thisPose3D;
        PointTypePose thisPose6D;

        // Use transformTobeMapped directly — it comes from IEKF (sole frontend estimator).
        float kfX = transformTobeMapped[3];
        float kfY = transformTobeMapped[4];
        float kfZ = transformTobeMapped[5];
        float kfRoll = transformTobeMapped[0];
        float kfPitch = transformTobeMapped[1];
        float kfYaw = transformTobeMapped[2];
        gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(kfYaw, kfPitch, kfRoll),
                                  gtsam::Point3(kfX, kfY, kfZ));

        thisPose3D.x = kfX;
        thisPose3D.y = kfY;
        thisPose3D.z = kfZ;
        thisPose3D.intensity = cloudKeyPoses3D->size();
        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.roll  = transformTobeMapped[0];
        thisPose6D.pitch = transformTobeMapped[1];
        thisPose6D.yaw   = transformTobeMapped[2];
        thisPose6D.time = timeLaserInfoCur;
        thisPose6D.intensity = thisPose3D.intensity;

        cloudKeyPoses3D->push_back(thisPose3D);
        cloudKeyPoses6D->push_back(thisPose6D);
        static int kfCount = 0;
        if (++kfCount % 50 == 0)
            ROS_INFO("DEBUG: keyframe #%d at (%.1f, %.1f, %.1f)", kfCount,
                     thisPose3D.x, thisPose3D.y, thisPose3D.z);

        // 保存当前帧特征点云（双雷达模式：两路分别经外参变换到机体系后拼接）
        pcl::PointCloud<PointType>::Ptr cornerKF(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfKF(new pcl::PointCloud<PointType>());
        if (useDualLidar)
        {
            for (int lid = 0; lid < 2; ++lid)
            {
                Eigen::Affine3f ext = extAffineOf(lid).cast<float>();
                for (const auto& p : laserCloudCornerLast[lid]->points)
                {
                    Eigen::Vector3f v = ext * Eigen::Vector3f(p.x, p.y, p.z);
                    PointType np = p; np.x = v.x(); np.y = v.y(); np.z = v.z();
                    cornerKF->push_back(np);
                }
                for (const auto& p : laserCloudSurfLast[lid]->points)
                {
                    Eigen::Vector3f v = ext * Eigen::Vector3f(p.x, p.y, p.z);
                    PointType np = p; np.x = v.x(); np.y = v.y(); np.z = v.z();
                    surfKF->push_back(np);
                }
            }
        }
        else
        {
            *cornerKF = *laserCloudCornerLast[0];
            *surfKF = *laserCloudSurfLast[0];
        }
        cornerCloudKeyFrames.push_back(cornerKF);
        surfCloudKeyFrames.push_back(surfKF);

        // 保存当前帧稠密点云（去畸变后全量点，经外参变换到机体系），用于生成稠密地图
        pcl::PointCloud<PointType>::Ptr denseKF(new pcl::PointCloud<PointType>());
        if (useDualLidar)
        {
            for (int lid = 0; lid < 2; ++lid)
            {
                Eigen::Affine3f ext = extAffineOf(lid).cast<float>();
                for (const auto& p : laserCloudDenseLast[lid]->points)
                {
                    Eigen::Vector3f v = ext * Eigen::Vector3f(p.x, p.y, p.z);
                    PointType np = p; np.x = v.x(); np.y = v.y(); np.z = v.z();
                    denseKF->push_back(np);
                }
            }
        }
        else
        {
            *denseKF = *laserCloudDenseLast[0];
        }
        pcl::PointCloud<PointType>::Ptr denseKFDS(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> dsDense;
        dsDense.setLeafSize(0.2f, 0.2f, 0.2f);
        dsDense.setInputCloud(denseKF);
        dsDense.filter(*denseKFDS);
        denseCloudKeyFrames.push_back(denseKFDS);

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
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].yaw,
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].pitch,
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].roll),
                gtsam::Point3(
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].x,
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].y,
                cloudKeyPoses6D->points[cloudKeyPoses6D->size()-2].z));

            // Strong odometry factors: odometry is ground truth in simulation.
            // Translation and orientation are equally reliable.
            noiseModel::Diagonal::shared_ptr odomNoise = noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8).finished());
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-2, cloudKeyPoses3D->size()-1,
                                                poseFromPrior.between(poseFrom), odomNoise));
            initialEstimate.insert(cloudKeyPoses3D->size()-1, poseFrom);
        }

        // Flag for correctPoses() to apply the ISAM2 correction
        aLoopIsClosed = hasLoop;

        // 执行ISAM2优化 (handle degenerate cases in corridors)
        try {
            isam->update(gtSAMgraph, initialEstimate);
            isam->update();

            if (hasLoop)
            {
                isam->update();
                isam->update();
                isam->update();
                isam->update();
                isam->update();
            }
        } catch (const std::exception& e) {
            ROS_WARN("GTSAM optimization failed: %s. Skipping this update.", e.what());
            gtSAMgraph.resize(0);
            initialEstimate.clear();
            return;
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

        updatePointAssociateToMap(0);
        if (useDualLidar) updatePointAssociateToMap(1);
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

        if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
        {
            static int lcSmallDbg = 0;
            if (++lcSmallDbg % 50 == 0)
                ROS_INFO("LCPAIR: cloud too small cur=%zu prev=%zu", cureKeyframeCloud->size(), prevKeyframeCloud->size());
            return;
        }

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
            ROS_INFO("LCPAIR: VGICP failed conv=%d dist=%.2f cur=%d pre=%d, trying ICP",
                     (int)vgicp_result.converged, dist, loopKeyCur, loopKeyPre);
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
            if (!icp.hasConverged() || icp.getFitnessScore() > historyKeyframeFitnessScore)
            {
                ROS_INFO("LCPAIR: ICP fallback failed conv=%d fitScore=%.3f cur=%d pre=%d",
                         (int)icp.hasConverged(), icp.getFitnessScore(), loopKeyCur, loopKeyPre);
                return;
            }
            ROS_INFO("LCPAIR: ICP fallback SUCCESS fitScore=%.3f cur=%d pre=%d",
                     icp.getFitnessScore(), loopKeyCur, loopKeyPre);

            // 构建回环约束
            Eigen::Affine3f correctionFrame(icp.getFinalTransformation());
            Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
            Eigen::Affine3f tCorrect = correctionFrame * tWrong;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);

            gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(yaw, pitch, roll), gtsam::Point3(x, y, z));
            gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
            // Fixed loop closure noise: ~20cm position, ~10deg rotation.
            // Using ICP/VGICP fitness_score directly is unreliable (VGICP returns
            // huge values ~1.5-70, making constraints 3000x weaker than odometry).
            gtsam::Vector Vector6(6);
            Vector6 << 0.04, 0.04, 0.04, 0.01, 0.01, 0.01;
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
            if (dist > vgicp.distance_threshold || dist < 0.1)
            {
                ROS_INFO("LCPAIR: VGICP OK but dist check failed dist=%.2f cur=%d pre=%d",
                         dist, loopKeyCur, loopKeyPre);
                return; // 论文公式(28)验证
            }
            ROS_INFO("LCPAIR: VGICP SUCCESS dist=%.2f fitScore=%.4f cur=%d pre=%d",
                     dist, vgicp_result.fitness_score, loopKeyCur, loopKeyPre);

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

            gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(yaw, pitch, roll), gtsam::Point3(x, y, z));
            gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
            gtsam::Vector Vector6(6);
            Vector6 << 0.04, 0.04, 0.04, 0.01, 0.01, 0.01;
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

        // ---- End-to-start closure: when near origin, match with start keyframes ----
        float curX = copy_cloudKeyPoses3D->back().x;
        float curY = copy_cloudKeyPoses3D->back().y;
        float distToOrigin = sqrt(curX * curX + curY * curY);

        if (distToOrigin > maxDistFromOrigin_) maxDistFromOrigin_ = distToOrigin;

        if (distToOrigin < 30.0 && maxDistFromOrigin_ > 80.0) {
            // Search for keyframes within 20m of the origin
            PointType originPt;
            originPt.x = 0; originPt.y = 0; originPt.z = 0;
            std::vector<int> originSearch;
            std::vector<float> originSqDist;
            kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
            kdtreeHistoryKeyPoses->radiusSearch(originPt, 30.0, originSearch, originSqDist, 0);

            // Find the earliest keyframe among origin-area matches
            int bestId = -1;
            float bestTime = std::numeric_limits<float>::max();
            for (size_t i = 0; i < originSearch.size(); ++i) {
                int id = originSearch[i];
                float t = copy_cloudKeyPoses6D->points[id].time;
                if (t < bestTime) { bestTime = t; bestId = id; }
            }

            if (bestId >= 0 && bestId != loopKeyCur &&
                abs(bestTime - timeLaserInfoCur) > 30.0) {
                loopKeyPre = bestId;
                ROS_INFO("LC_ORIGIN: cur=%d pos=(%.1f,%.1f) matched start=%d (dist=%.1fm from origin)",
                         loopKeyCur, curX, curY, bestId, distToOrigin);
            } else {
                ROS_INFO("LC_ORIGIN_SKIP: nFound=%zu bestId=%d bestTime=%.1f curTime=%.1f dt=%.1f maxDist=%.1f",
                         originSearch.size(), bestId, bestTime, timeLaserInfoCur,
                         abs(bestTime - timeLaserInfoCur), maxDistFromOrigin_);
            }
        }

        // ---- Local closure: match with keyframes outside time window ----
        if (loopKeyPre == -1) {
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
        }

        static int lcDbgCount = 0;
        if (++lcDbgCount % 20 == 0 || loopKeyPre >= 0)
            ROS_INFO("LCDETECT #%d: latest=%d pos=(%.1f,%.1f) distOrig=%.1f found=%d",
                     lcDbgCount, loopKeyCur, curX, curY, distToOrigin, loopKeyPre);

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
        // Publish pose from last keyframe (already corrected by GTSAM).
        // Keyframes now come from IEKF estimation, not odometry.
        float pubX, pubY, pubZ, pubRoll, pubPitch, pubYaw;
        if (!cloudKeyPoses6D->points.empty())
        {
            auto& lastKf = cloudKeyPoses6D->points.back();
            pubX = lastKf.x; pubY = lastKf.y; pubZ = lastKf.z;
            pubRoll = lastKf.roll; pubPitch = lastKf.pitch; pubYaw = lastKf.yaw;
        }
        else
        {
            pubX = transformTobeMapped[3]; pubY = transformTobeMapped[4]; pubZ = transformTobeMapped[5];
            pubRoll = transformTobeMapped[0]; pubPitch = transformTobeMapped[1]; pubYaw = transformTobeMapped[2];
        }
        laserOdometry.pose.pose.position.x = pubX;
        laserOdometry.pose.pose.position.y = pubY;
        laserOdometry.pose.pose.position.z = pubZ;
        tf::Quaternion q = tf::createQuaternionFromRPY(pubRoll, pubPitch, pubYaw);
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
        if (useDualLidar)
        {
            pcl::PointCloud<PointType>::Ptr cornerBody(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr surfBody(new pcl::PointCloud<PointType>());
            for (int lid = 0; lid < 2; ++lid)
            {
                Eigen::Affine3f ext = extAffineOf(lid).cast<float>();
                for (const auto& p : laserCloudCornerLast[lid]->points)
                {
                    Eigen::Vector3f v = ext * Eigen::Vector3f(p.x, p.y, p.z);
                    PointType np = p; np.x = v.x(); np.y = v.y(); np.z = v.z();
                    cornerBody->push_back(np);
                }
                for (const auto& p : laserCloudSurfLast[lid]->points)
                {
                    Eigen::Vector3f v = ext * Eigen::Vector3f(p.x, p.y, p.z);
                    PointType np = p; np.x = v.x(); np.y = v.y(); np.z = v.z();
                    surfBody->push_back(np);
                }
            }
            *cloudOut += *transformPointCloud(cornerBody, &thisPose6D);
            *cloudOut += *transformPointCloud(surfBody, &thisPose6D);
        }
        else
        {
            *cloudOut += *transformPointCloud(laserCloudCornerLast[0], &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLast[0], &thisPose6D);
        }
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

    void pointAssociateToMap(PointType const * const pi, PointType * const po, int lid)
    {
        const Eigen::Affine3f& T = transPointAssociateToMap[lid];
        po->x = T(0,0) * pi->x + T(0,1) * pi->y + T(0,2) * pi->z + T(0,3);
        po->y = T(1,0) * pi->x + T(1,1) * pi->y + T(1,2) * pi->z + T(1,3);
        po->z = T(2,0) * pi->x + T(2,1) * pi->y + T(2,2) * pi->z + T(2,3);
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
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.pitch), double(thisPoint.roll)),
                            gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[2], transformIn[1], transformIn[0]),
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

        // 稠密地图：全量去畸变点经关键帧位姿变换到地图系，体素滤波去重
        pcl::PointCloud<PointType>::Ptr globalDenseCloud(new pcl::PointCloud<PointType>());
        for (size_t i = 0; i < denseCloudKeyFrames.size(); i++)
            *globalDenseCloud += *transformPointCloud(denseCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        pcl::PointCloud<PointType>::Ptr globalDenseCloudDS(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> dsGlobalDense;
        dsGlobalDense.setLeafSize(0.2f, 0.2f, 0.2f);
        dsGlobalDense.setInputCloud(globalDenseCloud);
        dsGlobalDense.filter(*globalDenseCloudDS);
        int retDense = pcl::io::savePCDFileBinary(saveMapDirectory + "/DenseMap.pcd", *globalDenseCloudDS);

        res.success = (ret == 0) && (retDense == 0);
        cout << "\nSaving map completed (" << globalDenseCloudDS->size() << " dense pts).\n" << endl;
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

    ROS_INFO("\033[1;32m----> OURS SLAM: IEKF + VGICP Loop Detection Ready.\033[0m");

    ros::spin();

    loopthread.join();
    vizThread.join();
    return 0;
}
