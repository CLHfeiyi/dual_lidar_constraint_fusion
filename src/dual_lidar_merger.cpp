#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>

class DualLidarMerger
{
public:
    DualLidarMerger()
    {
        ros::NodeHandle nh("~");

        nh.param<std::string>("topic_front", topicFront_, "/ouster/points");
        nh.param<std::string>("topic_back", topicBack_, "/rslidar_points");
        nh.param<std::string>("output_topic", outputTopic_, "/points_raw");
        nh.param<double>("max_dt", maxDt_, 0.1);

        pub_ = nh.advertise<sensor_msgs::PointCloud2>(outputTopic_, 10);
        subFront_ = nh.subscribe(topicFront_, 10, &DualLidarMerger::frontCB, this);
        subBack_  = nh.subscribe(topicBack_,  10, &DualLidarMerger::backCB,  this);

        ROS_INFO("Dual-LiDAR Merger: %s + %s -> %s (max_dt=%.3f)",
                 topicFront_.c_str(), topicBack_.c_str(),
                 outputTopic_.c_str(), maxDt_);
    }

private:
    void frontCB(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        frontCloud_ = *msg;
        tryMerge();
    }

    void backCB(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        backCloud_ = *msg;
        tryMerge();
    }

    void tryMerge()
    {
        if (frontCloud_.data.empty() || backCloud_.data.empty())
            return;

        double dt = std::abs(
            (frontCloud_.header.stamp - backCloud_.header.stamp).toSec());
        if (dt > maxDt_)
            return;

        pcl::PointCloud<pcl::PointXYZI> ca, cb, merged;
        pcl::fromROSMsg(frontCloud_, ca);
        pcl::fromROSMsg(backCloud_, cb);

        merged = ca;
        merged += cb;

        sensor_msgs::PointCloud2 out;
        pcl::toROSMsg(merged, out);
        out.header.stamp = frontCloud_.header.stamp;
        out.header.frame_id = frontCloud_.header.frame_id;
        pub_.publish(out);
    }

    ros::Publisher pub_;
    ros::Subscriber subFront_, subBack_;
    sensor_msgs::PointCloud2 frontCloud_, backCloud_;
    std::mutex mutex_;
    std::string topicFront_, topicBack_, outputTopic_;
    double maxDt_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dual_lidar_merger");
    DualLidarMerger merger;
    ros::spin();
    return 0;
}
