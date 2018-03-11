#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace message_filters;

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;

ros::Publisher cloud_pub;

void odomCloudCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    // get orientation and translation
    Eigen::Quaternionf q;
    q.w() = odom->pose.pose.orientation.w;
    q.x() = odom->pose.pose.orientation.x;
    q.y() = odom->pose.pose.orientation.y;
    q.z() = odom->pose.pose.orientation.z;

    // create transform matrix
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block(0, 0, 3, 3) = Eigen::Matrix3f(q);
    transform(0, 3) = odom->pose.pose.position.x;
    transform(1, 3) = odom->pose.pose.position.y;
    transform(2, 3) = odom->pose.pose.position.z;
    std::cout << transform.matrix() << "\n\n";

    // convert cloud to pcl form
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud, *cloud_in);

    // transform to world frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(*cloud_in, *cloud_out, transform.inverse());

    // add to global cloud and down-sample
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_out);
    float res = 0.1f;
    sor.setLeafSize(res, res, res);
    sor.filter(*cloud_filtered);

    // *_cloud += *cloud_filtered;

    // publish transformed point cloud
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud_filtered, cloud_msg);
    cloud_msg.header.frame_id = "world";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_pub.publish(cloud_msg);
    cloud_pub.publish(cloud_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_mapping");
    ros::NodeHandle nh;

    // define synchronizer
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/zed/odom", 30);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/zed/point_cloud/cloud_registered", 30);
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, pcl_sub);
    sync.registerCallback(boost::bind(&odomCloudCallback, _1, _2));

    // to publish transformed cloud
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);

    // initialize point cloud
    // _cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    // _cloud->height = 1;

    // start loop
    std::cout << "Start mapping!" << std::endl;
    ros::spin();

    return 0;
}