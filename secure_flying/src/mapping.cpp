#include <ewok/ed_nor_ring_buffer.h>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace message_filters;
using namespace std;

// global declaration
ros::Time _last_time;

bool initialized = false;
const double resolution = 0.1;
static const int POW = 7;
static const int N = (1 << POW);
ewok::EuclideanDistanceNormalRingBuffer<POW>::Ptr rrb;

ros::Publisher occ_marker_pub, free_marker_pub, dist_marker_pub, norm_marker_pub;
ros::Publisher cloud2_pub, center_pub, traj_pub;

// this callback use input cloud to update ring buffer, and update odometry of UAV
void odomCloudCallback(const nav_msgs::OdometryConstPtr& odom, const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    double elp = ros::Time::now().toSec() - _last_time.toSec();

    tf::Quaternion q1(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                      odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    tf::Matrix3x3 m(q1);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // ROS_INFO("Updating ringbuffer map");
    // update ring buffer
    // tranform from optical frame to uav frame
    Eigen::Matrix4f t_c_b = Eigen::Matrix4f::Zero();
    t_c_b(0, 2) = 1;
    t_c_b(1, 0) = -1;
    t_c_b(2, 1) = -1;
    t_c_b(3, 3) = 1;

    // transform from uav to world
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
    // std::cout << transform.matrix() << "\n\n";

    // convert cloud to pcl form
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud, *cloud_in);

    // transform to world frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>());
    //pcl::transformPointCloud(*cloud_in, *cloud_1, t_c_b);
    //pcl::transformPointCloud(*cloud_1, *cloud_2, transform);
    pcl::transformPointCloud(*cloud_in, *cloud_2, transform);

    // down-sample
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud_2);
    float res = 0.1f;
    sor.setLeafSize(res, res, res);
    sor.filter(*cloud_filtered);

    // compute ewol pointcloud and origin
    Eigen::Vector3f origin = (transform * Eigen::Vector4f(0, 0, 0, 1)).head<3>();
    ewok::EuclideanDistanceNormalRingBuffer<POW>::PointCloud cloud_ew;
    std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> > points =
        cloud_filtered->points;
    for(int i = 0; i < points.size(); ++i)
    {
        cloud_ew.push_back(Eigen::Vector4f(points.at(i).x, points.at(i).y, points.at(i).z, 0));
    }

    // initialize the ringbuffer map
    if(!initialized)
    {
        Eigen::Vector3i idx;
        rrb->getIdx(origin, idx);
        ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());
        rrb->setOffset(idx);
        initialized = true;
    }
    else
    {
        // move buffer when its center is not the same as UAV
        while(true)
        {
            Eigen::Vector3i origin_idx, offset, diff;
            rrb->getIdx(origin, origin_idx);
            offset = rrb->getVolumeCenter();
            std::cout << "origin :" << origin_idx << " center:" << offset << std::endl;
            diff = origin_idx - offset;
            if(diff.array().any())
                rrb->moveVolume(diff.head<3>());
            else
                break;
        }
    }

    // insert point cloud to ringbuffer
    double t1 = ros::Time::now().toSec();
    rrb->insertPointCloud(cloud_ew, origin);
    rrb->updateDistance();
    double t2 = ros::Time::now().toSec();
    ROS_INFO("Updating ringbuffer time: %lf ms", 1000 * (t2 - t1));

    _last_time = ros::Time::now();
}

void timerCallback(const ros::TimerEvent& e)
{
    if(!initialized) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Vector3d center;
    rrb->getBufferAsCloud(cloud, center);

    // convert to ROS message and publish
    sensor_msgs::PointCloud2 cloud2;
    pcl::toROSMsg(cloud, cloud2);

    // message publish should have the same time stamp
    cloud2.header.stamp = ros::Time::now();
    cloud2.header.frame_id = "world";
    cloud2_pub.publish(cloud2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sim_example");
    ros::NodeHandle nh;

    // ringbuffer cloud2
    cloud2_pub = nh.advertise<sensor_msgs::PointCloud2>("ring_buffer/cloud2", 1, true);

    // synchronized subscriber for pointcloud and odometry
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/zed/odom", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh, "/zed/point_cloud/cloud_registered", 1);
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, pcl_sub);
    sync.registerCallback(boost::bind(&odomCloudCallback, _1, _2));

    // timer for publish ringbuffer as pointcloud
    ros::Timer timer1 = nh.createTimer(ros::Duration(1.0), timerCallback);

    ros::Duration(0.5).sleep();

    // setup ring buffer
    rrb = ewok::EuclideanDistanceNormalRingBuffer<POW>::Ptr(
        new ewok::EuclideanDistanceNormalRingBuffer<POW>(resolution, 1.0));

    _last_time = ros::Time::now();
    std::cout << "Start mapping!" << std::endl;

    ros::spin();

    return 0;
}
