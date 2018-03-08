#include <ros/ros.h>
#include <ewok/ed_nor_ring_buffer.h>


static const int POW = 6;
static const int N = (1 << POW);

int main(int argc, char** argv) {

    ros::init(argc, argv, "rolling_buffer_test");
    ros::NodeHandle nh;

    ros::Publisher occ_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 1, true);
    ros::Publisher free_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 1, true);
    ros::Publisher updated_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/updated", 1, true);
    ros::Publisher dist_marker_pub = nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 1, true);

    //ewok::QuasiEuclideanDistanceRingBuffer<POW> rrb(0.1, 1.0);
    //ewok::QuasiEuclideanDistanceRingBuffer<POW>::PointCloud cloud;

    ewok::EuclideanDistanceNormalRingBuffer<POW> rrb(0.1, 1.0);
    ewok::EuclideanDistanceNormalRingBuffer<POW>::PointCloud cloud;

    cloud.push_back(Eigen::Vector4f(-1, 1, 2, 0));
    cloud.push_back(Eigen::Vector4f(1, 1, 2, 0));
    cloud.push_back(Eigen::Vector4f(-1, -1, 2, 0));
    cloud.push_back(Eigen::Vector4f(1, -1, 2, 0));
    cloud.push_back(Eigen::Vector4f(0, 0, 5, 0));

    cloud.push_back(Eigen::Vector4f(0, 3, 5, 0));
    cloud.push_back(Eigen::Vector4f(3, 0, 5, 0));
    cloud.push_back(Eigen::Vector4f(0, 3, -5, 0));
    cloud.push_back(Eigen::Vector4f(3, 0, -5, 0));


    rrb.insertPointCloud(cloud, Eigen::Vector3f(0,0,0));
    rrb.updateDistance();

    rrb.insertPointCloud(cloud, Eigen::Vector3f(0,0,0));
    rrb.updateDistance();

    rrb.insertPointCloud(cloud, Eigen::Vector3f(0,0,0));
    rrb.updateDistance();

    ros::Rate r(1);
    while (ros::ok())
    {
        r.sleep();

        visualization_msgs::Marker m_occ, m_free, m_dist, m_updated;
        rrb.getMarkerOccupied(m_occ);
        rrb.getMarkerFree(m_free);
        rrb.getMarkerUpdated(m_updated);
        rrb.getMarkerDistance(m_dist, 0.9);

        occ_marker_pub.publish(m_occ);
        free_marker_pub.publish(m_free);
        updated_marker_pub.publish(m_updated);
        dist_marker_pub.publish(m_dist);

        // rrb.moveVolume(Eigen::Vector3i(1,0,0));

        ros::spinOnce();
    }

    return 0;
}