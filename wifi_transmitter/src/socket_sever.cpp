#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include "ros/ros.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#define PORT 10001
#define BYTE_SEND_INTERVAL 1

using namespace std;
int sock_sev;
struct sockaddr_in s_in;  // address structure
struct sockaddr_in c_in;
socklen_t in_len;

// global mat for cloud
pcl::PointCloud<pcl::PointXYZ> cloud;
bool cloud_ready = false;
int point_num = 0;
int cur_num = 0;

int send_data(double *data, long int size)  // 255k at most
{
    /*package number and last package size*/
    int pkg_num = size / 1024;
    int last_pkg_bytes = size % 1024;
    if(last_pkg_bytes > 0)
        pkg_num += 1;
    else
        last_pkg_bytes = 1024;

    double buffer[1024];
    for(int i = 0; i < 1024; i++) buffer[i] = 0;

    /*send mark*/
    buffer[0] = 1;
    buffer[1] = 1;
    buffer[2] = 1;
    buffer[3] = 1;
    buffer[4] = point_num;

    buffer[5] = (double)pkg_num;  // 255k at most
    buffer[6] = (double)(last_pkg_bytes / 256);
    buffer[7] = (double)(last_pkg_bytes % 256);

    sendto(sock_sev, buffer, 1024, 0, (struct sockaddr *)&c_in, sizeof(struct sockaddr));

    /*send data*/
    for(int j = 0; j < pkg_num; j++)
    {
        int pkg_size = 1024;
        if(j == pkg_num - 1) pkg_size = last_pkg_bytes;

        for(int i = 0; i < pkg_size; i++) buffer[i] = *(data + 1024 * j + i);

        sendto(sock_sev, buffer, 1024, 0, (struct sockaddr *)&c_in, sizeof(struct sockaddr));
    }

    return 1;
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    cout << "new cloud " << endl;
    if(cloud_ready == true) return;

    cloud.points.clear();
    pcl::fromROSMsg(*msg, cloud);
    cloud_ready = true;
}

void timerCallback(const ros::TimerEvent &e)
{
    cout << "timer create cloud" << endl;
    if(cloud_ready == true) return;

    // create some cloud
    cloud.points.clear();
    int num = 0;
    for(int i = -20; i <= 20; ++i)
        for(int j = -20; j <= 20; ++j)
            for(int k = -20; k <= 20; ++k)
            {
                pcl::PointXYZ p;
                p.x = i * 0.1;
                p.y = j * 0.1;
                p.z = k * 0.1;
                num++;
                cloud.points.push_back(p);
            }
    // cout << "cloud received, mat:" << cloud_mat << endl;
    cloud_ready = true;
}

/* @function main */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "socket_server");
    ros::NodeHandle n;

    // subscribe to cloud2
    // ros::Subscriber cloud_sub = n.subscribe("/zed/point_cloud/cloud_registered", 1, cloudCallback);
    ros::Subscriber cloud_sub = n.subscribe("/ring_buffer/cloud2", 1, cloudCallback);
    ros::Duration(0.5).sleep();

    // just for test. Create some cloud and publish
    ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);

    // socklen_t len;
    unsigned char buf[1024] = "";  // content buff area
    char recvbuf[8] = "";

    s_in.sin_family = AF_INET;          // IPV4 communication domain
    s_in.sin_addr.s_addr = INADDR_ANY;  // accept any address
    s_in.sin_port = htons(PORT);        // change port to netchar

    sock_sev = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    bind(sock_sev, (struct sockaddr *)&s_in, sizeof(struct sockaddr));
    socklen_t len = sizeof(struct sockaddr);

    vector<double> trans_buff;  // buffer for coding
    cout << "begin" << endl;
    in_len = sizeof(c_in);

    memset(recvbuf, 0, sizeof(recvbuf));

    // main loop, wait for request from client and send message
    while(ros::ok())
    {
        // wait for request from client
        int n = recvfrom(sock_sev, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&c_in, &in_len);
        // cout << (int)recvbuf[5] << (int)recvbuf[6] << (int)recvbuf[7] << endl;

        if(n > 0)
        {
            // wait for cloud ready
            while(!cloud_ready && ros::ok())
            {
                cout << "wait for cloud" << endl;
                ros::spinOnce();
                ros::Duration(0.2).sleep();
            }

            // send all cloud
            point_num = cloud.points.size() / 42;
            if(cloud.points.size() % 42 != 0) point_num += 1;
            cur_num = 0;

            while(cloud_ready && ros::ok())
            {
                // encode point cloud
                trans_buff.clear();
                for(int i = 0; i < 42 && i < cloud.points.size(); ++i)
                {
                    trans_buff.push_back(cloud.points.at(i).x);
                    trans_buff.push_back(cloud.points.at(i).y);
                    trans_buff.push_back(cloud.points.at(i).z);
                }
                if(cloud.points.size() >= 42)
                    cloud.points.erase(cloud.points.begin(), cloud.points.begin() + 42);
                else
                    cloud.points.clear();

                double *buff = &trans_buff[0];
                send_data(buff, trans_buff.size());

                ++cur_num;
                if(cur_num % 100 == 0 || cur_num == point_num)
                    cout << "total pkg num: " << point_num << " , sent num:" << cur_num << endl;

                if(cloud.size() == 0)
                {
                    cloud_ready = false;
                    cout << endl;
                }
            }
        }

        // quit_flag = waitKey(33);

        ros::spinOnce();
    }

    close(sock_sev);

    return 0;
}
