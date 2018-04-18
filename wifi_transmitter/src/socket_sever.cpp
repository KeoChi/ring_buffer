#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "ros/ros.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#define PORT 10001
#define BYTE_SEND_INTERVAL 1

using namespace std;
using namespace cv;
int sock_sev;
struct sockaddr_in s_in;  // address structure
struct sockaddr_in c_in;
socklen_t in_len;

// global mat for cloud
Mat cloud_mat;
pcl::PointCloud<pcl::PointXYZ> cloud;
bool cloud_ready = false;
int point_num = 0;

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
    cout << "point num:" << point_num << " buf: " << buffer[4] << endl;
    buffer[5] = (double)pkg_num;  // 255k at most
    buffer[6] = (double)(last_pkg_bytes / 256);
    buffer[7] = (double)(last_pkg_bytes % 256);

    sendto(sock_sev, buffer, 1024, 0, (struct sockaddr *)&c_in, sizeof(struct sockaddr));
    waitKey(BYTE_SEND_INTERVAL);

    /*send data*/
    for(int j = 0; j < pkg_num; j++)
    {
        int pkg_size = 1024;
        if(j == pkg_num - 1) pkg_size = last_pkg_bytes;

        for(int i = 0; i < pkg_size; i++) buffer[i] = *(data + 1024 * j + i);

        sendto(sock_sev, buffer, 1024, 0, (struct sockaddr *)&c_in, sizeof(struct sockaddr));
        waitKey(BYTE_SEND_INTERVAL);
    }

    return 1;
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    cloud.points.clear();
    pcl::fromROSMsg(*msg, cloud);
    cloud_ready = true;
}

void timerCallback(const ros::TimerEvent &e)
{
    cout << "timer create cloud" << endl;

    // create some cloud
    cloud.points.clear();
    int num = 0;
    for(int i = -5; i <= 5; ++i)
        for(int j = -5; j <= 5; ++j)
            for(int k = -5; k <= 5; ++k)
            {
                pcl::PointXYZ p;
                p.x = i;
                p.y = j;
                p.z = k;
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
    ros::Subscriber cloud_sub = n.subscribe("ring_buffer/cloud2", 1, cloudCallback);

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

    /*compress paras init*/
    vector<int> param = vector<int>(2);
    param[0] = CV_IMWRITE_JPEG_QUALITY;
    param[1] = 50;  // default(95) 0-100

    vector<double> trans_buff;  // buffer for coding

    cout << "begin" << endl;

    char quit_flag = ' ';

    while(ros::ok())
    {
        in_len = sizeof(c_in);
        //清空接收缓存数组
        memset(recvbuf, 0, sizeof(recvbuf));
        //开始接收数据
        int n = recvfrom(sock_sev, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&c_in, &in_len);
        cout << (int)recvbuf[5] << (int)recvbuf[6] << (int)recvbuf[7] << endl;

        if(n > 0)
        {
            // wait for cloud ready
            while(!cloud_ready && ros::ok())
            {
                cout << "wait for cloud" << endl;
                ros::spinOnce();
                ros::Duration(0.2).sleep();
            }

            // encode point cloud
            trans_buff.clear();
            point_num = 0;
            for(int i = 0; i < 42 && cloud.size() > 0; ++i)
            {
                trans_buff.push_back(cloud.points.at(0).x);
                trans_buff.push_back(cloud.points.at(0).y);
                trans_buff.push_back(cloud.points.at(0).z);
                cloud.points.erase(cloud.points.begin());

                ++point_num;
            }

            cout << "trans buff: " << endl;
            for(int i = 0; i < trans_buff.size(); ++i)
            {
                cout << trans_buff[i] << endl;
                if(i % 3 == 2) cout << endl;
            }

            double *buff = &trans_buff[0];

            send_data(buff, trans_buff.size());
            cout << "send cloud to client, mat size " << trans_buff.size() << endl;

            if(cloud.size() == 0) cloud_ready = false;
        }

        quit_flag = waitKey(33);

        ros::spinOnce();
    }

    close(sock_sev);

    return 0;
}
