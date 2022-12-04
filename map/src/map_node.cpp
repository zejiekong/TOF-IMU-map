#define RESOLUTION 0.1
#define WIDTH 1000
#define HEIGHT 1000

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <algorithm>
#include <vector>
#include <array>
#include <math.h>


int grid[WIDTH][HEIGHT];
int off_x = WIDTH/2;
int off_y = HEIGHT/2;
float tof_range;

void range_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    tof_range = msg->ranges[0];
}

int main(int argc,char* argv[])
{
    ros::init(argc,argv,"map_node");
    ros::NodeHandle nh;
    ros::Publisher occ_pub;
    tf::TransformListener imu_pose;
    nav_msgs::OccupancyGrid map_msg;
    std::fill((int*)grid,(int*)grid+sizeof(grid)/sizeof(int),-1);
    map_msg.header.frame_id = "map";
    map_msg.info.resolution = RESOLUTION;
    map_msg.info.width = WIDTH;
    map_msg.info.height = HEIGHT;
    map_msg.info.origin.position.x = (int)(-WIDTH/2)*RESOLUTION;
    map_msg.info.origin.position.y = (int)(-HEIGHT/2)*RESOLUTION;
    occ_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map",1);
    ros::Subscriber range_sub = nh.subscribe("/scan",1,range_callback);
    while(ros::ok())
    {
        if(tof_range != 0)
        {
            try
            {
                tf::StampedTransform transform;
                ros::Time now = ros::Time::now();
                imu_pose.waitForTransform("/imu","/map",ros::Time(0),ros::Duration(1));
                imu_pose.lookupTransform("/imu","/map",ros::Time(0),transform);
                tf::Matrix3x3 m(transform.getRotation());
                double roll,pitch,yaw;
                m.getRPY(roll,pitch,yaw);
                float angle = yaw - (M_PI/2);
                //rotation matrix
                int obstacle_x =  -sin(angle)*(tof_range/RESOLUTION) + off_x;
                int obstacle_y =  cos(angle)*(tof_range/RESOLUTION) + off_y;
                grid[obstacle_x][obstacle_y] = 100;
                ROS_INFO("%d,%d",obstacle_x,obstacle_y);
                std::vector<int8_t> grid_flatten;
                for(int i=0;i<WIDTH;i++)
                {
                    for(int j=0;j<HEIGHT;j++)
                    {
                        grid_flatten.push_back(grid[i][j]);
                    }
                }
                map_msg.header.stamp = now;
                map_msg.data = grid_flatten;
                occ_pub.publish(map_msg);
            }
            catch(tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
            }
        }
        ros::spinOnce();
    }
    return 0;
}