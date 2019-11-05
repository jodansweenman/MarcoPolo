#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
//#include "message_filters/subscriber.h"
//#include "message_filters/synchronizer.h"
//#include <message_filters/sync_policies/approximate_time.h>
#include <stdio.h>
#include <iostream>
#include <cmath>

using namespace std;
//using namespace message_filters;

ros::Publisher point;
ros::Subscriber pcloudsub;
ros::Subscriber ballstatus;
ros::Subscriber ballstatus2;

geometry_msgs::Transform robot;
geometry_msgs::Transform ball_initial;
std_msgs::Int16MultiArray ballstate;

int data0 = 0;
int data1 = 0;
int data2 = 0;
float world_ball_x = 0; 
float world_ball_y = 0; 
float world_ball_z = 0;


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 *  const int u, const int v, geometry_msgs::Point &p
 */
void ballpose(const std_msgs::Int16MultiArray state1)
{
    data0 = state1.data[0];
    data1 = state1.data[1];
    data2 = state1.data[2];
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped ball;
    if(state1.data[0]==1){
        ball.header.stamp = ros::Time::now();
        ball.header.frame_id = "map";
        ball.child_frame_id = "ball_position";
        ball.transform.translation.x = ball_initial.translation.x;
        ball.transform.translation.y = ball_initial.translation.y;
        ball.transform.translation.z = 0;
        ball.transform.rotation.x = 0;
        ball.transform.rotation.y = 0;
        ball.transform.rotation.z = 0;
        ball.transform.rotation.w = 1;
    }
    else{
        ball.header.stamp = ros::Time::now();
        ball.header.frame_id = "map";
        ball.child_frame_id = "ball_position";
        ball.transform.translation.x = 0;
        ball.transform.translation.y = 0;
        ball.transform.translation.z = 0;
        ball.transform.rotation.x = 0;
        ball.transform.rotation.y = 0;
        ball.transform.rotation.z = 0;
        ball.transform.rotation.w = 1;
    }
    br.sendTransform(ball);
}

void depth_find(const sensor_msgs::PointCloud2 pCloud)
{
    if(data0==1){
        int width = pCloud.width;
        int height = pCloud.height;
        
        // Convert from u (column / width), v (row/height) to position in array
        // where X,Y,Z data starts
        int arrayPosition = data1*pCloud.row_step + data2*pCloud.point_step;

        // compute position in array where x,y,z data start
        int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
        int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
        int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

        float X = 0.0;
        float Y = 0.0;
        float Z = 0.0;

        memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
        memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
        memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

        float ball_angle_r = atan(X/Z);
        float ball_hyp = sqrt((X*X)+(Z*Z));

        ball_initial.translation.x = sin(ball_angle_r)*ball_hyp;
        ball_initial.translation.y = cos(ball_angle_r)*ball_hyp;
    }
  // get width and height of 2D point cloud data
      int width = pCloud.width;
      int height = pCloud.height;

      // Convert from u (column / width), v (row/height) to position in array
      // where X,Y,Z data starts
      int arrayPosition = 320*pCloud.row_step + 240*pCloud.point_step;

      // compute position in array where x,y,z data start
      int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
      int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
      int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

      float X = 0.0;
      float Y = 0.0;
      float Z = 0.0;

      memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
      memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
      memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

      // put data into the point p
      //p.x = X;
      //p.y = Y;
      //p.z = Z;

      cout<<X<<" "<<Y<<" "<<Z<<endl;


}

void geometrypoint(){
    geometry_msgs::Point msg;
    msg.x = world_ball_x;      
    msg.y = world_ball_y;      
    msg.z = world_ball_z;      
    point.publish(msg);
    
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depthcloud");

  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  ball_initial.translation.x = 0;
  ball_initial.translation.y = 0;

  ballstatus = n.subscribe("placeholder0", 1000, ballpose);
  ballstatus2 = n.subscribe("placeholder0", 1000, depth_find);

  pcloudsub = n.subscribe("/camera/depth/points", 1000, depth_find);
  point = n.advertise<geometry_msgs::Point>("geopoint", 1000);

  //message_filters::Subscriber<std_msgs::Int16MultiArray> ballstatus2(n, "placeholder0", 1);
  //message_filters::Subscriber<sensor_msgs::PointCloud2> pcloudsub(n, "/camera/depth/points", 1);

  //typedef sync_policies::ApproximateTime<std_msgs::Int16MultiArray, sensor_msgs::PointCloud2> MySyncPolicy;

  //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), ballstatus2, pcloudsub);
  //sync.registerCallback(boost::bind(&depth_find, _1, _2));

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  while(ros::ok()){
      geometry_msgs::TransformStamped robot_coord;
      geometry_msgs::TransformStamped robot_ball_transform;
      try{
          robot_coord = tfBuffer.lookupTransform("base_footprint", "odom", ros::Time(0));
      }
      catch (tf2::TransformException &ex) {
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0).sleep();
          continue;
      }

      robot.translation.x = robot_coord.transform.translation.x;
      robot.translation.y = robot_coord.transform.translation.y;
      robot.translation.z = robot_coord.transform.translation.y;

      robot.rotation.x = robot_coord.transform.rotation.x;
      robot.rotation.y = robot_coord.transform.rotation.y;
      robot.rotation.z = robot_coord.transform.rotation.z;
      robot.rotation.w = robot_coord.transform.rotation.w;

      try{
          robot_coord = tfBuffer.lookupTransform("base_footprint", "odom", ros::Time(0));
      }
      catch (tf2::TransformException &ex) {
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0).sleep();
          continue;
      }
      world_ball_x = robot_ball_transform.transform.translation.x;
      world_ball_y = robot_ball_transform.transform.translation.y;

      ros::spinOnce();
      loop_rate.sleep();
      geometrypoint();
  }
  //ros::spin();

  return 0;
}