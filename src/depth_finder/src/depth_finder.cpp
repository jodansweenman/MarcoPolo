//Importing libraries
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <iostream>
#include <cmath>

//Std namepsace to clean up code
using namespace std;

//Declaration of ROS Publishers and Subscribers
ros::Publisher point;
ros::Subscriber pcloudsub;
ros::Subscriber ballstatus;
ros::Subscriber ballstatus2;

//Global Variable Declarations for this node
geometry_msgs::Transform robot;
geometry_msgs::Transform ball_initial;
geometry_msgs::PointStamped initial_pt;
geometry_msgs::PointStamped transformed_pt;
std_msgs::Int16MultiArray ballstate;

int data0 = 0;
int u = 0;
int v = 0;
int ball_flag = 0;
float world_ball_x = 0; 
float world_ball_y = 0; 
float world_ball_z = 0;

float X = 0.0;
float Y = 0.0;
float Z = 0.0;


/**
 * This subscriber publishes a tf for the location of the ball and records it's current state.
 */
void ballpose(const std_msgs::Int16MultiArray state1)
{
    data0 = state1.data[0];
    u = state1.data[1];
    v = state1.data[2];
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


/**
 * This subscriber looks at point cloud information and takes in information from the object recognition node, and records it to an initial reference point 
 */
void depth_find(const sensor_msgs::PointCloud2 pCloud)
{
    if(data0==1){
        ball_flag = 1;

        initial_pt.point.x = Z;
        initial_pt.point.y = X;
        initial_pt.point.z = Y;
    }
    int width = pCloud.width;
    int height = pCloud.height;

    // Convert from u (column / width), v (row/height) to position in array
    // where X,Y,Z data starts
    int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

    // compute position in array where x,y,z data start
    int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

    memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

    //cout<<"XYZ "<<Z<<" "<<X<<" "<<Y<<endl;


}


/**
 * This function publishes a geometry point to be taken in by the speech node to aloow the robot to state it's relative position to the ball.
 */
void geometrypoint(){
    geometry_msgs::Point msg;

    //Creating Quaternion for robot quaternion orientation for manipulation
    tf::Quaternion q_robot(robot.rotation.x, robot.rotation.y, robot.rotation.z, robot.rotation.w);
  
    //Putting data into matrix 3x3 format
    tf::Matrix3x3 m_robot(q_robot);

    //Variables for RPY, yaw in degrees, counters, and imar ground truth x and y
    double r_roll, r_pitch, r_yaw, r_yawDeg;

    //Conversion to Euler Imar
    m_robot.getRPY(r_roll, r_pitch, r_yaw);
    r_yawDeg = r_yaw * 180/M_PI;


    float angle_w = 0 - tan((transformed_pt.point.x-robot.translation.x)/(transformed_pt.point.y-robot.translation.y));
    float relative_x = sin(angle_w-r_yaw)*abs((transformed_pt.point.x-robot.translation.x));
    float relative_y = cos(angle_w-r_yaw)*abs((transformed_pt.point.y-robot.translation.y));
    //cout<<"relative_x "<<relative_x<<endl;
    //msg.x = relative_x;      
    //msg.y = relative_y;      
    msg.z = 0.0;

    if(isnan(relative_x) || relative_x == 0){
        point.publish(msg);
    }
    else{
        msg.x = -transformed_pt.point.y;      
        msg.y = transformed_pt.point.x;
        point.publish(msg);
    }      
    
    
}

/**
 * This main function instantiates the ROS node, and triggers the publishers and subscribers running..
 */
int main(int argc, char **argv)
{
  //Node intitialization  
  ros::init(argc, argv, "depthcloud");

  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  //Ball /tf initialization
  ball_initial.translation.x = 0;
  ball_initial.translation.y = 0;

  //Subscribers being started
  ballstatus = n.subscribe("/marco/ball_uv", 1000, ballpose);
  //ballstatus2 = n.subscribe("/marco/redball_uv", 1000, depth_find);

  pcloudsub = n.subscribe("/camera/depth/points", 1000, depth_find);

  //Start of publisher
  point = n.advertise<geometry_msgs::Point>("geopoint", 1000);

  //ROS tf2 initialization for tf listener
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  //Loop for listener, publisher, and ROS transform
  while(ros::ok()){
      //Variables for transform
      geometry_msgs::TransformStamped robot_coord;
      geometry_msgs::TransformStamped robot_ball_transform;
      try{
          //Listening to transform between robot and world coordinates
          robot_coord = tfBuffer.lookupTransform("base_footprint", "odom", ros::Time(0));

          //Transform to give relative position of ball to robot translation and rotation
          tf2::doTransform(initial_pt, transformed_pt, robot_coord);
          //cout<<"initial point "<<initial_pt.point.x<<" transformed point "<<transformed_pt.point.x<<endl;
      }
      catch (tf2::TransformException &ex) {
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0).sleep();
          continue;
      }

      //storage of robot transform
      robot.translation.x = robot_coord.transform.translation.x;
      robot.translation.y = robot_coord.transform.translation.y;
      robot.translation.z = robot_coord.transform.translation.y;

      robot.rotation.x = robot_coord.transform.rotation.x;
      robot.rotation.y = robot_coord.transform.rotation.y;
      robot.rotation.z = robot_coord.transform.rotation.z;
      robot.rotation.w = robot_coord.transform.rotation.w;

      ros::spinOnce();
      loop_rate.sleep();
      geometrypoint();
  }

  return 0;
}
