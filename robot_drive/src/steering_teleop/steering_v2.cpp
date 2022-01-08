#include <iostream>
#include <signal.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "std_msgs/Float64.h"
#include <cmath>
#include <iomanip>
#include "definitions.hpp"

using namespace std;


void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

void joy_cb(const sensor_msgs::Joy::ConstPtr& msg){
    joy_msg.set_buttons(msg->buttons[0],msg->buttons[1],msg->buttons[2],msg->buttons[3],msg->buttons[4],msg->buttons[5],msg->buttons[6],msg->buttons[7],msg->buttons[8],msg->buttons[9],msg->buttons[10],msg->buttons[11]);
}



class steering
{
private:
    double Pi = M_PI;


    ros::Publisher pub_steer;
    ros::Publisher pub_wheel[4];
    geometry_msgs::Twist twist;
    ros::Subscriber twistSub;
    double _eps = pow(10,-7);
    double zeroPos[4]={Pi/2,Pi/2,Pi/2,Pi/2};
    //double zeroPos[4]={0,0,0,0};

    trajectory_msgs::JointTrajectory steering_msg;
    trajectory_msgs::JointTrajectoryPoint steering_points;

    double steer_arr[4] = {0,0,0,0};
    int    wheel_arr[4] = {0,0,0,0};


public:
    steering(ros::NodeHandle *nh){
        std::cout << std::fixed;
        std::cout << std::setprecision(11);
        pub_steer = nh->advertise<trajectory_msgs::JointTrajectory>("/rover_steering_controller/command",10);
        ros::Rate rate(150);
        pub_wheel[0] = nh->advertise<std_msgs::Float64>("/rover_wheel_leftfront/command" ,10);
        pub_wheel[1] = nh->advertise<std_msgs::Float64>("/rover_wheel_leftrear/command" ,10);
        pub_wheel[2] = nh->advertise<std_msgs::Float64>("/rover_wheel_rightfront/command",10);
        pub_wheel[3] = nh->advertise<std_msgs::Float64>("/rover_wheel_rightrear/command" ,10);

        pub_steer = nh->advertise<trajectory_msgs::JointTrajectory>("/rover_steering_controller/command",10);


        steering_msg.joint_names= {"steering_leftfront_joint",
                                    "steering_leftrear_joint" ,
                                    "steering_rightrear_joint",
                                    "steering_rightfront_joint"} ; 

      
        twistSub = nh->subscribe("/drive_system/twist",10,&steering::twist_cb,this);
        ros::Subscriber joy_steering_Sub = nh->subscribe("/joy", 10, &steering::steering_joy_cb,this);
        while (ros::ok && second_stage)
        {
                if ((joy_msg.get_button(7) == 1) && !(first_stage))
                {
                    first_stage = true;
                }
                if ((first_stage) && joy_msg.get_button(7) == 0)
                {
                    if (second_stage)
                    {
                        second_stage = false;
                    }

                    else
                    {
                        second_stage = true;
                    }
            
                    first_stage = false;
            
                }
            
            ros::spinOnce();
            action();
            rate.sleep();
            signal(1, mySigintHandler);
            
        }
        
    }

    void steering_joy_cb(const sensor_msgs::Joy::ConstPtr& msg){
        joy_msg.set_buttons(msg->buttons[0],msg->buttons[1],msg->buttons[2],msg->buttons[3],msg->buttons[4],msg->buttons[5],msg->buttons[6],msg->buttons[7],msg->buttons[8],msg->buttons[9],msg->buttons[10],msg->buttons[11]);
        //cout << joy_msg.get_button(7)<<endl;
    }
    void twist_cb(const geometry_msgs::Twist& msg){
        twist =msg;
    }

    void tank_rotation(){

        trajectory_msgs::JointTrajectory steering_msg;
        steering_msg.joint_names= {"steering_leftfront_joint",
                                    "steering_leftrear_joint" ,
                                    "steering_rightrear_joint",
                                    "steering_rightfront_joint"} ; 
        trajectory_msgs::JointTrajectoryPoint steering_points;
        for (int i = 0; i < 4; i++) steer_arr[i]= Pi/4 *(((i%2==1)-.5)*2);

        for(double& c: steer_arr) steering_points.positions.push_back(c);
        steering_points.time_from_start =ros::Duration(0.005); 
        steering_msg.points.push_back(steering_points);
        pub_steer.publish(steering_msg);

        std_msgs::Float64 z[4];

        for (int i = 0; i < 4; i++) z[i].data = twist.angular.z*1.7584/2*Pi*(((i>1)-.5)*2);
        for (int i = 0; i < 4; i++) pub_wheel[i].publish(z[i]); 
        
        //cout << twist.angular.z<< endl;
    }

    double stLim(double steer_deg, double dDeg) {
        //cout<<fmod(  steer_deg+dDeg  ,  Pi+_eps)  -Pi/2. << endl;;
        return fmod(  steer_deg+dDeg  ,  Pi+_eps)  -Pi/2. ;}

    void cartesian_wheel_rot(double deg){
        
        trajectory_msgs::JointTrajectory steering_msg;
        steering_msg.joint_names= {"steering_leftfront_joint",
                                    "steering_leftrear_joint" ,
                                    "steering_rightrear_joint",
                                    "steering_rightfront_joint"} ; 
        trajectory_msgs::JointTrajectoryPoint steering_points;

        for (int i = 0; i < 4; i++) steer_arr[i]= stLim(zeroPos[i],deg);
        //cout<< steer_arr[0]<<"  "<< steer_arr[1]<<"  "<< steer_arr[2]<<"  "<< steer_arr[3]<<"  "<<endl;
        for(double& c: steer_arr) steering_points.positions.push_back(c);
        steering_points.time_from_start =ros::Duration(0.005); 
        steering_msg.points.push_back(steering_points);
        pub_steer.publish(steering_msg);

        std_msgs::Float64 z[4];
        for (int i = 0; i < 4; i++) z[i].data = pow(pow(twist.linear.x,2)+pow(twist.linear.y,2),.5)*(((twist.linear.x>=0)-.5)*2);
        for (int i = 0; i < 4; i++) pub_wheel[i].publish(z[i]); 

    }

    void cartesian_motion(){
        double degWheel = atan((twist.linear.y)/(twist.linear.x+_eps));
        if  (isnan(degWheel))  {degWheel=0;} 
        //cout << "vec degree --->  "<<degWheel<<endl;
        cartesian_wheel_rot(degWheel);
    }

    void action(){
        if (twist.angular.z){
            tank_rotation();
        }else{
            cartesian_motion();
        }
    }

    ~steering(){

    }
};

/*
steering::steering()
{
}

steering::~steering()
{
}*/


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Steering_system",ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::Subscriber joySub = nh.subscribe("/joy", 10, joy_cb);
    while (ros::ok())
    {
        ros::spinOnce();
        if ((joy_msg.get_button(7) == 1) && !(first_stage))
        {
            first_stage = true;
        }
        if ((first_stage) && joy_msg.get_button(7) == 0)
        {
            if (second_stage)
            {
                second_stage = false;
            }

            else
            {
                second_stage = true;
            }
            
            first_stage = false;
            
        }
        
        if (!(second_stage))
        {
            while (ros::ok() && !(second_stage))
            {
                RobotArm arm(&nh);
                signal(1, mySigintHandler);
            }
            
            
        }
        
        else
        {
            while (ros::ok() && second_stage)
            {
                steering Steering(&nh);
                RobotArm arm(&nh); // do not why necessery but it is essential for keep arm position
                signal(1, mySigintHandler);
            }
            

        }
        
        
    }
    
    
    return 0;
}



