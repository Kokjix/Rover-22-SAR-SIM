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


bool first_stage = false;
bool second_stage = false;

void SigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

class joy_message{

    double axes[8];
    double buttons[12];


public:
	void set_axes(double,double,double,double,double,double, double, double);
	double get_axis(int);
	void set_buttons(double,double,double,double,double,double,double,double,double,double,double,double);
	double get_button(int);
};

void joy_message::set_axes(double new_axis_0, double new_axis_1, double new_axis_2, double new_axis_3, double new_axis_4, double new_axis_5, double new_axis_6, double new_axis_7){

	axes[0] = new_axis_0;
	axes[1] = new_axis_1;
	axes[2] = new_axis_2;
	axes[3] = new_axis_3;
	axes[4] = new_axis_4;
	axes[5] = new_axis_5;
	axes[6] = new_axis_6;
	axes[7] = new_axis_7;
}



double joy_message::get_axis(int axis_index){

	return axes[axis_index];
}



void joy_message::set_buttons(double new_button_0, double new_button_1, double new_button_2, double new_button_3, double new_button_4, double new_button_5, double new_button_6, double new_button_7, double new_button_8, double new_button_9, double new_button_10, double new_button_11){

	buttons[0] = new_button_0;
	buttons[1] = new_button_1;
	buttons[2] = new_button_2;
	buttons[3] = new_button_3;
	buttons[4] = new_button_4;
	buttons[5] = new_button_5;
	buttons[6] = new_button_6;
	buttons[7] = new_button_7;
	buttons[8] = new_button_8;
	buttons[9] = new_button_9;
	buttons[10] = new_button_10;
	buttons[11] = new_button_11;
}



double joy_message::get_button(int button_index){

	return buttons[button_index];
}

joy_message joy_msg;

class RobotArm
{
private:
	ros::Publisher arm_publisher;
	ros::Publisher right_finger_publisher;
	ros::Publisher left_finger_publisher;
	ros::Subscriber arm_joy;
	ros::Subscriber arm_mod_switch;
	double arm[6];
	double arm_delta_theta[6];
	double gripper_delta_theta[2];
	double gripper[2];


public:
	RobotArm(ros::NodeHandle *nh){
		arm_publisher = nh->advertise<trajectory_msgs::JointTrajectory>("rover_arm_controller/command",10);
		ros::Rate rate(150);
		right_finger_publisher = nh->advertise<std_msgs::Float64>("rover_arm_right_finger/command",10);
		left_finger_publisher = nh->advertise<std_msgs::Float64>("/rover_arm_left_finger/command",10);
		arm_joy = nh->subscribe("/joy", 10, &RobotArm::arm_joy_cb,this);
		//arm_mod_switch = nh->subscribe("/joy", 10, &RobotArm::arm_joy_mod_switch,this);

		
		/*for (int i = 0; i < 6; i++)
		{
			arm[i] = 0.0;
			arm_delta_theta[i] = 0.0;
		}
		
		for (int i = 0; i < 2; i++)
		{
			gripper_delta_theta[i] = 0.0;
			gripper[i] = 0.0;
		}*/
		
		while (ros::ok && !(second_stage))
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
            //std::cout<<"arm"<<std::endl;            
            ros::spinOnce();
            arm_joints();
            rate.sleep();
            signal(1, SigintHandler);
            
        }
	}
	// void arm_joy_mod_switch(const sensor_msgs::Joy::ConstPtr& msg)
	// {
	// 	joy_msg.set_buttons(msg->buttons[0],msg->buttons[1],msg->buttons[2],msg->buttons[3],msg->buttons[4],msg->buttons[5],msg->buttons[6],msg->buttons[7],msg->buttons[8],msg->buttons[9],msg->buttons[10],msg->buttons[11]);
    //     //cout << joy_msg.get_button(7)<<endl;
	// }
	void arm_joy_cb(const sensor_msgs::Joy::ConstPtr& msg)
	{
		joy_msg.set_axes(msg->axes[0],msg->axes[1],msg->axes[2],msg->axes[3],msg->axes[4],msg->axes[5], msg->axes[6], msg->axes[7]);
		joy_msg.set_buttons(msg->buttons[0],msg->buttons[1],msg->buttons[2],msg->buttons[3],msg->buttons[4],msg->buttons[5],msg->buttons[6],msg->buttons[7],msg->buttons[8],msg->buttons[9],msg->buttons[10],msg->buttons[11]);

	}
	void arm_joints()
	{	
		trajectory_msgs::JointTrajectory arm_msg;
		trajectory_msgs::JointTrajectoryPoint arm_point;
		arm_msg.joint_names = {"axis_1", "axis_2", "axis_3", "axis_4", "axis_5", "axis_6"};
		arm_delta_theta[0] = joy_msg.get_axis(0) * 0.03;
    	arm_delta_theta[1] = joy_msg.get_axis(1) * 0.03;
   	 	arm_delta_theta[2] = joy_msg.get_axis(4) * 0.04;
    	arm_delta_theta[3] = -joy_msg.get_axis(3) * 0.03;
    	arm_delta_theta[4] = -joy_msg.get_axis(7) * 0.03;
    	arm_delta_theta[5] = joy_msg.get_axis(6) * 0.06;
		gripper_delta_theta[0] = (joy_msg.get_button(1) - joy_msg.get_button(3)) * 0.03;  // right finger 
    	gripper_delta_theta[1] = (-joy_msg.get_button(1) + joy_msg.get_button(3)) * 0.03; // left finger


		for (int i = 0; i < 6; i++)
		{
			arm[i] += arm_delta_theta[i];
		}
		
		

		gripper[0] += gripper_delta_theta[0];
        gripper[1] += gripper_delta_theta[1];
		for(double& c: arm) arm_point.positions.push_back(c);
		arm_point.time_from_start =ros::Duration(0.005); 
        arm_msg.points.push_back(arm_point);
		std::cout << arm_msg << std::endl;
        arm_publisher.publish(arm_msg);
		std_msgs::Float64 gripper_pub[2];
		gripper_pub[0].data = gripper[0];
		gripper_pub[1].data = gripper[1];
		right_finger_publisher.publish(gripper_pub[0]);
		left_finger_publisher.publish(gripper_pub[1]);
	}

	~RobotArm()
	{

	}
};


