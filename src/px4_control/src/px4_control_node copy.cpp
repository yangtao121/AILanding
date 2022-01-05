#include <ros/ros.h>
#include<std_msgs/Float32.h>
#include<mavros_msgs/CommandBool.h>
#include<mavros_msgs/SetMode.h>
#include<mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include<geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// recieve target velocity
std_msgs::Float32 target_vx;
std_msgs::Float32 target_vy;
std_msgs::Float32 target_vz;

void velocityx_cb(const std_msgs::Float32::ConstPtr& msg){
    target_vx = *msg;
}

void velocityy_cb(const std_msgs::Float32::ConstPtr& msg){
    target_vy = *msg;
}

void velocityz_cb(const std_msgs::Float32::ConstPtr& msg){
    target_vz = *msg;
}
// recieve px4's info
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

int main(int argc, char** argv){
    ros::init(argc,argv,"px4_node");
    ros::NodeHandle nh;

    // target velocity subscriber
    ros::Subscriber target_vel_x_sub = nh.subscribe<std_msgs::Float32>("velocity_x",10,velocityx_cb);
    ros::Subscriber target_vel_y_sub = nh.subscribe<std_msgs::Float32>("velocity_y",10,velocityy_cb);
    ros::Subscriber target_vel_z_sub = nh.subscribe<std_msgs::Float32>("velocity_z",10,velocityz_cb);

    // subscribe px4's info
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);

    // mavros service
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // publisher
    ros::Publisher yaw_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    ros::Rate rate(20);

    while (ros::ok() && current_state.connected) {
        ROS_INFO("conneted");
		ros::spinOnce();
		rate.sleep();
	}

    ros::Time last_request = ros::Time::now();
	ros::Time request;

   
//  waiting for remote control
       
    while (ros::ok()) //loop always
	{
        ROS_INFO("waiting for remote control.");
		if (current_state.mode == "AUTO.LOITER")
			break;
 
		ros::spinOnce();
		rate.sleep();
	}

    // checking status 

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    
        

     //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::PositionTarget yaw;
    geometry_msgs::Vector3 target_vel;
    // zero = geometry_msgs::Vector3()

    // zero.x.data = 0;
    // zero.y.data = 0;
    // zero.z.data = 0;

    yaw.type_mask=1+2+4+64+128+256+512+1024+2048;
    yaw.coordinate_frame = 8;
    // yaw.position = geometry_msgs::Vector3()
    while (ros::ok())
    {
        if (current_state.mode == "AUTO.LAND"){
            ROS_INFO("Terminate!!!!!");
            break;
        }
        if( current_state.mode != "OFFBOARD" &&(ros::Time::now() - last_request > ros::Duration(5.0)))
			{
				if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
				{
                        ROS_INFO("Offboard enabled");
					
				}
					last_request = ros::Time::now();
			}
        else
			{
				if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
				{
					if( arming_client.call(arm_cmd) &&arm_cmd.response.success)
					{
                            ROS_INFO("Vehicle armed");
					}
					last_request = ros::Time::now();
				}
			}

        target_vel.x = target_vx.data*0.4;
        target_vel.y = target_vy.data*0.4;
        target_vel.z = target_vz.data*0.8;
        yaw.velocity = target_vel;
        yaw_pub.publish(yaw);
        // ROS_INFO("velocity:%f,%f,%f",target_vx.data,target_vy.data,target_vz.data);
        // ROS_INFO("%s",current_state.mode);
        if (current_state.mode=="OFFBOARD" ){
            // ROS_INFO("offboard");
            ROS_INFO("velocity:%f,%f,%f",target_vx.data,target_vy.data,target_vz.data);

        }
        if (current_state.mode=="AUTO.LOITER"){
            ROS_INFO("AUTO.LOITER");
        }

        ros::spinOnce();
        rate.sleep();
    }
}