#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <array>
#include <angles/angles.h>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <keyboard/Key.h>
#include <math.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#define PI 3.1415926

geometry_msgs::PoseStamped oriPos,ps,currentPos,nextPos,initCirclePos;
geometry_msgs::PoseStamped uavCurrentLocalPose,uavCurrentViconPose;
geometry_msgs::TransformStamped vicon_currentPos;
geometry_msgs::PoseStamped vicon_mocaptf;

ros::Publisher setPositionPublisher;
ros::Publisher currentViconPositionPublisher;
ros::ServiceClient arming_client;

mavros_msgs::CommandBool arm_cmd;

bool flag_offboard_mode = false;
bool isLockLocalPos;
bool isGoTarget;
bool XhasReach;
bool YhasReach;
bool hasSet;

double tmpRoll,tmpPitch,tmpYaw;
double target_x, target_y, target_z;

float pos_value;

// 判断飞行模式
mavros_msgs::State current_state;
void stateReceived(const mavros_msgs::State::ConstPtr& msg) 
{
    static bool init_flag = false;
    current_state = *msg;

    if (current_state.mode == "OFFBOARD")
    {
        if(!init_flag)
        {
            ROS_INFO_STREAM("Offboard Mode"); 
            init_flag = true;         
        }

        flag_offboard_mode = true;
    }
    else
    {
        flag_offboard_mode = false;
        init_flag = false;  
    }
}

// 无人机当前位置和姿态(Pixhawk LPE 滤波结果)
void localPositionReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{
    uavCurrentLocalPose.pose.position.x = msg->pose.position.x;
    uavCurrentLocalPose.pose.position.y = msg->pose.position.y;
    uavCurrentLocalPose.pose.position.z = msg->pose.position.z;
    uavCurrentLocalPose.pose.orientation.x = msg->pose.orientation.x;
    uavCurrentLocalPose.pose.orientation.y = msg->pose.orientation.y;
    uavCurrentLocalPose.pose.orientation.z = msg->pose.orientation.z;
    uavCurrentLocalPose.pose.orientation.w = msg->pose.orientation.w;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(uavCurrentLocalPose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(tmpRoll, tmpPitch, tmpYaw);

    ROS_INFO("Roll,Pitch,Yaw:[%0.3f,%0.3f,%0.3f]",tmpRoll/3.14*180,tmpPitch/3.14*180,tmpYaw/3.14*180);

    if (!isLockLocalPos)
    {
        ps = uavCurrentLocalPose;
        hasSet = true;
    }
}

// Vicon位姿数据转换为Mavros位姿数据(转换原因：消息类型不匹配)
void PosetopicTransport(geometry_msgs::TransformStamped& viconPos, geometry_msgs::PoseStamped& rosPos)
{
    rosPos.pose.position.x = viconPos.transform.translation.x;
    rosPos.pose.position.y = viconPos.transform.translation.y;
    rosPos.pose.position.z = viconPos.transform.translation.z;
    rosPos.pose.orientation.x = viconPos.transform.rotation.x;
    rosPos.pose.orientation.y = viconPos.transform.rotation.y;
    rosPos.pose.orientation.z = viconPos.transform.rotation.z;
    rosPos.pose.orientation.w = viconPos.transform.rotation.w;
}

// Vicon 数据接收处理
void viconPositionReceived(const geometry_msgs::TransformStampedConstPtr& vicon_msg)
{
    vicon_currentPos = *vicon_msg;
    PosetopicTransport(vicon_currentPos, uavCurrentViconPose);
}

/***************** 键盘控制 ********************/
void sendCommand(const keyboard::Key &key)
{
  switch(key.code)
  {
      case 'w':
      {
        // Up
        ps.pose.position.z += pos_value;      
        ROS_INFO_STREAM("Up: " << pos_value);
        break;
      }
      case 's':
      {
        // Down
        ps.pose.position.z -= pos_value;
        ROS_INFO_STREAM("Down: "<< pos_value);
        break;
      }
      case 'i':
      {
        // Forward
        ps.pose.position.x = ps.pose.position.x + pos_value * cos(tmpYaw);
        ps.pose.position.y = ps.pose.position.y + pos_value * sin(tmpYaw);
        ROS_INFO_STREAM("Forward: " << pos_value);        
        break;
      }
      case 'k':
      {
        // Backward
        ps.pose.position.x = ps.pose.position.x - pos_value * cos(tmpYaw);
        ps.pose.position.y = ps.pose.position.y - pos_value * sin(tmpYaw);
        ROS_INFO_STREAM("Backward: " << pos_value);       
        break;
      }
      case 'j':
      {
        // left
        ps.pose.position.x = ps.pose.position.x - pos_value * sin(tmpYaw);
        ps.pose.position.y = ps.pose.position.y + pos_value * cos(tmpYaw);
        ROS_INFO_STREAM("Left: " << pos_value);     
        break;
      }
      case 'l':
      {
        // right
        ps.pose.position.x = ps.pose.position.x + pos_value * sin(tmpYaw);
        ps.pose.position.y = ps.pose.position.y - pos_value * cos(tmpYaw);       
        ROS_INFO_STREAM("Right: " << pos_value);
        break;
      }
      case 'a':
      {
        // turn left
        tmpYaw += 0.05;
        if (tmpYaw > PI)
        {
           tmpYaw = tmpYaw - 2*PI;
        }
        ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpYaw);
        ROS_INFO("Yaw:[%0.3f]",tmpYaw/PI*180);
        break;
      }
      case 'd':
      {
        // turn right
        tmpYaw -= 0.05;
        if (tmpYaw < -PI)
        {
           tmpYaw = tmpYaw + 2*PI;
        }
        ps.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,tmpYaw);
		    ROS_INFO("Yaw:[%0.3f]",tmpYaw/PI*180);
        break;
      }
      case 'p':
      {
        // back to position control
        isGoTarget = false;
        isLockLocalPos = false;
        XhasReach = false;
        YhasReach = false;
        break;
      }
      case 'g':
      {
        // go to target position
        isGoTarget = true;
        break;
      }
      case 'b':
      {
        // unlock local position
        isLockLocalPos = false;
        break;
      }
      case 'n':
      {
        // lock local position
        isLockLocalPos = true;
        break;
      }
      default:
      {

      }
  }
}

// Vicon自动降落位置控制
void auto_position_land()
{
    if(isGoTarget)
    {
      	// x方向 当前位置与目标位置差值小于阈值，next_pos 设定为目标值，认为已经达到目标位置
        if(fabs(uavCurrentLocalPose.pose.position.x - target_x) < 0.3)
      	{
        		ps.pose.position.x = target_x;  		
        		XhasReach = true;
        		ROS_INFO_STREAM("X has Reach");
      	}
        // x方向 当前位置与目标位置差值大于阈值，当前位置与设定位置小于阈值(目前0.2),每次发送固定累加位置(目前0.4)
        else
        {
        		if(target_x > uavCurrentLocalPose.pose.position.x)
        		{
        			  if (fabs(uavCurrentLocalPose.pose.position.x - ps.pose.position.x) < 0.2)
        			      ps.pose.position.x = ps.pose.position.x + 0.4; 
        		}
            else
            {
        			  if (fabs(uavCurrentLocalPose.pose.position.x - ps.pose.position.x) < 0.2)
        			      ps.pose.position.x = ps.pose.position.x - 0.4; 
        		}
      	}

        // y方向, 解释同上
      	if(fabs(uavCurrentLocalPose.pose.position.y - target_y) < 0.3)
      	{
      	    ps.pose.position.y = target_y;   		
        		YhasReach = true;
        		ROS_INFO_STREAM("Y has Reach");
      	}
        else
        {
        		if(target_y > uavCurrentLocalPose.pose.position.y)
        		{
          			if (fabs(uavCurrentLocalPose.pose.position.y - ps.pose.position.y) < 0.2)
          			    ps.pose.position.y = ps.pose.position.y + 0.4; 
        		}
            else
            {
          			if (fabs(uavCurrentLocalPose.pose.position.y - ps.pose.position.y) < 0.2)
          			    ps.pose.position.y = ps.pose.position.y - 0.4; 
        		}
      	}

        // x y 方向基本到达目标位置，开始逐渐降落
      	if(XhasReach && YhasReach)
      	{
        		ps.pose.position.x = target_x;  
        		ps.pose.position.y = target_y; 	
        		
        		if (fabs(uavCurrentLocalPose.pose.position.z - ps.pose.position.z) < 0.15)
        			 ps.pose.position.z = ps.pose.position.z - 0.2;

            // 高度小于 0.17m，认为已经降落到地面，飞机 disarm
        		if (uavCurrentLocalPose.pose.position.z < 0.17) 
    		    {
                arm_cmd.request.value = false;
                arming_client.call(arm_cmd);    
    		        if(arm_cmd.response.success)
                   ROS_INFO("Vehicle disarmed");
    	      }	
      	}
    }
    
    // 发布设定位置 next_pos
  	ps.header.seq++;
    ps.header.stamp = ros::Time::now();
    setPositionPublisher.publish(ps);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vicon_position_auto_land");
    ros::NodeHandle nodeHandle;

    printf("------------landing position control node running successfully-------------\n");

    setPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros04/setpoint_position/local",10);
    currentViconPositionPublisher = nodeHandle.advertise<geometry_msgs::PoseStamped>("/mavros04/mocap/pose",10);
    ros::Subscriber localPositionSubsciber = nodeHandle.subscribe("/mavros04/local_position/pose", 10, localPositionReceived);
    ros::Subscriber viconPositionSubsciber = nodeHandle.subscribe("/vicon/nuflie04/nuflie04", 10, viconPositionReceived);
    ros::Subscriber commandSubscriber = nodeHandle.subscribe("/keyboard/keydown",1,sendCommand);
    ros::Subscriber stateSubscriber = nodeHandle.subscribe("mavros04/state", 10, stateReceived);
    arming_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>("mavros04/cmd/arming");

    arm_cmd.request.value = false;  
    hasSet = false;
    isLockLocalPos = false;
    isGoTarget = false;
    XhasReach = false;
    YhasReach = false;

    pos_value = 0.2f;

    target_x = 0.095;
    target_y = -4.116;
    target_z = 0.0; 

    tmpRoll = 0.0;
    tmpPitch = 0.0;
    tmpYaw= 0.0;

    ros::Rate loopRate(50.0);
    while(ros::ok())
    {
        if(hasSet) 
        {       
            auto_position_land();
        } 

        // 向 mocap/pose 话题发送 Vicon 提供的位姿数据
        uavCurrentViconPose.header.stamp = ros::Time::now();
        uavCurrentViconPose.header.frame_id = "/world";
        currentViconPositionPublisher.publish(uavCurrentViconPose);  

        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
