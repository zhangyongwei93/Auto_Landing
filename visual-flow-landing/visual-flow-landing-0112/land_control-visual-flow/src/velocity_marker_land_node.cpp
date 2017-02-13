#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <keyboard/Key.h>

using namespace std;

// 主点坐标
# define U0 319.0
# define V0 220.0
// 归一化焦距
# define FOCAL_X 487.0
# define FOCAL_Y 488.0

geometry_msgs::TwistStamped vs_body_axis;
geometry_msgs::PoseStamped uavPose;

ros::Publisher bodyAxisVelocityPublisher;
ros::ServiceClient arming_client;

double uavRollENU, uavPitchENU, uavYawENU;

double err_x, err_y, err_z;
double err_roll, err_pitch, err_yaw;
double last_err_x, last_err_y, last_err_z;
double last_err_roll, last_err_pitch, last_err_raw;
double last_timestamp;

double xyP, xyI, xyD, zP, zI, zD, yawP, yawD;
double dt;

float uav_init_altitude = 0.0;
float uav_altitude = 0.0;     
float uav_x_distance = 0.0;
float uav_y_distance = 0.0;

bool flag_offboard_mode = false;
bool flag_enter_position_hold = false;
bool flag_enter_manual_adjust = false;

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

// 无人机位置和姿态，From 内部传感器
void uavPoseReceived(const geometry_msgs::PoseStampedConstPtr& msg)
{    
    uavPose.pose.position.x = msg->pose.position.x;
    uavPose.pose.position.y = msg->pose.position.y;
    uavPose.pose.position.z = msg->pose.position.z;
    uavPose.pose.orientation.x = msg->pose.orientation.x;
    uavPose.pose.orientation.y = msg->pose.orientation.y;
    uavPose.pose.orientation.z = msg->pose.orientation.z;
    uavPose.pose.orientation.w = msg->pose.orientation.w;
    
    // Using ROS tf to get RPY angle from Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(uavPose.pose.orientation, quat);  
    tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU);
    //ROS_INFO("Current UAV angles: roll=%0.3f, pitch=%0.3f, yaw=%0.3f", uavRollENU*180/3.1415926, uavPitchENU*180/3.1415926, uavYawENU*180/3.1415926);  
}

// 从超声传感器获取飞机高度,并计算高度偏差 
void uavAltitudeReceived(const sensor_msgs::Range& msg)
{
    static bool init_flag = false; 
    if (msg.range > 0.28f)
    {
        if(!init_flag)
        {
        	uav_init_altitude = msg.range;
    	    init_flag = true; 
        }
        else
        {           
            uav_altitude = msg.range;
        }

        //cout << "uav_altitude = " << uav_altitude << endl;
        // 首先控制高度恒定，z轴偏差为目前高度与初始高度之差
        err_z = uav_init_altitude - uav_altitude;
    }
}

// 获取 aruco 坐标中心，并计算无人机相对 x y 距离 
void markerCenterReceived(const geometry_msgs::Point& msg)
{	
	static int flag_not_found_mark = 0;
    static geometry_msgs::Point markcenter;

	// 获取 aruco 坐标中心
	markcenter.x = msg.x;
	markcenter.y = msg.y;

    // 当标志中心坐标有效时，计算无人机相对标志 x y 距离
    if (markcenter.x > 0.01f && markcenter.y > 0.01f) 
    {
    	uav_x_distance = uav_altitude*(markcenter.x-U0)/FOCAL_X;
    	uav_y_distance = uav_altitude*(markcenter.y-V0)/FOCAL_Y;

        // 将无人机与mark在 x y z 方向的距离偏差，分别表示为err_ ,便于控制部分的理解
        // 图像坐标系与Vicon坐标系，x轴反向，y轴重合
    	err_x = -uav_x_distance;
    	err_y = uav_y_distance;
        
        //一旦发现标志，将未发现mark的计数标志复位0
        flag_not_found_mark = 0;
    } 
    // 说明： 1.8m 只是尝试值，明显有点大
    // offboard 模式下，高度大于1.8m？，连续 10次 未发现标志，认为目标丢失，自动进入定点悬停模式
    // 高度小于1.8m？，由于 Tag 占图像大部分，飞机晃动，Tag很容易出视野，识别失败，此时保持继续降落
    else
    {
        flag_not_found_mark++;
        if (flag_offboard_mode && (flag_not_found_mark > 10))
        {           
            if(uav_altitude > 1.8)
            {
                ROS_INFO_STREAM("Fail to found mark, enter Position Hold");
                flag_enter_position_hold = true;
            }
            else
            {
                err_x = 0.0;
                err_y = 0.0;
                last_err_x = 0.0;
                last_err_y = 0.0;
            }

            flag_not_found_mark = 0;
        }
    }
}

// 飞机降落速度控制
void landingVelocityControl()
{
    vs_body_axis.header.seq++;
	vs_body_axis.header.stamp = ros::Time::now();

	dt = ros::Time::now().toSec() - last_timestamp;

    // 当 x y方向位置偏差小于阈值时(阈值与高度相关)，逐渐降落，同时x y方向在继续调整偏差(0.3待调整)
    if ((fabs(err_x) < 0.3*uav_altitude) && (fabs(err_y) < 0.3*uav_altitude))
    {
        vs_body_axis.twist.linear.z = -0.1;
    }

    else
    {
        vs_body_axis.twist.linear.z = 0.0;
    }

    // PD控制(x y方向)
    vs_body_axis.twist.linear.x = err_x * xyP + (err_x - last_err_x) / dt * xyD;
    vs_body_axis.twist.linear.y = err_y * xyP + (err_y - last_err_y) / dt * xyD;

    // 由于 x y方向的 err值与高度相关，当高度很小时，err很小，造成控制量很小，因此加大控制的 P 值
    if(uav_altitude < 1.0) 
    {
        // PD控制(x y方向)
        vs_body_axis.twist.linear.x = err_x * xyP * 1.3 + (err_x - last_err_x) / dt * xyD;              
        vs_body_axis.twist.linear.y = err_y * xyP * 1.3 + (err_y - last_err_y) / dt * xyD;
    }

    // 更新偏差值 
    last_err_x = err_x;
    last_err_y = err_y;
    //last_err_z = err_z;
    //last_err_raw = - err_yaw;
    last_timestamp = ros::Time::now().toSec();
}


/***************** 键盘控制 按下一直飞行 松开停止********************/
// 键盘按下部分
void sendflightCommand(const keyboard::Key &key)
{
   vs_body_axis.header.seq++;
   vs_body_axis.header.stamp = ros::Time::now();

   switch (key.code)
   {
        case 'q':
        {
            ROS_INFO_STREAM("manual quit, enter Position Hold");
            flag_enter_position_hold = true;
            break;
        }
        case 'p':
        {
            flag_enter_position_hold = false;
            flag_enter_manual_adjust = false;
            ROS_INFO_STREAM("manual recover Offboard Mode");
            break;
        }
        case 'm':
        {
            flag_enter_manual_adjust = true;
            ROS_INFO_STREAM("enter manual adjust");
            break;
        }
        case 'w':
        {
            ROS_INFO_STREAM("up");
            vs_body_axis.twist.linear.z = 0.1;
            break;
        }
        case 's':
        {
            ROS_INFO_STREAM("down");
            vs_body_axis.twist.linear.z = -0.1;
            break;
        }
        case 'j':
        {
            ROS_INFO_STREAM("left");
            vs_body_axis.twist.linear.x = 0.2;
            break;
        }
        case 'l':
        {
            ROS_INFO_STREAM("right");
            vs_body_axis.twist.linear.x = -0.2;
            break;
        }
        case 'i':
        {
            ROS_INFO_STREAM("forward");
            vs_body_axis.twist.linear.y = -0.2;
            break;
        }
        case 'k':
        {
            ROS_INFO_STREAM("backward");
            vs_body_axis.twist.linear.y = 0.2;
            break;
        }
        case 'z':
        {
            vs_body_axis.twist.linear.x = 0.0;
            vs_body_axis.twist.linear.y = 0.0;
            vs_body_axis.twist.linear.z = 0.0;
            vs_body_axis.twist.angular.z = 0.0;
            break;
        }
        default:
        {
            
        }
    }
}

// 键盘松开部分
void sendholdCommand(const keyboard::Key &key)
{
   vs_body_axis.header.seq++;
   vs_body_axis.header.stamp = ros::Time::now();

   switch (key.code)
   {
        case 'w':
        {
            ROS_INFO_STREAM("quit up");
            vs_body_axis.twist.linear.z = 0.0;
            break;
        }
        case 's':
        {
            ROS_INFO_STREAM("quit down");
            vs_body_axis.twist.linear.z = 0.0;
            break;
        }
        case 'j':
        {
            ROS_INFO_STREAM("quit left");
            vs_body_axis.twist.linear.x = 0.0;
            break;
        }
        case 'l':
        {
            ROS_INFO_STREAM("quit right");
            vs_body_axis.twist.linear.x = 0.0;
            break;
        }
        case 'i':
        {
            ROS_INFO_STREAM("quit forward");
            vs_body_axis.twist.linear.y = 0.0;
            break;
        }
        case 'k':
        {
            ROS_INFO_STREAM("quit backward");
            vs_body_axis.twist.linear.y = 0.0;
            break;
        }
        default:
        {
            
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_marker_land_node");
    ros::NodeHandle nh;

    // arm 与 disarm service 使用
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    int flag_time = 0;
    int flag_low_altitude = 0;

    ros::Time time_disarm(0);

    printf("------------landing control node running successfully-------------\n");

    bodyAxisVelocityPublisher = nh.advertise<geometry_msgs::TwistStamped>("/mavros04/setpoint_velocity/cmd_vel",10);
    ros::Subscriber stateSubscriber = nh.subscribe("mavros04/state", 10, stateReceived);
    ros::Subscriber markerCenterSubscriber = nh.subscribe("/aruco_single/pixel",1000,markerCenterReceived);  
    ros::Subscriber uavPoseSubscriber = nh.subscribe("/mavros04/local_position/pose", 1000, uavPoseReceived);
    ros::Subscriber uavAltitudeSubscriber = nh.subscribe("/mavros04/px4flow/ground_distance", 1000, uavAltitudeReceived);
    ros::Subscriber flightcommandSubscriber = nh.subscribe("/keyboard/keydown",1,sendflightCommand);
    ros::Subscriber holdcommandSubscriber = nh.subscribe("/keyboard/keyup",1,sendholdCommand);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros04/cmd/arming");

    last_err_x = 0;
    last_err_y = 0;
    last_err_z = 0;
    last_err_raw = 0;

    // 获取 PID 参数值
    ros::param::param("~xyP", xyP, 0.0);
    ros::param::param("~xyI", xyI, 0.0);
    ros::param::param("~xyD", xyD, 0.0);
    ros::param::param("~zP", zP, 0.0);
    ros::param::param("~zI", zI, 0.0);
    ros::param::param("~zD", zD, 0.0);

    cout << "got xyP = " << xyP << endl;
    cout << "got xyD = " << xyD << endl;

    ros::Rate loopRate(50.0);
    while(ros::ok())
    {	 
	  if(flag_offboard_mode)
      {
          // 高度大于0.35m，正常控制飞行
          if(uav_altitude >= 0.35)
          {             
              // 切换到 offboard 模式，且未进入位置保持状态
              if(flag_offboard_mode && !flag_enter_position_hold && !flag_enter_manual_adjust)
              {
                    landingVelocityControl();   
                    ROS_INFO_STREAM("Offboard auto control");     
              }
              // 在 offboard 模式下，进入位置保持状态, 未进入手动调节模式
              else if(flag_enter_position_hold && !flag_enter_manual_adjust)
              {
                    ROS_INFO_STREAM("Position hold control");
                    vs_body_axis.header.seq++;
	            vs_body_axis.header.stamp = ros::Time::now();
                    vs_body_axis.twist.linear.x = 0.0;
                    vs_body_axis.twist.linear.y = 0.0;
                    vs_body_axis.twist.linear.z = 0.0;
                    vs_body_axis.twist.angular.z = 0.0;
              }

            flag_low_altitude = 0;
          }

          // 高度小于 0.35m，继续降落，1s 后飞机锁定(防止超声波数据出错，要求低高度预警连续触发3次)
          else
          {     
               flag_low_altitude++;
               if(flag_low_altitude > 2)  
               {        
                   flag_low_altitude = 3;
		           vs_body_axis.header.seq++;
	               vs_body_axis.header.stamp = ros::Time::now();
                   vs_body_axis.twist.linear.x = 0;
                   vs_body_axis.twist.linear.y = 0;
                   vs_body_axis.twist.linear.z = -0.1;

                   if (flag_time == 0)
                   {        
                        time_disarm = ros::Time::now();
                        flag_time = 1;
                   }

                   if ((ros::Time::now() - time_disarm) > ros::Duration(1.0))
                   {
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_INFO("Vehicle disarmed");
                            return 0;
                        }
                   }
               }
          }
      }

// 此句为测试代码，不用 arm 飞机，直接看速度控制输出量 
//landingVelocityControl();

      //发布速度控制量
      bodyAxisVelocityPublisher.publish(vs_body_axis);
      
	  ros::spinOnce();
  	  loopRate.sleep();
    }
    return 0;
}
