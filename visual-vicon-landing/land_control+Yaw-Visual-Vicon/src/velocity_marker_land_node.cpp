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

#define PI 3.1415926
// 主点坐标
# define U0 319.0
# define V0 220.0
// 归一化焦距
# define FOCAL_X 487.0
# define FOCAL_Y 488.0

geometry_msgs::TwistStamped vs_body_axis;
geometry_msgs::PoseStamped uavCurrentLocalPose,uavCurrentViconPose;
geometry_msgs::PoseStamped ps;
geometry_msgs::TransformStamped vicon_CurrentPos;

ros::Publisher bodyAxisVelocityPublisher;
ros::Publisher currentViconPositionPublisher;
ros::Publisher setPositionPublisher;
ros::ServiceClient arming_client;

double tmpRoll,tmpPitch,tmpYaw;
double Yaw_Init, Yaw_Curr;

double err_x, err_y, err_z, err_Yaw;
double last_err_x, last_err_y, last_err_z;
double last_err_roll, last_err_pitch, last_err_raw;
double last_timestamp;

double xyP, xyI, xyD, zP, zI, zD, yawP, yawD;

float uav_init_altitude = 0.0;
float uav_altitude = 0.0;     
float uav_x_distance = 0.0;
float uav_y_distance = 0.0;

bool flag_offboard_mode = false;
bool flag_enter_position_hold = false;
bool flag_enter_manual_adjust = false;
bool isLockLocalPos = false;
bool isVelMode = false;
bool has_set = false;

float pos_value;
double dt;

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
    static bool Yaw_Init_flag = false;

    uavCurrentLocalPose.pose.position.x = msg->pose.position.x;
    uavCurrentLocalPose.pose.position.y = msg->pose.position.y;
    uavCurrentLocalPose.pose.position.z = msg->pose.position.z;
    uavCurrentLocalPose.pose.orientation.x = msg->pose.orientation.x;
    uavCurrentLocalPose.pose.orientation.y = msg->pose.orientation.y;
    uavCurrentLocalPose.pose.orientation.z = msg->pose.orientation.z;
    uavCurrentLocalPose.pose.orientation.w = msg->pose.orientation.w;

    // 无人机在地面初始高度约为 0.15m
    uav_altitude = uavCurrentLocalPose.pose.position.z - 0.15;

    // 将四元数转换为欧拉角形式
    tf::Quaternion quat;
    tf::quaternionMsgToTF(uavCurrentLocalPose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(tmpRoll, tmpPitch, tmpYaw);

    //ROS_INFO("Roll,Pitch,Yaw:[%0.3f,%0.3f,%0.3f]",tmpRoll/3.14*180,tmpPitch/3.14*180,tmpYaw/3.14*180);

    // 计算 Yaw 偏差
    if(!Yaw_Init_flag)
    {
        Yaw_Init = tmpYaw;
        Yaw_Init_flag = true;
    }
    else
    {
        Yaw_Curr = tmpYaw;
    }

    err_Yaw = Yaw_Init - Yaw_Curr;

    // Yaw 偏差限幅
    if (err_Yaw < -PI)
    {
        err_Yaw = err_Yaw + 2 * PI; 
    }
    if (err_Yaw > PI)
    {
        err_Yaw = err_Yaw - 2 * PI; 
    }

    // 判断当前位置是否锁定
    if (!isLockLocalPos)
    {
        ps = uavCurrentLocalPose;
        has_set = true;
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
    vicon_CurrentPos = *vicon_msg;
    PosetopicTransport(vicon_CurrentPos, uavCurrentViconPose);
}

// 获取 aruco 坐标中心，并计算无人机相对 x y 距离 
void markerCenterReceived(const geometry_msgs::Point& msg)
{	
	static int flag_not_found_mark = 0;
    static geometry_msgs::Point markcenter;
	// 获取 aruco 坐标中心
	markcenter.x = msg.x;
	markcenter.y = msg.y;

    // 当标志中心坐标有效时，计算无人机相对标志 x y 距离((0,0)表明未检测到Tag,属于无效值)
    if (markcenter.x > 0.01f && markcenter.y > 0.01f) 
    {
    	uav_x_distance = uav_altitude*(markcenter.x-U0)/FOCAL_X;
    	uav_y_distance = uav_altitude*(markcenter.y-V0)/FOCAL_Y;

        // 将无人机与Tag在 x y z 方向的距离偏差，分别表示为err_ ,便于控制部分的理解
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

    // 当 x y方向位置偏差小于阈值时(阈值与高度相关)，逐渐降落，同时x y方向在继续调整偏差(0.23是实验的结果，可调整)
    if ((fabs(err_x) < 0.23*uav_altitude) && (fabs(err_y) < 0.23*uav_altitude))
    {
        vs_body_axis.twist.linear.z = -0.15;
    }
    else
    {
        vs_body_axis.twist.linear.z = 0.0;
    }

    // PD控制(x y方向)  P控制(偏航角Yaw)
    vs_body_axis.twist.linear.x = err_x * xyP + (err_x - last_err_x) / dt * xyD;
    vs_body_axis.twist.linear.y = err_y * xyP + (err_y - last_err_y) / dt * xyD;

    vs_body_axis.twist.angular.x = 0;
    vs_body_axis.twist.angular.y = 0;
    vs_body_axis.twist.angular.z = err_Yaw * yawP;

    // 由于 x y方向的 err值与高度相关，当高度很小时，err很小，造成控制量很小，因此加大控制的 P 值
    if(uav_altitude < 1.0) 
    {
        // PD控制(x y方向)
        vs_body_axis.twist.linear.x = err_x * xyP * 1.3 + (err_x - last_err_x) / dt * xyD;              
        vs_body_axis.twist.linear.y = err_y * xyP * 1.3 + (err_y - last_err_y) / dt * xyD;
    }

    // 速度限幅
    if(vs_body_axis.twist.linear.x > 0.8)
        vs_body_axis.twist.linear.x = 0.8;
    if(vs_body_axis.twist.linear.x < -0.8)
        vs_body_axis.twist.linear.x = -0.8;   
    if(vs_body_axis.twist.linear.y > 0.8)
        vs_body_axis.twist.linear.y = 0.8;
    if(vs_body_axis.twist.linear.y < -0.8)
        vs_body_axis.twist.linear.y = -0.8;  
    if(vs_body_axis.twist.angular.z > 0.4)
        vs_body_axis.twist.angular.z = 0.4;
    if(vs_body_axis.twist.angular.z < -0.4)
        vs_body_axis.twist.angular.z = -0.4;    
    
    // 更新偏差值   
    last_err_x = err_x;
    last_err_y = err_y;
    //last_err_z = err_z;
    //last_err_Yaw = - err_Yaw;
    last_timestamp = ros::Time::now().toSec();
}


/***************** 键盘控制 ********************/
void sendflightCommand(const keyboard::Key &key)
{
   vs_body_axis.header.seq++;
   vs_body_axis.header.stamp = ros::Time::now();

   switch (key.code)
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
        case 'v':
        {
            // enter velocity control
            isVelMode = true;
            flag_enter_position_hold = false;
            break;
        }
        case 'p':
        {
            // back to position control
            isVelMode = false;
            isLockLocalPos = false;
            flag_enter_position_hold = false;
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
    ros::Time time_disarm(0);

    bool time_init_flag = false;
    int flag_low_altitude = 0;

    printf("------------landing control node running successfully-------------\n");

    setPositionPublisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros04/setpoint_position/local",10);   
    bodyAxisVelocityPublisher = nh.advertise<geometry_msgs::TwistStamped>("/mavros04/setpoint_velocity/cmd_vel",10);
    currentViconPositionPublisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros04/mocap/pose",10);
    ros::Subscriber stateSubscriber = nh.subscribe("mavros04/state", 10, stateReceived);
    ros::Subscriber markerCenterSubscriber = nh.subscribe("/aruco_single/pixel",1000,markerCenterReceived);  
    ros::Subscriber uavPoseSubscriber = nh.subscribe("/mavros04/local_position/pose", 1000, localPositionReceived);
    ros::Subscriber viconPositionSubsciber = nh.subscribe("/vicon/nuflie04/nuflie04", 10, viconPositionReceived);    
    // ros::Subscriber uavAltitudeSubscriber = nh.subscribe("/mavros04/px4flow/ground_distance", 1000, uavAltitudeReceived);
    ros::Subscriber flightcommandSubscriber = nh.subscribe("/keyboard/keydown",1,sendflightCommand);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros04/cmd/arming");

    last_err_x = 0;
    last_err_y = 0;
    last_err_z = 0;
    last_err_raw = 0;

    pos_value = 0.2f;

    Yaw_Init = 0.0;
    Yaw_Curr = 0.0;
    err_Yaw = 0.0;

    // 获取 PID 参数值
    ros::param::param("~xyP", xyP, 0.0);
    ros::param::param("~xyI", xyI, 0.0);
    ros::param::param("~xyD", xyD, 0.0);
    ros::param::param("~zP", zP, 0.0);
    ros::param::param("~zI", zI, 0.0);
    ros::param::param("~zD", zD, 0.0);
    ros::param::param("~yawP", yawP, 0.0);

    cout << "got xyP = " << xyP << endl;
    cout << "got xyD = " << xyD << endl;
    cout << "got yawP = " << yawP << endl;

    ros::Rate loopRate(50.0);
    while(ros::ok())
    {	 
	  if(has_set)
      {
           if(!isVelMode)
           {
               ROS_INFO_STREAM("Pos Control Mode");
               ps.header.seq++;
               ps.header.stamp = ros::Time::now();
               setPositionPublisher.publish(ps);
           }  
           else 
           {  
              ROS_INFO_STREAM("Vel Control Mode");
              // 高度大于0.05m，正常控制飞行
              if(uav_altitude >= 0.05)
              {             
                  // 切换到 offboard 模式，且未进入位置保持状态
                  if(flag_offboard_mode && !flag_enter_position_hold)
                  {
                        landingVelocityControl();   
                        ROS_INFO_STREAM("Offboard auto control");     
                  }
                  // 在 offboard 模式下，进入位置保持状态
                  else if(flag_enter_position_hold)
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

              // 高度小于 0.05m，继续降落，0.3s后飞机disarm (防止Vicon数据出错，要求低高度预警连续触发2次)
              else
              {     
                   flag_low_altitude++;
                   if(flag_low_altitude > 1)  
                   {        
                       flag_low_altitude = 2;
    		           vs_body_axis.header.seq++;
    	               vs_body_axis.header.stamp = ros::Time::now();
                       vs_body_axis.twist.linear.x = 0.0;
                       vs_body_axis.twist.linear.y = 0.0;
                       vs_body_axis.twist.linear.z = -0.15;
                       vs_body_axis.twist.angular.z = 0.0;

                       if (!time_init_flag)
                       {        
                            time_disarm = ros::Time::now();
                            time_init_flag = true;
                       }

                       if ((ros::Time::now() - time_disarm) > ros::Duration(0.3))
                       {
                            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                            {
                                ROS_INFO("Vehicle disarmed");
                                return 0;
                            }
                       }
                   }
              }

              // 此句为测试代码，不用 arm 飞机，直接看速度控制输出量 
              // landingVelocityControl();

              //发布速度控制量
              bodyAxisVelocityPublisher.publish(vs_body_axis);
          }
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
