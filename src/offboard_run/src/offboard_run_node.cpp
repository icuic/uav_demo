//一系列头文件

//ros库
#include <ros/ros.h>

//发布的位置消息体对应的头文件，该消息体的类型为geometry_msgs::PoseStamped
//用来进行发送目标位置
/*
ros官网上这样定义
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose
实际上就是一个带有头消息和位姿的消息
*/
#include <geometry_msgs/PoseStamped.h>

//CommandBool服务的头文件，该服务的类型为mavros_msgs::CommandBool
//用来进行无人机解锁
/*
其结构如下（来源于ros wiki）
# Common type for switch commands
bool value
---
bool success
uint8 result
可以看到，发送的请求是一个bool类型的数据，为True则解锁，为False则上锁
返回的响应中success是一个bool类型的参数，表示上电/断电操作是否成功执行。
如果操作成功执行，success值为True，否则为False。
result是一个int32类型的参数，表示执行上电/断电操作的结果。
如果解锁/上锁操作成功执行，result值为0，
否则为其他值，表示执行解锁/上锁操作时发生了某种错误或异常。可以根据这个数值查看是哪种问题导致
*/
#include <mavros_msgs/CommandBool.h>

//SetMode服务的头文件，该服务的类型为mavros_msgs::SetMode
//用来设置无人机的飞行模式，切换offboard
/*
wiki上的消息定义如下
# set FCU mode
#
# Known custom modes listed here:
# http://wiki.ros.org/mavros/CustomModes

# basic modes from MAV_MODE
uint8 MAV_MODE_PREFLIGHT = 0
uint8 MAV_MODE_STABILIZE_DISARMED = 80
uint8 MAV_MODE_STABILIZE_ARMED = 208
uint8 MAV_MODE_MANUAL_DISARMED = 64
uint8 MAV_MODE_MANUAL_ARMED = 192
uint8 MAV_MODE_GUIDED_DISARMED = 88
uint8 MAV_MODE_GUIDED_ARMED = 216
uint8 MAV_MODE_AUTO_DISARMED = 92
uint8 MAV_MODE_AUTO_ARMED = 220
uint8 MAV_MODE_TEST_DISARMED = 66
uint8 MAV_MODE_TEST_ARMED = 194

uint8 base_mode # filled by MAV_MODE enum value or 0 if custom_mode != ''
string custom_mode # string mode representation or integer
---
bool success

String类型的变量custom_mode就是我们想切换的模式，有如下选择：
MANUAL，ACRO，ALTCTL，POSCTL，OFFBOARD，STABILIZED，RATTITUDE，AUTO.MISSION
AUTO.LOITER，AUTO.RTL，AUTO.LAND，AUTO.RTGS，AUTO.READY，AUTO.TAKEOFF
*/
#include <mavros_msgs/SetMode.h>

//订阅的消息体的头文件，该消息体的类型为mavros_msgs::State
//查看无人机的状态
/*
wiki上是这样的
std_msgs/Header header
bool connected
bool armed
bool guided
bool manual_input
string mode
uint8 system_status

解析如下：
header：消息头，包含时间戳和框架信息；
connected：表示是否连接到了 mavros 节点；
armed：表示无人机当前是否上锁；
guided：表示无人机当前是否处于 GUIDED 模式；
mode：表示当前无人机所处的模式，包括以下几种：
MANUAL，ACRO，ALTCTL，POSCTL，OFFBOARD，STABILIZED，RATTITUDE，AUTO.MISSION
AUTO.LOITER，AUTO.RTL，AUTO.LAND，AUTO.RTGS，AUTO.READY，AUTO.TAKEOFF
*/
#include <mavros_msgs/State.h>  

//建立一个订阅消息体类型的变量，用于存储订阅的信息
mavros_msgs::State current_state;
 
//订阅时的回调函数，接受到该消息体的内容时执行里面的内容，内容是储存飞控当前的状态
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    //ros系统的初始化，argc和argv在后期节点传值会使用，最后一个参数offb_node为节点名称
    ros::init(argc, argv, "offb_node"); 

    //实例化ROS句柄，这个ros::NodeHandle类封装了ROS中的一些常用功能
    ros::NodeHandle nh;

    //这是一个订阅者对象，可以订阅无人机的状态信息（状态信息来源为MAVROS发布），用来储存无人机的状态，在回调函数中会不断更新这个状态变量
    //<>里面为模板参数，传入的是订阅的消息体类型，（）里面传入三个参数，分别是该消息体的位置、缓存大小（通常为1000）、回调函数
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    //这是一个发布者对象，用来在本地坐标系下发布目标点，后面会以20Hz频率发布目标点
    //<>里面为模板参数，传入的是发布的消息体类型，（）里面传入两个参数，分别是该消息体的位置、缓存大小（通常为1000）
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    //一个客户端，用来解锁无人机，这是因为无人机如果降落后一段时间没有收到信号输入，会自动上锁来保障安全
    //启动服务的函数为nh下的serviceClient<>()函数，<>里面是该服务的类型，（）里面是该服务的路径
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    //一个客户端，用来切换飞行模式
    //启动服务的函数为nh下的serviceClient<>()函数，<>里面是该服务的类型，（）里面是该服务的路径
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //官方要求local_pos_pub发布速率必须快于2Hz，这里设置为20Hz
    //PX4在两个Offboard命令之间有一个500ms的延时，如果超过此延时，系统会将回到无人机进入Offboard模式之前的最后一个模式。
    ros::Rate rate(20.0);

    // 等待飞控和MAVROS建立连接，current_state是我订阅的MAVROS的状态，在收到心跳包之后连接成功跳出循环
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //实例化一个geometry_msgs::PoseStamped类型的对象，并对其赋值，最后将其发布出去
    //尽管PX4在航空航天常用的NED坐标系下操控飞机，但MAVROS将自动将该坐标系切换至常规的ENU坐标系下
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //在进入Offboard模式之前，必须已经启动了local_pos_pub数据流，否则模式切换将被拒绝。
    //这里的100可以被设置为任意数
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //建立一个类型为SetMode的服务端offb_set_mode，并将其中的模式mode设为"OFFBOARD"，作用便是用于后面的客户端与服务端之间的通信（服务）
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    //建立一个类型为CommandBool的服务端arm_cmd，并将其中的是否解锁设为"true"，作用便是用于后面的客户端与服务端之间的通信（服务）
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //更新时间
    ros::Time last_request = ros::Time::now();

    double radius;
    double angular_speed;
    double current_angle;

    //大循环，只要节点还在ros::ok()的值就为正
    while(ros::ok())
    {
        //首先判断当前模式是否为offboard模式，如果不是，则进入if语句内部
        //这里是5秒钟进行一次判断，避免飞控被大量的请求阻塞
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            //客户端set_mode_client向服务端offb_set_mode发起请求call，然后服务端回应response将模式返回，这就打开了offboard模式
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                //打开Offboard模式后在终端打印信息
                ROS_INFO("Offboard enabled");
            }
            //更新时间
            last_request = ros::Time::now();
        }

        //else指已经为offboard模式，则进入if语句内部
        else 
        {
            //判断当前状态是否解锁，如果没有解锁，则进入if语句内部
            //这里是5秒钟进行一次判断，避免飞控被大量的请求阻塞
            if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                //客户端arming_client向服务端arm_cmd发起请求call，然后服务端回应response成功解锁，则解锁成功
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    //解锁后在终端打印信息
                    ROS_INFO("Vehicle armed");
                }
                //更新时间
                last_request = ros::Time::now();
            }
        }



        // 设置圆周运动参数
        radius = 2.0; // 圆周半径
        angular_speed = 0.2; // 角速度，控制绕圆周的速度

        // 更新无人机当前位置
        current_angle += angular_speed * 0.1; // 0.1秒的时间间隔
        double x = radius * cos(current_angle);
        double y = radius * sin(current_angle);  

        pose.pose.position.x = x;
        pose.pose.position.y = y;              

        //发布位置信息，所以综上飞机只有先打开offboard模式然后解锁才能飞起来
        local_pos_pub.publish(pose); 
        
        //当spinOnce函数被调用时，会调用回调函数队列中第一个回调函数，这里回调函数是state_cb函数
        ros::spinOnce();

        //根据前面ros::Rate rate(20.0);制定的发送频率自动休眠 休眠时间 = 1/频率
        rate.sleep();
    }

    return 0;
}

