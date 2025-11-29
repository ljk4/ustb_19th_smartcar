#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <map>
#include <algorithm>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <vector>
#include <string>
#include <std_srvs/SetBool.h> //计时器开关服务
#include <qr_msgs/qr.h>  //QR码识别
#include <sound_play/SoundRequest.h>  //语音播放
#include <sound_play/sound_play.h>
#include <nav_msgs/Odometry.h> //里程计消息
#include <geometry_msgs/Twist.h> //速度消息

class recognitionCommand{
public:
    recognitionCommand();
    bool send_goal(geometry_msgs::PoseStamped goal);//发送目的地
    bool qr_recognition();  //qr识别
    bool handle_nav_start(bool if_start);  //计时器开关启动控制
    void update_state();  //状态变换函数
    bool moveDistancePID(double distance, double max_speed, char axis = 'x', double tol = 0.02); //PID控制 
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg); //odom回调函数
    bool parking_car(); //去停车位
    nav_msgs::Odometry current_odom_; // 当前里程计信息 即当前位姿
    int game_state = -1; //状态变换
    int qr_cal[2];  // 0:货物 1~2:计算送达到的位置
    int qr_count = 0; //计算取到第几个qr码

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;  // move_base客户端
    ros::ServiceClient nav_start_client; // 计时器客户端  /start_stop_service（服务）
    ros::ServiceClient qr_client; // qr码识别客户端 /qr/scan_qr（服务）
    ros::Publisher cmd_vel_pub; // 速度发布器  /cmd_vel（话题）
    ros::Subscriber odom_sub;  // 里程计订阅器 /odom （话题）
};

recognitionCommand::recognitionCommand() : nh_(), ac_("/move_base", true) {
    ac_.waitForServer(ros::Duration(3)); // 等待move_base服务器连接
    // 计时器客户端  /start_stop_service（服务）
    nav_start_client = nh_.serviceClient<std_srvs::SetBool>("/start_stop_service");
    // qr码识别客户端 /qr/scan_qr（服务）

    // 速度发布器  /cmd_vel（话题）

    // 里程计订阅器 /odom （话题）

}
