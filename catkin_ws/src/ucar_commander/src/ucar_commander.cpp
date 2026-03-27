#include <ucar_commander.h>

// 生成目标点
geometry_msgs::PoseStamped goal(float pose_x, float pose_y, float pose_z, float pose_yaw)
{
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time();
    goal.pose.position.x = pose_x;
    goal.pose.position.y = pose_y;
    goal.pose.position.z = pose_z;
    geometry_msgs::Quaternion map_q;
    double roll, pitch, yaw;
    roll = 0.0;
    pitch = 0.0;
    yaw = pose_yaw;
    map_q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    goal.pose.orientation = map_q;
    return goal;
}

// odom回调函数
void recognitionCommand::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_odom_ = *msg;
}

// PID控制上坡 (可用pid也可不用)
bool recognitionCommand::moveDistancePID(double distance, double max_speed, char axis, double tol) {

}

//发布目标点，包含超时取消
bool recognitionCommand::send_goal(geometry_msgs::PoseStamped goal) {

}

// 去停车位A、B、C
bool recognitionCommand::parking_car(){

}

//qr码识别
bool recognitionCommand::qr_recognition(){

}

// 计时器开关启动控制
bool recognitionCommand::handle_nav_start(bool if_start){
    std_srvs::SetBool srv;                   // 构造 std_srvs::SetBool 服务消息（包含 request 和 response）
    srv.request.data = if_start;             // 将传入的布尔值设置到请求的 data 字段（true 表示启动，false 表示停止）
    if (nav_start_client.call(srv)) {        // 使用事先创建的服务客户端调用 /start_stop_service
        return true;                         
    }
    return false;   
}

//状态变换函数 用于控制状态机决策
void recognitionCommand::update_state(){
    if(game_state == 0){
        setlocale(LC_ALL, "");
        ROS_INFO("等待开始");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ucar_commander");
    ros::NodeHandle nh;
    recognitionCommand rec;
    ros::Rate rate(5.0);
    while(ros::ok()){
        rec.update_state();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}