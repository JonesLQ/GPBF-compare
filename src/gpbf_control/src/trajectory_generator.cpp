#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cmath>

class TrajectoryGenerator {
public:
    TrajectoryGenerator(ros::NodeHandle& nh) : nh_(nh) {
        // 从参数服务器获取轨迹参数
        nh_.param<double>("traj/a", a_, 4.0);            // x方向幅值
        nh_.param<double>("traj/b", b_, 2.0);            // y方向幅值
        nh_.param<double>("traj/h", h_, 1.5);            // 高度
        nh_.param<double>("traj/omega", omega_, 0.1);    // 角频率

        // 创建轨迹目标点发布器
        setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        
        // 创建可视化路径发布器
        path_pub_ = nh_.advertise<nav_msgs::Path>("/trajectory/path", 10);
        
        // 创建轨迹生成定时器
        double rate;
        nh_.param<double>("traj/rate", rate, 20.0);  // 默认20Hz
        timer_ = nh_.createTimer(ros::Duration(1.0/rate), &TrajectoryGenerator::timerCallback, this);
        
        // 初始化时间
        start_time_ = ros::Time::now();
        
        // 初始化路径消息
        path_msg_.header.frame_id = "map";
        
        // 预先计算完整路径用于可视化
        precomputePath();
        
        ROS_INFO("Trajectory generator initialized with parameters: a=%.2f, b=%.2f, h=%.2f, omega=%.2f", 
                 a_, b_, h_, omega_);
    }
    
    void precomputePath() {
        // 计算8字形的整个周期
        double period = 2.0 * M_PI / omega_;
        int points = 200;  // 路径上的点数
        
        for (int i = 0; i < points; i++) {
            double t = i * period / points;
            
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            
            // 8字轨迹位置
            pose.pose.position.x = a_ * sin(omega_ * t);
            pose.pose.position.y = b_ * sin(2 * omega_ * t);
            pose.pose.position.z = h_;
            
            // 简单的航向角设置（朝向运动方向）
            double dx = a_ * omega_ * cos(omega_ * t);
            double dy = 2 * b_ * omega_ * cos(2 * omega_ * t);
            double yaw = atan2(dy, dx);
            
            // 四元数表示航向
            double cy = cos(yaw * 0.5);
            double sy = sin(yaw * 0.5);
            pose.pose.orientation.w = cy;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = sy;
            
            path_msg_.poses.push_back(pose);
        }
    }
    
    void timerCallback(const ros::TimerEvent& event) {
        // 计算当前时间
        ros::Time now = ros::Time::now();
        double t = (now - start_time_).toSec();
        
        // 创建位置消息
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = now;
        pose.header.frame_id = "map";
        
        // 8字轨迹位置
        pose.pose.position.x = a_ * sin(omega_ * t);
        pose.pose.position.y = b_ * sin(2 * omega_ * t);
        pose.pose.position.z = h_;
        
        // 简单的航向角设置（朝向运动方向）
        double dx = a_ * omega_ * cos(omega_ * t);
        double dy = 2 * b_ * omega_ * cos(2 * omega_ * t);
        double yaw = atan2(dy, dx);
        
        // 四元数表示航向
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        pose.pose.orientation.w = cy;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = sy;
        
        // 发布目标点
        setpoint_pub_.publish(pose);
        
        // 更新并发布路径可视化
        path_msg_.header.stamp = now;
        path_pub_.publish(path_msg_);
    }
    
private:
    ros::NodeHandle nh_;
    ros::Publisher setpoint_pub_;
    ros::Publisher path_pub_;
    ros::Timer timer_;
    
    double a_;      // x方向幅值
    double b_;      // y方向幅值
    double h_;      // 高度
    double omega_;  // 角频率
    
    ros::Time start_time_;
    nav_msgs::Path path_msg_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;
    
    TrajectoryGenerator trajectory_generator(nh);
    
    ros::spin();
    
    return 0;
}
