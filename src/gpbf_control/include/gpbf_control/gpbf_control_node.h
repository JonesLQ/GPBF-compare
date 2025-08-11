#ifndef GPBF_CONTROL_NODE_H
#define GPBF_CONTROL_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <gpbf_control/gp_model.h>
#include <gpbf_control/pbf_controller.h>
#include <gpbf_control/predictive_cbf.h>
#include <gpbf_msgs/ObstacleArray.h>

#include <memory>
#include <Eigen/Dense>

namespace gpbf {

class GPBFControlNode {
public:
    /**
     * 构造函数
     * @param nh ROS节点句柄
     */
    GPBFControlNode(const ros::NodeHandle& nh);
    
    /**
     * 析构函数
     */
    ~GPBFControlNode();
    
    /**
     * 初始化节点
     * @return 是否初始化成功
     */
    bool initialize();
    
    /**
     * 运行节点
     */
    void run();
    
private:
    // 回调函数
    /**
     * 状态回调函数
     * @param msg 里程计消息
     */
    void stateCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    /**
     * 目标点回调函数
     * @param msg 位姿消息
     */
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    /**
     * 障碍物回调函数
     * @param msg 障碍物数组消息
     */
    void obstacleCallback(const gpbf_msgs::ObstacleArray::ConstPtr& msg);
    
    /**
     * 控制循环定时器回调
     * @param event 定时器事件
     */
    void controlLoopCallback(const ros::TimerEvent& event);
    
    /**
     * 发布控制命令
     * @param control 控制输入
     */
    void publishControl(const Eigen::VectorXd& control);
    
    /**
     * 估计当前干扰
     * @param state 当前状态
     * @param control 上一个控制输入
     * @return 干扰估计
     */
    Eigen::VectorXd estimateDisturbance(const Eigen::VectorXd& state, 
                                      const Eigen::VectorXd& control);
    
    // ROS接口
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber obstacle_sub_;
    ros::Publisher control_pub_;
    ros::Timer control_timer_;
    
    // TF监听器
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 控制器组件
    std::unique_ptr<GPModel> gp_model_;
    std::unique_ptr<PBFController> pbf_controller_;
    std::unique_ptr<PredictiveCBF> predictive_cbf_;
    
    // 系统状态
    Eigen::VectorXd current_state_;         // 当前ROM状态 [p, v]
    Eigen::VectorXd current_attitude_;      // 当前姿态状态 [R, w]
    Eigen::VectorXd desired_position_;      // 目标位置
    Eigen::VectorXd last_control_;          // 上一次控制输入
    std::vector<Obstacle> obstacles_;       // 障碍物列表
    ros::Time last_state_time_;             // 上一次状态更新时间
    
    // 控制配置
    double control_rate_;                   // 控制频率
    std::string base_frame_;                // 基准坐标系
    
    // 调试数据
    struct DebugData {
        Eigen::VectorXd disturbance_mean;
        Eigen::VectorXd disturbance_var;
        double predictive_margin;
    } debug_data_;
    
    // 系统状态
    bool initialized_;
    bool received_state_;
    bool received_goal_;
};

} // namespace gpbf

#endif // GPBF_CONTROL_NODE_H