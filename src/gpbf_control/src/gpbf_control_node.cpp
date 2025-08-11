#include <gpbf_control/gpbf_control_node.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
// #include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace gpbf {

GPBFControlNode::GPBFControlNode(const ros::NodeHandle& nh) 
  : nh_(nh), tf_listener_(tf_buffer_), initialized_(false), received_state_(false), received_goal_(false) {
  
    // 设置默认状态
    current_state_ = Eigen::VectorXd::Zero(6);      // [x, y, z, vx, vy, vz]
    current_attitude_ = Eigen::VectorXd::Zero(12);  // [R(9), wx, wy, wz]
    desired_position_ = Eigen::VectorXd::Zero(3);   // [x, y, z]
    last_control_ = Eigen::VectorXd::Zero(3);       // [ax, ay, az]
    
    // 获取参数
    nh_.param<double>("control/rate", control_rate_, 50.0);
    nh_.param<std::string>("frames/base", base_frame_, "map");
    
    ROS_INFO("GPBF Control Node initialized");
}

GPBFControlNode::~GPBFControlNode() {
    ROS_INFO("GPBF Control Node destroyed");
}

bool GPBFControlNode::initialize() {
    // 创建组件
    gp_model_ = std::make_unique<GPModel>(nh_);
    pbf_controller_ = std::make_unique<PBFController>(nh_);
    predictive_cbf_ = std::make_unique<PredictiveCBF>(nh_);
    
    // 初始化组件
    std::vector<double> length_scales = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    double signal_var = 0.1;
    double noise_var = 0.01;
    
    if (!gp_model_->initialize(length_scales, signal_var, noise_var)) {
        ROS_ERROR("Failed to initialize GP Model");
        return false;
    }
    
    if (!pbf_controller_->initialize()) {
        ROS_ERROR("Failed to initialize PBF Controller");
        return false;
    }
    
    if (!predictive_cbf_->initialize()) {
        ROS_ERROR("Failed to initialize Predictive CBF");
        return false;
    }
    
    // 创建订阅器
    state_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &GPBFControlNode::stateCallback, this);
    goal_sub_ = nh_.subscribe("/mavros/setpoint_position/local", 10, &GPBFControlNode::goalCallback, this);
    obstacle_sub_ = nh_.subscribe("/obstacles", 10, &GPBFControlNode::obstacleCallback, this);
    
    // 创建发布器
    // control_pub_ = nh_.advertise<geometry_msgs::Accel>("/mavros/setpoint_accel/accel", 10);
    // control_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_accel/accel", 10);
    control_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/mavros/setpoint_accel/accel", 10);
    control_timer_ = nh_.createTimer(ros::Duration(1.0/control_rate_), &GPBFControlNode::controlLoopCallback, this);
    
    initialized_ = true;
    return true;
}

void GPBFControlNode::run() {
    if (!initialized_) {
        ROS_ERROR("Attempted to run before initialization");
        return;
    }
    
    ROS_INFO("GPBF Control Node running");
    ros::spin();
}

void GPBFControlNode::stateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 提取位置和速度
    Eigen::Vector3d position(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );
    
    Eigen::Vector3d velocity(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z
    );
    
    // 提取姿态信息
    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
    );
    
    Eigen::Matrix3d rotation = q.toRotationMatrix();
    
    // 提取角速度
    Eigen::Vector3d angular_velocity(
        msg->twist.twist.angular.x,
        msg->twist.twist.angular.y,
        msg->twist.twist.angular.z
    );
    
    // 更新当前状态
    current_state_.head(3) = position;
    current_state_.tail(3) = velocity;
    
    // 更新姿态状态
    current_attitude_.segment(0, 3) = rotation.col(0);
    current_attitude_.segment(3, 3) = rotation.col(1);
    current_attitude_.segment(6, 3) = rotation.col(2);
    current_attitude_.tail(3) = angular_velocity;
    
    // 记录时间戳
    last_state_time_ = msg->header.stamp;
    
    received_state_ = true;
}

void GPBFControlNode::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // 更新目标位置
    desired_position_(0) = msg->pose.position.x;
    desired_position_(1) = msg->pose.position.y;
    desired_position_(2) = msg->pose.position.z;
    
    received_goal_ = true;
}

void GPBFControlNode::obstacleCallback(const gpbf_msgs::ObstacleArray::ConstPtr& msg) {
    // 清空当前障碍物列表
    obstacles_.clear();
    
    // 添加新障碍物
    for (const auto& obs_msg : msg->obstacles) {
        Obstacle obs;
        obs.position = Eigen::Vector3d(obs_msg.position.x, obs_msg.position.y, obs_msg.position.z);
        obs.radius = obs_msg.radius;
        obs.id = obs_msg.id;
        
        obstacles_.push_back(obs);
    }
}

void GPBFControlNode::controlLoopCallback(const ros::TimerEvent& event) {
    // 检查是否收到必要数据
    if (!received_state_ || !received_goal_) {
        ROS_WARN_THROTTLE(1.0, "Waiting for state/goal data...");
        return;
    }
    
    try {
        // 估计当前干扰
        Eigen::VectorXd disturbance = estimateDisturbance(current_state_, last_control_);
        
        // 更新GP模型
        gp_model_->updateTrainingData(current_state_, disturbance);
        
        // 获取干扰预测
        Eigen::VectorXd mean, variance;
        gp_model_->predictDisturbance(current_state_, mean, variance);
        
        // 预测ROM-FOM安全裕度
        double predictive_margin = predictive_cbf_->predictSafetyMargin(current_state_, current_attitude_);
        
        // 设置控制器参数
        pbf_controller_->setDisturbancePrediction(mean, variance);
        pbf_controller_->setPredictiveMargin(predictive_margin);
        
        // 计算PD控制器标称控制输入
        Eigen::VectorXd position_error = desired_position_ - current_state_.head(3);
        Eigen::VectorXd velocity = current_state_.tail(3);
        
        // 简单PD控制器
        double kp = 1.0;
        double kd = 0.5;
        Eigen::VectorXd nominal_control = kp * position_error - kd * velocity;
        
        // 计算安全控制输入
        Eigen::VectorXd safe_control;
        if (pbf_controller_->computeSafeControl(current_state_, nominal_control, desired_position_, obstacles_, safe_control)) {
            // 发布控制输入
            publishControl(safe_control);
            
            // 保存控制输入供下次干扰估计使用
            last_control_ = safe_control;
            
            // 发布安全信息
            pbf_controller_->publishSafetyInfo(obstacles_, current_state_);
        } else {
            ROS_WARN("Failed to compute safe control, using nominal control");
            publishControl(nominal_control);
            last_control_ = nominal_control;
        }
        
        // 发布干扰预测结果
        gp_model_->publishDisturbancePrediction(current_state_);
    }
    catch (const std::exception& e) {
        ROS_ERROR("Control loop exception: %s", e.what());
    }
}

Eigen::VectorXd GPBFControlNode::estimateDisturbance(const Eigen::VectorXd& state, const Eigen::VectorXd& control) {
    // 计算两次状态更新之间的时间差
    static ros::Time prev_time = ros::Time::now();
    ros::Time curr_time = last_state_time_;
    double dt = (curr_time - prev_time).toSec();
    prev_time = curr_time;
    
    // 无效时间差，返回零干扰
    if (dt <= 0.0 || dt > 0.1) {
        return Eigen::VectorXd::Zero(3);
    }
    
    // 提取当前速度
    Eigen::Vector3d velocity = state.tail(3);
    
    // 上一个控制输入应该导致的加速度
    Eigen::Vector3d expected_accel = last_control_;
    
    // 重力补偿 (假设z轴向上)
    expected_accel(2) -= 9.81;
    
    // 估计实际加速度 (速度导数)
    static Eigen::Vector3d prev_vel = velocity;
    Eigen::Vector3d actual_accel = (velocity - prev_vel) / dt;
    prev_vel = velocity;
    
    // 干扰 = 实际加速度 - 期望加速度
    Eigen::VectorXd disturbance = actual_accel - expected_accel;
    
    // 简单滤波去除噪声
    static Eigen::VectorXd filtered_disturbance = Eigen::VectorXd::Zero(3);
    double alpha = 0.1;  // 滤波系数
    filtered_disturbance = alpha * disturbance + (1 - alpha) * filtered_disturbance;
    
    return filtered_disturbance;
}

// void GPBFControlNode::publishControl(const Eigen::VectorXd& control) {
//     // 创建加速度消息
//     geometry_msgs::Accel msg;
//     msg.header.stamp = ros::Time::now();
//     msg.header.frame_id = base_frame_;
    
//     // 填充加速度
//     msg.linear.x = control(0);
//     msg.linear.y = control(1);
//     msg.linear.z = control(2);
    
//     // 发布消息
//     control_pub_.publish(msg);
// }



// void GPBFControlNode::publishControl(const Eigen::VectorXd& control) {
//     // 创建TwistStamped消息
//     geometry_msgs::TwistStamped msg;
//     msg.header.stamp = ros::Time::now();
//     msg.header.frame_id = base_frame_;
    
//     // 填充线性加速度到linear字段
//     msg.twist.linear.x = control(0);
//     msg.twist.linear.y = control(1);
//     msg.twist.linear.z = control(2);
    
//     // 角加速度设为0
//     msg.twist.angular.x = 0;
//     msg.twist.angular.y = 0;
//     msg.twist.angular.z = 0;
    
//     // 发布消息
//     control_pub_.publish(msg);
//     }



void GPBFControlNode::publishControl(const Eigen::VectorXd& control) {
    // 创建Vector3Stamped消息
    geometry_msgs::Vector3Stamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = base_frame_;
        
    // 填充加速度到vector字段
    msg.vector.x = control(0);
    msg.vector.y = control(1);
    msg.vector.z = control(2);
        
    // 发布消息
    control_pub_.publish(msg);
    }




} // namespace gpbf

int main(int argc, char** argv) {
    ros::init(argc, argv, "gpbf_control_node");
    ros::NodeHandle nh;
    
    gpbf::GPBFControlNode node(nh);
    if (node.initialize()) {
        node.run();
    } else {
        ROS_ERROR("Failed to initialize GPBF Control Node");
    }
    
    return 0;
}
