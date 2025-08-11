#include <gpbf_control/pbf_controller.h>
#include <cmath>

namespace gpbf {

PBFController::PBFController(const ros::NodeHandle& nh) : nh_(nh) {
    // 初始化QP求解器
    qp_solver_ = std::make_unique<QPSolver>();
    
    // 初始化干扰预测
    disturbance_mean_ = Eigen::VectorXd::Zero(3);
    disturbance_var_ = Eigen::VectorXd::Ones(3) * 0.1;  // 默认方差
    
    // 初始化预测性CBF裕度
    predictive_margin_ = 0.0;
    
    // 创建安全信息发布器
    safety_pub_ = nh_.advertise<gpbf_msgs::SafetyInfo>("safety_info", 10);
    
    ROS_INFO("PBF Controller initialized");
}

PBFController::~PBFController() {
    ROS_INFO("PBF Controller destroyed");
}

bool PBFController::initialize() {
    // 加载参数
    nh_.param<double>("pbf/alpha_obs", alpha_obs_, 5.0);
    nh_.param<double>("pbf/alpha_obs_nominal", alpha_obs_nominal_, 5.0);
    nh_.param<double>("pbf/alpha_track", alpha_track_, 2.0);
    nh_.param<double>("pbf/r_safe_nominal", r_safe_nominal_, 0.5);
    nh_.param<double>("pbf/beta_uncertain", beta_uncertain_, 0.5);
    nh_.param<double>("pbf/track_margin", track_margin_, 0.5);
    nh_.param<double>("pbf/vel_scale", vel_scale_, 0.2);
    
    // 系统参数
    nh_.param<double>("system/mass", mass_, 2.5);
    nh_.param<double>("system/gravity", gravity_, 9.81);
    
    ROS_INFO("PBF Controller parameters loaded");
    ROS_INFO("alpha_obs: %.2f, r_safe_nominal: %.2f, beta_uncertain: %.2f", 
            alpha_obs_, r_safe_nominal_, beta_uncertain_);
    
    return true;
}

bool PBFController::computeSafeControl(const Eigen::VectorXd& state, 
                                     const Eigen::VectorXd& nominal_control,
                                     const Eigen::VectorXd& goal_position,
                                     const std::vector<Obstacle>& obstacles,
                                     Eigen::VectorXd& safe_control) {
    // 检查输入维度
    if (state.size() != 6 || nominal_control.size() != 3 || goal_position.size() != 3) {
        ROS_ERROR("Invalid input dimensions: state=%ld, nominal_control=%ld, goal=%ld", 
                 state.size(), nominal_control.size(), goal_position.size());
        return false;
    }
    
    // 提取状态
    Eigen::Vector3d position = state.head(3);
    Eigen::Vector3d velocity = state.tail(3);
    
    // 计算控制输入维度
    int n_vars = 3;  // [ax, ay, az]
    
    // 计算约束数量：障碍物约束 + 轨迹跟踪约束
    int n_obstacles = obstacles.size();
    int n_constraints = n_obstacles + 1;
    
    // 构建QP目标函数：min 0.5 * u^T * H * u + f^T * u
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(n_vars, n_vars);
    Eigen::VectorXd f = -nominal_control;
    
    // 构建约束矩阵和向量：A * u <= b
    Eigen::MatrixXd A(n_constraints, n_vars);
    Eigen::VectorXd b(n_constraints);
    
    // 清空CBF值列表
    cbf_values_.clear();
    
    // 动态调整参数
    // 基于不确定性的安全距离增益
    double uncertainty_factor = disturbance_var_.mean();
    current_safety_margin_ = r_safe_nominal_ + beta_uncertain_ * uncertainty_factor;
    
    // 基于速度和不确定性的CBF收敛率
    double vel_norm = velocity.norm();
    alpha_obs_ = alpha_obs_nominal_ * (1.0 + vel_scale_ * vel_norm);
    
    // 1. 添加障碍物约束
    for (int i = 0; i < n_obstacles; i++) {
        // 计算CBF值
        double h = computeCBF(state, obstacles[i]);
        cbf_values_.push_back(h);
        
        // 计算CBF导数
        Eigen::VectorXd dh_dx = computeCBFGradient(state, obstacles[i]);
        
        // 构建约束：dh/dx * (Ax + Bu + d_mean) >= -alpha * h
        Eigen::VectorXd dh_du = dh_dx.tail(3);  // 关于控制输入的导数
        A.row(i) = -dh_du;
        
        // 右侧包含：CBF项 + 干扰补偿
        b(i) = alpha_obs_ * h + dh_dx.head(3).dot(velocity) + 
               dh_du.dot(disturbance_mean_) - predictive_margin_;
    }
    
    // 2. 添加轨迹跟踪约束
    Eigen::Vector3d pos_error = goal_position - position;
    double h_track = track_margin_ * track_margin_ - pos_error.squaredNorm();
    Eigen::VectorXd dh_track_dx = -2.0 * Eigen::VectorXd::Zero(6);
    dh_track_dx.head(3) = -2.0 * pos_error;
    
    A.row(n_obstacles) = -dh_track_dx.tail(3);
    b(n_obstacles) = alpha_track_ * h_track + dh_track_dx.head(3).dot(velocity);
    
    // 3. 添加控制输入约束（可选）
    // 这里可以添加控制输入上下限约束
    
    // 求解QP问题
    safe_control = Eigen::VectorXd(3);
    bool success = solveQP(H, f, A, b, safe_control);
    
    if (!success) {
        ROS_WARN("QP求解失败，使用标称控制输入");
        safe_control = nominal_control;
    }
    
    return success;
}

double PBFController::computeCBF(const Eigen::VectorXd& state, const Obstacle& obstacle) {
    // 提取位置
    Eigen::Vector3d position = state.head(3);
    
    // 计算到障碍物的距离
    double dist = (position - obstacle.position).norm();
    
    // 计算CBF值: h(x) = ||p - p_obs||^2 - r_safe^2
    double r_safe = computeSafetyMargin(state, obstacle);
    double h = dist * dist - r_safe * r_safe;
    
    return h;
}

Eigen::VectorXd PBFController::computeCBFGradient(const Eigen::VectorXd& state, 
                                                const Obstacle& obstacle) {
    // 提取位置
    Eigen::Vector3d position = state.head(3);
    Eigen::Vector3d velocity = state.tail(3);
    
    // 计算到障碍物的向量
    Eigen::Vector3d diff = position - obstacle.position;
    double dist = diff.norm();
    
    // 避免除零
    if (dist < 1e-6) {
        diff = Eigen::Vector3d(1e-6, 0, 0);
        dist = 1e-6;
    }
    
    // 计算CBF关于位置的梯度: dh/dp = 2 * (p - p_obs)
    Eigen::Vector3d dh_dp = 2.0 * diff;
    
    // 计算CBF关于速度的梯度（都是0）
    Eigen::Vector3d dh_dv = Eigen::Vector3d::Zero();
    
    // 组合梯度向量 [dh/dp, dh/dv]
    Eigen::VectorXd dh_dx(6);
    dh_dx << dh_dp, dh_dv;
    
    return dh_dx;
}

double PBFController::computeSafetyMargin(const Eigen::VectorXd& state, 
                                        const Obstacle& obstacle) {
    // 提取速度
    Eigen::Vector3d velocity = state.tail(3);
    
    // 基本安全距离
    double r_safe = r_safe_nominal_;
    
    // 1. 基于干扰预测不确定性调整
    double uncertainty_factor = disturbance_var_.mean();
    r_safe += beta_uncertain_ * uncertainty_factor;
    
    // 2. 基于速度调整
    double vel_norm = velocity.norm();
    r_safe += vel_scale_ * vel_norm;
    
    // 3. 加上障碍物本身半径
    r_safe += obstacle.radius;
    
    return r_safe;
}

bool PBFController::solveQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& f,
                          const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                          Eigen::VectorXd& u) {
    // 使用QP求解器求解
    return qp_solver_->solve(H, f, A, b, u);
}

void PBFController::setDisturbancePrediction(const Eigen::VectorXd& mean, 
                                           const Eigen::VectorXd& variance) {
    disturbance_mean_ = mean;
    disturbance_var_ = variance;
}

void PBFController::setPredictiveMargin(double margin) {
    predictive_margin_ = margin;
}

void PBFController::publishSafetyInfo(const std::vector<Obstacle>& obstacles, 
                                    const Eigen::VectorXd& state) {
    // 创建安全信息消息
    gpbf_msgs::SafetyInfo msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    
    // 填充CBF值
    for (const auto& cbf_value : cbf_values_) {
        msg.cbf_values.push_back(cbf_value);
    }
    
    // 填充安全边界和预测裕度
    msg.safety_margin = current_safety_margin_;
    msg.predictive_margin = predictive_margin_;
    
    // 判断是否安全
    bool is_safe = true;
    for (const auto& cbf_value : cbf_values_) {
        if (cbf_value < 0) {
            is_safe = false;
            break;
        }
    }
    msg.is_safe = is_safe;
    
    // 发布消息
    safety_pub_.publish(msg);
}

} // namespace gpbf