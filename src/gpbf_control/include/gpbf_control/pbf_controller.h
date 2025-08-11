#ifndef PBF_CONTROLLER_H
#define PBF_CONTROLLER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <gpbf_control/qp_solver.h>
#include <gpbf_msgs/ObstacleArray.h>
#include <gpbf_msgs/SafetyInfo.h>

namespace gpbf {

// 障碍物结构
struct Obstacle {
    Eigen::Vector3d position;  // 障碍物位置
    double radius;            // 障碍物半径
    int id;                   // 障碍物ID
};

class PBFController {
public:
    /**
     * 构造函数
     * @param nh ROS节点句柄
     */
    PBFController(const ros::NodeHandle& nh);
    
    /**
     * 析构函数
     */
    ~PBFController();
    
    /**
     * 初始化控制器
     * @return 是否初始化成功
     */
    bool initialize();
    
    /**
     * 计算安全控制输入
     * @param state 当前系统状态 [位置(3), 速度(3)]
     * @param nominal_control 标称控制输入
     * @param goal_position 目标位置
     * @param obstacles 障碍物列表
     * @param safe_control 输出的安全控制输入
     * @return 是否成功计算安全控制
     */
    bool computeSafeControl(const Eigen::VectorXd& state, 
                           const Eigen::VectorXd& nominal_control,
                           const Eigen::VectorXd& goal_position,
                           const std::vector<Obstacle>& obstacles,
                           Eigen::VectorXd& safe_control);
    
    /**
     * 设置干扰预测结果
     * @param mean 干扰预测均值
     * @param variance 干扰预测方差
     */
    void setDisturbancePrediction(const Eigen::VectorXd& mean, 
                                const Eigen::VectorXd& variance);
    
    /**
     * 设置预测性CBF提供的安全裕度
     * @param margin 预测的安全裕度
     */
    void setPredictiveMargin(double margin);
    
    /**
     * 发布安全信息
     * @param obstacles 障碍物列表
     * @param state 当前状态
     */
    void publishSafetyInfo(const std::vector<Obstacle>& obstacles, 
                          const Eigen::VectorXd& state);
    
private:
    /**
     * 计算控制屏障函数值
     * @param state 当前状态
     * @param obstacle 障碍物
     * @return CBF值
     */
    double computeCBF(const Eigen::VectorXd& state, const Obstacle& obstacle);
    
    /**
     * 计算CBF导数
     * @param state 当前状态
     * @param obstacle 障碍物
     * @return CBF导数向量
     */
    Eigen::VectorXd computeCBFGradient(const Eigen::VectorXd& state, 
                                     const Obstacle& obstacle);
    
    /**
     * 计算安全边界
     * @param state 当前状态
     * @param obstacle 当前障碍物
     * @return 安全半径
     */
    double computeSafetyMargin(const Eigen::VectorXd& state, 
                             const Obstacle& obstacle);
    
    /**
     * 求解QP问题
     * @param H 目标函数二次项
     * @param f 目标函数线性项
     * @param A 不等式约束矩阵
     * @param b 不等式约束向量
     * @param u 输出控制
     * @return 是否求解成功
     */
    bool solveQP(const Eigen::MatrixXd& H, const Eigen::VectorXd& f,
                const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                Eigen::VectorXd& u);
    
    // 参数
    double alpha_obs_;              // CBF收敛率
    double alpha_obs_nominal_;      // 标称CBF收敛率
    double alpha_track_;            // 轨迹跟踪CBF收敛率
    double r_safe_nominal_;         // 标称安全距离
    double beta_uncertain_;         // 不确定性缩放因子
    double track_margin_;           // 轨迹跟踪容许误差
    double vel_scale_;              // 速度对安全距离的影响系数
    
    // 系统参数
    double mass_;                   // 质量(kg)
    double gravity_;                // 重力加速度(m/s^2)
    
    // 当前干扰预测
    Eigen::VectorXd disturbance_mean_;   // 干扰均值
    Eigen::VectorXd disturbance_var_;    // 干扰方差
    
    // 预测性CBF安全裕度
    double predictive_margin_;      // 模型差异补偿裕度
    
    // QP求解器
    std::unique_ptr<QPSolver> qp_solver_;
    
    // 安全信息
    std::vector<double> cbf_values_;
    double current_safety_margin_;
    
    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher safety_pub_;      // 安全信息发布器
};

} // namespace gpbf

#endif // PBF_CONTROLLER_H