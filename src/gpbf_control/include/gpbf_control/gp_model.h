#ifndef GP_MODEL_H
#define GP_MODEL_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <gpbf_msgs/Disturbance.h>

namespace gpbf {

class GPModel {
public:
    /**
     * 构造函数
     * @param nh ROS节点句柄
     */
    GPModel(const ros::NodeHandle& nh);
    
    /**
     * 析构函数
     */
    ~GPModel();
    
    /**
     * 初始化GP模型参数
     * @param length_scales 长度尺度参数
     * @param signal_var 信号方差
     * @param noise_var 噪声方差
     * @return 是否初始化成功
     */
    bool initialize(const std::vector<double>& length_scales, 
                   double signal_var, double noise_var);
    
    /**
     * 更新训练数据集
     * @param state 当前系统状态 [位置(3), 速度(3)]
     * @param estimated_disturbance 估计的干扰 [3]
     */
    void updateTrainingData(const Eigen::VectorXd& state, 
                           const Eigen::VectorXd& estimated_disturbance);
    
    /**
     * 预测当前状态下的干扰
     * @param state 当前系统状态
     * @param mean 输出的干扰均值
     * @param variance 输出的干扰方差
     */
    void predictDisturbance(const Eigen::VectorXd& state, 
                           Eigen::VectorXd& mean, 
                           Eigen::VectorXd& variance);
    
    /**
     * 保存模型到文件
     * @param filename 文件路径
     * @return 是否成功
     */
    bool saveModel(const std::string& filename);
    
    /**
     * 从文件加载模型
     * @param filename 文件路径
     * @return 是否成功
     */
    bool loadModel(const std::string& filename);
    
    /**
     * 发布干扰预测结果
     * @param state 当前状态
     */
    void publishDisturbancePrediction(const Eigen::VectorXd& state);
    
private:
    /**
     * 计算SE核函数
     * @param x1 输入向量1
     * @param x2 输入向量2
     * @return 核函数值
     */
    double computeKernel(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2);
    
    /**
     * 计算输入向量与所有训练数据的核向量
     * @param x 输入向量
     * @return 核向量
     */
    Eigen::VectorXd computeKernelVector(const Eigen::VectorXd& x);
    
    /**
     * 重新计算核矩阵的逆
     */
    void recomputeKernelMatrixInverse();
    
    // 模型参数
    Eigen::VectorXd length_scales_;   // 长度尺度
    double signal_var_;               // 信号方差
    double noise_var_;                // 噪声方差
    
    // 训练数据
    Eigen::MatrixXd X_train_;         // 训练输入
    Eigen::MatrixXd Y_train_;         // 训练输出
    
    // 核矩阵
    Eigen::MatrixXd K_inv_;           // 核矩阵的逆
    
    // 训练数据管理
    int max_samples_;                 // 最大样本数
    bool kernel_updated_;             // 核矩阵是否需要更新
    
    // ROS相关
    ros::NodeHandle nh_;
    ros::Publisher disturbance_pub_;   // 干扰预测发布器
};

} // namespace gpbf

#endif // GP_MODEL_H