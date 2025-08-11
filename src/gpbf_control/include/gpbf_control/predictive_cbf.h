#ifndef PREDICTIVE_CBF_H
#define PREDICTIVE_CBF_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace gpbf {

/**
 * 预测性控制屏障函数类
 * 用于补偿简化模型(ROM)和全阶模型(FOM)之间的差异
 */
class PredictiveCBF {
public:
    /**
     * 构造函数
     * @param nh ROS节点句柄
     */
    PredictiveCBF(const ros::NodeHandle& nh);
    
    /**
     * 析构函数
     */
    ~PredictiveCBF();
    
    /**
     * 初始化参数和模型
     * @return 是否初始化成功
     */
    bool initialize();
    
    /**
     * 加载安全退化模型
     * @param model_path 模型文件路径
     * @return 是否加载成功
     */
    bool loadModel(const std::string& model_path);
    
    /**
     * 保存安全退化模型
     * @param model_path 模型文件路径
     * @return 是否保存成功
     */
    bool saveModel(const std::string& model_path);
    
    /**
     * 预测ROM到FOM的安全裕度
     * @param state 当前状态，包含位置速度[3+3]
     * @param attitude 当前姿态，包含旋转矩阵[9]和角速度[3]
     * @return 预测的安全裕度
     */
    double predictSafetyMargin(const Eigen::VectorXd& state, 
                              const Eigen::VectorXd& attitude);
    
    /**
     * 在线更新模型
     * @param state 当前状态
     * @param attitude 当前姿态
     * @param observed_degradation 观测到的安全退化
     */
    void updateModel(const Eigen::VectorXd& state, 
                    const Eigen::VectorXd& attitude,
                    double observed_degradation);
    
private:
    /**
     * 从状态提取特征用于预测
     * @param state 系统状态
     * @param attitude 系统姿态
     * @return 特征向量
     */
    Eigen::VectorXd extractFeatures(const Eigen::VectorXd& state, 
                                  const Eigen::VectorXd& attitude);
    
    // 模型参数
    Eigen::VectorXd model_weights_;     // 线性模型权重
    
    // 数据收集
    struct DataPoint {
        Eigen::VectorXd features;       // 特征向量
        double degradation;             // 安全退化值
    };
    std::vector<DataPoint> training_data_;
    
    // 参数
    int max_samples_;                   // 最大样本数
    int feature_dim_;                   // 特征维度
    double learning_rate_;              // 学习率
    
    // ROS相关
    ros::NodeHandle nh_;
};

} // namespace gpbf

#endif // PREDICTIVE_CBF_H