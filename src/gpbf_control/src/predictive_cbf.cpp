#include <gpbf_control/predictive_cbf.h>
#include <fstream>
#include <cmath>

namespace gpbf {

PredictiveCBF::PredictiveCBF(const ros::NodeHandle& nh) : nh_(nh) {
    ROS_INFO("Predictive CBF initialized");
}

PredictiveCBF::~PredictiveCBF() {
    ROS_INFO("Predictive CBF destroyed");
}

bool PredictiveCBF::initialize() {
    // 加载参数
    nh_.param<int>("pcbf/max_samples", max_samples_, 1000);
    nh_.param<double>("pcbf/learning_rate", learning_rate_, 0.01);
    
    // 设置特征维度 - 基于我们的特征提取函数
    feature_dim_ = 12;  // 速度(3) + 姿态欧拉角(3) + 角速度(3) + 加速度请求(3)
    
    // 初始化模型权重
    model_weights_ = Eigen::VectorXd::Zero(feature_dim_);
    
    // 尝试加载预训练模型
    std::string model_path;
    if (nh_.getParam("pcbf/model_path", model_path)) {
        if (!loadModel(model_path)) {
            ROS_WARN("Failed to load pretrained model, using default weights");
        }
    } else {
        ROS_INFO("No pretrained model specified, using default weights");
        
        // 设置一些基于经验的默认权重
        // 这些权重应该反映姿态和速度对安全裕度的影响
        model_weights_(3) = 0.05;  // 欧拉角roll影响
        model_weights_(4) = 0.05;  // 欧拉角pitch影响
        model_weights_(6) = 0.01;  // 角速度x影响
        model_weights_(7) = 0.01;  // 角速度y影响
    }
    
    ROS_INFO("Predictive CBF initialized with feature_dim=%d", feature_dim_);
    
    return true;
}

bool PredictiveCBF::loadModel(const std::string& model_path) {
    try {
        std::ifstream file(model_path);
        if (!file.is_open()) {
            ROS_ERROR("Could not open model file: %s", model_path.c_str());
            return false;
        }
        
        // 读取特征维度
        int dim;
        file >> dim;
        
        if (dim != feature_dim_) {
            ROS_ERROR("Model dimension mismatch: expected %d, got %d", feature_dim_, dim);
            return false;
        }
        
        // 读取模型权重
        model_weights_.resize(dim);
        for (int i = 0; i < dim; i++) {
            file >> model_weights_(i);
        }
        
        file.close();
        
        ROS_INFO("Model loaded from %s", model_path.c_str());
        return true;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Error loading model: %s", e.what());
        return false;
    }
}

bool PredictiveCBF::saveModel(const std::string& model_path) {
    try {
        std::ofstream file(model_path);
        if (!file.is_open()) {
            ROS_ERROR("Could not open model file for writing: %s", model_path.c_str());
            return false;
        }
        
        // 写入特征维度
        file << feature_dim_ << std::endl;
        
        // 写入模型权重
        for (int i = 0; i < feature_dim_; i++) {
            file << model_weights_(i) << " ";
        }
        file << std::endl;
        
        file.close();
        
        ROS_INFO("Model saved to %s", model_path.c_str());
        return true;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Error saving model: %s", e.what());
        return false;
    }
}

double PredictiveCBF::predictSafetyMargin(const Eigen::VectorXd& state, 
                                        const Eigen::VectorXd& attitude) {
    // 提取特征
    Eigen::VectorXd features = extractFeatures(state, attitude);
    
    // 线性预测: margin = w^T * features
    double margin = model_weights_.dot(features);
    
    // 确保结果非负
    margin = std::max(0.0, margin);
    
    return margin;
}

void PredictiveCBF::updateModel(const Eigen::VectorXd& state, 
                              const Eigen::VectorXd& attitude,
                              double observed_degradation) {
    // 提取特征
    Eigen::VectorXd features = extractFeatures(state, attitude);
    
    // 当前预测
    double predicted = model_weights_.dot(features);
    
    // 计算误差
    double error = observed_degradation - predicted;
    
    // 更新权重 (梯度下降)
    model_weights_ += learning_rate_ * error * features;
    
    // 保存训练样本
    DataPoint data_point;
    data_point.features = features;
    data_point.degradation = observed_degradation;
    
    // 如果数据量超过限制，移除最老的
    if (training_data_.size() >= max_samples_) {
        training_data_.erase(training_data_.begin());
    }
    
    training_data_.push_back(data_point);
    
    ROS_DEBUG("Model updated: error=%.4f, samples=%zu", error, training_data_.size());
}

Eigen::VectorXd PredictiveCBF::extractFeatures(const Eigen::VectorXd& state, 
                                             const Eigen::VectorXd& attitude) {
    Eigen::VectorXd features = Eigen::VectorXd::Zero(feature_dim_);
    
    // 提取位置和速度
    Eigen::Vector3d velocity = state.tail(3);
    
    // 从姿态向量提取欧拉角和角速度
    // 姿态向量假设为: [R11, R12, R13, R21, R22, R23, R31, R32, R33, wx, wy, wz]
    Eigen::Matrix3d R;
    R << attitude(0), attitude(1), attitude(2),
         attitude(3), attitude(4), attitude(5),
         attitude(6), attitude(7), attitude(8);
    
    // 从旋转矩阵提取欧拉角 (ZYX顺序)
    double roll = atan2(R(2, 1), R(2, 2));
    double pitch = atan2(-R(2, 0), sqrt(R(2, 1)*R(2, 1) + R(2, 2)*R(2, 2)));
    double yaw = atan2(R(1, 0), R(0, 0));
    
    // 提取角速度
    Eigen::Vector3d angular_velocity = attitude.tail(3);
    
    // 填充特征向量
    // 1-3: 速度
    features.segment(0, 3) = velocity.cwiseAbs();  // 使用绝对值，因为方向不重要
    
    // 4-6: 欧拉角
    features(3) = std::abs(roll);   // 绝对值的roll角
    features(4) = std::abs(pitch);  // 绝对值的pitch角
    features(5) = 0;                // yaw角对安全影响不大，设为0
    
    // 7-9: 角速度
    features.segment(6, 3) = angular_velocity.cwiseAbs();
    
    // 10-12: 加速度请求的占位符
    // 在实际使用时，需要从外部传入加速度请求
    features.segment(9, 3) = Eigen::Vector3d::Zero();
    
    return features;
}

} // namespace gpbf