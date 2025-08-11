#include <gpbf_control/gp_model.h>
#include <fstream>
#include <iostream>

namespace gpbf {

GPModel::GPModel(const ros::NodeHandle& nh) : nh_(nh), kernel_updated_(false) {
    // 从参数服务器读取最大样本数
    nh_.param<int>("gp/max_samples", max_samples_, 100);
    
    // 创建发布器
    disturbance_pub_ = nh_.advertise<gpbf_msgs::Disturbance>("disturbance_prediction", 10);
    
    ROS_INFO("GP Model initialized with max_samples: %d", max_samples_);
}

GPModel::~GPModel() {
    ROS_INFO("GP Model destroyed");
}

bool GPModel::initialize(const std::vector<double>& length_scales, 
                       double signal_var, double noise_var) {
    // 检查参数
    if (length_scales.size() != 6) {
        ROS_ERROR("GP Model requires 6 length scales for [x, y, z, vx, vy, vz]");
        return false;
    }
    
    // 设置参数
    length_scales_ = Eigen::Map<const Eigen::VectorXd>(length_scales.data(), length_scales.size());
    signal_var_ = signal_var;
    noise_var_ = noise_var;
    
    // 初始化训练数据矩阵
    X_train_ = Eigen::MatrixXd(0, 6);  // 状态: [x, y, z, vx, vy, vz]
    Y_train_ = Eigen::MatrixXd(0, 3);  // 干扰: [dx, dy, dz]
    
    ROS_INFO("GP Model parameters set: signal_var=%.3f, noise_var=%.3f", signal_var_, noise_var_);
    
    return true;
}

void GPModel::updateTrainingData(const Eigen::VectorXd& state, 
                               const Eigen::VectorXd& estimated_disturbance) {
    // 检查输入
    if (state.size() != 6 || estimated_disturbance.size() != 3) {
        ROS_ERROR("Invalid input dimensions: state=%ld, disturbance=%ld", 
                 state.size(), estimated_disturbance.size());
        return;
    }
    
    // 添加新样本
    if (X_train_.rows() < max_samples_) {
        // 如果还未到最大样本数，直接添加
        Eigen::MatrixXd X_new(X_train_.rows() + 1, X_train_.cols());
        Eigen::MatrixXd Y_new(Y_train_.rows() + 1, Y_train_.cols());
        
        X_new.topRows(X_train_.rows()) = X_train_;
        Y_new.topRows(Y_train_.rows()) = Y_train_;
        
        X_new.bottomRows(1) = state.transpose();
        Y_new.bottomRows(1) = estimated_disturbance.transpose();
        
        X_train_ = X_new;
        Y_train_ = Y_new;
    } else {
        // 如果已到最大样本数，替换最旧的样本(FIFO)
        X_train_.topRows(X_train_.rows() - 1) = X_train_.bottomRows(X_train_.rows() - 1);
        Y_train_.topRows(Y_train_.rows() - 1) = Y_train_.bottomRows(Y_train_.rows() - 1);
        
        X_train_.bottomRows(1) = state.transpose();
        Y_train_.bottomRows(1) = estimated_disturbance.transpose();
    }
    
    // 标记核矩阵需要更新
    kernel_updated_ = false;
    
    ROS_DEBUG("Training data updated: %ld samples", X_train_.rows());
}

void GPModel::predictDisturbance(const Eigen::VectorXd& state, 
                               Eigen::VectorXd& mean, 
                               Eigen::VectorXd& variance) {
    // 检查是否有训练数据
    if (X_train_.rows() == 0) {
        // 无训练数据时，返回零均值和高方差
        mean = Eigen::VectorXd::Zero(3);
        variance = Eigen::VectorXd::Ones(3) * signal_var_;
        return;
    }
    
    // 检查是否需要更新核矩阵
    if (!kernel_updated_) {
        recomputeKernelMatrixInverse();
    }
    
    // 计算核向量 k*
    Eigen::VectorXd k_star = computeKernelVector(state);
    
    // 计算均值: mu = k*^T * K^-1 * y
    mean = Eigen::VectorXd::Zero(3);
    variance = Eigen::VectorXd::Zero(3);
    
    for (int i = 0; i < 3; i++) {
        // 对每个输出维度分别计算
        mean(i) = k_star.transpose() * K_inv_ * Y_train_.col(i);
        
        // 计算方差: sigma^2 = k(x*,x*) - k*^T * K^-1 * k*
        double k_star_star = signal_var_;  // k(x*,x*)
        variance(i) = k_star_star - k_star.transpose() * K_inv_ * k_star;
        
        // 确保方差为正
        variance(i) = std::max(variance(i), 1e-6);
    }
}

double GPModel::computeKernel(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2) {
    // 计算平方距离，考虑每个维度的长度尺度
    double dist_squared = 0.0;
    for (int i = 0; i < x1.size(); i++) {
        double diff = (x1(i) - x2(i)) / length_scales_(i);
        dist_squared += diff * diff;
    }
    
    // 平方指数核: k(x,x') = sigma_f^2 * exp(-0.5 * ||x-x'||^2)
    return signal_var_ * exp(-0.5 * dist_squared);
}

Eigen::VectorXd GPModel::computeKernelVector(const Eigen::VectorXd& x) {
    // 计算输入向量x与所有训练数据的核函数值
    Eigen::VectorXd k_star(X_train_.rows());
    
    for (int i = 0; i < X_train_.rows(); i++) {
        k_star(i) = computeKernel(x, X_train_.row(i).transpose());
    }
    
    return k_star;
}

void GPModel::recomputeKernelMatrixInverse() {
    int n = X_train_.rows();
    
    // 计算核矩阵 K
    Eigen::MatrixXd K(n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            K(i, j) = computeKernel(X_train_.row(i).transpose(), X_train_.row(j).transpose());
            
            // 对角线添加噪声方差
            if (i == j) {
                K(i, j) += noise_var_;
            }
        }
    }
    
    // 计算 K 的逆
    K_inv_ = K.inverse();
    
    // 标记更新完成
    kernel_updated_ = true;
}

bool GPModel::saveModel(const std::string& filename) {
    try {
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            ROS_ERROR("Could not open file for saving: %s", filename.c_str());
            return false;
        }
        
        // 保存超参数
        file.write((char*)length_scales_.data(), length_scales_.size() * sizeof(double));
        file.write((char*)&signal_var_, sizeof(double));
        file.write((char*)&noise_var_, sizeof(double));
        
        // 保存训练数据维度
        int rows = X_train_.rows();
        int x_cols = X_train_.cols();
        int y_cols = Y_train_.cols();
        
        file.write((char*)&rows, sizeof(int));
        file.write((char*)&x_cols, sizeof(int));
        file.write((char*)&y_cols, sizeof(int));
        
        // 保存训练数据
        file.write((char*)X_train_.data(), rows * x_cols * sizeof(double));
        file.write((char*)Y_train_.data(), rows * y_cols * sizeof(double));
        
        file.close();
        
        ROS_INFO("Model saved to: %s", filename.c_str());
        return true;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Error saving model: %s", e.what());
        return false;
    }
}

bool GPModel::loadModel(const std::string& filename) {
    try {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            ROS_ERROR("Could not open file for loading: %s", filename.c_str());
            return false;
        }
        
        // 加载超参数
        length_scales_ = Eigen::VectorXd(6);
        file.read((char*)length_scales_.data(), length_scales_.size() * sizeof(double));
        file.read((char*)&signal_var_, sizeof(double));
        file.read((char*)&noise_var_, sizeof(double));
        
        // 加载训练数据维度
        int rows, x_cols, y_cols;
        file.read((char*)&rows, sizeof(int));
        file.read((char*)&x_cols, sizeof(int));
        file.read((char*)&y_cols, sizeof(int));
        
        // 加载训练数据
        X_train_.resize(rows, x_cols);
        Y_train_.resize(rows, y_cols);
        
        file.read((char*)X_train_.data(), rows * x_cols * sizeof(double));
        file.read((char*)Y_train_.data(), rows * y_cols * sizeof(double));
        
        file.close();
        
        // 重新计算核矩阵
        kernel_updated_ = false;
        
        ROS_INFO("Model loaded from: %s with %d samples", filename.c_str(), rows);
        return true;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Error loading model: %s", e.what());
        return false;
    }
}

void GPModel::publishDisturbancePrediction(const Eigen::VectorXd& state) {
    // 预测当前状态的干扰
    Eigen::VectorXd mean, variance;
    predictDisturbance(state, mean, variance);
    
    // 创建消息
    gpbf_msgs::Disturbance msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    
    // 填充均值
    msg.mean.x = mean(0);
    msg.mean.y = mean(1);
    msg.mean.z = mean(2);
    
    // 填充方差
    msg.variance.x = variance(0);
    msg.variance.y = variance(1);
    msg.variance.z = variance(2);
    
    // 发布消息
    disturbance_pub_.publish(msg);
}

} // namespace gpbf