#ifndef QP_SOLVER_H
#define QP_SOLVER_H

#include <Eigen/Dense>
#include <ros/ros.h>

namespace gpbf {

/**
 * 二次规划求解器接口类
 * 定义求解二次规划问题的接口
 * 最小化 0.5 x^T H x + f^T x
 * 满足 Ax <= b
 */
class QPSolver {
public:
    /**
     * 构造函数
     */
    QPSolver();
    
    /**
     * 析构函数
     */
    virtual ~QPSolver();
    
    /**
     * 求解QP问题
     * @param H 目标函数二次项矩阵
     * @param f 目标函数线性项向量
     * @param A 不等式约束矩阵
     * @param b 不等式约束向量
     * @param x 输出的解
     * @return 是否求解成功
     */
    virtual bool solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& f,
                      const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                      Eigen::VectorXd& x);
    
private:
    // 求解器参数
    double eps_abs_;  // 绝对精度
    double eps_rel_;  // 相对精度
    int max_iter_;    // 最大迭代次数
};

} // namespace gpbf

#endif // QP_SOLVER_H