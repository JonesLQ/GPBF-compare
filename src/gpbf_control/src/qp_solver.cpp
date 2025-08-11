#include <gpbf_control/qp_solver.h>
#include <OsqpEigen/OsqpEigen.h>

namespace gpbf {

QPSolver::QPSolver() {
    // 设置默认求解参数
    eps_abs_ = 1e-3;
    eps_rel_ = 1e-3;
    max_iter_ = 100;
    
    ROS_INFO("QP Solver initialized with OSQP");
}

QPSolver::~QPSolver() {
    ROS_INFO("QP Solver destroyed");
}

bool QPSolver::solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& f,
                    const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                    Eigen::VectorXd& x) {
    // 检查输入维度
    int n_vars = H.rows();  // 变量数量
    int n_cons = A.rows();  // 约束数量
    
    if (H.cols() != n_vars || f.size() != n_vars || 
        A.cols() != n_vars || b.size() != n_cons) {
        ROS_ERROR("QP Solver: Invalid input dimensions");
        return false;
    }
    
    try {
        // 创建OSQP求解器实例
        OsqpEigen::Solver solver;
        
        // 设置求解器参数
        solver.settings()->setVerbosity(false);
        solver.settings()->setAbsoluteTolerance(eps_abs_);
        solver.settings()->setRelativeTolerance(eps_rel_);
        solver.settings()->setMaxIteration(max_iter_);
        solver.settings()->setWarmStart(true);
        
        // 设置优化问题维度
        solver.data()->setNumberOfVariables(n_vars);
        solver.data()->setNumberOfConstraints(n_cons);
        
        
        // 设置Hessian矩阵 (确保对称正定并转换为稀疏格式)
        Eigen::MatrixXd H_sym = 0.5 * (H + H.transpose());
        Eigen::SparseMatrix<double> H_sparse = H_sym.sparseView();
        solver.data()->setHessianMatrix(H_sparse);
        
        // 设置梯度
        // solver.data()->setGradient(f);

        // 将梯度转换为稀疏格式
        // Eigen::SparseVector<double> f_sparse = f.sparseView();
        // solver.data()->setGradient(f_sparse);

        // 创建梯度的副本以避免Ref构造函数问题
        Eigen::VectorXd f_copy = f;
        solver.data()->setGradient(f_copy);
        
        // 设置线性约束
        // 对于OSQP，约束为 lb <= Ax <= ub
        // 我们的约束为 Ax <= b，所以lb设为负无穷
        Eigen::VectorXd constraint_lb = Eigen::VectorXd::Constant(n_cons, -OsqpEigen::INFTY);
        Eigen::VectorXd constraint_ub = b;

        // 将约束矩阵转换为稀疏格式
        Eigen::SparseMatrix<double> A_sparse = A.sparseView();
        solver.data()->setLinearConstraintsMatrix(A_sparse);
        solver.data()->setLowerBound(constraint_lb);
        solver.data()->setUpperBound(constraint_ub);
        
        // 初始化求解器
        if (!solver.initSolver()) {
            ROS_ERROR("QP Solver: Failed to initialize OSQP");
            return false;
        }
        
        // // 求解问题
        // if (!solver.solveProblem()) {
        //     ROS_WARN("QP Solver: OSQP solution may not be optimal");
        //     return false;
        // }
        // 替换为
        OsqpEigen::ErrorExitFlag exitflag = solver.solveProblem();
        if (exitflag != OsqpEigen::ErrorExitFlag::NoError) {
        ROS_WARN("QP Solver: OSQP solution may not be optimal, error code: %d", static_cast<int>(exitflag));
        return false;
        }
        
        // 获取解
        x = solver.getSolution();
        
        return true;
    }
    catch (const std::exception& e) {
        ROS_ERROR("QP Solver exception: %s", e.what());
        return false;
    }
}

} // namespace gpbf