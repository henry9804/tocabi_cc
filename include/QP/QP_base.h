#ifndef QP_BASE_H
#define QP_BASE_H

#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>

#include "OsqpEigen/OsqpEigen.h"

#include "suhan_benchmark.h"

namespace QP
{
    struct TimeDuration
    {
        double set_qp;
        double set_solver;
        double solve_qp;
        void setZero(){set_qp=0; set_solver=0; solve_qp=0;}
    };
    
    class Base
    {
        public:
            Base();
            void setQPsize(const int &nx, const int &nc);
            int solveQP(Eigen::MatrixXd &sol, TimeDuration &time_status);
        
        private:
            virtual void setCost() = 0;
            virtual void setConstraint() = 0;
        
        protected:
            int nx_; // number of decision variables
            int nc_; // number of constraints
            Eigen::MatrixXd P_ds_;
            Eigen::VectorXd q_ds_;
            Eigen::MatrixXd A_ds_;
            Eigen::VectorXd l_ds_;
            Eigen::VectorXd u_ds_;

            OsqpEigen::Status qp_status_;
            SuhanBenchmark timer_;
            TimeDuration time_status_;
    };
} // namespace QP
#endif