#ifndef QP_CARTESIAN_VELOCITY_H
#define QP_CARTESIAN_VELOCITY_H

#include "ros/ros.h"
#include "qp.h"
#include "math_type_define.h"

#define JOINT_DOF 8 // 8 for TOCABI arm
#define TASK_DOF 6  // 6 for end-effector JOINT_dofS

namespace QP
{
    class CartesianVelocity : public CQuadraticProgram
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            CartesianVelocity();
            ~CartesianVelocity() override = default;
            void setCurrentState(const Eigen::Matrix<double, JOINT_DOF, 1> &q_current, const Eigen::Matrix<double, JOINT_DOF, 1> &qdot_current, const Eigen::Matrix<double, TASK_DOF, JOINT_DOF> &j_current);
            void setDesiredEEVel(const Eigen::Matrix<double, TASK_DOF, 1> &xdot_desired);
            bool getOptJointVel(Eigen::Matrix<double, JOINT_DOF, 1> &opt_qdot);
            ros::NodeHandle nh_qp_;

        private:
            static constexpr int nq_ = JOINT_DOF;
            static constexpr int ns_ = TASK_DOF;
            static constexpr int nx_ = ns_ + nq_;       // decision variables
            static constexpr int nc_ = ns_ + nq_ * 3;   // constraints

            struct QPIndex
            {
                int s1 = 0; // slack 1
                int s2 = 1; // slack 2
                int s3 = 2; // slack 3
                int s4 = 3; // slack 4
                int s5 = 4; // slack 5
                int s6 = 5; // slack 6

                int dq1 = 6;  // qdot 1 
                int dq2 = 7;  // qdot 2 
                int dq3 = 8;  // qdot 3 
                int dq4 = 9;  // qdot 4 
                int dq5 = 10; // qdot 5 
                int dq6 = 11; // qdot 6 
                int dq7 = 12; // qdot 7 
                int dq8 = 13; // qdot 8 

                int con_slack = 0;  // constraints for slack (0-5)
                int con_q = 6;      // constraints for q (6-13)
                int con_qdot = 14;  // constraints for qdot (14-21)
                int con_qddot = 22; // constraints for qddot (22-29)
            }si_index_;

            Eigen::Matrix<double, JOINT_DOF, 1> q_current_;
            Eigen::Matrix<double, JOINT_DOF, 1> qdot_current_;
            Eigen::Matrix<double, TASK_DOF, JOINT_DOF> j_current_;
            Eigen::Matrix<double, TASK_DOF, 1> xdot_desired_;

            const double hz_ = 2000.;

            Eigen::Matrix<double, JOINT_DOF, 1> q_upper_;
            Eigen::Matrix<double, JOINT_DOF, 1> q_lower_;
            Eigen::Matrix<double, JOINT_DOF, 1> qdot_upper_;
            Eigen::Matrix<double, JOINT_DOF, 1> qdot_lower_;
            Eigen::Matrix<double, JOINT_DOF, 1> qddot_upper_;
            Eigen::Matrix<double, JOINT_DOF, 1> qddot_lower_;

            Eigen::Matrix<double, TASK_DOF, TASK_DOF> slack_weight_;
            Eigen::Matrix<double, JOINT_DOF, JOINT_DOF> damping_weight_;
            Eigen::Matrix<double, JOINT_DOF, JOINT_DOF> qddot_weight_;
            // double mani_weight_;

            bool is_first_{true};

            /* 
            min   1/2 x' P x + q' x
            x

            subject to
            l <= A x <= u

            with :
            P (nx x nx) positive definite
            q (nx x 1)
            A (nc x nx)
            l (nc x 1)
            u (nc x 1)
            */
            Eigen::Matrix<double, nx_, nx_> P_;
            Eigen::Matrix<double, nx_, 1> q_;
            Eigen::Matrix<double, nc_, nx_> A_;
            Eigen::Matrix<double, nc_, 1> l_;
            Eigen::Matrix<double, nc_, 1> u_;
            
            void setJointLimit();
            void setWeightMatrix();
            void setCost();
            void setConstraint();
    };
} // namespace QP
#endif