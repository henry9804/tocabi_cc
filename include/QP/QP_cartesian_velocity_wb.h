#ifndef QP_CARTESIAN_VELOCITY_WB_H
#define QP_CARTESIAN_VELOCITY_WB_H

#include "ros/ros.h"
#include "qp.h"
#include "math_type_define.h"

#define JOINT_DOF 21 // 3 for waist, 8 for larm, 2 for neck, 8 for rarm
#define TASK_DOF 18  // 6 for each end-effector (lhand, head, rhand)

namespace QP
{
    class CartesianVelocityWB : public CQuadraticProgram
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            CartesianVelocityWB();
            ~CartesianVelocityWB() override = default;
            void setCurrentState(const Eigen::Matrix<double, JOINT_DOF, 1> &q_current, const Eigen::Matrix<double, JOINT_DOF, 1> &qdot_current, const Eigen::Matrix<double, TASK_DOF, JOINT_DOF> &j_current);
            void setDesiredEEVel(const Eigen::Matrix<double, 6, 1> &lhand_dot_desired, const Eigen::Matrix<double, 6, 1> &head_dot_desired, const Eigen::Matrix<double, 6, 1> &rhand_dot_desired);
            bool getOptJointVel(Eigen::Matrix<double, JOINT_DOF, 1> &opt_qdot);
            ros::NodeHandle nh_qp_;

        private:
            static constexpr int nq_ = JOINT_DOF;
            static constexpr int ns_ = TASK_DOF;
            static constexpr int nx_ = ns_ + nq_;       // decision variables
            static constexpr int nc_ = ns_ + nq_ * 3;   // constraints

            struct QPIndex
            {
                int s = 0;  // slack start index

                int dq  = 18; // qdot start index

                int con_slack = 0;  // constraints for hand slack (0-17)
                int con_q = 18;     // constraints for q (18-38)
                int con_qdot = 39;  // constraints for qdot (39-59)
                int con_qddot = 60; // constraints for qddot (60-80)
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