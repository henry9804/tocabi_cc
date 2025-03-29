#include "QP/QP_cartesian_velocity.h"

namespace QP
{
    CartesianVelocity::CartesianVelocity()
    :Base()
    {
        Base::setQPsize(ns_ + nq_, ns_ + nq_ * 3);

        q_upper_.setZero();
        q_lower_.setZero();
        qdot_upper_.setZero();
        qdot_lower_.setZero();
        qddot_upper_.setZero();
        qddot_lower_.setZero();
        slack_weight_.setZero();
        damping_weight_.setZero();

        setJointLimit();
        setWeightMatrix();

        std::cout << "======================================================================" << std::endl;
        std::cout << "=================== QP Cartesian Velocity loaded!! ===================" << std::endl;
        std::cout << "======================================================================" << std::endl;
    }

    void CartesianVelocity::setCurrentState(const Eigen::Matrix<double, DOF, 1> &q_current, const Eigen::Matrix<double, DOF, 1> &qdot_current, const Eigen::Matrix<double, 6, DOF> &j_current)
    {
        q_current_ = q_current;
        qdot_current_ = qdot_current;
        j_current_ = j_current;
    }
    
    void CartesianVelocity::setDesiredEEVel(const Eigen::Matrix<double, 6, 1> &xdot_desired)
    {
        xdot_desired_ = xdot_desired;
    }

    void CartesianVelocity::setJointLimit()
    {
        std::vector<float> q_upper_wb;
        nh_qp_.getParam("tocabi_controller/jointlimit_u", q_upper_wb);
        q_upper_(0) = q_upper_wb[17];
        q_upper_(1) = q_upper_wb[16];
        q_upper_(2) = q_upper_wb[9];
        q_upper_(3) = q_upper_wb[8];
        q_upper_(4) = q_upper_wb[10];
        q_upper_(5) = q_upper_wb[11];
        q_upper_(6) = q_upper_wb[2];
        q_upper_(7) = q_upper_wb[3];

        std::vector<float> q_lower_wb;
        nh_qp_.getParam("tocabi_controller/jointlimit_l", q_lower_wb);
        q_lower_(0) = q_lower_wb[17];
        q_lower_(1) = q_lower_wb[16];
        q_lower_(2) = q_lower_wb[9];
        q_lower_(3) = q_lower_wb[8];
        q_lower_(4) = q_lower_wb[10];
        q_lower_(5) = q_lower_wb[11];
        q_lower_(6) = q_lower_wb[2];
        q_lower_(7) = q_lower_wb[3];

        std::vector<float> qdot_lim_wb;
        nh_qp_.getParam("tocabi_controller/vellimit", qdot_lim_wb);
        for(int i = 0; i < DOF; i++){
            qdot_upper_(i) = qdot_lim_wb[25+i];
            qdot_lower_(i) = -qdot_lim_wb[25+i];
            qddot_upper_(i) = 10000;
            qddot_lower_(i) = -10000;
        }

        q_upper_ *= 1.0;
        q_lower_ *= 1.0;

        qdot_upper_ *= 0.5;
        qdot_lower_ *= 0.5;

        qddot_upper_ *= 0.3;
        qddot_lower_ *= 0.3;
    }

    void CartesianVelocity::setWeightMatrix()
    {
        std::vector<float> slack;
        nh_qp_.getParam("tocabi_cc/QP/Slack", slack);
        slack_weight_(0,0) = slack[0];
        slack_weight_(1,1) = slack[1];
        slack_weight_(2,2) = slack[2];
        slack_weight_(3,3) = slack[3];
        slack_weight_(4,4) = slack[4];
        slack_weight_(5,5) = slack[5];

        std::vector<float> damping;
        nh_qp_.getParam("tocabi_cc/QP/Damping", damping);
        damping_weight_(0,0) = damping[0];
        damping_weight_(1,1) = damping[1];
        damping_weight_(2,2) = damping[2];
        damping_weight_(3,3) = damping[3];
        damping_weight_(4,4) = damping[4];
        damping_weight_(5,5) = damping[5];
        damping_weight_(6,6) = damping[6];
        damping_weight_(7,7) = damping[7];

        // mani_weight_ = 0
    }

    void CartesianVelocity::setCost()
    {
        P_ds_.block(si_index_.s1,  si_index_.s1,  ns_, ns_) = 2.0 * slack_weight_;
        P_ds_.block(si_index_.dq1, si_index_.dq1, nq_, nq_) = 2.0 * damping_weight_;

        // double mani = robot_model_.getManipulability(q_current_);
        // double mani_cubic_weight;
        // if(mani > 0.05) 
        // {
        //     mani_cubic_weight = 0.0;
        // }
        // else if(mani < 0.01)
        // {
        //     mani_cubic_weight = mani_weight_;
        // }
        // else
        // {
        //     mani_cubic_weight = DyrosMath::cubic(mani, 0.01, 0.05, mani_weight_, 0., 0., 0.);
        // }
        // // double mani_cubic_weight = mani_weight_;
        // q_ds_.block(si_index_.dq1, 0, nq_, 1) = -mani_weight_ * robot_model_.getDManipulability(q_current_);
        q_ds_.block(si_index_.dq1, 0, nq_, 1).setZero();
    }

    void CartesianVelocity::setConstraint()
    {
        A_ds_.block(si_index_.con_slack, si_index_.s1,  ns_, ns_).setIdentity(ns_, ns_);
        A_ds_.block(si_index_.con_slack, si_index_.dq1, ns_, nq_) = j_current_;
        A_ds_.block(si_index_.con_q,     si_index_.dq1, nq_, nq_).setIdentity(nq_, nq_);
        A_ds_.block(si_index_.con_qdot,  si_index_.dq1, nq_, nq_).setIdentity(nq_, nq_);
        A_ds_.block(si_index_.con_qddot, si_index_.dq1, nq_, nq_).setIdentity(nq_, nq_);

        l_ds_.block(si_index_.con_slack, 0, ns_, 1) = xdot_desired_;
        l_ds_.block(si_index_.con_q,     0, nq_, 1) = hz_ * (q_lower_ - q_current_);
        l_ds_.block(si_index_.con_qdot,  0, nq_, 1) = qdot_lower_;
        l_ds_.block(si_index_.con_qddot, 0, nq_, 1) = qddot_lower_ / hz_ + qdot_current_;
        
        u_ds_.block(si_index_.con_slack, 0, ns_, 1) = xdot_desired_;
        u_ds_.block(si_index_.con_q,     0, nq_, 1) = hz_ * (q_upper_ - q_current_);
        u_ds_.block(si_index_.con_qdot,  0, nq_, 1) = qdot_upper_;
        u_ds_.block(si_index_.con_qddot, 0, nq_, 1) = qddot_upper_ / hz_ + qdot_current_;
    }

    int CartesianVelocity::getOptJointVel(Eigen::Matrix<double, DOF, 1> &opt_qdot, TimeDuration &time_status)
    {
        Eigen::MatrixXd sol;
        int status = solveQP(sol, time_status);
        if(status == 0)
        {
            opt_qdot = sol.block(si_index_.dq1, 0, nq_, 1);
        }
        else
        {
            opt_qdot.setZero(nq_, 1);
        }
        return status;
    }

} // namespace QP
