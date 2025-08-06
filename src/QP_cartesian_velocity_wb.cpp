#include "QP/QP_cartesian_velocity_wb.h"

namespace QP
{
    CartesianVelocityWB::CartesianVelocityWB()
    :CQuadraticProgram()
    {
        InitializeProblemSize(nx_, nc_);

        q_upper_.setZero();
        q_lower_.setZero();
        qdot_upper_.setZero();
        qdot_lower_.setZero();
        qddot_upper_.setZero();
        qddot_lower_.setZero();
        slack_weight_.setZero();
        damping_weight_.setZero();
        qddot_weight_.setZero();

        P_.setZero();
        q_.setZero();
        A_.setZero();
        l_.setZero();
        u_.setZero();

        setJointLimit();
        setWeightMatrix();

        std::cout << "======================================================================" << std::endl;
        std::cout << "============== QP Cartesian Velocity WholeBody loaded!! ==============" << std::endl;
        std::cout << "======================================================================" << std::endl;
    }

    void CartesianVelocityWB::setCurrentState(const Eigen::Matrix<double, JOINT_DOF, 1> &q_current, const Eigen::Matrix<double, JOINT_DOF, 1> &qdot_current, const Eigen::Matrix<double, TASK_DOF, JOINT_DOF> &j_current)
    {
        q_current_ = q_current;
        qdot_current_ = qdot_current;
        j_current_ = j_current;
    }
    
    void CartesianVelocityWB::setDesiredEEVel(const Eigen::Matrix<double, 6, 1> &lhand_dot_desired, const Eigen::Matrix<double, 6, 1> &head_dot_desired, const Eigen::Matrix<double, 6, 1> &rhand_dot_desired)
    {
        xdot_desired_.segment(0, 6) = lhand_dot_desired;
        xdot_desired_.segment(6, 6) = head_dot_desired;
        xdot_desired_.segment(12, 6) = rhand_dot_desired;
    }

    void CartesianVelocityWB::setJointLimit()
    {
        std::vector<float> q_upper_wb;
        nh_qp_.getParam("tocabi_controller/jointlimit_u", q_upper_wb);
        q_upper_(0)  = q_upper_wb[18];
        q_upper_(1)  = q_upper_wb[19];
        q_upper_(2)  = q_upper_wb[26];
        q_upper_(3)  = q_upper_wb[14];
        q_upper_(4)  = q_upper_wb[15];
        q_upper_(5)  = q_upper_wb[6];
        q_upper_(6)  = q_upper_wb[7];
        q_upper_(7)  = q_upper_wb[13];
        q_upper_(8)  = q_upper_wb[12];
        q_upper_(9)  = q_upper_wb[5];
        q_upper_(10) = q_upper_wb[4];
        q_upper_(11) = q_upper_wb[0];
        q_upper_(12) = q_upper_wb[1];
        q_upper_(13) = q_upper_wb[17];
        q_upper_(14) = q_upper_wb[16];
        q_upper_(15) = q_upper_wb[9];
        q_upper_(16) = q_upper_wb[8];
        q_upper_(17) = q_upper_wb[10];
        q_upper_(18) = q_upper_wb[11];
        q_upper_(19) = q_upper_wb[2];
        q_upper_(20) = q_upper_wb[3];

        std::vector<float> q_lower_wb;
        nh_qp_.getParam("tocabi_controller/jointlimit_l", q_lower_wb);
        q_lower_(0)  = q_lower_wb[18];
        q_lower_(1)  = q_lower_wb[19];
        q_lower_(2)  = q_lower_wb[26];
        q_lower_(3)  = q_lower_wb[14];
        q_lower_(4)  = q_lower_wb[15];
        q_lower_(5)  = q_lower_wb[6];
        q_lower_(6)  = q_lower_wb[7];
        q_lower_(7)  = q_lower_wb[13];
        q_lower_(8)  = q_lower_wb[12];
        q_lower_(9)  = q_lower_wb[5];
        q_lower_(10) = q_lower_wb[4];
        q_lower_(11) = q_lower_wb[0];
        q_lower_(12) = q_lower_wb[1];
        q_lower_(13) = q_lower_wb[17];
        q_lower_(14) = q_lower_wb[16];
        q_lower_(15) = q_lower_wb[9];
        q_lower_(16) = q_lower_wb[8];
        q_lower_(17) = q_lower_wb[10];
        q_lower_(18) = q_lower_wb[11];
        q_lower_(19) = q_lower_wb[2];
        q_lower_(20) = q_lower_wb[3];

        std::vector<float> qdot_lim_wb;
        nh_qp_.getParam("tocabi_controller/vellimit", qdot_lim_wb);
        for(int i = 0; i < JOINT_DOF; i++){
            qdot_upper_(i) = qdot_lim_wb[12+i];
            qdot_lower_(i) = -qdot_lim_wb[12+i];
            qddot_upper_(i) = 10000;
            qddot_lower_(i) = -10000;
        }

        q_upper_ *= 1.0;
        q_lower_ *= 1.0;

        qdot_upper_ *= 1.0;
        qdot_lower_ *= 1.0;

        qddot_upper_ *= 0.3;
        qddot_lower_ *= 0.3;
    }

    void CartesianVelocityWB::setWeightMatrix()
    {
        std::vector<float> slack;
        nh_qp_.getParam("tocabi_cc/QP/Slack", slack);
        for(int i = 0; i < 6; i++){
            slack_weight_(i,i) = slack[i];
            slack_weight_(6+i,6+i) = slack[i];
            slack_weight_(12+i,12+i) = slack[i];
        }

        std::vector<float> damping;
        nh_qp_.getParam("tocabi_cc/QP/Damping", damping);
        for(int i = 0; i < JOINT_DOF; i++){
            damping_weight_(i,i) = damping[i];
        }

        std::vector<float> qddot_weight;
        nh_qp_.getParam("tocabi_cc/QP/qddot", qddot_weight);
        for(int i = 0; i < JOINT_DOF; i++){
            qddot_weight_(i,i) = qddot_weight[i];
        }

        // mani_weight_ = 0
    }

    void CartesianVelocityWB::setCost()
    {
        P_.block(si_index_.s,  si_index_.s,  ns_, ns_) = 2.0 * slack_weight_;
        P_.block(si_index_.dq, si_index_.dq, nq_, nq_) = 2.0 * damping_weight_;
        P_.block(si_index_.dq, si_index_.dq, nq_, nq_) += 2.0 * qddot_weight_;

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
        // q_.block(si_index_.dq1, 0, nq_, 1) = -mani_weight_ * robot_model_.getDManipulability(q_current_);
        q_.block(si_index_.dq, 0, nq_, 1) = -2. * qddot_weight_ * qdot_current_;
    }

    void CartesianVelocityWB::setConstraint()
    {
        A_.block(si_index_.con_slack, si_index_.s,  ns_, ns_).setIdentity(ns_, ns_);
        A_.block(si_index_.con_slack, si_index_.dq, ns_, nq_) = j_current_;
        A_.block(si_index_.con_q,     si_index_.dq, nq_, nq_).setIdentity(nq_, nq_);
        A_.block(si_index_.con_qdot,  si_index_.dq, nq_, nq_).setIdentity(nq_, nq_);
        A_.block(si_index_.con_qddot, si_index_.dq, nq_, nq_).setIdentity(nq_, nq_);

        l_.block(si_index_.con_slack, 0, ns_, 1) = xdot_desired_;
        l_.block(si_index_.con_q,     0, nq_, 1) = hz_ * (q_lower_ - q_current_);
        l_.block(si_index_.con_qdot,  0, nq_, 1) = qdot_lower_;
        l_.block(si_index_.con_qddot, 0, nq_, 1) = qddot_lower_ / hz_ + qdot_current_;
        
        u_.block(si_index_.con_slack, 0, ns_, 1) = xdot_desired_;
        u_.block(si_index_.con_q,     0, nq_, 1) = hz_ * (q_upper_ - q_current_);
        u_.block(si_index_.con_qdot,  0, nq_, 1) = qdot_upper_;
        u_.block(si_index_.con_qddot, 0, nq_, 1) = qddot_upper_ / hz_ + qdot_current_;
    }

    bool CartesianVelocityWB::getOptJointVel(Eigen::Matrix<double, JOINT_DOF, 1> &opt_qdot)
    {
        setCost();
        setConstraint();

        UpdateMinProblem(P_, q_);
        UpdateSubjectToAx(A_, l_, u_);
        DeleteSubjectToX();

        // cout << "P_: " << endl << P_ << endl;
        // cout << "q_: " << q_.transpose() << endl;
        // cout << "A_: " << endl << A_ << endl;
        // cout << "l_: " << l_.transpose() << endl;
        // cout << "u_: " << u_.transpose() << endl;

        // PrintMinProb();
        // PrintSubjectToAx();

        Eigen::VectorXd sol;
        sol.setZero(nx_);

        if (SolveQPoases(300, sol))
        {
            opt_qdot = sol.segment(si_index_.dq, JOINT_DOF);
            return true;
        }
        else
        {
            std::cout << "task solve failed" << std::endl;
            opt_qdot.setZero(JOINT_DOF, 1);
            return false;
        }
    }

} // namespace QP
