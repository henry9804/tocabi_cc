#include "cc.h"

std::default_random_engine generator;
ros::Publisher new_cup_pos_pub;
geometry_msgs::Point new_cup_pos_msg_;

using namespace TOCABI;


CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    nh_cc_.setCallbackQueue(&queue_cc_);
    mode_sub = nh_cc_.subscribe("/tocabi/act/mode", 1, &CustomController::ModeCallback, this);
    joint_target_sub = nh_cc_.subscribe("/tocabi/act/joint_target", 1, &CustomController::JointTargetCallback, this);
    ControlVal_.setZero();

    for(int i = 0; i < MODEL_DOF; i++){
        JOINT_INDEX.insert({JOINT_NAME[i], i});
    }
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

double getRandomPosition(double minValue, double maxValue) 
        {
            std::uniform_real_distribution<double> distribution(minValue, maxValue);
            return distribution(generator);
        }

void CustomController::computeSlow()
{
    //MODE 6,7,8 are reserved for cc
    //MODE 6: ready pose
    //MODE 7: joint command
    //MODE 8: task command
    queue_cc_.callAvailable(ros::WallDuration());

    
    if (rd_.tc_.mode == 6)
    {
        // initialize tc, executed only once
        if (rd_.tc_init){
            std::cout << "mode 6 init!" << std::endl;
            rd_.tc_init = false;
            q_init_ = rd_.q_;
            time_init_ = rd_.control_time_;
        }
        resetRobotPose(3.0);
    }
    else if (rd_.tc_.mode == 7)
    {
        // initialize tc, executed only once
        if (rd_.tc_init){
            std::cout << "mode 7 init!" << std::endl;
            rd_.tc_init = false;

            target_received = false;
            time_init_ = rd_.control_time_;

            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = rd_.q_[i];
                desired_qdot_[i] = 0.0;
            }

            // initialize file for data logging
            std::stringstream folderPathSS;
            // [Check] ros::Time::now().sec is int32 type.
            auto now = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
            folderPathSS << "/home/dyros/catkin_ws/src/tocabi_cc/log/" << std::put_time(std::localtime(&currentTime), "%Y%m%d_%H%M%S");
            folderPath = folderPathSS.str();

            int result = mkdir(folderPath.c_str(), 0777);
            if(result != 0){
                ROS_ERROR("Couldn't make folder(dir), Check folderPath");
            }

            std::stringstream filePathSS_joint;
            filePathSS_joint << folderPath << "/joint.txt";
            filePath_joint = filePathSS_joint.str();
            fout.open(filePath_joint);
            fout << "elapsed_time" << "\t" 
                 << "current_joint_pos" << "\t" << "target_joint_pos" << "\t" 
                 << "current_joint_vel" << "\t" << "target_joint_vel" << "\t" 
                 << "recieved_target" << endl; 
            if(!fout.is_open()){
                ROS_ERROR("Couldn't open text file2");
            }
        }
        
        if(target_received){
            double duration = 0.1;
            for(int i = 0; i < joint_names_.size(); i++){
                int j = JOINT_INDEX[joint_names_[i]];
                desired_q_[j] = DyrosMath::cubic(rd_.control_time_, t_0_, t_0_+duration, q_0_[j], joint_target_[i], qdot_0_[j], 0);
                desired_qdot_[j] = DyrosMath::cubicDot(rd_.control_time_, t_0_, t_0_+duration, q_0_[j], joint_target_[i], qdot_0_[j], 0);
            }

            // write data to the file
            fout <<  rd_.control_time_ - time_init_ << "\t";
            for(int i = 0; i < MODEL_DOF; i++){
                fout << rd_.q_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout << desired_q_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout << rd_.q_dot_[i] << "\t";
            }
            for(int i = 0; i < MODEL_DOF; i++){
                fout << desired_qdot_[i] << "\t";
            }
            for(int i = 0; i < joint_names_.size(); i++){
                fout << joint_target_[i] << "\t";
            }
            fout << endl;
        }
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        }
        // gravity compensation
        WBC::SetContact(rd_, 1, 1);
        rd_.torque_desired = rd_.torque_desired + WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
    }
    else if (rd_.tc_.mode == 9)
    {
        double ang2rad = 0.0174533;

        static bool init_qp;
        
        static VectorQd init_q_;

        static Matrix3d rot_hand_init;
        static Matrix3d rot_haptic_init;

        static Vector3d pos_hand_init;
        static Vector3d pos_haptic_init;


        if (rd_.tc_init) //한번만 실행됩니다(gui)
        {
            init_qp = true;

            std::cout << "mode 8 init!" << std::endl;
            rd_.tc_init = false;
            rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;

            time_init_ = rd_.control_time_;
            init_q_ = rd_.q_;

            rot_hand_init = rd_.link_[Right_Hand].rotm;
            pos_hand_init = rd_.link_[Right_Hand].xpos;
        
            // initialize file for data collection
            std::stringstream folderPathSS;
            // [Check] ros::Time::now().sec is int32 type.
            auto now = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
            folderPathSS << "/home/dyros/catkin_ws/src/tocabi_cc/log/" << std::put_time(std::localtime(&currentTime), "%Y%m%d-%H%M%S");
            folderPath = folderPathSS.str();
            
            int result = mkdir(folderPath.c_str(), 0777);
            if(result != 0){
                ROS_ERROR("Couldn't make folder(dir), Check folderPath");
            }

            std::stringstream filePathSS_hand;
            filePathSS_hand << folderPath << "/hand.txt";
            filePath_hand = filePathSS_hand.str();
            fout.open(filePath_hand);
            fout << "elapsed_time" << "\t" << "hand_pose_x" << "\t" << "hand_pose_y" << "\t" << "hand_pose_z" << "\t" 
                 << "hand_pose_roll" << "\t" << "hand_pose_pitch" << "\t" << "hand_pose_yaw" << endl; 
            if(!fout.is_open()){
                ROS_ERROR("Couldn't open text file");
            }

        }

        WBC::SetContact(rd_, 1, 1);
        if (rd_.tc_.customTaskGain)
        {
            rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
            rd_.link_[Right_Hand].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
        }

        rd_.link_[Pelvis].x_desired = rd_.tc_.ratio * rd_.link_[Left_Foot].x_init + (1 - rd_.tc_.ratio) * rd_.link_[Right_Foot].x_init;
        rd_.link_[Pelvis].x_desired(2) = rd_.tc_.height;
        rd_.link_[Pelvis].rot_desired = DyrosMath::rotateWithY(rd_.tc_.pelv_pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.link_[Pelvis].yaw_init);

        rd_.link_[Right_Hand].x_desired = rd_.link_[Right_Hand].x_init;
        rd_.link_[Right_Hand].x_desired(0) += rd_.tc_.r_x;
        rd_.link_[Right_Hand].x_desired(1) += rd_.tc_.r_y;
        rd_.link_[Right_Hand].x_desired(2) += rd_.tc_.r_z;
        rd_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(rd_.tc_.r_roll * ang2rad) * DyrosMath::rotateWithY(rd_.tc_.r_pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.tc_.r_yaw * ang2rad) * DyrosMath::Euler2rot(0, 1.5708, -1.5708).transpose();

        rd_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithX(rd_.tc_.roll * ang2rad) * DyrosMath::rotateWithY(rd_.tc_.pitch * ang2rad) * DyrosMath::rotateWithZ(rd_.tc_.yaw * ang2rad);

        rd_.link_[Pelvis].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, rd_.link_[Pelvis].xi_init, rd_.link_[Pelvis].x_desired);
        rd_.link_[Pelvis].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        Vector3d hand_pos_desired = rd_.link_[Right_Hand].x_desired;
        Matrix3d hand_rot_desired = rot_hand_init;

        // write data to the file
        fout << rd_.control_time_ - time_init_ << "\t" << rd_.link_[Right_Hand].xpos(0) << "\t" << rd_.link_[Right_Hand].xpos(1) << "\t" << rd_.link_[Right_Hand].xpos(2) << "\t" 
                << rd_.link_[Right_Hand].roll << "\t"<< rd_.link_[Right_Hand].pitch << "\t"<< rd_.link_[Right_Hand].yaw << endl;

        rd_.link_[Right_Hand].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, hand_pos_desired);
        
        rd_.link_[Upper_Body].SetTrajectoryRotation(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time);

        rd_.torque_grav = WBC::GravityCompensationTorque(rd_);
        TaskSpace ts_(6);
        Eigen::MatrixXd Jtask = rd_.link_[Pelvis].JacCOM();
        Eigen::VectorXd fstar = WBC::GetFstar6d(rd_.link_[Pelvis], true, true);

        ts_.Update(Jtask, fstar);
        WBC::CalcJKT(rd_, ts_);
        WBC::CalcTaskNull(rd_, ts_);
        static CQuadraticProgram task_qp_;
        WBC::TaskControlHQP(rd_, ts_, task_qp_, rd_.torque_grav, MatrixXd::Identity(MODEL_DOF, MODEL_DOF), init_qp);

        VectorQd torque_Task2 = ts_.torque_h_ + rd_.torque_grav;

        TaskSpace ts1_(6);
        Eigen::MatrixXd Jtask1 = rd_.link_[Right_Hand].Jac();
        Eigen::VectorXd fstar1 = WBC::GetFstar6d(rd_.link_[Right_Hand], true);

        ts1_.Update(Jtask1, fstar1);
        WBC::CalcJKT(rd_, ts1_);
        WBC::CalcTaskNull(rd_, ts1_);
        static CQuadraticProgram task_qp1_;
        WBC::TaskControlHQP(rd_, ts1_, task_qp1_, torque_Task2, ts_.Null_task, init_qp);

        torque_Task2 = ts_.torque_h_ + ts_.Null_task * ts1_.torque_h_ + rd_.torque_grav;

        TaskSpace ts2_(3);
        Eigen::MatrixXd Jtask2 = rd_.link_[Upper_Body].Jac().bottomRows(3);
        Eigen::VectorXd fstar2 = WBC::GetFstarRot(rd_.link_[Upper_Body]);
        ts2_.Update(Jtask2, fstar2);
        WBC::CalcJKT(rd_, ts2_);

        static CQuadraticProgram task_qp2_;
        WBC::TaskControlHQP(rd_, ts2_, task_qp2_, torque_Task2, ts_.Null_task * ts1_.Null_task, init_qp);

        torque_Task2 = ts_.torque_h_ + ts_.Null_task * ts1_.torque_h_ + ts_.Null_task * ts1_.Null_task * ts2_.torque_h_ + rd_.torque_grav;

        VectorQd torque_pos_hold;

        for (int i=0;i<MODEL_DOF;i++)
        {
            torque_pos_hold[i] = rd_.pos_kp_v[i] * (init_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * ( - rd_.q_dot_[i]);
        }
        torque_pos_hold += WBC::ContactForceRedistributionTorque(rd_, rd_.torque_grav);

        torque_pos_hold.segment(25,8).setZero();

        VectorQd torque_right_arm;
        
        torque_right_arm.setZero();

        torque_right_arm.segment(25,8) = WBC::ContactForceRedistributionTorque(rd_, torque_Task2).segment(25,8);

        for (int i=12;i<MODEL_DOF;i++)
        {
            rd_.torque_desired[i] = torque_pos_hold[i] + torque_right_arm[i];
        }
    }
    else if (rd_.tc_.mode == 8){        
        double ang2rad = 0.0174533;

        static bool init_qp;

        static Matrix3d r_rot_hand_init;

        static Vector3d r_pos_hand_init;
        static Vector3d r_pos_des_init;

        double alpha = 0.1; // 필터 계수 (조정 가능)
        Eigen::VectorXd filtered_qdot_(MODEL_DOF);
        Eigen::VectorXd filtered_q_(MODEL_DOF);

        if (rd_.tc_init)
        {
            init_qp = true;

            std::cout << "mode 8 init!" << std::endl;
            rd_.tc_init = false;
            rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;

            r_rot_hand_init = rd_.link_[Right_Hand].rotm;
            r_pos_hand_init = rd_.link_[Right_Hand].xpos;
            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = rd_.q_[i];
                desired_qdot_[i] = 0.0;
            }
            des_r_pos_ = rd_.link_[Right_Hand].xpos;
            des_r_pos_(0) += rd_.tc_.r_x;
            des_r_pos_(1) += rd_.tc_.r_y;
            des_r_pos_(2) += rd_.tc_.r_z;
            des_r_orientation_ = r_rot_hand_init;
        }
                
        WBC::SetContact(rd_, 1, 1);
        // if (rd_.tc_.customTaskGain)
        // {
        //     // rd_.link_[Pelvis].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
        //     rd_.link_[Upper_Body].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
        //     rd_.link_[Right_Hand].SetGain(rd_.tc_.pos_p, rd_.tc_.pos_d, rd_.tc_.acc_p, rd_.tc_.ang_p, rd_.tc_.ang_d, 1);
        // }
        
        // ARM // right ram jac
        // rd_.link_[Right_Hand].x_desired = des_r_pos_;
        // Vector3d hand_r_pos_desired = rd_.link_[Right_Hand].x_desired;

        rd_.link_[Right_Hand].SetTrajectoryQuintic(rd_.control_time_, rd_.tc_time_, rd_.tc_time_ + rd_.tc_.time, des_r_pos_);
        Vector3d hand_r_pos_desired = rd_.link_[Right_Hand].x_traj;
        Matrix3d hand_r_rot_desired = des_r_orientation_;

        DyrosMath::rot2Euler_tf2(hand_r_rot_desired, drr_, drp_, dry_);

        Eigen::MatrixXd J = rd_.link_[Right_Hand].Jac();
        Eigen::MatrixXd J_r_arm = J.block(0, 6+33-8, 6, 8);
        Eigen::VectorXd r_pose_current(6); 
        r_pose_current << rd_.link_[Right_Hand].xpos(0), rd_.link_[Right_Hand].xpos(1), rd_.link_[Right_Hand].xpos(2), rr_, rp_, ry_;

        Eigen::VectorXd r_pose_desired(6); 
        r_pose_desired << hand_r_pos_desired(0), hand_r_pos_desired(1), hand_r_pos_desired(2), drr_, drp_, dry_;

        Eigen::VectorXd r_rot_diff(3); 
        r_rot_diff = DyrosMath::getPhi(rd_.link_[Right_Hand].rotm, hand_r_rot_desired);

        Eigen::VectorXd r_pose_error(6); 
        r_pose_error = r_pose_desired - r_pose_current;
        r_pose_error(3)=-r_rot_diff(0);
        r_pose_error(4)=-r_rot_diff(1);
        r_pose_error(5)=-r_rot_diff(2);

        Eigen::MatrixXd JtJ_r= J_r_arm.transpose() * J_r_arm;

        Eigen::VectorXd Kp_r_(6);
        // Kp_r_ << 2.0, 2.0, 2.0, 1.5, 1.5, 1.5; 
        Kp_r_ << 1.5, 1.5, 1.5, 1.5, 1.5, 1.5; 

        Eigen::MatrixXd dq_r = JtJ_r.ldlt().solve(J_r_arm.transpose() * Kp_r_.cwiseProduct(r_pose_error));

        for(int i=0; i<8; i++)
        {
            desired_qdot_[MODEL_DOF-8+i] = dq_r(i);
            desired_q_[MODEL_DOF-8+i] += dq_r(i) * 0.0005;
        }

        // for (int i = 0; i < MODEL_DOF; i++){
        //     rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        // }

        // 필터 초기값 설정
        for (int i = 0; i < MODEL_DOF; i++) {
            filtered_qdot_[i] = desired_qdot_[i];
            filtered_q_[i] = desired_q_[i];
        }

        // 기존 코드 ...
        for (int i = 0; i < 8; i++)
        {
            desired_qdot_[MODEL_DOF - 8 + i] = dq_r(i);
            // 필터 적용
            filtered_qdot_[MODEL_DOF - 8 + i] = alpha * desired_qdot_[MODEL_DOF - 8 + i] + (1 - alpha) * filtered_qdot_[MODEL_DOF - 8 + i];
            
            desired_q_[MODEL_DOF - 8 + i] += dq_r(i) * 0.0005;
            // 필터 적용
            filtered_q_[MODEL_DOF - 8 + i] = alpha * desired_q_[MODEL_DOF - 8 + i] + (1 - alpha) * filtered_q_[MODEL_DOF - 8 + i];
        }

        // 나머지 코드 ...
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (filtered_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (filtered_qdot_[i] - rd_.q_dot_[i]);
        }
        rd_.torque_desired = rd_.torque_desired + WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));

        init_qp = false;
    }

}

void CustomController::resetRobotPose(double duration)
{
    Eigen::Matrix<double, MODEL_DOF, 1> q_target;
    Eigen::Matrix<double, MODEL_DOF, 1> q_cubic;
    q_target << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, 0.0,
            0.0, -0.3, 1.57, -1.2, -1.57, 1.5, 0.4, -0.2,
            0.0, 0.3,
            0.0, 0.3, -1.57, 1.2, 1.57, -1.5, -0.4, 0.2;

    for (int i = 0; i <MODEL_DOF; i++)
    {
        q_cubic(i) = DyrosMath::cubic(rd_.control_time_, time_init_, time_init_ +duration, q_init_(i), q_target(i), 0.0, 0.0);
    }
    for (int i = 0; i < MODEL_DOF; i++)
    {
        rd_.torque_desired[i] = rd_.pos_kp_v[i] * (q_cubic[i] - rd_.q_[i]) - rd_.pos_kv_v[i] * rd_.q_dot_[i];
    }

    WBC::SetContact(rd_, 1, 1);
    rd_.torque_desired =  rd_.torque_desired + WBC::ContactForceRedistributionTorque(rd_, WBC::GravityCompensationTorque(rd_));
}


void CustomController::computeFast()
{
    if (rd_.tc_.mode == 6)
    {
    }
    else if (rd_.tc_.mode == 7)
    {
    }
}

void CustomController::ModeCallback(const std_msgs::Int32Ptr &msg)
{
    std::cout << "act mode msg: " << msg->data << std::endl;
    if (msg->data == 0) {
        rd_.tc_.mode = 6;
        rd_.tc_init = true;
    }
    else if (msg->data == 1) {
        rd_.tc_.mode = 7;
        rd_.tc_init = true;
        desired_q_ = rd_.q_;
    }
}

void CustomController::JointTargetCallback(const sensor_msgs::JointStatePtr &msg)
{
    t_0_ = rd_.control_time_;
    q_0_ = desired_q_;
    qdot_0_ = rd_.q_dot_;
    joint_names_ = msg->name;
    joint_target_ = msg->position;
    target_received = true;
}

Eigen::Matrix3d CustomController::Quat2rotmatrix(double q0, double q1, double q2, double q3)
{
    double r00 = 2 * (q0 * q0 + q1 * q1) - 1 ;
    double r01 = 2 * (q1 * q2 - q0 * q3) ;
    double r02 = 2 * (q1 * q3 + q0 * q2) ;

    double r10 = 2 * (q1 * q2 + q0 * q3) ;
    double r11 = 2 * (q0 * q0 + q2 * q2) - 1 ;
    double r12 = 2 * (q2 * q3 - q0 * q1) ;

    double r20 = 2 * (q1 * q3 - q0 * q2) ;
    double r21 = 2 * (q2 * q3 + q0 * q1) ;
    double r22 = 2 * (q0 * q0 + q3 * q3) - 1 ;

    Eigen::Matrix3d rot_matrix;
    rot_matrix << r00, r01, r02,
                    r10, r11, r12,
                    r20, r21, r22;
    return rot_matrix;
}

void CustomController::computePlanner()
{
}

void CustomController::copyRobotData(RobotData &rd_l)
{
    std::memcpy(&rd_cc_, &rd_l, sizeof(RobotData));
}

Eigen::MatrixXd CustomController::LowPassFilter(const Eigen::MatrixXd &input, const Eigen::MatrixXd &prev_res, const double &sampling_freq, const double &cutoff_freq)
{
    double rc = 1. / (cutoff_freq * 2 * M_PI);
    double dt = 1. / sampling_freq;
    double a = dt / (rc + dt);
    return prev_res + a * (input - prev_res);
}