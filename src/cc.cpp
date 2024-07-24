#include "cc.h"

std::default_random_engine generator;
ros::Publisher new_cup_pos_pub;
geometry_msgs::Point new_cup_pos_msg_;
ros::Time init;

using namespace TOCABI;


CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    nh_cc_.setCallbackQueue(&queue_cc_);
    haptic_pose_sub_ = nh_cc_.subscribe("/haptic/pose", 1, &CustomController::HapticPoseCallback, this);
    cup_pose_sub = nh_cc_.subscribe("/cup_pose", 1, &CustomController::CupPoseCallback, this);
    joint_trajectory_sub = nh_cc_.subscribe("/tocabi/srmt/trajectory", 1, &CustomController::JointTrajectoryCallback, this);
    joint_target_sub = nh_cc_.subscribe("/tocabi/act/joint_target", 1, &CustomController::JointTargetCallback, this);
    haptic_force_pub_ = nh_cc_.advertise<geometry_msgs::Vector3>("/haptic/force", 10);
    ControlVal_.setZero();
    image_transport::ImageTransport it(nh_cc_);
    camera_flag_pub = nh_cc_.advertise<std_msgs::Bool>("/mujoco_ros_interface/camera/flag", 1);
    camera_image_sub = it.subscribe("/mujoco_ros_interface/camera/image", 1, &CustomController::camera_img_callback, this);
    new_cup_pos_pub = nh_cc_.advertise<geometry_msgs::Point>("/new_cup_pos", 1);
    terminate_pub = nh_cc_.advertise<std_msgs::Bool>("/tocabi/act/terminate", 1);
}

Eigen::VectorQd CustomController::getControl()
{
    return ControlVal_;
}

// void CustomController::taskCommandToCC(TaskCommand tc_)
// {
//     tc = tc_;
// }

void CustomController::PublishHapticData()
{
    geometry_msgs::Vector3 force;
    force.x = haptic_force_[0];
    force.y = haptic_force_[1];
    force.z = haptic_force_[2];

    haptic_force_pub_.publish(force);
}

double getRandomPosition(double minValue, double maxValue) 
        {
            std::uniform_real_distribution<double> distribution(minValue, maxValue);
            return distribution(generator);
        }



bool CustomController::saveImage(const sensor_msgs::ImageConstPtr &image_msg) {
    cv::Mat image;
    auto t_ = image_msg->header.stamp - init;
    try
    {
      image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
    }
    catch(cv_bridge::Exception)
    {
      ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), "bgr8");
      return false;
    }

    if (!image.empty()) {
        std::ostringstream oss;
        oss << std::setw(9) << std::setfill('0') << t_.nsec;
        std::stringstream fileNameSS;
        fileNameSS << "image_" << camera_tick_ << "_" << t_.sec << "_" << oss.str() << ".jpg";
        fileName_image = fileNameSS.str();

        std::stringstream filePathSS;
        filePathSS << folderPath_image << "/" << fileName_image;
        filePath_image = filePathSS.str();

        cv::imwrite(filePath_image, image);
        // ROS_INFO("Saved image %s", fileName.c_str());
    }
    else
    {
        ROS_ERROR("Couldn't save image, no data!");
        return false;
    }

    return true;
}

void CustomController::camera_img_callback(const sensor_msgs::ImageConstPtr &msg)
{
    if(data_collect_start_){
        try
        {
            // ROS_INFO("Camera Callback. '%d'", camera_tick_);
            // save the image
            if (!saveImage(msg)) return;
            // ROS_INFO("Image Saved.");
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
}


void CustomController::computeSlow()
{
    //MODE 6,7,8,9 is reserved for cc
    //MODE 6: joint target tracking for ACT
    //MODE 7: Homing & new cup pose
    //MODE 8: joint trajectory tracking for RRT & data collection
    //MODE 9: GUI end-effector pose tracking w/ HQP
    queue_cc_.callAvailable(ros::WallDuration());

    
    if (rd_.tc_.mode == 6)
    {
        // initialize tc, executed only once
        if (rd_.tc_init){
            std::cout << "mode 6 init!" << std::endl;
            rd_.tc_init = false;
            prev_mode = 6;

            camera_tick_ = 0;
            target_received = false;
            terminate_msg.data = false;
            terminate_pub.publish(terminate_msg);

            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = rd_.q_[i];
                desired_qdot_[i] = 0.0;
            }
        
            // initialize file for data collection
            std::stringstream folderPathSS;
            // [Check] ros::Time::now().sec is int32 type.
            auto now = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
            // folderPathSS << "/home/hokyun20/2024winter_ws/src/camera_pubsub/" << ros::Time::now().sec;
            folderPathSS << "/home/lyh/catkin_ws/src/tocabi/result/" << std::put_time(std::localtime(&currentTime), "%Y%m%d-%H%M%S");
            folderPath = folderPathSS.str();
            
            folderPathSS << "/image";
            folderPath_image = folderPathSS.str();

            int result1 = mkdir(folderPath.c_str(), 0777);
            int result2 = mkdir(folderPath_image.c_str(), 0777);
            if(result1 != 0){
                ROS_ERROR("Couldn't make folder(dir), Check folderPath");
            }
            if(result2 != 0){
                ROS_ERROR("Couldn't make folder2(dir), Check folderPath2");
            }

            std::stringstream filePathSS_hand;
            filePathSS_hand << folderPath << "/hand.txt";
            filePath_hand = filePathSS_hand.str();
            fout.open(filePath_hand);
            fout << "camera_tick" << "\t" << "ros_time" << "\t" << "hand_pose_x" << "\t" << "hand_pose_y" << "\t" << "hand_pose_z" << "\t" 
                    << "hand_pose_roll" << "\t" << "hand_pose_pitch" << "\t" << "hand_pose_yaw" << "\t" << "distance" << endl; 
            if(!fout.is_open()){
                ROS_ERROR("Couldn't open text file");
            }

            std::stringstream filePathSS_joint;
            filePathSS_joint << folderPath << "/joint.txt";
            filePath_joint = filePathSS_joint.str();
            fout2.open(filePath_joint);
            fout2 << "camera_tick" << "\t" << "ros_time" << "\t" 
                    << "current_joint_pos" << "\t" << "target_joint_pos" << "\t" 
                    << "current_joint_vel" << "\t" << "target_joint_vel" << endl; 
            if(!fout2.is_open()){
                ROS_ERROR("Couldn't open text file2");
            }

            std::stringstream filePathSS_info;
            filePathSS_info << folderPath << "/info.txt";
            filePath_info = filePathSS_info.str();
            fout3.open(filePath_info);
            fout3 << "cup_pose_x" << "\t" << "cup_pose_y" << "\t" << "cup_pose_z" << "\t" << "success" << endl;
            fout3 << cup_pos_(0) << "\t" << cup_pos_(1) << "\t" << cup_pos_(2) << "\t";
            if(!fout3.is_open()){
                ROS_ERROR("Couldn't open text file3");
            }
        }
        
        if(target_received){
            if(!data_collect_start_){
                data_collect_start_ = true;
                init = ros::Time::now();
            }
            // compute current distance between hand and cup for terminal condition check
            Eigen::Vector3d rhand_pos_;
            rhand_pos_ << rd_.link_[Right_Hand].xpos;
            
            float dx = cup_pos_(0) - rhand_pos_(0);
            float dy = cup_pos_(1) - rhand_pos_(1);
            float dz = cup_pos_(2) - rhand_pos_(2);
            distance_hand2obj = std::sqrt(dx*dx + dy*dy + dz*dz);

            WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
            // rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

            double t_ = (ros::Time::now() - init).toSec();
            double dt = 1/30;
            for(int i = 0; i < 8; i++){
                desired_q_[MODEL_DOF-8+i] = DyrosMath::cubic(t_, t_0_, t_0_+dt, q_0_[i], right_arm_target_[i], qdot_0_[i], 0);
                desired_qdot_[MODEL_DOF-8+i] = DyrosMath::cubicDot(t_, t_0_, t_0_+dt, q_0_[i], right_arm_target_[i], qdot_0_[i], 0);
            }

            if(camera_tick_%16 == 0){
                // write data to the file
                fout << camera_tick_ << "\t" << t_ << "\t" << rhand_pos_(0) << "\t" << rhand_pos_(1) << "\t" << rhand_pos_(2) << "\t" 
                        << rd_.link_[Right_Hand].roll << "\t"<< rd_.link_[Right_Hand].pitch << "\t"<< rd_.link_[Right_Hand].yaw << "\t" << distance_hand2obj << endl;
                
                fout2 << camera_tick_ << "\t" << t_ << "\t";
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << rd_.q_[i] << "\t";
                }
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << desired_q_[i] << "\t";
                }
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << rd_.q_dot_[i] << "\t";
                }
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << desired_qdot_[i] << "\t";
                }
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << right_arm_target_[i] << "\t";
                }
                fout2 << endl;
            }
            camera_tick_++;
            
            if (t_ > 10.0){
                bool is_reached = (distance_hand2obj < 0.3);
                fout3 << is_reached << endl;
                std::cout<< (is_reached ? "Reached" : "Failed") << std::endl;

                terminate_msg.data = true;
                terminate_pub.publish(terminate_msg);

                data_collect_start_ = false;
                rd_.tc_.mode = 7;
                rd_.tc_init = true;
            }
        }
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        }
    }
    else if (rd_.tc_.mode == 7)
    {
        // std::cout << "tc 7" << std::endl;
        if(fout.is_open()==true)
        {
            fout.close();
        }
        if(fout2.is_open()==true)
        {
            fout2.close();
        }
        if(fout3.is_open()==true)
        {
            fout3.close();
        }
        
        double duration = 3.0;
        if (!target_reached_)
        {
            target_reached_ = true;
            q_init_ = rd_.q_;
            time_init_ = rd_.control_time_;
        }
        resetRobotPose(duration);

        if (rd_.control_time_ > time_init_ + duration)
        {
            const double minX = -0.1;
            const double maxX = 0.1;
            const double minY = -0.3;
            const double maxY = 0.3;

            new_cup_pos_msg_.x = getRandomPosition(minX, maxX);
            new_cup_pos_msg_.y = getRandomPosition(minY, maxY);
            new_cup_pos_msg_.z = 0.0;
            new_cup_pos_pub.publish(new_cup_pos_msg_);

            rd_.tc_.mode = prev_mode;
            target_reached_ = false;
        }
    }
    else if (rd_.tc_.mode == 8)
    {
        // initialize tc, executed only once
        if (rd_.tc_init){
            std::cout << "mode 8 init!" << std::endl;
            rd_.tc_init = false;
            prev_mode = 8;

            camera_tick_ = 0;

            for(int i = 0; i < MODEL_DOF; i++){
                desired_q_[i] = rd_.q_[i];
                desired_qdot_[i] = 0.0;
            }
        
            // initialize file for data collection
            std::stringstream folderPathSS;
            // [Check] ros::Time::now().sec is int32 type.
            auto now = std::chrono::system_clock::now();
            std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
            // folderPathSS << "/home/hokyun20/2024winter_ws/src/camera_pubsub/" << ros::Time::now().sec;
            folderPathSS << "/home/lyh/catkin_ws/src/tocabi/data/" << std::put_time(std::localtime(&currentTime), "%Y%m%d-%H%M%S");
            folderPath = folderPathSS.str();
            
            folderPathSS << "/image";
            folderPath_image = folderPathSS.str();

            int result1 = mkdir(folderPath.c_str(), 0777);
            int result2 = mkdir(folderPath_image.c_str(), 0777);
            if(result1 != 0){
                ROS_ERROR("Couldn't make folder(dir), Check folderPath");
            }
            if(result2 != 0){
                ROS_ERROR("Couldn't make folder2(dir), Check folderPath2");
            }

            std::stringstream filePathSS_hand;
            filePathSS_hand << folderPath << "/hand.txt";
            filePath_hand = filePathSS_hand.str();
            fout.open(filePath_hand);
            fout << "camera_tick" << "\t" << "ros_time" << "\t" << "hand_pose_x" << "\t" << "hand_pose_y" << "\t" << "hand_pose_z" << "\t" 
                    << "hand_pose_roll" << "\t" << "hand_pose_pitch" << "\t" << "hand_pose_yaw" << "\t" << "distance" << endl; 
            if(!fout.is_open()){
                ROS_ERROR("Couldn't open text file");
            }

            std::stringstream filePathSS_joint;
            filePathSS_joint << folderPath << "/joint.txt";
            filePath_joint = filePathSS_joint.str();
            fout2.open(filePath_joint);
            fout2 << "camera_tick" << "\t" << "ros_time" << "\t" 
                    << "current_joint_pos" << "\t" << "target_joint_pos" << "\t" 
                    << "current_joint_vel" << "\t" << "target_joint_vel" << endl; 
            if(!fout2.is_open()){
                ROS_ERROR("Couldn't open text file2");
            }

            std::stringstream filePathSS_info;
            filePathSS_info << folderPath << "/info.txt";
            filePath_info = filePathSS_info.str();
            fout3.open(filePath_info);
            fout3 << "cup_pose_x" << "\t" << "cup_pose_y" << "\t" << "cup_pose_z" << "\t" << "success" << endl;
            fout3 << cup_pos_(0) << "\t" << cup_pos_(1) << "\t" << cup_pos_(2) << "\t";
            if(!fout3.is_open()){
                ROS_ERROR("Couldn't open text file3");
            }
        }
            
        if(data_collect_start_){
            // compute current distance between hand and cup for terminal condition check
            Eigen::Vector3d rhand_pos_;
            rhand_pos_ << rd_.link_[Right_Hand].xpos;
            
            float dx = cup_pos_(0) - rhand_pos_(0);
            float dy = cup_pos_(1) - rhand_pos_(1);
            float dz = cup_pos_(2) - rhand_pos_(2);
            distance_hand2obj = std::sqrt(dx*dx + dy*dy + dz*dz);

            WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
            // rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

            if (init_time){
                traj_index = 0;
                init_time = false;
                init = ros::Time::now();
            }
            double t_ = (ros::Time::now() - init).toSec();
            if (traj_index != -1){
                if (t_ > points[traj_index+1].time_from_start.toSec()){
                    traj_index++;
                }
                if (traj_index < num_waypoint-1){
                    double t_0 = points[traj_index].time_from_start.toSec();
                    auto   p_0 = points[traj_index].positions;
                    auto   v_0 = points[traj_index].velocities;
                    double t_f = points[traj_index+1].time_from_start.toSec();
                    auto   p_f = points[traj_index+1].positions;
                    auto   v_f = points[traj_index+1].velocities;
                    for(int i = 0; i < 8; i++){                    
                        desired_q_[MODEL_DOF-8+i] = DyrosMath::cubic(t_, t_0, t_f, p_0[i], p_f[i], v_0[i], v_f[i]);
                        desired_qdot_[MODEL_DOF-8+i] = DyrosMath::cubicDot(t_, t_0, t_f, p_0[i], p_f[i], v_0[i], v_f[i]);
                    }
                }
                else{
                    for(int i = 0; i < 8; i++){                    
                        desired_q_[MODEL_DOF-8+i] = points[traj_index].positions[i];
                        desired_qdot_[MODEL_DOF-8+i] = points[traj_index].velocities[i];
                    }
                }
            }

            // write data to the file
            if(camera_tick_%16 == 0){
                // ROS_INFO("I heard: [%d]", camera_tick_);
                camera_flag_msg.data = true;
                camera_flag_pub.publish(camera_flag_msg);
                fout << camera_tick_ << "\t" << t_ << "\t" << rhand_pos_(0) << "\t" << rhand_pos_(1) << "\t" << rhand_pos_(2) << "\t" 
                     << rd_.link_[Right_Hand].roll << "\t"<< rd_.link_[Right_Hand].pitch << "\t"<< rd_.link_[Right_Hand].yaw << "\t" << distance_hand2obj << endl;
                fout2 << camera_tick_ << "\t" << t_ << "\t";
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << rd_.q_[i] << "\t";
                }
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << desired_q_[i] << "\t";
                }
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << rd_.q_dot_[i] << "\t";
                }
                for(int i = 0; i < MODEL_DOF; i++){
                    fout2 << desired_qdot_[i] << "\t";
                }
                fout2 << endl;
            }
            camera_tick_++;

            if (traj_index == num_waypoint-1){
                bool is_reached = (distance_hand2obj < 0.3);
                fout3 << is_reached << endl;
                std::cout<< (is_reached ? "Reached" : "Failed") << std::endl;
                data_collect_start_ = false;
                rd_.tc_.mode = 7;
                rd_.tc_init = true;
                traj_index = -1;
            }
        }
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        }
    }

    if (rd_.tc_.mode == 9)
    {
        double ang2rad = 0.0174533;

        static bool init_qp;
        
        static VectorQd init_q_mode6;

        static Matrix3d rot_hand_init;
        static Matrix3d rot_haptic_init;

        static Vector3d pos_hand_init;
        static Vector3d pos_haptic_init;


        if (rd_.tc_init) //한번만 실행됩니다(gui)
        {
            init_qp = true;

            std::cout << "mode 9 init!" << std::endl;
            rd_.tc_init = false;
            rd_.link_[COM_id].x_desired = rd_.link_[COM_id].x_init;

            init_q_mode6 = rd_.q_;

            rot_hand_init = rd_.link_[Right_Hand].rotm;
            pos_hand_init = rd_.link_[Right_Hand].xpos;
        }

        WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
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
            torque_pos_hold[i] = rd_.pos_kp_v[i] * (init_q_mode6[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * ( - rd_.q_dot_[i]);
        }

        torque_pos_hold.segment(25,8).setZero();

        VectorQd torque_right_arm;
        
        torque_right_arm.setZero();

        torque_right_arm.segment(25,8) = WBC::ContactForceRedistributionTorque(rd_, torque_Task2).segment(25,8);

        for (int i=12;i<MODEL_DOF;i++)
        {
            rd_.torque_desired[i] = torque_pos_hold[i] + torque_right_arm[i];
        }
    }

}

void CustomController::resetRobotPose(double duration)
{
    Eigen::Matrix<double, MODEL_DOF, 1> q_target;
    Eigen::Matrix<double, MODEL_DOF, 1> q_cubic;
    q_target << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, 0.0,
            0.3, 0.3, 1.5, -1.27, -1.0, 0.0, -1.0, 0.0,
            0.0, 0.0,
            -0.3, -0.9, -1.5, 1.57, 1.9, 0.0, 0.5, 0.0;
    for (int i = 0; i <MODEL_DOF; i++)
    {
        q_cubic(i) = DyrosMath::cubic(rd_.control_time_, time_init_, time_init_ +duration, q_init_(i), q_target(i), 0.0, 0.0);
    }
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kp;
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kv;

    kp.setZero();
    kv.setZero();
    kp.diagonal() <<   2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
                        6000.0, 10000.0, 10000.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0,
                        100.0, 100.0,
                        400.0, 1000.0, 400.0, 400.0, 400.0, 400.0, 100.0, 100.0;
    kv.diagonal() << 15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        15.0, 50.0, 20.0, 25.0, 24.0, 24.0,
                        200.0, 100.0, 100.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0,
                        2.0, 2.0,
                        10.0, 28.0, 10.0, 10.0, 10.0, 10.0, 3.0, 3.0;


    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
    rd_.torque_desired =  kp*(q_cubic-rd_.q_)-kv* rd_.q_dot_ +  WBC::GravityCompensationTorque(rd_);
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

void CustomController::HapticPoseCallback(const geometry_msgs::PoseConstPtr &msg)
{

    float pos_x = CustomController::PositionMapping(msg -> position.x, 0);
    float pos_y = CustomController::PositionMapping(msg -> position.y, 1);
    float pos_z = CustomController::PositionMapping(msg -> position.z, 2);
    float ori_x = CustomController::PositionMapping(msg -> orientation.x, 3);
    float ori_y = CustomController::PositionMapping(msg -> orientation.y, 4);
    float ori_z = CustomController::PositionMapping(msg -> orientation.z, 5);
    float ori_w = CustomController::PositionMapping(msg -> orientation.w, 6);

    // double posx = static_cast<double>(pos_x);
    // double posy = static_cast<double>(pos_y); 
    // double posz = static_cast<double>(pos_z);
    double orix = static_cast<double>(ori_x);
    double oriy = static_cast<double>(ori_y); 
    double oriz = static_cast<double>(ori_z);
    double oriw = static_cast<double>(ori_w);

    haptic_pos_[0] = pos_x;
    haptic_pos_[1] = pos_y; 
    haptic_pos_[2] = pos_z;

    haptic_orientation_ = CustomController::Quat2rotmatrix(orix, oriy, oriz, oriw);

}

void CustomController::CupPoseCallback(const geometry_msgs::PoseConstPtr &msg)
{
    float cup_x = msg->position.x;
    float cup_y = msg->position.y;
    float cup_z = msg->position.z;
 
    cup_pos_ << cup_x, cup_y, cup_z;
    // std::cout << "CupPos subscribed!" << std::endl;
}

void CustomController::JointTrajectoryCallback(const trajectory_msgs::JointTrajectoryPtr &msg)
{
    points = msg->points;
    data_collect_start_ = true;
    init_time = true;
    num_waypoint = points.size();
}

void CustomController::JointTargetCallback(const sensor_msgs::JointStatePtr &msg)
{
    t_0_ = (msg->header.stamp - init).toSec();
    q_0_ = rd_.q_.block<8,1>(25,0);
    qdot_0_ = rd_.q_dot_.block<8,1>(25,0);
    right_arm_target_ = msg->position;
    target_received = true;
}

float CustomController::PositionMapping(float haptic_val, int i)
{
    if (i == 0){
        return -1 * (haptic_val + 0.051448) * 5.0 ;
    }

    else if(i == 1){
        return -1 * (haptic_val + 0.000152) * 5.0;
    }

    else if (i == 2){
        return (haptic_val - 0.007794) * 5.0;
    }
    else {
     return haptic_val;
    }
    
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