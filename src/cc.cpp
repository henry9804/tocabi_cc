#include "cc.h"

std::default_random_engine generator;
ros::Publisher new_obj_pose_pub;
geometry_msgs::Pose new_obj_pose_msg_;
ros::Time init;

using namespace TOCABI;


CustomController::CustomController(RobotData &rd) : rd_(rd) //, wbc_(dc.wbc_)
{
    nh_cc_.setCallbackQueue(&queue_cc_);
    haptic_pose_sub_ = nh_cc_.subscribe("/haptic/pose", 1, &CustomController::HapticPoseCallback, this);
    obj_pose_sub = nh_cc_.subscribe("/obj_pose", 1, &CustomController::ObjPoseCallback, this);
    joint_trajectory_sub = nh_cc_.subscribe("/tocabi/srmt/trajectory", 1, &CustomController::JointTrajectoryCallback, this);
    joint_target_sub = nh_cc_.subscribe("/tocabi/act/joint_target", 1, &CustomController::JointTargetCallback, this);
    pose_target_sub = nh_cc_.subscribe("/tocabi/act/pose_target", 1, &CustomController::PoseTargetCallback, this);
    haptic_force_pub_ = nh_cc_.advertise<geometry_msgs::Vector3>("/haptic/force", 10);
    ControlVal_.setZero();
    image_transport::ImageTransport it(nh_cc_);
    robot_pose_pub = nh_cc_.advertise<geometry_msgs::PoseArray>("/tocabi/robot_poses", 1);
    robot_pose_msg.poses.resize(3);
    camera_image_sub = it.subscribe("/mujoco_ros_interface/camera/image", 1, &CustomController::camera_img_callback, this);
    new_obj_pose_pub = nh_cc_.advertise<geometry_msgs::Pose>("/new_obj_pose", 1);
    terminate_pub = nh_cc_.advertise<std_msgs::Bool>("/tocabi/act/terminate", 1);
    hand_open_pub = nh_cc_.advertise<std_msgs::Int32>("/mujoco_ros_interface/hand_open", 1);
    hand_open_msg.data = 0;

    for(int i = 0; i < MODEL_DOF; i++){
        JOINT_INDEX.insert({JOINT_NAME[i], i});
    }

    kp.setZero();
    kv.setZero();
    kp.diagonal() << 2000.0, 5000.0, 4000.0, 3700.0, 3200.0, 3200.0,
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

    qp_cartesian_velocity_ = std::make_unique<QP::CartesianVelocity>();
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
        // std::ostringstream oss;
        // oss << std::setw(9) << std::setfill('0') << t_.nsec;
        std::stringstream fileNameSS;
        // fileNameSS << "image_" << camera_tick_ << "_" << t_.sec << "_" << oss.str() << ".jpg";
        fileNameSS << "image_" << camera_tick_ << ".jpg";
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

            double t_ = (ros::Time::now() - init).toSec();

            Eigen::Quaterniond hand_quat(rd_.link_[Right_Hand].rotm);
            Eigen::Quaterniond head_quat(rd_.link_[Head].rotm);
            // write data to the file
            fout1 << t_ << "\t"
                << rd_.link_[Right_Hand].xpos(0) << "\t" << rd_.link_[Right_Hand].xpos(1) << "\t" << rd_.link_[Right_Hand].xpos(2) << "\t" 
                << hand_quat.coeffs()[0] << "\t"<< hand_quat.coeffs()[1] << "\t" << hand_quat.coeffs()[2] << "\t" << hand_quat.coeffs()[3] << "\t" 
                << rd_.link_[Head].xpos(0) << "\t" << rd_.link_[Head].xpos(1) << "\t" << rd_.link_[Head].xpos(2) << "\t" 
                << head_quat.coeffs()[0] << "\t"<< head_quat.coeffs()[1] << "\t" << head_quat.coeffs()[2] << "\t" << head_quat.coeffs()[3] << "\t" 
                << hand_open_msg.data << endl;
            
            fout2 << t_ << "\t";
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

            camera_tick_++;
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
    //MODE 7: Homing & new obj pose
    //MODE 8: joint trajectory tracking for RRT & data collection
    //MODE 9: right hand task space control with QP by jh
    queue_cc_.callAvailable(ros::WallDuration());
    publishRobotPoses();
    
    if (rd_.tc_.mode == 6)
    {
        // initialize tc, executed only once
        if (rd_.tc_init){
            std::cout << "mode 6 init!" << std::endl;
            rd_.tc_init = false;
            prev_mode = 6;

            camera_tick_ = 0;
            target_received = false;
            // uncomment following lines for automatic policy test
            // terminate_msg.data = false;
            // terminate_pub.publish(terminate_msg);

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
            fout1.open(filePath_hand);
            fout1 << "camera_tick" << "\t" << "ros_time" << "\t" << "hand_pose_x" << "\t" << "hand_pose_y" << "\t" << "hand_pose_z" << "\t" 
                    << "hand_pose_roll" << "\t" << "hand_pose_pitch" << "\t" << "hand_pose_yaw" << "\t" << "distance" << endl; 
            if(!fout1.is_open()){
                ROS_ERROR("Couldn't open text file1");
            }

            std::stringstream filePathSS_joint;
            filePathSS_joint << folderPath << "/joint.txt";
            filePath_joint = filePathSS_joint.str();
            fout2.open(filePath_joint);
            fout2 << "camera_tick" << "\t" << "ros_time" << "\t" 
                    << "current_joint_pos" << "\t" << "target_joint_pos" << "\t" 
                    << "current_joint_vel" << "\t" << "target_joint_vel" << "\t" 
                    << "recieved_target" << endl; 
            if(!fout2.is_open()){
                ROS_ERROR("Couldn't open text file2");
            }

            std::stringstream filePathSS_info;
            filePathSS_info << folderPath << "/info.txt";
            filePath_info = filePathSS_info.str();
            fout3.open(filePath_info);
            fout3 << "obj_pose_x" << "\t" << "obj_pose_y" << "\t" << "obj_pose_z" << "\t" << "success" << endl;
            fout3 << obj_pos_(0) << "\t" << obj_pos_(1) << "\t" << obj_pos_(2) << "\t";
            if(!fout3.is_open()){
                ROS_ERROR("Couldn't open text file3");
            }
        }
        
        if(target_received){
            if(!data_collect_start_){
                data_collect_start_ = true;
                init = ros::Time::now();
            }
            // compute current distance between hand and obj for terminal condition check
            Eigen::Vector3d rhand_pos_;
            rhand_pos_ << rd_.link_[Right_Hand].xpos;
            
            float dx = obj_pos_(0) - rhand_pos_(0);
            float dy = obj_pos_(1) - rhand_pos_(1);
            float dz = obj_pos_(2) - rhand_pos_(2);
            distance_hand2obj = std::sqrt(dx*dx + dy*dy + dz*dz);

            WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
            // rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

            double t_ = (ros::Time::now() - init).toSec();
            double dt = 1/30;
            for(int i = 0; i < joint_names_.size(); i++){
                int j = JOINT_INDEX[joint_names_[i]];
                desired_q_[j] = DyrosMath::cubic(t_, t_0_, t_0_+dt, q_0_[j], joint_target_[i], qdot_0_[j], 0);
                desired_qdot_[j] = DyrosMath::cubicDot(t_, t_0_, t_0_+dt, q_0_[j], joint_target_[i], qdot_0_[j], 0);
            }

            if(camera_tick_%16 == 0){
                // write data to the file
                fout1 << camera_tick_ << "\t" << t_ << "\t" << rhand_pos_(0) << "\t" << rhand_pos_(1) << "\t" << rhand_pos_(2) << "\t" 
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
                for(int i = 0; i < joint_names_.size(); i++){
                    fout2 << joint_target_[i] << "\t";
                }
                fout2 << endl;
            }
            camera_tick_++;
            
            // uncomment following lines for automatic policy test
            // if (t_ > 10.0){
            //     bool is_reached = (distance_hand2obj < 0.3);
            //     fout3 << is_reached << endl;
            //     std::cout<< (is_reached ? "Reached" : "Failed") << std::endl;

            //     terminate_msg.data = true;
            //     terminate_pub.publish(terminate_msg);

            //     data_collect_start_ = false;
            //     rd_.tc_.mode = 7;
            //     rd_.tc_init = true;
            // }
        }
        for (int i = 0; i < MODEL_DOF; i++){
            rd_.torque_desired[i] = rd_.pos_kp_v[i] * (desired_q_[i] - rd_.q_[i]) + rd_.pos_kv_v[i] * (desired_qdot_[i] - rd_.q_dot_[i]);
        }
    }
    else if (rd_.tc_.mode == 7)
    {
        // initialize tc, executed only once
        if (rd_.tc_init){
            std::cout << "mode 7 init!" << std::endl;
            rd_.tc_init = false;
            q_init_ = rd_.q_;
            time_init_ = rd_.control_time_ + 0.5;   // wait 0.5s for hand closed
        }
        // move to ready pose
        double duration = 2.0;
        resetRobotPose(duration);

        if (rd_.control_time_ > time_init_ + duration)
        {
            // check and record whether the task has succeeded
            bool is_reached = (obj_pos_(2) > 1.0);
            fout3 << is_reached << endl;
            std::cout<< (is_reached ? "Success" : "Fail") << std::endl;

            // end data collection
            data_collect_start_ = false;
            if(fout1.is_open()==true)
            {
                fout1.close();
            }
            if(fout2.is_open()==true)
            {
                fout2.close();
            }
            if(fout3.is_open()==true)
            {
                fout3.close();
            }

            // open hand
            hand_open_msg.data = 0;
            hand_open_pub.publish(hand_open_msg);

            // relocate object to new position
            const double minX = -0.1;
            const double maxX = 0.1;
            const double minY = -0.1;
            const double maxY = 0.1;

            new_obj_pose_msg_.position.x = getRandomPosition(minX, maxX);
            new_obj_pose_msg_.position.y = getRandomPosition(minY, maxY);
            new_obj_pose_msg_.position.z = 0.0;
            double yaw = getRandomPosition(0, 0.75);
            Eigen::Quaterniond quaternion(DyrosMath::rotateWithZ(yaw));
            new_obj_pose_msg_.orientation.x = quaternion.coeffs()[0];
            new_obj_pose_msg_.orientation.y = quaternion.coeffs()[1];
            new_obj_pose_msg_.orientation.z = quaternion.coeffs()[2];
            new_obj_pose_msg_.orientation.w = quaternion.coeffs()[3];
            new_obj_pose_pub.publish(new_obj_pose_msg_);

            rd_.tc_.mode = prev_mode;
            rd_.tc_init = true;
        }
    }
    else if (rd_.tc_.mode == 8)
    {
        // initialize tc, executed only once
        if (rd_.tc_init){
            std::cout << "mode 8 init!" << std::endl;
            rd_.tc_init = false;
            prev_mode = 8;

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
            fout1.open(filePath_hand);
            fout1 << "camera_tick" << "\t" << "ros_time" << "\t" 
                << "hand_pose_x" << "\t" << "hand_pose_y" << "\t" << "hand_pose_z" << "\t" 
                << "hand_pose_qx" << "\t" << "hand_pose_qy" << "\t" << "hand_pose_qz" << "hand_pose_qw" << "\t" 
                << "head_pose_x" << "\t" << "head_pose_y" << "\t" << "heand_pose_z" << "\t" 
                << "head_pose_qx" << "\t" << "head_pose_qy" << "\t" << "head_pose_qz" << "head_pose_qw" << "\t" 
                << "hand_open_state" << endl; 
            if(!fout1.is_open()){
                ROS_ERROR("Couldn't open text file1");
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
            fout3 << "obj_pose_x" << "\t" << "obj_pose_y" << "\t" << "obj_pose_z" << "\t" << "success" << endl;
            if(!fout3.is_open()){
                ROS_ERROR("Couldn't open text file3");
            }
        }
            
        if(data_collect_start_){
            WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
            // rd_.torque_grav = WBC::GravityCompensationTorque(rd_);

            // get target q, qvel from trajectory waypoints & cubic spline
            double t_ = (ros::Time::now() - init).toSec();
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
                for(int i = 0; i < joint_names_.size(); i++){                    
                    desired_q_[JOINT_INDEX[joint_names_[i]]] = DyrosMath::cubic(t_, t_0, t_f, p_0[i], p_f[i], v_0[i], v_f[i]);
                    desired_qdot_[JOINT_INDEX[joint_names_[i]]] = DyrosMath::cubicDot(t_, t_0, t_f, p_0[i], p_f[i], v_0[i], v_f[i]);
                }
            }
            else{
                // end of trajectory following
                for(int i = 0; i < joint_names_.size(); i++){                    
                    desired_q_[JOINT_INDEX[joint_names_[i]]] = points[traj_index].positions[i];
                    desired_qdot_[JOINT_INDEX[joint_names_[i]]] = points[traj_index].velocities[i];
                }
                rd_.tc_.mode = 7;
                rd_.tc_init = true;

                // close hand to grab the object
                hand_open_msg.data = 1;
                hand_open_pub.publish(hand_open_msg);
            }
        }
        // torque PD control
        rd_.torque_desired =  kp * (desired_q_ - rd_.q_) + kv * (desired_qdot_ - rd_.q_dot_);
    }
    
    else if (rd_.tc_.mode == 9)
    {
        cc_timer_.reset();
        if (rd_.tc_init){
            std::cout << "mode 9 init!" << std::endl;
            rd_.tc_init = false;
            
            rhand_pos_init_ = rd_.link_[Right_Hand].xpos;
            rhand_rot_init_ = rd_.link_[Right_Hand].rotm;
            
            desired_q_ = rd_.q_;
            desired_qdot_.setZero();
            target_received = false;
        }

        // get desired pose from GUI taskcommand msg
        if (!target_received){
            Vector3d rhand_pos_desired_l;
            rhand_pos_desired_l << rd_.tc_.r_x, rd_.tc_.r_y, rd_.tc_.r_z;
            rhand_target_pos_ = rhand_pos_init_ + rhand_pos_desired_l;
            rhand_target_rotm_ = DyrosMath::rotateWithX(rd_.tc_.r_roll * DEG2RAD) * DyrosMath::rotateWithY(rd_.tc_.r_pitch * DEG2RAD) * DyrosMath::rotateWithZ(rd_.tc_.r_yaw * DEG2RAD) * DyrosMath::Euler2rot(0, M_PI_2, M_PI).transpose();
        }
        // else get desired pose from /tocabi/act/pose_target topic callback

        // set QP problem
        Eigen::Vector6d desired_xdot;
        desired_xdot.head(3) = rhand_target_pos_ - rd_.link_[Right_Hand].xpos;
        desired_xdot.tail(3) = DyrosMath::getPhi(rhand_target_rotm_, rd_.link_[Right_Hand].rotm);
        qp_cartesian_velocity_->setCurrentState(rd_.q_.tail(8), rd_.q_dot_.tail(8), rd_.link_[Right_Hand].Jac().block(0, 31, 6, 8));
        qp_cartesian_velocity_->setDesiredEEVel(desired_xdot);

        // solve QP
        Eigen::Matrix<double, 8, 1> opt_qdot;
        QP::TimeDuration time_status;
        int status = qp_cartesian_velocity_->getOptJointVel(opt_qdot, time_status);
        if(status != 0)
        {
            ROS_INFO("QP did not solved!!!");
            std::cout << "\t\t\t\t error code: " << status << std::endl;
        }

        desired_qdot_.tail(8) = opt_qdot;        
        desired_q_ += desired_qdot_ / hz_;

        // torque PD control
        rd_.torque_desired =  kp * (desired_q_ - rd_.q_) + kv * (desired_qdot_ - rd_.q_dot_);

        double elapsed_time = cc_timer_.elapsedAndReset();
        // cout << elapsed_time * 1000 << " ms" << endl;
    } 
}

void CustomController::resetRobotPose(double duration)
{
    Eigen::Matrix<double, MODEL_DOF, 1> q_target;
    q_target << 0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
            0.0, 0.0, 0.0,
            0.3, 0.3, 1.5, -1.27, -1, 0.0, -1, 0.0,
            0.0, 0.3,
            0.0, 0.3, -1.57, 1.2, 1.57, -1.5, -0.4, 0.2;
    for (int i = 0; i < MODEL_DOF; i++)
    {
        desired_q_(i) = DyrosMath::cubic(rd_.control_time_, time_init_, time_init_ + duration, q_init_(i), q_target(i), 0.0, 0.0);
        desired_qdot_(i) = DyrosMath::cubicDot(rd_.control_time_, time_init_, time_init_ + duration, q_init_(i), q_target(i), 0.0, 0.0);
    }

    WBC::SetContact(rd_, rd_.tc_.left_foot, rd_.tc_.right_foot, rd_.tc_.left_hand, rd_.tc_.right_hand);
    rd_.torque_desired =  kp * (desired_q_ - rd_.q_) + kv * (desired_qdot_ - rd_.q_dot_);
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

void CustomController::ObjPoseCallback(const geometry_msgs::PoseConstPtr &msg)
{
    float obj_x = msg->position.x;
    float obj_y = msg->position.y;
    float obj_z = msg->position.z;
 
    obj_pos_ << obj_x, obj_y, obj_z;
    // std::cout << "Obj Pose subscribed!" << std::endl;
}

void CustomController::JointTrajectoryCallback(const trajectory_msgs::JointTrajectoryPtr &msg)
{
    joint_names_ = msg->joint_names;
    points = msg->points;
    traj_index = 0;
    num_waypoint = points.size();

    data_collect_start_ = true;
    camera_tick_ = 0;
    init = ros::Time::now();
    fout3 << obj_pos_(0) << "\t" << obj_pos_(1) << "\t" << obj_pos_(2) << "\t";
}

void CustomController::JointTargetCallback(const sensor_msgs::JointStatePtr &msg)
{
    t_0_ = (msg->header.stamp - init).toSec();
    q_0_ = rd_.q_;
    qdot_0_ = rd_.q_dot_;
    joint_names_ = msg->name;
    joint_target_ = msg->position;
    target_received = true;
}

void CustomController::PoseTargetCallback(const geometry_msgs::PosePtr &msg)
{
    rhand_target_pos_ << msg->position.x, msg->position.y, msg->position.z;
    rhand_target_rotm_ = CustomController::Quat2rotmatrix(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
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

void CustomController::publishRobotPoses()
{
    robot_pose_msg.header.stamp = ros::Time::now();
    robot_pose_msg.header.frame_id = "world";
    // left hand
    robot_pose_msg.poses[0].position.x = rd_.link_[Left_Hand].xpos(0);
    robot_pose_msg.poses[0].position.y = rd_.link_[Left_Hand].xpos(1);
    robot_pose_msg.poses[0].position.z = rd_.link_[Left_Hand].xpos(2);
    Eigen::Quaterniond quat_lhand(rd_.link_[Left_Hand].rotm);
    robot_pose_msg.poses[0].orientation.x = quat_lhand.x();
    robot_pose_msg.poses[0].orientation.y = quat_lhand.y();
    robot_pose_msg.poses[0].orientation.z = quat_lhand.z();
    robot_pose_msg.poses[0].orientation.w = quat_lhand.w();
    // head
    robot_pose_msg.poses[1].position.x = rd_.link_[Head].xpos(0);
    robot_pose_msg.poses[1].position.y = rd_.link_[Head].xpos(1);
    robot_pose_msg.poses[1].position.z = rd_.link_[Head].xpos(2);
    Eigen::Quaterniond q_head(rd_.link_[Head].rotm);
    robot_pose_msg.poses[1].orientation.x = q_head.x();
    robot_pose_msg.poses[1].orientation.y = q_head.y();
    robot_pose_msg.poses[1].orientation.z = q_head.z();
    robot_pose_msg.poses[1].orientation.w = q_head.w();
    // right hand
    robot_pose_msg.poses[2].position.x = rd_.link_[Right_Hand].xpos(0);
    robot_pose_msg.poses[2].position.y = rd_.link_[Right_Hand].xpos(1);
    robot_pose_msg.poses[2].position.z = rd_.link_[Right_Hand].xpos(2);
    Eigen::Quaterniond quat_rhand(rd_.link_[Right_Hand].rotm);
    robot_pose_msg.poses[2].orientation.x = quat_rhand.x();
    robot_pose_msg.poses[2].orientation.y = quat_rhand.y();
    robot_pose_msg.poses[2].orientation.z = quat_rhand.z();
    robot_pose_msg.poses[2].orientation.w = quat_rhand.w();

    robot_pose_pub.publish(robot_pose_msg);
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