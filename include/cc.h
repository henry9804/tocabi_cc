#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <random>
#include <fstream>
#include <ctime>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sys/stat.h>
#include <chrono>
// #include <boost/shared_ptr.hpp>


class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();

    //void taskCommandToCC(TaskCommand tc_);
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);

    void ModeCallback(const std_msgs::Int32Ptr &msg);
    void JointTargetCallback(const sensor_msgs::JointStatePtr &msg);
    Eigen::Matrix3d Quat2rotmatrix(double q0, double q1, double q2, double q3);
    Eigen::MatrixXd LowPassFilter(const Eigen::MatrixXd &input, const Eigen::MatrixXd &prev_res, const double &sampling_freq, const double &cutoff_freq);


    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;
    ros::Subscriber mode_sub;
    ros::Subscriber joint_target_sub;

    bool des_r_subscribed=false;
    Eigen::Vector3d des_r_pos_;
    Eigen::Vector3d des_r_ori_;
    Eigen::Matrix3d des_r_orientation_;
    
    Eigen::VectorQd desired_q_;
    Eigen::VectorQd desired_qdot_;

    bool target_received = false;
    double t_0_;
    std::vector<double> right_arm_target_first;
    std::vector<double> right_arm_target;

    std::vector<std::string> joint_names_;
    std::vector<double> joint_target_;
    Eigen::VectorQd q_0_;
    Eigen::VectorQd qdot_0_;


    void resetRobotPose(double duration);
    Eigen::VectorQd q_init_;
    double time_init_ = 0.0;
    
    std::string folderPath, filePath_hand, filePath_joint;   // for hand pose and joint
    std::ofstream fout;

    // float pos_x_;

    //WholebodyController &wbc_;
    //TaskCommand tc;

    double rr_, rp_, ry_;
    double drr_, drp_, dry_;
    Matrix3d hand_r_rot_desired_past;

    Eigen::Matrix3d eulerRotation(double roll, double pitch, double yaw) {
        // Yaw, pitch, roll 순서로 회전 행렬 생성
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        return rotation;
    }


private:
    Eigen::VectorQd ControlVal_;
    map<std::string, int> JOINT_INDEX;
};
