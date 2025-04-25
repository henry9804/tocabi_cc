#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
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
#include "QP/QP_cartesian_velocity.h"

class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl() {return ControlVal_;}
    
    void computeSlow();
    void computeFast();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);

    void resetRobotPose(double duration);

    void TargetPosesCallback(const geometry_msgs::PoseArrayPtr &msg);
    void TargetJointCallback(const sensor_msgs::JointStatePtr &msg);
    void TargetRHandPoseCallback(const geometry_msgs::PoseStampedPtr &msg);

    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;
    ros::Subscriber mode_sub;
    ros::Subscriber joint_target_sub;

    VectorQd q_init_;
    VectorQd qdot_init_;
    double time_init_ = 0.0;
    
    ros::Publisher robot_pose_pub;
    geometry_msgs::PoseArray robot_pose_msg;

    ros::Subscriber target_pose_sub_; // left hand, upper body, head, right hand
    ros::Subscriber target_rhand_pose_sub_; // only for right hand

    ros::Subscriber joint_target_sub_;
    ros::Subscriber pose_target_sub_;

    //////////jh OSF & QP controller performance checking data////////
    ros::Publisher desired_robot_pose_pub_; // left hand, head, right hand
    geometry_msgs::PoseArray desired_robot_pose_msg_; // left hand, head, right hand
    ros::Publisher desired_joint_pub_; 
    ros::Publisher robot_joint_pub_;


    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kp;
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kv;

    std::vector<Eigen::Affine3d> target_robot_poses_local_; // left hand, upper body, head, right hand
    std::vector<Eigen::Affine3d> target_robot_poses_world_; // left hand, upper body, head, right hand

    std::unique_ptr<QP::CartesianVelocity> qp_cartesian_velocity_;

    bool is_world_base_{true};
    bool is_qp_first_{true};

    Eigen::VectorQd q_desired_, q_dot_desired_;
    std::vector<double> target_q_, target_q_dot_;
    std::vector<std::string> target_names_;
    bool is_q_target_{false};
    

private:
    Eigen::VectorQd ControlVal_;
    const double hz_ = 2000.0;
    map<std::string, int> JOINT_INDEX;
};
