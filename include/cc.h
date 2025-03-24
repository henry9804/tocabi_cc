#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
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
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
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
    void PublishHapticData();

    void HapticPoseCallback(const geometry_msgs::PoseConstPtr &msg);
    void ObjPoseCallback(const geometry_msgs::PoseConstPtr &msg);
    void JointTrajectoryCallback(const trajectory_msgs::JointTrajectoryPtr &msg);
    void JointTargetCallback(const sensor_msgs::JointStatePtr &msg);
    Eigen::Matrix3d Quat2rotmatrix(double q0, double q1, double q2, double q3);
    float PositionMapping( float haptic_pos, int i);
    bool saveImage(const sensor_msgs::ImageConstPtr &image_msg);
    void camera_img_callback(const sensor_msgs::ImageConstPtr &msg);
    // sensor_msgs::ImageConstPtr

    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;
    ros::Subscriber haptic_pose_sub_;
    ros::Subscriber joint_trajectory_sub;
    ros::Subscriber joint_target_sub;
    ros::Publisher haptic_force_pub_;
    ros::Subscriber obj_pose_sub;
    
    Eigen::Vector3d haptic_pos_;
    Eigen::Vector4d haptic_ori_;
    Eigen::Matrix3d haptic_orientation_;
    Eigen::Vector3d obj_pos_;
    Eigen::VectorQd desired_q_;
    Eigen::VectorQd desired_qdot_;
    std::vector<trajectory_msgs::JointTrajectoryPoint> points;
    bool init_time = false;
    int traj_index = 0;
    int num_waypoint = 0;

    bool target_received = false;
    double t_0_;
    std::vector<std::string> joint_names_;
    std::vector<double> joint_target_;
    Eigen::VectorQd q_0_;
    Eigen::VectorQd qdot_0_;
    ros::Publisher terminate_pub;
    std_msgs::Bool terminate_msg;
    ros::Publisher hand_open_pub;
    std_msgs::Int32 hand_open_msg;


    void resetRobotPose(double duration);
    bool target_reached_ = false;
    Eigen::VectorQd q_init_;
    double time_init_ = 0.0;
    
    std::string folderPath, filePath_hand, filePath_joint, filePath_info;   // for hand pose and joint
    std::string folderPath_image, fileName_image, filePath_image;           // for images
    std::ofstream fout1, fout2, fout3;

    // float pos_x_;

    //WholebodyController &wbc_;
    //TaskCommand tc;

    double haptic_force_[3];

    ros::Publisher camera_flag_pub;
    std_msgs::Bool camera_flag_msg;

    image_transport::Subscriber camera_image_sub;

    int camera_tick_ = 0;
    bool data_collect_start_ = false;
    bool make_dir = true;
    bool terminate = false;

    float distance_hand2obj;
    int prev_mode = 8;

    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kp;
    Eigen::Matrix<double, MODEL_DOF, MODEL_DOF> kv;
    
private:
    Eigen::VectorQd ControlVal_;
    map<std::string, int> JOINT_INDEX;
};
