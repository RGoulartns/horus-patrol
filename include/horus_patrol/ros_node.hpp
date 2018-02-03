#ifndef horus_patrol_ROSNODE_HPP_
#define horus_patrol_ROSNODE_HPP_

#include <QThread>
#include <QPixmap>
#include <ros/ros.h>
#include <ros/network.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "move_base_msgs/MoveBaseActionResult.h"
#include <tf/transform_listener.h>
#include "actionlib_msgs/GoalID.h"
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/MotorPower.h>
#include <kobuki_msgs/WheelDropEvent.h>
#include <log4cxx/logger.h>
#include "opencv2/imgproc/imgproc.hpp"


namespace HorusPatrol
{

class ROSNode : public QThread
{
    Q_OBJECT

public:
    ROSNode(int argc, char** argv, const std::string &name );
    virtual ~ROSNode();

    geometry_msgs::PoseWithCovarianceStamped custom_initial_pose;
    ros::Publisher goal_pub, goal_cancel_pub, initial_pose_pub, threat_pub, motors_pub;
    std::vector<float> patrol_places_x, patrol_places_y, patrol_places_orientation;
    std::vector<QString> patrol_places_name;

    bool bumper_event_sequence, manual_control_enabled, mapping_mode, patrolling_mode, patrolling_mode_run, managing_route, robot_localized, at_base,
    streaming, custom_move_requested, turn360_requested, custom_goal, sentido_horario, clock_rotation, threat_detected_;
    short int limit, nlaps, nrept, vertices_rotation, custom_move_counter, total_patrol_places, patrolplace_it;
    float side1, side2, sup_turn, patrol_initial_pose_x, patrol_initial_pose_y, patrol_initial_pose_orientation, linear_, angular_, current_linear, current_angular;
    double pose_x, pose_y, Start_Point_x , Start_Point_y, acc_dist, acc_angle, orientation, orientation_old, Start_Orientation;
    QPixmap captured_image;
    QString patrol_initial_pose_name, selected_route, selected_cmove;

    bool init(), init(const std::string &master_url, const std::string &host_url);
    const std::string& nodeName() { return node_name; }
    QPixmap pixmapModel (){ return captured_image; }
    void moveToInitialPosition();
    void moveToPosition(int index);
    void patrolSystemSetup();
    void publishVel(float, float, int move_type = false);
    void rosCommsInit();
    void run();
    void KobukiMotors(bool);
    void move(char direction, float linear = 0, float angular = 0);


Q_SIGNALS:
    void threatDetected();
    void bumperEvent(bool);
    void customMoveEnded(bool);
    void setupPatrollingTab();
    void robotLocalized();
    void updateImage(const QPixmap*);
    void updatePatrolInfoBox(int);
    void updateLinearVelocity(QString);
    void updateAngularVelocity(QString);


private:    
    cv::Mat threat_image;
    const std::string node_name;
    geometry_msgs::Twist vel;
    geometry_msgs::PoseWithCovarianceStamped initial_pose;
    geometry_msgs::PoseStamped patrol_pose;
    image_transport::Subscriber kinect_rgb_sub, kinect_depth_sub;
    ros::Publisher  vel_pub;
    ros::Subscriber goalresult_sub, pose_sub, bumper_sub, wheels_sub, odom_sub, laser_scan_sub, velocities_sub, initial_pose_sub, goal_sub;
    char** init_argv;
    int init_argc, amcl_error_counter_;
    QImage cvtCvMat2QImage(const cv::Mat & image);


    void customMovement();
    //void automatic_movement(char direction);
    void setInitialPoses();
    void turn360();

    void GoalResultCallback(const move_base_msgs::MoveBaseActionResultPtr& msg);
    void GoalTempCallback(const geometry_msgs::PoseStampedPtr& msg);
    void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void KinectRGBCallback(const sensor_msgs::ImageConstPtr& msg);
    void KinectDepthCallback(const sensor_msgs::ImageConstPtr& msg);
    void BumperCallback(const kobuki_msgs::BumperEventPtr &msg);
    void WheelsDropCallback(const kobuki_msgs::WheelDropEventPtr &msg);
    void LaserScanCallback (const sensor_msgs::LaserScan::ConstPtr& msg);
    void PoseCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg);
    void SensorCallback(const nav_msgs::OdometryPtr &msg);
    void VelocitiesCallback(const geometry_msgs::TwistPtr& msg);
};
}  // namespace horus_patrol

#endif
