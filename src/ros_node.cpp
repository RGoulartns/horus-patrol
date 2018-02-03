#include "../include/horus_patrol/ros_node.hpp"

namespace HorusPatrol
{

ROSNode::ROSNode(int argc, char** argv, const std::string &name ) :
    init_argc(argc),
    init_argv(argv),
    manual_control_enabled(false),
    patrolling_mode(false),
    patrolling_mode_run(false),
    custom_move_requested(false),
    turn360_requested(false),
    bumper_event_sequence(false),
    managing_route(false),
    robot_localized(false),
    custom_goal(false),
    threat_detected_(false),
    linear_(0),
    angular_(0),
    current_linear(0),
    current_angular(0),
    patrolplace_it(0),
    at_base(0),
    streaming(0),
    custom_move_counter(0),
    nrept(0),
    amcl_error_counter_(0),
    node_name(name)
{}

//destructor
ROSNode::~ROSNode()
{
    if(ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}

//LOCAL
bool ROSNode::init()
{
    ros::init(init_argc,init_argv,node_name);
    if (!ros::master::check())
    {
        return false;
    }
    rosCommsInit();
    return true;
}

//NETWORK
bool ROSNode::init(const std::string &master_url, const std::string &host_url)
{
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"horus_patrol");
    if (!ros::master::check())
    {
        return false;
    }
    rosCommsInit();
}

void ROSNode::rosCommsInit()
{
    ros::start();
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME)->setLevel(ros::console::g_level_lookup[ros::console::levels::Warn]);
    ros::console::notifyLoggerLevelsChanged();

    motors_pub       = n.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 5, true);
    vel_pub          = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 5, true);
    initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>( "/initialpose", 5 ,true);
    goal_pub         = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5, true);
    goal_cancel_pub  = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 5, true);
    threat_pub       = n.advertise<geometry_msgs::PointStamped>("/clicked_point", 5, true);
    pose_sub         = n.subscribe("/amcl_pose", 3, &ROSNode::PoseCallback, this);
    goalresult_sub   = n.subscribe("/move_base/result", 3, &ROSNode::GoalResultCallback, this);
    bumper_sub       = n.subscribe("/mobile_base/events/bumper",3, &ROSNode::BumperCallback, this);
    wheels_sub       = n.subscribe("/mobile_base/events/wheels_drop",3, &ROSNode::WheelsDropCallback, this);
    kinect_rgb_sub   = it.subscribe("/camera/rgb/image_raw", 3, &ROSNode::KinectRGBCallback, this, image_transport::TransportHints("compressed"));
    kinect_depth_sub = it.subscribe("/camera/depth/image_raw", 3, &ROSNode::KinectDepthCallback, this);
    odom_sub         = n.subscribe("/odom", 1, &ROSNode::SensorCallback, this, ros::TransportHints().tcpNoDelay(true));
    laser_scan_sub   = n.subscribe("/scan",1, &ROSNode::LaserScanCallback, this);
    velocities_sub   = n.subscribe("cmd_vel_mux/input/navi",3, &ROSNode::VelocitiesCallback, this);
    initial_pose_sub = n.subscribe("/initialpose", 3, &ROSNode::InitialPoseCallback, this);
    goal_sub         = n.subscribe("/goal", 3, &ROSNode::GoalTempCallback, this);

    start();
    KobukiMotors(false);
}

void ROSNode::run()
{
    ros::Rate rate(50);
    //ros::Rate rate(10);
    while ( ros::ok() )
    {
        if(bumper_event_sequence)
        {
            if(current_linear > -0.0000001)
            {
                bumper_event_sequence = false;
            }
            publishVel(angular_,linear_);
        }
        else
        {
            if(custom_move_requested)
            {
                customMovement();
            }
            if(turn360_requested)
            {
                turn360();
            }
            if( (manual_control_enabled)||(turn360_requested)||(custom_move_requested) )
            {
                //std::cout << acc_angle << " " << acc_dist << std::endl;
                publishVel(angular_,linear_);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void ROSNode::KobukiMotors(bool state)
{
    kobuki_msgs::MotorPower power_cmd;
    if(state)
    {
        power_cmd.state = kobuki_msgs::MotorPower::ON;
    }
    else
    {
        power_cmd.state = kobuki_msgs::MotorPower::OFF;
    }
    motors_pub.publish(power_cmd);
}

void ROSNode::BumperCallback(const kobuki_msgs::BumperEventPtr& msg)
{
    if(msg->state)
    {
        manual_control_enabled = false;
        custom_move_requested = false;
        turn360_requested = false;
        bumper_event_sequence = true;
        angular_ = 0;
        linear_ = -0.3;
        publishVel(angular_,linear_, 1);
        Q_EMIT bumperEvent(msg->state);
    }
    else
    {
        angular_ = 0;
        linear_ = 0;
    }
}

void ROSNode::WheelsDropCallback(const kobuki_msgs::WheelDropEventPtr& msg)
{
    if(msg->state)
    {

    }
    else
    {

    }
}

void ROSNode::SensorCallback(const nav_msgs::OdometryPtr& msg)
{
    pose_x = msg->pose.pose.position.x;
    pose_y = msg->pose.pose.position.y;
    orientation = tf::getYaw(msg->pose.pose.orientation);
    orientation = 3.14159 + orientation;													    			//0~2PI
    if( (manual_control_enabled) || (custom_move_requested)||(turn360_requested) )
    {
        acc_dist = sqrt( pow( (pose_y - Start_Point_y), 2) + pow( (pose_x - Start_Point_x), 2) );			//5 -> fator de seguranca. A mudanca deve ser relativamente grande. ex: 6.2 para 0.1
        if( (!sentido_horario)&&(5*orientation < orientation_old) )
        {
            limit++;                                                                                        //limit -> remover o limite de 0~2PI
        }
        if( (sentido_horario)&&(orientation > 5*orientation_old) )
        {
            limit--;
        }
        acc_angle = sqrt( pow((orientation - Start_Orientation +2*3.14159*limit),2) );                      //acc_angle -> rotacao acumulada
        orientation_old = orientation;																		//orientation_old -> orientation[n-1] para fazer comparacoes
    }
}

double right_dist, left_dist, dist, test;
void ROSNode::LaserScanCallback (const sensor_msgs::LaserScan::ConstPtr& msg){	//640 max
    dist = msg->ranges[320];
    left_dist = msg->ranges[639];
    right_dist = msg->ranges[1];

    for(int i=270;i<=370;i++){
        if( (msg->ranges[i]>0.45)&&(msg->ranges[i]<11) ){
            if(dist > msg->ranges[i]) dist = msg->ranges[i];
        }
    }
}

QImage ROSNode::cvtCvMat2QImage(const cv::Mat & image)                                                      //internet fragment code
{
    QImage qtemp;
    if(!image.empty() && image.depth() == CV_8U)
    {
        const unsigned char * data = image.data;
        qtemp = QImage(image.cols, image.rows, QImage::Format_RGB32);
        for(int y = 0; y < image.rows; ++y, data += image.cols*image.elemSize())
        {
            for(int x = 0; x < image.cols; ++x)
            {
                QRgb * p = ((QRgb*)qtemp.scanLine (y)) + x;
                *p = qRgb(data[x * image.channels()+2], data[x * image.channels()+1], data[x * image.channels()]);
            }
        }
    }
    return qtemp;
}

void ROSNode::KinectRGBCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //uchar *p;
    cv::Mat thresh_image;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::inRange(cv_ptr->image, cv::Scalar(0,0,100), cv::Scalar(50,50,255), thresh_image);
    cv::morphologyEx(thresh_image, threat_image, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));
    /*for(short int i=0;i<480;++i){
        p = threat_image.ptr<uchar>(i);
        for(short int j=0;j<640;++j){
            if(p[j]>0) n_points_detected++;
        }
    }
    if(n_points_detected > 480*640*0.1) std::cout << "!!!" << std::endl;*/
    //captured_image = QPixmap::fromImage(cvtCvMat2QImage(threat_image));
    captured_image = QPixmap::fromImage(cvtCvMat2QImage(cv_ptr->image));
    Q_EMIT updateImage(&captured_image);
}

void ROSNode::KinectDepthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if((patrolling_mode_run)&&(!manual_control_enabled))
    {
        if(!threat_image.empty())
        {
            uchar *p;
            double n_points = 0, score = 0;
            cv::Mat depth_mat;
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
            cv::convertScaleAbs(cv_ptr->image, depth_mat, 100, 0.0);
            QImage depth_image( depth_mat.data, depth_mat.cols, depth_mat.rows, depth_mat.step, QImage::Format_Indexed8);
            static QVector<QRgb>  sColorTable;
            for(short int i=0; i<256; ++i) sColorTable.push_back(qRgb(i, i, i));
            depth_image.setColorTable(sColorTable);

            for(short int i=0;i<480;++i){
                p = threat_image.ptr<uchar>(i);
                for(short int j=100;j<540;++j){
                    if(p[j]>0)
                    {
                        n_points++;
                        score += depth_image.pixelIndex(j,i);
                    }
                    else
                    {
                        depth_image.setPixel(j,i,0);
                    }
                }
            }
            if(n_points > 3000)
            {
                double detection_thresh = 1.5 - log(score/n_points)/3.5;
                if(detection_thresh < 0.03) detection_thresh = 0.010;
                if( (!threat_detected_)&&(n_points/(480*640) > detection_thresh) )
                {
                    threat_detected_ = true;
                    Q_EMIT threatDetected();
                }
            }
            else
            {
                threat_detected_ = false;
            }
            //captured_image = QPixmap::fromImage(depth_image);
            //Q_EMIT updateImage(&captured_image);
        }
    }
}

void ROSNode::InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    robot_localized = true;
    custom_initial_pose = msg;
    initial_pose = custom_initial_pose;
    Q_EMIT  robotLocalized();
}

void ROSNode::PoseCallback(const geometry_msgs::PoseWithCovarianceStampedPtr& msg)
{
    custom_initial_pose = *msg;
    if(!selected_route.isEmpty())
    {
        if ( (!at_base)&&( sqrt(pow( (msg->pose.pose.position.y - initial_pose.pose.pose.position.y), 2) + pow( (msg->pose.pose.position.x - initial_pose.pose.pose.position.x), 2)) < 0.3 ) )
        {
            at_base = true;
            Q_EMIT updatePatrolInfoBox(42);
        }
        if ( (at_base)&&( sqrt(pow( (msg->pose.pose.position.y - initial_pose.pose.pose.position.y), 2) + pow( (msg->pose.pose.position.x - initial_pose.pose.pose.position.x), 2)) > 0.3 ) )
        {
            at_base = false;
        }
    }
}

void ROSNode::GoalTempCallback(const geometry_msgs::PoseStampedPtr& msg)
{
    goal_pub.publish(msg);
}

void ROSNode::GoalResultCallback(const move_base_msgs::MoveBaseActionResultPtr& msg)
{
    if( (patrolling_mode_run)&&(!turn360_requested) )
    {
        if(msg->status.status == 3)
        {
            //Q_EMIT TakeAScreenShot();
            if(patrolplace_it < total_patrol_places)
            {
                if( (patrolplace_it!=0)&&(!custom_goal) )
                {
                    turn360_requested = true;
                }
                patrol_pose.pose.position.x = patrol_places_x[patrolplace_it];
                patrol_pose.pose.position.y = patrol_places_y[patrolplace_it];
                patrol_pose.pose.orientation = tf::createQuaternionMsgFromYaw(patrol_places_orientation[patrolplace_it]);
                ros::Duration(1).sleep();
                goal_pub.publish(patrol_pose);
                patrolplace_it++;
            }
            else if( patrolplace_it == total_patrol_places )
            {
                ros::Duration(1).sleep();
                moveToInitialPosition();
            }
            else
            {
                //WHY??? dont remember
                patrolplace_it = 0;
                Q_EMIT setupPatrollingTab();
            }
            Q_EMIT updatePatrolInfoBox(3);
        }
    }
    else if(msg->status.status == 3)                                                    //Custom movement
    {
        patrolplace_it++;
        Q_EMIT updatePatrolInfoBox(3);
    }
    if( (msg->status.status == 3)&&(custom_goal) )
    {
        custom_goal = false;
    }
    if(msg->status.status == 4)
    {
        amcl_error_counter_++;
        if(amcl_error_counter_>3)
        {
            updatePatrolInfoBox(4);
        }
        else
        {
            goal_pub.publish(patrol_pose);
        }

    }
}

void ROSNode::VelocitiesCallback(const geometry_msgs::TwistPtr& msg)
{
    if( (!manual_control_enabled)&&(!turn360_requested) )
    {
        Q_EMIT updateLinearVelocity(QString::number(msg->linear.x, 'f', 2));
        Q_EMIT updateAngularVelocity(QString::number(msg->angular.z, 'f', 2));
    }
}

void ROSNode::patrolSystemSetup()
{
    setInitialPoses();
    patrolling_mode_run = false;
    initial_pose_pub.publish(initial_pose);
}

void ROSNode::setInitialPoses()
{
    //set initial position
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = ros::Time::now();
    if(patrol_initial_pose_name == "Custom Initial Pose")
    {
        initial_pose = custom_initial_pose;
    }
    else
    {
        initial_pose.pose.pose.position.x = patrol_initial_pose_x;
        initial_pose.pose.pose.position.y = patrol_initial_pose_y;
        initial_pose.pose.pose.position.z = 0;
        initial_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(patrol_initial_pose_orientation);
        initial_pose.pose.covariance = {0.15, 0.0, 0.0, 0.0, 0.0, 0.0,                    //x
                                        0.0, 0.15, 0.0, 0.0, 0.0, 0.0,                    //y
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                    //z
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                    //row
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                    //pitch
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.2};                   //yaw
    }
    // set first goal pose
    patrol_pose.header.frame_id = "map";
    patrol_pose.header.stamp = ros::Time::now();
    patrol_pose.pose.position.x = patrol_places_x[patrolplace_it];
    patrol_pose.pose.position.y = patrol_places_y[patrolplace_it];
    patrol_pose.pose.position.z = 0;
    patrol_pose.pose.orientation = tf::createQuaternionMsgFromYaw(patrol_places_orientation[patrolplace_it]);
}

void ROSNode::moveToInitialPosition()
{
    patrol_pose.pose.position.x = initial_pose.pose.pose.position.x;
    patrol_pose.pose.position.y = initial_pose.pose.pose.position.y;
    patrol_pose.pose.orientation = initial_pose.pose.pose.orientation;
    patrolplace_it = total_patrol_places + 1;
    actionlib_msgs::GoalID goal_id;
    goal_cancel_pub.publish(goal_id);
    //usleep(1000);
    goal_pub.publish(patrol_pose);
}

void ROSNode::moveToPosition(int index)
{
    patrol_pose.pose.position.x = patrol_places_x[index];
    patrol_pose.pose.position.y = patrol_places_y[index];
    patrol_pose.pose.orientation = tf::createQuaternionMsgFromYaw(patrol_places_orientation[index]);
    actionlib_msgs::GoalID goal_id;
    goal_cancel_pub.publish(goal_id);
    usleep(1000);
    goal_pub.publish(patrol_pose);
}

void ROSNode::turn360()
{
    switch(custom_move_counter)
    {
    case 0:
        move('r');
        custom_move_counter = 1;
        break;
    case 1:
        if(acc_angle >= 2*3.14159)
        {
            custom_move_counter = 0;
            move('s');
            turn360_requested = false;
        }
        break;
    }
}

void ROSNode::customMovement()
{
    if(nrept != 4*nlaps)
    {
        switch(custom_move_counter)
        {
        case 0:
            move('f');
            custom_move_counter = 1;
            break;
        case 1:
            if( ( (acc_dist >= side1)&&(!(nrept%2)) ) || ( (acc_dist >= side2)&&(nrept%2) ) )
            {
                move('s');
                custom_move_counter = 2;
            }
            break;
        case 2:
            if(!clock_rotation)
            {
                move('r');
            }
            else
            {
                move('l');
            }
            custom_move_counter = 3;
        case 3:
            if(acc_angle >= 3.14159/2 + sup_turn + vertices_rotation*2*3.141)
            {
                nrept++;
                custom_move_counter = 0;
                move('s');
            }
            break;
        }
    }
    else
    {
        custom_move_requested = false;
        custom_move_counter = 0;
        nrept = 0;
        Q_EMIT customMoveEnded(true);
    }
}

void ROSNode::move(char direction, float linear, float angular)
{
    acc_angle = 0;
    acc_dist = 0;
    sentido_horario = false;
    Start_Point_x = pose_x;
    Start_Point_y = pose_y;
    Start_Orientation = orientation;
    orientation_old = orientation;
    limit = 0;
    switch(direction)
    {
    case 'f':
        if(linear_ < 1.5)
            linear_ += 0.25;
        if((linear_< 0.001)&&(linear_> -0.001))
            linear_ = 0;
        break;
    case 'b':
        if(linear_ > -1.5)
            linear_ -= 0.25;
        if((linear_< 0.001)&&(linear_> -0.001))
            linear_ = 0;
        break;
    case 'l':
        if(angular_ < 3)
            angular_ += 0.6;
        if((angular_< 0.08)&&(angular_ > -0.08))
            angular_ = 0;
        break;
    case 'r':
        sentido_horario = true;
        if(angular_ > -3)
            angular_ -= 0.6;
        if((angular_< 0.08)&&(angular_ > -0.08))
            angular_ = 0;
        break;
    case 's':
        angular_ = 0;
        linear_ = 0;
        if(ros::ok()){
            publishVel(angular_, linear_, 1);
            ros::spinOnce();
        }
        break;
    case 'o': //other speed
        linear_  = linear;
        angular_ = angular;
        if(angular_ < 0)
        {
            sentido_horario = true;
        }
        break;
    }
}

void ROSNode::publishVel(float angular, float linear, int move_type)
{
    //move_type values:
    //0 - smooth movement
    //1 - "absolute" movement;
    if(move_type)
    {
        current_linear = linear;
        current_angular = angular;
        vel.linear.x = linear;
        vel.angular.z = angular;
        Q_EMIT updateLinearVelocity(QString::number(current_linear, 'f', 2));
        Q_EMIT updateAngularVelocity(QString::number(current_angular, 'f', 2));
    }
    else
    {
        if( fabs(linear - current_linear) > 0.000001)
        {
            if(current_linear < linear)
                vel.linear.x = current_linear + 0.025;
            else if (current_linear > linear)
                vel.linear.x = current_linear - 0.025;

            current_linear = vel.linear.x;
            Q_EMIT updateLinearVelocity(QString::number(current_linear, 'f', 2));
        }
        if( fabs(angular - current_angular) > 0.000001)
        {
            if(current_angular < angular)
                vel.angular.z = current_angular + 0.1;
            else if (current_angular > angular)
                vel.angular.z = current_angular - 0.1;
            current_angular = vel.angular.z;
            Q_EMIT updateAngularVelocity(QString::number(current_angular, 'f', 2));
        }
    }
    vel_pub.publish(vel);
    return;
}
}
