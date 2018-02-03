#include "../include/horus_patrol/threat_detection.hpp"

namespace HorusPatrol
{

bool ThreatDetection::dialog_openned_ = false;

ThreatDetection::ThreatDetection(ROSNode* node)
    : ros_node_(node), accepted_(false),
      time_detected_(QDateTime::currentDateTime())
{
  //create routes threats folder
  QString horus_threats_path =
      QSettings("Horus Settings", "horus_patrol_settings")
          .value("horus_path")
          .toString() +
      "/Detected Threats/";
  if (!QDir(horus_threats_path).exists())
    QDir().mkdir(horus_threats_path);

  //create folder for a route
  if (!QDir(horus_threats_path + ros_node_->selected_route).exists())
  {
    QDir(horus_threats_path).mkdir(ros_node_->selected_route);
    HorusLogger* logger = new HorusLogger(
        this, "PatrolDetections.txt",
        "Error reading Detected Threats folder. New folder created.");
    logger->start();
  }

  //save threat image
  QFile file(horus_threats_path + ros_node_->selected_route + "/Detection - " +
             time_detected_.toString("yyyy-MM-dd  hh-mm-ss") + ".png");
  file.open(QIODevice::WriteOnly);
  ros_node_->captured_image.save(&file, "PNG");

  //log
  HorusLogger* logger = new HorusLogger(
      this, "PatrolDetections.txt",
      QString("[THREAT] Possible threat detected at " + threat_place_ +
              " (x=%1, y=%2")
          .arg(ros_node_->custom_initial_pose.pose.pose.position.x)
          .arg(ros_node_->custom_initial_pose.pose.pose.position.y));
  logger->start();

  connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
}

ThreatDetection::~ThreatDetection() {}

void ThreatDetection::run()
{
  calculateThreatPosition();

  QSqlQuery query;
  query.exec("USE threats;");
  query.exec(QString(
      "INSERT INTO `" + ros_node_->selected_route +
      "` (`Time`,`SurroundingArea`,`PositionX`,`PositionY`,`Orientation`)"
      " values ('" +
      time_detected_.toString("yyyy-MM-dd  hh-mm-ss") + "','" +
      threat_place_ + "','" +
      QString::number(ros_node_->custom_initial_pose.pose.pose.position.x) +
      "','" +
      QString::number(ros_node_->custom_initial_pose.pose.pose.position.y) +
      "','" + QString::number(tf::getYaw(
                  ros_node_->custom_initial_pose.pose.pose.orientation)) +
      "');"));

  if (!dialog_openned_)
  {
    QDialog* dialog = new QDialog;
    dialog->setWindowTitle("Horus Detection Alarm");
    dialog->setFixedSize(360, 150);
    QGridLayout* gridLayout = new QGridLayout(dialog);
    gridLayout->setHorizontalSpacing(20);
    gridLayout->setContentsMargins(11, 11, 11, 11);
    QLabel* label = new QLabel(dialog);
    label->setText("THREAT DETECTED!");
    QFont font;
    font.setPointSize(20);
    font.setBold(true);
    font.setWeight(75);
    label->setFont(font);
    label->setStyleSheet("color:rgb(255, 0, 0)");
    label->setFrameShadow(QFrame::Raised);
    label->setAlignment(Qt::AlignCenter);
    gridLayout->addWidget(label, 0, 0, 1, 2);
    QPushButton* pushButton = new QPushButton(dialog);
    pushButton->setText("Ignore");
    gridLayout->addWidget(pushButton, 1, 0, 1, 1);
    pushButton->setDefault(false);
    pushButton->setAutoDefault(false);
    QPushButton* pushButton_2 = new QPushButton(dialog);
    pushButton_2->setText("Go to detections");
    gridLayout->addWidget(pushButton_2, 1, 1, 1, 1);
    pushButton_2->setDefault(true);
    pushButton_2->setAutoDefault(true);
    QObject::connect(pushButton, SIGNAL(clicked()), dialog, SLOT(reject()));
    QObject::connect(pushButton_2, SIGNAL(clicked()), dialog, SLOT(accept()));

    QSoundEffect* sound = new QSoundEffect(dialog);
    sound->setSource(QUrl("qrc:/sounds/TDetected.wav"));
    sound->setLoopCount(QSoundEffect::Infinite);
    sound->setVolume(0.75f);
    sound->play();

    dialog_openned_ = true;
    dialog->exec();
    dialog_openned_ = false;

    sound->stop();
    delete sound;
    if (dialog->result())
    {
      accepted_ = true;
    }
  }
}

void ThreatDetection::calculateThreatPosition()
{
  if (ros_node_->patrolplace_it <
      2) // comparison between initial pose and the first place
  {
    if (sqrt(pow(ros_node_->patrol_places_x[0] -
                     ros_node_->custom_initial_pose.pose.pose.position.x,
                 2) +
             pow(ros_node_->patrol_places_y[0] -
                     ros_node_->custom_initial_pose.pose.pose.position.y,
                 2)) >
        sqrt(pow(ros_node_->patrol_initial_pose_x -
                     ros_node_->custom_initial_pose.pose.pose.position.x,
                 2) +
             pow(ros_node_->patrol_initial_pose_y -
                     ros_node_->custom_initial_pose.pose.pose.position.y,
                 2)))
    {
      threat_place_ = ros_node_->patrol_initial_pose_name;
    }
    else
    {
      threat_place_ = ros_node_->patrol_places_name[0];
    }
  }
  else if (ros_node_->patrolplace_it >= ros_node_->total_patrol_places)
  {
    threat_place_ = "Unknown (Returning base)";
  }
  else // comparison between 2 places
  {
    if (sqrt(pow(ros_node_->patrol_places_x[ros_node_->patrolplace_it - 1] -
                     ros_node_->custom_initial_pose.pose.pose.position.x,
                 2) +
             pow(ros_node_->patrol_places_y[ros_node_->patrolplace_it - 1] -
                     ros_node_->custom_initial_pose.pose.pose.position.y,
                 2)) >
        sqrt(pow(ros_node_->patrol_places_x[ros_node_->patrolplace_it - 2] -
                     ros_node_->custom_initial_pose.pose.pose.position.x,
                 2) +
             pow(ros_node_->patrol_places_y[ros_node_->patrolplace_it - 2] -
                     ros_node_->custom_initial_pose.pose.pose.position.y,
                 2)))
    {
      threat_place_ =
          ros_node_->patrol_places_name[ros_node_->patrolplace_it - 2];
    }
    else
    {
      threat_place_ =
          ros_node_->patrol_places_name[ros_node_->patrolplace_it - 1];
    }
  }
}
}
