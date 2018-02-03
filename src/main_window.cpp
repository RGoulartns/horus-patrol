#include "include/horus_patrol/main_window.hpp"

namespace HorusPatrol
{

MainWindow::MainWindow(ROSNode* node, bool admin, QWidget* parent)
    : QMainWindow(parent), requested_destination_index_(0), ros_node_(node),
      admin_profile_(admin), threat_window_open(false)
{

  ui.setupUi(this);
  ui.mainWindow_tabWidget->setCurrentIndex(0);
  ui.mainWindow_tabWidget->setTabEnabled(2, false);
  ui.mainWindow_tabWidget->setTabEnabled(3, false);
  QApplication::instance()->installEventFilter(this); // buttons

  readSettings();
  if (ui.checkbox_remember_settings->isChecked())
  {
    ui.button_connect->setChecked(true);
    on_button_connect_clicked(true);
  }

  // check horus path
  if (horus_path_.isEmpty() || !QDir(horus_path_).exists())
    horus_path_ =
        QFileDialog::getExistingDirectory(this, tr("Choose Horus Path"), "/");
  if (horus_path_.isEmpty())
    exit(1);
  QDir::setCurrent(horus_path_);

  if (!QDir(horus_path_ + "/Captured Images").exists())
    QDir(horus_path_).mkdir("Captured Images");
  if (!QDir(horus_path_ + "/Maps").exists())
    QDir(horus_path_).mkdir("Maps");


  // Load map
  QString file = horus_path_ + "/Maps/" + map_name_ + "/" + map_name_ + ".pgm";
  QFileInfo file_info(file);
  if (file_info.isFile())
  {
    if (QFileInfo(file.left(file.length() - 3) + "yaml").isFile())
      loaded_map_.load(file);
    else
    {
      QMessageBox::critical(
          this, "Horus Patrol Error",
          "Map cannot be loaded because its 'yaml' file is missing.");
      loaded_map_.load(":/images/nomap.gif");
      map_name_.clear();
    }
  }
  else
  {
    loaded_map_.load(":/images/nomap.gif");
    map_name_.clear();
  }
  ui.map_preview->setPixmap(loaded_map_);

  // setup user profile access
  if (!admin_profile_)
  {
    ui.clear_patrol_log_button->setVisible(false);
    ui.clear_robot_log_button->setVisible(false);
  }

  // setup stream screen
  video_stream_label_ = new QLabel;
  video_stream_label_->setFrameShape(QFrame::WinPanel);
  video_stream_label_->setFrameShadow(QFrame::Plain);
  video_stream_label_->setSizePolicy(QSizePolicy::Ignored,
                                     QSizePolicy::Ignored);
  video_stream_label_->setLineWidth(3);
  video_stream_label_->setScaledContents(true);

  ui.welcome_label->setText(tr("<h2>Welcome to Horus Patrol.</h2>"
                               "<p>Please make sure that you are connected in "
                               "the same network as the robot.</p>"
                               "<p>For more information about this "
                               "application, see 'Help' located in the menu "
                               "bar.</p>"));

  ui.ros_groupBox->setStyleSheet("QGroupBox#ros_groupBox {"
                                 "border: 1px solid silver;"
                                 "border-radius: 6px;"
                                 "margin-top: 10px;"
                                 "font-weight: bold;}"
                                 "QGroupBox::title#ros_groupBox{"
                                 "subcontrol-origin:  margin;"
                                 "left: 11px;"
                                 "padding: 0 5px 0 5px;}");

  ui.map_groupBox->setStyleSheet("QGroupBox#map_groupBox{"
                                 "border: 1px solid silver;"
                                 "border-radius: 6px;"
                                 "margin-top: 10px;"
                                 "font-weight: bold;}"
                                 "QGroupBox::title#map_groupBox{"
                                 "subcontrol-origin:  margin;"
                                 "left: 11px;"
                                 "padding: 0 5px 0 5px;}");

  ui.robotDashboard_groupBox->setStyleSheet(
      "QGroupBox#robotDashboard_groupBox { "
      "border: 2px solid silver;"
      "border-radius: 6px;"
      "margin-top: 10px;"
      "font-weight: bold;} "
      "QGroupBox::title#robotDashboard_groupBox {"
      "subcontrol-origin:  margin;"
      "padding: 0 5px 0 5px;"
      "subcontrol-position: top;}");

  ui.selectedRobot_groupBox->setStyleSheet(
      "QGroupBox#selectedRobot_groupBox{"
      "border: 1px solid silver;"
      "border-radius: 6px;"
      "margin-top: 10px;"
      "font-weight: bold;}"
      "QGroupBox::title#selectedRobot_groupBox{"
      "subcontrol-origin:  margin;"
      "left: 11px;"
      "padding: 0 5px 0 5px;}");

  ui.velocities_groupBox->setStyleSheet("QGroupBox#velocities_groupBox{"
                                        "border: 1px solid silver;"
                                        "border-radius: 6px;"
                                        "margin-top: 10px;"
                                        "font-weight: bold;}"
                                        "QGroupBox::title#velocities_groupBox{"
                                        "subcontrol-origin:  margin;"
                                        "left: 11px;"
                                        "padding: 0 5px 0 5px;}");

  // mainwindow signals | slots
  QObject::connect(ros_node_, SIGNAL(robotLocalized()), this,
                   SLOT(robotLocalized()));
  QObject::connect(ros_node_, SIGNAL(updateImage(const QPixmap*)), this,
                   SLOT(updateStreamImage(const QPixmap*)));
  QObject::connect(ros_node_, SIGNAL(setupPatrollingTab()), this,
                   SLOT(setupPatrollingTab()));
  QObject::connect(ros_node_, SIGNAL(updatePatrolInfoBox(int)), this,
                   SLOT(updatePatrolInfoBox(int)));
  QObject::connect(ros_node_, SIGNAL(bumperEvent(bool)), this,
                   SLOT(bumperEvent(bool)));
  QObject::connect(ros_node_, SIGNAL(threatDetected()), this,
                   SLOT(threatDetected()));
  QObject::connect(ui.start_patrolling_button, SIGNAL(clicked(bool)),
                   ui.stop_patrolling_button, SLOT(setDisabled(bool)));
  QObject::connect(ros_node_, SIGNAL(customMoveEnded(bool)),
                   ui.custom_movement_button, SLOT(setEnabled(bool)));
  QObject::connect(ros_node_, SIGNAL(updateAngularVelocity(QString)),
                   ui.angular_label, SLOT(setText(QString)));
  QObject::connect(ros_node_, SIGNAL(updateLinearVelocity(QString)),
                   ui.linear_label, SLOT(setText(QString)));

  /*
  map = new QSignalMapper(this);
  QObject::connect(map, SIGNAL(mapped(int)), ui.tabWidget,
  SLOT(setCurrentIndex(int)));
  QShortcut *shortcut1 = new QShortcut(QKeySequence("Ctrl+1"), this);
  QShortcut *shortcut2 = new QShortcut(QKeySequence("Ctrl+2"), this);
  shortcut3 = new QShortcut(QKeySequence("Ctrl+3"), this);
  shortcut4 = new QShortcut(QKeySequence("Ctrl+4"), this);
  QObject::connect(shortcut1, SIGNAL(activated()), map, SLOT(map()));
  QObject::connect(shortcut2, SIGNAL(activated()), map, SLOT(map()));
  QObject::connect(shortcut3, SIGNAL(activated()), map, SLOT(map()));
  QObject::connect(shortcut4, SIGNAL(activated()), map, SLOT(map()));
  map->setMapping(shortcut1, 0);
  map->setMapping(shortcut2, 1);*/

  // open logs files
  HorusLogger* logger =
      new HorusLogger(this, "PatrolDetections.txt", "_HORUS PATROL STARTED_");
  logger->start();
  logger = new HorusLogger(this, "RobotEvents.txt", "_HORUS PATROL STARTED_");
  logger->start();

  model_ = new QSqlTableModel(this);
  model_->setEditStrategy(QSqlTableModel::OnFieldChange);
  ui.threatslog_tableView->setSelectionMode(QAbstractItemView::SingleSelection);

  // TODO remove it
  // setup ros initial state
  ros_node_->manual_control_enabled = false;
  ros_node_->mapping_mode = false;
  ros_node_->patrolling_mode = false;
  ros_node_->patrolling_mode_run = false;
}

MainWindow::~MainWindow()
{
  writeSettings();
  if ((ros_node_->patrolling_mode) || (ros_node_->mapping_mode))
  {
    delete myviz_widget_;
    process_->terminate();
    process_->waitForFinished(-1);
    delete process_;
  }
  if (ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }

  HorusLogger* logger = new HorusLogger(this, "PatrolDetections.txt",
                                        "_HORUS PATROL TERMINATED_");
  logger->start();
}

/*****************************************************************************
** MENUBAR
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(
      this, tr("About Horus Patrol"),
      tr("<h2>About Horus Patrol </h2>"
         "Version: <b>0.20</b> <sub>(Alpha)</sub> "
         "<p>Developed by RGoulart</p>"
         "<p>This application is a Graphical User Interface for the Horus "
         "Patrol system, responsible for allowing the user to interact with "
         "the robot system. </p>"
         "Features:"
         "<ul><li>Remote Robot Control</li><li>Video, Audio "
         "Stream</li><li>Autonomous Patrol System</li><li>Manual Mapping "
         "System%</li></ul>"
         "<br><br><br><br><br>For more information about the TurtleBot robot, "
         "visit: "
         "<ul><li><a href='http://www.turtlebot.com/'>TurtleBot "
         "Homepage;</a></li>"
         "<li><a href='http://www.ros.org/reps/rep-0119.html'>TurtleBot "
         "Specification;</a></li>"
         "<li><a href='http://wiki.ros.org/Robots/TurtleBot'>TurtleBot at "
         "ROS.</a></li></ul>"));
}

void MainWindow::on_actionAbout_Qt_triggered() { QApplication::aboutQt(); }

void MainWindow::on_action_Preferences_triggered()
{
  // preferences..
}

void MainWindow::on_actionNetwork_Settings_triggered()
{
  // Network Settings...
}

void MainWindow::on_actionLoad_Map_triggered()
{
  // Load Map...
  on_load_map_clicked();
}

/*****************************************************************************
** Home and Others
*****************************************************************************/

void MainWindow::on_checkbox_use_environment_stateChanged(int state)
{
  ui.line_edit_master->setDisabled(state);
  ui.line_edit_host->setDisabled(state);
}

void MainWindow::on_button_connect_clicked(bool checked)
{
  if (checked)
  {
    if (ui.checkbox_use_environment->isChecked())
      ros_node_->init();
    else
      ros_node_->init(ui.line_edit_master->text().toStdString(),
                      ui.line_edit_host->text().toStdString());

    if (ros::isStarted())
    {
      ui.button_connect->setText("&Disconnect");
      ui.connection_status->setText(
          "<b>STATUS:</b> <font color='Green'>Connected</font>");
      ui.robotDashboard_groupBox->setEnabled(true);
      ui.checkbox_use_environment->setEnabled(false);
      ui.mainWindow_tabWidget->setTabEnabled(2, true);
      ui.mainWindow_tabWidget->setTabEnabled(3, true);
      HorusLogger* logger = new HorusLogger(this, "PatrolDetections.txt",
                                            "Connection with ROS establish.");
      logger->start();
    }
    else
    {
      ui.button_connect->setChecked(false);
      QMessageBox::information(this, tr("Horus Patrol information"),
                               tr("Couldn't find the ros master."));
    }
  }
  else
  {
    ui.button_connect->setText("&Connect");
    ui.connection_status->setText(
        "<b>STATUS:</b> <font color='Red'>Disconnected</font>");
    ui.robotDashboard_groupBox->setEnabled(false);
    ui.checkbox_use_environment->setEnabled(true);
    ui.mainWindow_tabWidget->setTabEnabled(2, false);
    ui.mainWindow_tabWidget->setTabEnabled(3, false);

    // disable Robot Dashboard
    if (ui.motor_button->isChecked())
    {
      ui.motor_button->setChecked(false);
      on_motor_button_clicked(false);
    }
    if (ui.kinect_button->isChecked())
    {
      ui.kinect_button->setChecked(false);
      on_kinect_button_clicked(false);
    }
    if (ui.audio_stream_button->isChecked())
    {
      ui.audio_stream_button->setChecked(false);
      on_audio_stream_button_clicked(false);
    }
    if (ui.video_stream_button->isChecked())
    {
      ui.video_stream_button->setChecked(false);
      on_video_stream_button_clicked(false);
    }
    if (ui.microphone_button->isChecked())
    {
      ui.microphone_button->setChecked(false);
      on_microphone_button_clicked(false);
    }
    if (ui.video_record_button->isChecked())
    {
      ui.video_record_button->setChecked(false);
      on_video_record_button_clicked(false);
    }
    if (ui.manual_control_button->isChecked())
    {
      ui.manual_control_button->setChecked(false);
      on_manual_control_button_clicked(false);
    }

    if (ros_node_->patrolling_mode) // Disable patrolling mode
    {
      ui.patrolling_groupbox->setChecked(false);
      on_patrolling_groupbox_clicked(false);
    }
    if (ros_node_->mapping_mode) // Disable mapping mode
    {
      on_cancel_mapping_button_clicked();
    }
    ui.mainWindow_tabWidget->setTabEnabled(2, false);
    ui.mainWindow_tabWidget->setTabEnabled(3, false);
  }
}

void MainWindow::readSettings() // Load up qt program settings at startup
{
  QSettings settings("Horus Settings", "horus_patrol_settings");
  restoreGeometry(settings.value("geometry").toByteArray());
  restoreState(settings.value("windowState").toByteArray());
  ui.line_edit_master->setText(settings.value("master_url").toString());
  ui.line_edit_host->setText(settings.value("host_url").toString());
  map_name_ = settings.value("horus_map").toString();
  horus_path_ = settings.value("horus_path").toString();

  bool remember = settings.value("remember_settings", false).toBool();
  ui.checkbox_remember_settings->setChecked(remember);
  ui.button_connect->setChecked(remember);

  bool checked = settings.value("use_environment_variables", false).toBool();
  ui.checkbox_use_environment->setChecked(checked);
  if (checked)
  {
    ui.line_edit_master->setEnabled(false);
    ui.line_edit_host->setEnabled(false);
  }
}

void MainWindow::writeSettings() // Save program settings when closing
{
  QSettings settings("Horus Settings", "horus_patrol_settings");
  settings.setValue("horus_path", horus_path_);
  settings.setValue("horus_map", map_name_);
  settings.setValue("master_url", ui.line_edit_master->text());
  settings.setValue("host_url", ui.line_edit_host->text());
  settings.setValue("use_environment_variables",
                    ui.checkbox_use_environment->isChecked());
  settings.setValue("remember_settings",
                    ui.checkbox_remember_settings->isChecked());
  settings.setValue("geometry", saveGeometry());
  settings.setValue("mainWindow_state", saveState());
  // TODO add last route used (after updating ros_node)
}

void MainWindow::on_mainWindow_tabWidget_currentChanged(int index)
{
  // move RViz and VideoStream
  switch (index)
  {
  case 0: // home
  {
    ui.robotDashboard_groupBox->setVisible(true);
    if (ui.video_stream_button->isChecked())
      ui.gridLayout_7->addWidget(video_stream_label_, 1, 1, 1, 1);
    break;
  }
  case 1: // events logs
  {
    ui.robotDashboard_groupBox->setVisible(false);

    QFile logfile(horus_path_ + "/Log/PatrolDetections.txt");
    logfile.open(QIODevice::ReadOnly);
    QTextStream out(&logfile);
    ui.horus_patrol_logs_textbox->setPlainText(out.readAll());
    logfile.close();

    logfile.setFileName(horus_path_ + "/Log/RobotEvents.txt");
    logfile.open(QIODevice::ReadOnly);
    ui.robot_logs_textbox->setPlainText(out.readAll());
    logfile.close();
    break;
  }
  case 2: // Patrolling
  {
    ui.robotDashboard_groupBox->setVisible(true);
    if (ui.video_stream_button->isChecked())
      ui.splitter_2->addWidget(video_stream_label_);
    if (ros_node_->patrolling_mode)
    {
      ui.splitter_3->addWidget(myviz_widget_);
      ui.splitter_3->setSizes(QList<int>() << 1 << 500);
      if (ui.sequence_var->text().isEmpty())
        myviz_widget_->modeSelection(0);
      else
        myviz_widget_->modeSelection(2);
    }
    break;
  }
  case 3: // Mapping
  {
    ui.robotDashboard_groupBox->setVisible(true);
    if (ui.video_stream_button->isChecked())
    {
      ui.splitter_6->addWidget(video_stream_label_);
      ui.splitter_6->setSizes(QList<int>() << 1 << 500);
    }
    break;
  }
  case 4: // Threats
  {
    ui.robotDashboard_groupBox->setVisible(true);

    // update comboBox
    updateThreatsComboBox();

    // Setup Rviz + show button
    if (ros_node_->patrolling_mode)
    {
      ui.routeThreats_comboBox->setCurrentText(ros_node_->selected_route);
      if (ros_node_->robot_localized)
        ui.showRobot_pushButton->setEnabled(true);
      else
        ui.showRobot_pushButton->setEnabled(false);

      if (ui.showRobot_pushButton->isChecked())
        myviz_widget_->modeSelection(3);
      else
        myviz_widget_->modeSelection(1);

      ui.threatRviz_widget->layout()->addWidget(myviz_widget_);
    }
    break;
  }
  default:
    QMessageBox::critical(this, "Horus Patrol Error", "Invalid Tab Selection.");
    break;
  }
}

void MainWindow::on_load_map_clicked()
{
  QString file = QFileDialog::getOpenFileName(
      this, tr("Open Map"), horus_path_ + "/Maps", tr("Map Files (*.pgm)"));
  QFileInfo file_info(file);
  if (file_info.isFile())
  {
    map_name_ = file_info.baseName();

    if (QString::compare(file_info.absolutePath(),
                         horus_path_ + "/Maps/" + map_name_))
    {
      QDir(horus_path_ + "/Maps/").mkdir(map_name_);
      QFile::copy(file, horus_path_ + "/Maps/" + map_name_ + "/" + map_name_ +
                            ".pgm");
      QFile::copy(file_info.path() + "/" + map_name_ + ".yaml",
                  horus_path_ + "/Maps/" + map_name_ + "/" + map_name_ +
                      ".yaml");
    }
    loaded_map_.load(file);
    ui.map_preview->setPixmap(loaded_map_);
    HorusLogger* logger =
        new HorusLogger(this, "PatrolDetections.txt", "Map Loaded: " + file);
    logger->start();
  }
}

void MainWindow::updateStreamImage(const QPixmap* image)
{
  if (ui.video_stream_button->isChecked())
    video_stream_label_->setPixmap(*image);
}

/*****************************************************************************
**  Detection LOG Tab
*****************************************************************************/

void MainWindow::on_clear_patrol_log_button_clicked()
{
  QFile::remove(horus_path_ + "/Log/PatrolDetections.txt");
  ui.horus_patrol_logs_textbox->clear();
}

void MainWindow::on_clear_robot_log_button_clicked()
{
  QFile::remove(horus_path_ + "/Log/RobotEvents.txt");
  ui.robot_logs_textbox->clear();
}

/*****************************************************************************
**  Patrolling Tab
*****************************************************************************/
void MainWindow::threatDetected()
{
  ThreatDetection* tDetection = new ThreatDetection(ros_node_);
  tDetection->run();
  if (tDetection->accepted_)
  {
    ui.mainWindow_tabWidget->setCurrentIndex(4);
  }
  model_->select();
  delete tDetection;
}

void MainWindow::on_patrolling_groupbox_clicked(bool checked)
{
  if (checked)
  {
    if (!map_name_.isEmpty())
    {
      ui.mainWindow_tabWidget->setTabEnabled(3, false);
      process_ = new QProcess();
      process_->start(QString("xterm -iconic -e roslaunch turtlebot_gazebo "
                              "amcl_demo.launch map_file:=" +
                              horus_path_ + "/Maps/" + map_name_ + "/" +
                              map_name_ + ".yaml"));
      process_->waitForStarted(-1);
      ros_node_->patrolling_mode = true;
      horusRVizSetup();
      updatePatrolInfoBox(42);
      HorusLogger* logger = new HorusLogger(this, "PatrolDetections.txt",
                                            "Horus patrolling system: ON.");
      logger->start();
    }
    else
    {
      ui.patrolling_groupbox->setChecked(false);
      QMessageBox::information(this, tr("Horus Patrol information"),
                               tr("No map Loaded.<br>Please Load a map before "
                                  "running the patrol system."));
    }
  }
  else
  {
    process_->terminate();
    process_->waitForFinished(-1);
    delete process_;
    delete myviz_widget_;
    ros_node_->patrolling_mode = false;
    ros_node_->patrolling_mode_run = false;
    ros_node_->robot_localized = false;
    requested_destination_index_ = 0;
    clearRouteSelection();

    ui.mainWindow_tabWidget->setTabEnabled(3, true);
    ui.start_patrolling_button->setEnabled(false);
    ui.start_patrolling_button->setText("S&tart");
    ui.start_patrolling_button->setStyleSheet("");
    ui.stop_patrolling_button->setEnabled(false);
    ui.stop_patrolling_button->setText("Ca&ncel");
    ui.stop_patrolling_button->setStyleSheet("");
    ui.goto_place_combobox->blockSignals(true);
    ui.goto_place_combobox->clear();
    ui.goto_place_combobox->setEnabled(false);
    ui.goto_place_combobox->blockSignals(false);
    ui.route_selection->setEnabled(true);
    ui.setgoal_pushbutton->setEnabled(false);

    updatePatrolInfoBox(0);
    HorusLogger* logger = new HorusLogger(this, "PatrolDetections.txt",
                                          "Horus patrolling system: OFF.");
    logger->start();
  }
}

void MainWindow::horusRVizSetup()
{
  myviz_widget_ = new horusRViz(this);
  myviz_widget_->setSizePolicy(QSizePolicy::MinimumExpanding,
                               QSizePolicy::MinimumExpanding);
  if (ros_node_->patrolling_mode)
  {
    ui.splitter_3->addWidget(myviz_widget_);
    ui.splitter_3->setSizes(QList<int>() << 1 << 500);
  }
  else if (ros_node_->mapping_mode)
  {
    ui.splitter_5->addWidget(myviz_widget_);
    ui.splitter_5->setSizes(QList<int>() << 1 << 500);

    myviz_widget_->modeSelection(2);
  }
}

// control patrol tab state
void MainWindow::setupPatrollingTab()
{
  ui.start_patrolling_button->setEnabled(true);
  ui.start_patrolling_button->setText("S&tart");
  ui.stop_patrolling_button->setEnabled(false);
  ui.stop_patrolling_button->setText("Ca&ncel");
  ui.goto_place_combobox->addItem("Go to..");
  ui.goto_place_combobox->setEnabled(true);
  ui.route_selection->setEnabled(true);

  requested_destination_index_ = 0;
  updatePatrolInfoBox(42);
}

// Clear Selected Route Information
void MainWindow::clearRouteSelection()
{
  ros_node_->patrol_initial_pose_name.clear();
  ros_node_->patrol_initial_pose_x = 0;
  ros_node_->patrol_initial_pose_y = 0;
  ros_node_->patrol_initial_pose_orientation = 0;
  ros_node_->patrol_places_name.clear();
  ros_node_->patrol_places_x.clear();
  ros_node_->patrol_places_y.clear();
  ros_node_->patrol_places_orientation.clear();
  ros_node_->total_patrol_places = 0;
  ros_node_->selected_route.clear();
}

void MainWindow::on_route_selection_clicked()
{
  patrol_dialog_ =
      new PatrolDialog(ros_node_, myviz_widget_, admin_profile_, this);
  patrol_dialog_->exec();
  if (patrol_dialog_->result())
  {
    setupPatrollingTab();
    for (short int i = 0; i < ros_node_->total_patrol_places; i++)
      ui.goto_place_combobox->addItem(ros_node_->patrol_places_name[i]);
    ros_node_->patrolSystemSetup();
  }
  else
  {
    clearRouteSelection();
  }
  ui.splitter_3->addWidget(myviz_widget_);
  ui.splitter_3->setSizes(QList<int>() << 1 << 500);
  updateThreatsComboBox();
  delete patrol_dialog_;
}

void MainWindow::on_goto_place_combobox_currentIndexChanged(int index)
{
  if (index == 0)
  {
    ui.start_patrolling_button->setText("Start");
    ui.start_patrolling_button->setStyleSheet("");
    ui.start_patrolling_button->setEnabled(true);
    ui.stop_patrolling_button->setText("C&ancel");
    ui.stop_patrolling_button->setStyleSheet("");
    ui.stop_patrolling_button->setEnabled(false);
  }
  else
  {
    if (requested_destination_index_ != index)
      ui.start_patrolling_button->setEnabled(true);
    else
      ui.start_patrolling_button->setEnabled(false);
    ui.start_patrolling_button->setText("GO!");
    ui.start_patrolling_button->setStyleSheet("font:bold italic");
    ui.stop_patrolling_button->setText("C&ancel");
    ui.stop_patrolling_button->setStyleSheet("color:red");
    ui.stop_patrolling_button->setEnabled(true);
  }
}

void MainWindow::on_start_patrolling_button_clicked()
{
  if (ui.goto_place_combobox->currentIndex() == 0) // Patrol start
  {
    ui.goto_place_combobox->setEnabled(false);
    ros_node_->patrolling_mode_run = true;
    HorusLogger* logger = new HorusLogger(
        this, "PatrolDetections.txt",
        "Patrol sequence started. Selected: " + ros_node_->selected_route);
    logger->start();
    ros_node_->moveToInitialPosition();
  }
  else //"Go To Place" start
  {
    requested_destination_index_ = ui.goto_place_combobox->currentIndex();
    HorusLogger* logger = new HorusLogger(
        this, "PatrolDetections.txt",
        "Custom movement requested: " +
            ros_node_->patrol_places_name[requested_destination_index_ - 1]);
    logger->start();
    ros_node_->moveToPosition(requested_destination_index_ - 1);
  }
  ui.route_selection->setEnabled(false);
  ros_node_->patrolplace_it = 0;
  updatePatrolInfoBox(3);
  ui.start_patrolling_button->setEnabled(false);
}

void MainWindow::on_stop_patrolling_button_clicked()
{
  if (!(ros_node_->patrolling_mode_run))
  {
    ui.goto_place_combobox->setCurrentIndex(0);
    ui.start_patrolling_button->setEnabled(false);
    HorusLogger* logger = new HorusLogger(this, "PatrolDetections.txt",
                                          "Custom movement mode cancelled.");
    logger->start();
  }
  else
  {
    HorusLogger* logger = new HorusLogger(this, "PatrolDetections.txt",
                                          "Patrol sequence aborted.");
    logger->start();
  }
  updatePatrolInfoBox(11);
  ros_node_->moveToInitialPosition();
  ui.stop_patrolling_button->setEnabled(false);
}

void MainWindow::on_set_route_pose_pushbutton_clicked()
{
  myviz_widget_->toolmanager_ = myviz_widget_->manager_->getToolManager();
  myviz_widget_->mytool_ =
      myviz_widget_->toolmanager_->addTool("rviz/SetInitialPose");
  myviz_widget_->toolmanager_->setCurrentTool(myviz_widget_->mytool_);
}

void MainWindow::on_setgoal_pushbutton_clicked()
{
  ros_node_->custom_goal;
  myviz_widget_->toolmanager_ = myviz_widget_->manager_->getToolManager();
  myviz_widget_->mytool_ = myviz_widget_->toolmanager_->addTool("rviz/SetGoal");
  myviz_widget_->toolmanager_->setCurrentTool(myviz_widget_->mytool_);
  HorusLogger* logger =
      new HorusLogger(this, "PatrolDetections.txt", "Custom Goal Sent.");
  logger->start();
  updatePatrolInfoBox(12);
  if ((ros_node_->patrolling_mode_run) && (ros_node_->patrolplace_it > 0))
  {
    ros_node_->patrolplace_it--;
  }
}

void MainWindow::robotLocalized()
{
  myviz_widget_->modeSelection(2);
  if (!ros_node_->managing_route)
  {
    ui.setgoal_pushbutton->setEnabled(true);
    ros_node_->robot_localized = true;
  }
  else
  {
    ui.setgoal_pushbutton->setEnabled(false);
    ros_node_->robot_localized = false;
  }
}

void MainWindow::updatePatrolInfoBox(
    int status) // patrol information box and buttons(Enabled/Disable)
{
  switch (status)
  {
  case -2: // Manual Disabled
    ui.status_var->setText(status_var_text_);
    ui.patrolling_buttons_widget->setEnabled(true);
    ui.movingto_var->setVisible(true);
    ui.sequence_var->setVisible(true);
    ui.currentplace_var->setVisible(true);
    break;
  case -1: // Manual Enabled
    ui.status_var->setText("<b><font color='Red'>AT MANUAL</b></font>");
    ui.patrolling_buttons_widget->setEnabled(false);
    ui.movingto_var->setVisible(false);
    ui.sequence_var->setVisible(false);
    ui.currentplace_var->setVisible(false);
    ui.linear_label->setNum(0);
    ui.angular_label->setNum(0);
    break;
  case 0:
    ui.status_var->clear();
    ui.sequence_var->clear();
    ui.movingto_var->clear();
    ui.currentplace_var->clear();
    break;
  case 3:
    if (ros_node_->patrolling_mode_run) // patrolling
    {
      if (ros_node_->patrolplace_it == (ros_node_->total_patrol_places + 1))
      {
        status_var_text_ = "Patrol Sequence Completed";
        ui.status_var->setText(status_var_text_);
        ui.movingto_var->setText(ros_node_->patrol_initial_pose_name);
        ui.currentplace_var->setText("");
        ui.stop_patrolling_button->setEnabled(false);
        HorusLogger* logger = new HorusLogger(
            this, "PatrolDetections.txt",
            "Patrol sequence " + ros_node_->selected_route + " completed.");
        logger->start();
      }
      else if (!(ros_node_->patrolplace_it))
      {
        status_var_text_ = "Patrolling";
        ui.status_var->setText(status_var_text_);
        ui.movingto_var->setText(ros_node_->patrol_initial_pose_name);
      }
      else
      {
        status_var_text_ = "Patrolling";
        ui.status_var->setText(status_var_text_);
        ui.movingto_var->setText(
            ros_node_->patrol_places_name[ros_node_->patrolplace_it - 1]);
        ui.currentplace_var->setText(QString("%1 out of %2")
                                         .arg(ros_node_->patrolplace_it)
                                         .arg(ros_node_->total_patrol_places));
      }
    }
    else if (requested_destination_index_ != 0)
    {
      if (ros_node_->patrolplace_it == 0)
      {
        status_var_text_ = "Moving to Requested Destination";
        ui.status_var->setText(status_var_text_);
        ui.movingto_var->setText(
            ros_node_->patrol_places_name[requested_destination_index_ - 1]);
      }
      else
      {
        status_var_text_ = "At Requested Destination";
        ui.status_var->setText(status_var_text_);
        ui.movingto_var->setText("");
        ros_node_->patrolplace_it = 0;
        requested_destination_index_ = 0;
      }
    }
    break;
  case 4: // ERROR
    status_var_text_ = "<b><font color='Red'>ERROR!</b></font>";
    ui.status_var->setText(status_var_text_);
    ui.start_patrolling_button->setEnabled(false);
    ui.stop_patrolling_button->setEnabled(true);
    break;
  case 11:
    if (ros_node_->patrolling_mode_run)
    {
      status_var_text_ = "Patrol Sequence Aborted";
      ui.status_var->setText(status_var_text_);
      ui.movingto_var->setText(ros_node_->patrol_initial_pose_name);
      ui.currentplace_var->setText("");
    }
    else
    {
      status_var_text_ = "Custom Destination Mode Cancelled";
      ui.status_var->setText(status_var_text_);
      ui.movingto_var->setText(ros_node_->patrol_initial_pose_name);
    }
    break;
  case 12:
    status_var_text_ = "Moving to Requested Goal";
    ui.status_var->setText(status_var_text_);
    ui.movingto_var->setText("Goal");
    ui.currentplace_var->clear();
    break;
  case 42:
    if (!ros_node_->selected_route.isEmpty())
    {
      status_var_text_ =
          QString("Standby at %1").arg(ros_node_->patrol_initial_pose_name);
      ui.status_var->setText(status_var_text_);
      ui.sequence_var->setText(ros_node_->selected_route);
      ui.movingto_var->clear();
      ui.currentplace_var->clear();
    }
    else
    {
      status_var_text_ = "Waiting Route Selection";
      ui.status_var->setText(status_var_text_);
      ui.sequence_var->clear();
      ui.movingto_var->clear();
      ui.currentplace_var->clear();
    }
    break;
  }
}

/*****************************************************************************
**  Build a Map Tab
*****************************************************************************/

void MainWindow::on_start_mapping_button_clicked()
{
  ui.start_mapping_button->setEnabled(false);
  ui.cancel_mapping_button->setEnabled(true);
  ui.custom_movement_button->setEnabled(true);
  ui.save_map_button->setEnabled(true);
  ui.mainWindow_tabWidget->setTabEnabled(2, false);
  ui.mainWindow_tabWidget->setTabEnabled(4, false);

  process_ = new QProcess();
  process_->start(
      "xterm -iconic -e roslaunch turtlebot_gazebo gmapping_demo.launch");
  process_->waitForStarted(-1);
  ros_node_->mapping_mode = true;
  horusRVizSetup();
  HorusLogger* logger =
      new HorusLogger(this, "PatrolDetections.txt", "Horus mapping mode: ON.");
  logger->start();
  updateMappingInfoBox(1);
}

// what should it do when the mapping mode is aborted
void MainWindow::on_cancel_mapping_button_clicked()
{
  if (!ros_node_->custom_move_requested)
  {
    ui.start_mapping_button->setEnabled(true);
    ui.cancel_mapping_button->setEnabled(false);
    ui.custom_movement_button->setEnabled(false);
    ui.save_map_button->setEnabled(false);
    ui.mainWindow_tabWidget->setTabEnabled(2, true);
    ui.mainWindow_tabWidget->setTabEnabled(4, true);

    process_->terminate();
    process_->waitForFinished(-1);
    delete process_;
    delete myviz_widget_;
    ros_node_->mapping_mode = false;
    HorusLogger* logger = new HorusLogger(this, "PatrolDetections.txt",
                                          "Horus mapping mode: OFF.");
    logger->start();
    updateMappingInfoBox(2);
  }
  else
  {
    ros_node_->custom_move_requested = false;
    ui.custom_movement_button->setEnabled(true);
    ros_node_->move('s');
    updateMappingInfoBox(3);
  }
}

void MainWindow::on_save_map_button_clicked()
{
  if (ui.manual_control_button->isChecked())
  {
    ui.manual_control_button->setChecked(false);
    on_manual_control_button_clicked(false);
  }

  QString file = QFileDialog::getSaveFileName(
      this, tr("Save Map"), horus_path_ + "/Maps", tr("Map Files (*.pgm)"));
  QFileInfo file_info(file);
  QDir(file_info.absolutePath()).mkdir(file_info.baseName());
  if (!file.isEmpty())
  {
    QProcess* process_ = new QProcess();
    process_->start(QString("xterm -iconic -e rosrun map_server map_saver -f " +
                            file_info.baseName()));
    process_->waitForFinished(-1);
    process_->terminate();
    delete process_;
    QFile::rename(QDir::currentPath() + "/" + file_info.baseName() + ".pgm",
                  file_info.filePath() + "/" + file_info.baseName() + ".pgm");
    QFile::rename(QDir::currentPath() + "/" + file_info.baseName() + ".yaml",
                  file_info.filePath() + "/" + file_info.baseName() + ".yaml");
    updateMappingInfoBox(4);
    HorusLogger* logger = new HorusLogger(this, "PatrolDetections.txt",
                                          "Map saved at [" + file + "].");
    logger->start();
  }
}

void MainWindow::on_custom_movement_button_clicked()
{
  custom_move_dialog_ = new CustomMoveDialog(ros_node_, this);
  custom_move_dialog_->exec();
  if (custom_move_dialog_->result())
  {
    ros_node_->custom_move_requested = true;
    ui.custom_movement_button->setEnabled(false);
    ui.move_method_var->setText(QString("%1 (%2x%3)  Repeat: x%4")
                                    .arg(ros_node_->selected_cmove)
                                    .arg(ros_node_->side1)
                                    .arg(ros_node_->side2)
                                    .arg(ros_node_->nlaps));
  }
  delete custom_move_dialog_;
}

void MainWindow::updateMappingInfoBox(int status)
{
  switch (status)
  {
  case 1: // start button
    mapping_time_ = new QTime;
    mapping_time_->start();
    timer_ = new QTimer(this);
    timer_->setInterval(1000);
    QObject::connect(timer_, SIGNAL(timeout()), this,
                     SLOT(updateMappingTime()));
    timer_->start();
    ui.status_var_2->setText("Mapping");
    ui.move_method_var->setText("Manual");
    ui.lastsavetime_var->setText("No saved map");
    break;
  case 2: // stop mapping button
    ui.status_var_2->setText("Standby");
    ui.move_method_var->clear();
    ui.elapsedtime_var->clear();
    ui.lastsavetime_var->clear();
    delete mapping_time_;
    delete timer_;
    break;
  case 3: // stop custom button
    ui.status_var_2->setText("Mapping");
    ui.move_method_var->setText("Manual");
    break;
  case 4:
    ui.lastsavetime_var->setText(
        QTime(0, 0).addMSecs(mapping_time_->elapsed()).toString("hh:mm:ss"));
    break;
  }
}

void MainWindow::updateMappingTime()
{
  ui.elapsedtime_var->setText(
      QTime(0, 0).addMSecs(mapping_time_->elapsed()).toString("hh:mm:ss"));
}

/*****************************************************************************
** Turtlebot Panel
*****************************************************************************/

void MainWindow::bumperEvent(bool msg)
{
  ui.manual_control_button->setChecked(false);
  on_manual_control_button_clicked(false);
  if (ros_node_->patrolling_mode)
  {
    updatePatrolInfoBox(-2);
  }
  HorusLogger* logger = new HorusLogger(this, "RobotEvents.txt",
                                        "[WARNING] Robot Bumper Pressed.");
  logger->start();
}

void MainWindow::on_motor_button_clicked(bool checked)
{
  if (checked)
  {
    ros_node_->KobukiMotors(true);
    ui.motor_button->setIcon(QIcon(":/images/motor/motor (green).svg"));
    ui.manual_control_button->setEnabled(true);
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "Motors enabled.");
    logger->start();
  }
  else
  {
    ros_node_->KobukiMotors(false);
    ui.motor_button->setIcon(QIcon(":/images/motor/motor (red).svg"));
    if (ui.manual_control_button->isChecked())
    {
      ui.manual_control_button->setChecked(false);
      on_manual_control_button_clicked(false);
    }
    ui.manual_control_button->setEnabled(false);
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "Motors disabled.");
    logger->start();
  }
}

void MainWindow::on_manual_control_button_clicked(bool checked)
{
  if (checked)
  {
    ui.manual_control_button->setIcon(
        QIcon(":/images/steering wheel/steering-wheel (green).svg"));
    ui.joystick_up->setEnabled(true);
    ui.joystick_back->setEnabled(true);
    ui.joystick_left->setEnabled(true);
    ui.joystick_right->setEnabled(true);
    ui.joystick_stop->setEnabled(true);
    ros_node_->manual_control_enabled = true;
    ros_node_->custom_move_requested = false;
    ros_node_->turn360_requested = false;
    if (ros_node_->patrolling_mode)
    {
      updatePatrolInfoBox(-1);
    }
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "Manual mode: ON");
    logger->start();
  }
  else
  {
    on_joystick_stop_clicked();
    ui.manual_control_button->setIcon(
        QIcon(":/images/steering wheel/steering-wheel (red).svg"));
    ui.joystick_up->setEnabled(false);
    ui.joystick_back->setEnabled(false);
    ui.joystick_left->setEnabled(false);
    ui.joystick_right->setEnabled(false);
    ui.joystick_stop->setEnabled(false);
    ros_node_->manual_control_enabled = false;
    if (ros_node_->patrolling_mode)
    {
      updatePatrolInfoBox(-2);
    }
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "Manual mode: OFF");
    logger->start();
  }
}

void MainWindow::on_kinect_button_clicked(bool checked)
{
  if (checked)
  {
    ui.kinect_button->setIcon(QIcon(":/images/kinect/kinect (green).svg"));
    ui.capture_image->setEnabled(true);
    ui.video_stream_button->setEnabled(true);
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "Camera enabled.");
    logger->start();
  }
  else
  {
    ui.kinect_button->setIcon(QIcon(":/images/kinect/kinect (red).svg"));
    ui.capture_image->setEnabled(false);
    if (ui.video_stream_button->isChecked())
    {
      ui.video_record_button->setChecked(false);
      on_video_stream_button_clicked(false);
    }
    ui.video_stream_button->setEnabled(false);
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "Camera disabled.");
    logger->start();
  }
}

void MainWindow::on_video_stream_button_clicked(bool checked)
{
  if (checked)
  {
    ui.video_stream_button->setIcon(
        QIcon(":/images/streams/video/video (green).svg"));
    ros_node_->streaming = true;
    video_stream_label_->setVisible(true);
    switch (ui.mainWindow_tabWidget->currentIndex())
    {
    case 0:
      ui.gridLayout_7->addWidget(video_stream_label_, 1, 1, 2, 2);
      break;
    case 2:
      ui.splitter_2->addWidget(video_stream_label_);
      ui.splitter_2->setSizes(QList<int>() << 1 << 500);
      break;
    case 3:
      ui.splitter_6->addWidget(video_stream_label_);
      ui.splitter_6->setSizes(QList<int>() << 1 << 500);
      break;
    default:
      break;
    }

    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "VIDEO Streaming: ON.");
    logger->start();
  }
  else
  {
    ui.video_stream_button->setChecked(false);
    ui.video_stream_button->setIcon(
        QIcon(":/images/streams/video/video (red).svg"));
    ros_node_->streaming = false;
    ui.video_stream_button->setChecked(false);
    video_stream_label_->setVisible(false);
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "VIDEO Streaming: OFF.");
    logger->start();
  }
}

void MainWindow::on_video_record_button_clicked(bool checked)
{
  if (checked)
  {
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "Video Recording: ON.");
    logger->start();
    ui.video_record_button->setIcon(
        QIcon(":/images/video record/video record (green).svg"));
  }
  else
  {
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "Video Recording: OFF.");
    logger->start();
    ui.video_record_button->setIcon(
        QIcon(":/images/video record/video record (red).svg"));
  }
}

void MainWindow::on_audio_stream_button_clicked(bool checked)
{
  if (checked)
  {
    ui.audio_stream_button->setIcon(
        QIcon(":/images/streams/audio/audio (green).svg"));
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "AUDIO Streaming: ON.");
    logger->start();
  }
  else
  {
    ui.audio_stream_button->setIcon(
        QIcon(":/images/streams/audio/audio (red).svg"));
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "AUDIO Streaming: OFF.");
    logger->start();
  }
}

void MainWindow::on_microphone_button_clicked(bool checked)
{
  if (checked)
  {
    ui.microphone_button->setIcon(
        QIcon(":/images/streams/microphone/microphone (green).svg"));
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "VOICE Streaming: ON.");
    logger->start();
  }
  else
  {
    ui.microphone_button->setIcon(
        QIcon(":/images/streams/microphone/microphone (red).svg"));
    HorusLogger* logger =
        new HorusLogger(this, "RobotEvents.txt", "VOICE Streaming: OFF.");
    logger->start();
  }
}

void MainWindow::on_capture_image_pressed()
{
  ui.capture_image->setIcon(QIcon(":/images/camera/camera (green).svg"));
  QFile file(horus_path_+"/Captured Images/Screenshot - " +
             QDateTime::currentDateTime().toString("yyyy-MM-dd  hh-mm-ss") +
             ".png");
  file.open(QIODevice::WriteOnly);
  ros_node_->captured_image.save(&file, "PNG");
  HorusLogger* logger =
      new HorusLogger(this, "RobotEvents.txt", "Photo taken.");
  logger->start();
}

void MainWindow::on_capture_image_released()
{
  ui.capture_image->setIcon(QIcon(":/images/camera/camera (red).svg"));
}

void MainWindow::on_joystick_up_clicked() { ros_node_->move('f'); }
void MainWindow::on_joystick_back_clicked() { ros_node_->move('b'); }
void MainWindow::on_joystick_left_clicked() { ros_node_->move('l'); }
void MainWindow::on_joystick_right_clicked() { ros_node_->move('r'); }
void MainWindow::on_joystick_stop_clicked() { ros_node_->move('s'); }

bool MainWindow::eventFilter(QObject* object, QEvent* event)
{
  if ((event->type() == QEvent::KeyPress) &&
      (ros_node_->manual_control_enabled))
  {
    QKeyEvent* key_event = static_cast<QKeyEvent*>(event);
    switch (key_event->key())
    {
    case Qt::Key_Up:
      ros_node_->move('f');
      return true;
    case Qt::Key_Down:
      ros_node_->move('b');
      return true;
    case Qt::Key_Left:
      ros_node_->move('l');
      return true;
    case Qt::Key_Right:
      ros_node_->move('r');
      return true;
    case Qt::Key_W:
      ros_node_->move('f');
      return true;
    case Qt::Key_S:
      ros_node_->move('b');
      return true;
    case Qt::Key_A:
      ros_node_->move('l');
      return true;
    case Qt::Key_D:
      ros_node_->move('r');
      return true;
    case Qt::Key_R:
      ros_node_->move('s');
      return true;
    default:
      event->ignore();
      return false;
    }
  }
}

/*****************************************************************************
** Threats Detection Tab
*****************************************************************************/

void MainWindow::on_clearThreats_pushButton_clicked()
{
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(
      this, QString::fromUtf8("Delete Threat"),
      QString::fromUtf8("Are you sure you want to delete all threats?"),
      QMessageBox::Yes | QMessageBox::No);
  if (reply == QMessageBox::Yes)
  {
    QSqlQuery query;
    query.exec("TRUNCATE TABLE `threats`.`" +
               ui.routeThreats_comboBox->currentText() + "`;");
    QDir dir = QDir(horus_path_+"/Detected Threats/" +
                    ui.routeThreats_comboBox->currentText());
    dir.removeRecursively();
    dir.mkdir(horus_path_+"/Detected Threats/" +
              ui.routeThreats_comboBox->currentText());

    model_->select();
    ui.clearThreats_pushButton->setEnabled(false);
    ui.removeThreat_pushButton->setEnabled(false);
    ui.threatPhoto_label->clear();

    ui.threatslog_tableView->setVisible(false);
    ui.threatslog_tableView->resizeColumnsToContents();
    ui.threatslog_tableView->setFixedWidth(
        ui.threatslog_tableView->horizontalHeader()->length() +
        ui.threatslog_tableView->verticalHeader()->width() + 2);
    ui.threatslog_tableView->setVisible(true);

    if (ros_node_->patrolling_mode)
    {
      if (ui.showRobot_pushButton->isChecked())
        myviz_widget_->modeSelection(2);
      else
        myviz_widget_->modeSelection(0);
    }

    QMessageBox::information(this, tr("Horus Patrol information"),
                             tr("All Threats were deleted."));
  }
}

void MainWindow::threatsTableViewRowChanged(const QModelIndex& index)
{
  ui.threatPhoto_label->setPixmap(
      QPixmap(horus_path_+"/Detected Threats/" +
              ui.routeThreats_comboBox->currentText() + "/Detection - " +
              model_->index(index.row(), 0).data().toString() + ".png"));
  if (ros_node_->patrolling_mode)
  {
    geometry_msgs::PointStamped threat_point;
    threat_point.header.frame_id = "map";
    threat_point.point.x = model_->index(index.row(), 2).data().toDouble();
    threat_point.point.y = model_->index(index.row(), 3).data().toDouble();
    ros_node_->threat_pub.publish(threat_point);
  }
}

void MainWindow::updateThreatsComboBox()
{
  ui.routeThreats_comboBox->blockSignals(true);
  ui.routeThreats_comboBox->clear();
  QSqlQuery query;
  query.exec("USE `threats`");
  if (query.exec("SHOW TABLES FROM `threats`;"))
  {
    while (query.next())
      ui.routeThreats_comboBox->addItem(query.value(0).toString());

    if (!ui.routeThreats_comboBox->count())
    {
      ui.routeThreats_comboBox->setEnabled(false);
      ui.routeThreats_comboBox->addItem("No Routes...");
    }
    else
      ui.routeThreats_comboBox->setEnabled(true);
  }
  else
    QMessageBox::critical(this, tr("Horus Patrol error"),
                          tr("Could not access database."));

  on_routeThreats_comboBox_currentIndexChanged(0);
  ui.routeThreats_comboBox->blockSignals(false);
}

void MainWindow::on_routeThreats_comboBox_currentIndexChanged(int index)
{
  model_->setTable("`" + ui.routeThreats_comboBox->currentText() + "`");
  model_->select();
  ui.threatslog_tableView->setModel(model_);
  ui.threatslog_tableView->setVisible(false);
  ui.threatslog_tableView->hideColumn(2);
  ui.threatslog_tableView->hideColumn(3);
  ui.threatslog_tableView->hideColumn(4);
  ui.threatslog_tableView->resizeColumnsToContents();
  ui.threatslog_tableView->setFixedWidth(
      ui.threatslog_tableView->horizontalHeader()->length() +
      ui.threatslog_tableView->verticalHeader()->width() + 2);
  ui.threatslog_tableView->setVisible(true);

  if (!model_->rowCount())
  {
    ui.clearThreats_pushButton->setEnabled(false);
    ui.removeThreat_pushButton->setEnabled(false);
    ui.threatPhoto_label->clear();
  }
  else
  {
    ui.clearThreats_pushButton->setEnabled(true);
    ui.removeThreat_pushButton->setEnabled(true);
    ui.threatslog_tableView->setCurrentIndex(model_->index(0, 0));
    ui.threatPhoto_label->setPixmap(
        QPixmap(horus_path_+"/Detected Threats/" +
                ui.routeThreats_comboBox->currentText() + "/Detection - " +
                model_->index(0, 0).data().toString() + ".png"));
  }

  QObject::connect(ui.threatslog_tableView->selectionModel(),
                   SIGNAL(currentRowChanged(QModelIndex, QModelIndex)), this,
                   SLOT(threatsTableViewRowChanged(QModelIndex)));
}

void MainWindow::on_showRobot_pushButton_clicked(bool checked)
{
  if (checked)
  {
    ui.showRobot_pushButton->setText("Hide Robot");
    myviz_widget_->modeSelection(3);
  }
  else
  {
    ui.showRobot_pushButton->setText("Show Robot");
    myviz_widget_->modeSelection(1);
  }
}

void MainWindow::on_removeThreat_pushButton_clicked()
{
  short int row = ui.threatslog_tableView->currentIndex().row();
  QMessageBox::StandardButton reply;
  reply = QMessageBox::question(
      this, QString::fromUtf8("Delete Threat"),
      QString::fromUtf8("Are you sure you want to delete threat number: %1?")
          .arg(row + 1),
      QMessageBox::Yes | QMessageBox::No);
  if (reply == QMessageBox::Yes)
  {
    QDir dir = QDir(horus_path_+"/Detected Threats/" +
                    ui.routeThreats_comboBox->currentText());
    dir.remove("Detection - " + model_->index(row, 0).data().toString() +
               ".png");

    QSqlQuery query;
    query.exec("USE `threats`");
    query.exec("DELETE FROM `" + ui.routeThreats_comboBox->currentText() +
               "` WHERE `Time` = '" + model_->index(row, 0).data().toString() +
               "';");
    model_->select();

    if (!model_->rowCount())
    {
      ui.clearThreats_pushButton->setEnabled(false);
      ui.removeThreat_pushButton->setEnabled(false);
      ui.threatPhoto_label->clear();
    }
    else
    {
      ui.threatslog_tableView->setCurrentIndex(model_->index(0, 0));
      ui.threatPhoto_label->setPixmap(
          QPixmap(horus_path_+"/Detected Threats/" +
                  ui.routeThreats_comboBox->currentText() + "/" +
                  model_->index(0, 0).data().toString() + ".png"));
    }
  }
}
} // namespace HorusPatrol
