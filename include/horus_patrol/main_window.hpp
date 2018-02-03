#ifndef HORUS_PATROL_MAIN_WINDOW_H
#define HORUS_PATROL_MAIN_WINDOW_H

#include <QProcess>
#include <QTime>
#include <QTimer>
#include <QtSql>
#include <QShortcut>
#include <QFileDialog>
#include <QKeyEvent>
#include "ros_node.hpp"
#include "patrol_dialog.hpp"
#include "cmove_dialog.hpp"
#include "rviz_interface.hpp"
#include "horus_logger.hpp"
#include "login_dialog.hpp"
#include "ui_main_window.h"
#include "threat_detection.hpp"

namespace HorusPatrol
{

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(ROSNode* node, bool admin, QWidget* parent = 0);
  ~MainWindow();

  horusRViz* myviz_widget_;
  QTimer* timer_;
  QTime* mapping_time_;
  short int requested_destination_index_;

  void readSettings();
  void writeSettings();

public Q_SLOTS:
  void bumperEvent(bool msg);
  void threatDetected();
  void robotLocalized();
  void setupPatrollingTab();
  void updatePatrolInfoBox(int status);
  void updateMappingInfoBox(int status);
  void updateMappingTime();
  void updateStreamImage(const QPixmap* image);
  void threatsTableViewRowChanged(const QModelIndex& index);

  void on_actionAbout_triggered();
  void on_actionAbout_Qt_triggered();
  void on_button_connect_clicked(bool check);
  void on_checkbox_use_environment_stateChanged(int state);
  void on_action_Preferences_triggered();
  void on_actionNetwork_Settings_triggered();
  void on_actionLoad_Map_triggered();
  void on_load_map_clicked();
  void on_joystick_up_clicked();
  void on_joystick_back_clicked();
  void on_joystick_left_clicked();
  void on_joystick_right_clicked();
  void on_joystick_stop_clicked();
  void on_manual_control_button_clicked(bool checked);
  void on_motor_button_clicked(bool checked);
  void on_kinect_button_clicked(bool checked);
  void on_mainWindow_tabWidget_currentChanged(int index);
  void on_route_selection_clicked();
  void on_patrolling_groupbox_clicked(bool checked);
  void on_start_patrolling_button_clicked();
  void on_stop_patrolling_button_clicked();
  void on_goto_place_combobox_currentIndexChanged(int index);
  void on_video_stream_button_clicked(bool checked);
  void on_clear_patrol_log_button_clicked();
  void on_clear_robot_log_button_clicked();
  void on_start_mapping_button_clicked();
  void on_cancel_mapping_button_clicked();
  void on_save_map_button_clicked();
  void on_custom_movement_button_clicked();
  void on_set_route_pose_pushbutton_clicked();
  void on_setgoal_pushbutton_clicked();
  void on_audio_stream_button_clicked(bool checked);
  void on_microphone_button_clicked(bool checked);
  void on_capture_image_pressed();
  void on_capture_image_released();
  void on_video_record_button_clicked(bool checked);
  void on_clearThreats_pushButton_clicked();
  void on_routeThreats_comboBox_currentIndexChanged(int index);
  void on_showRobot_pushButton_clicked(bool checked);
  void on_removeThreat_pushButton_clicked();

private:
  Ui::MainWindowDesign ui;
  CustomMoveDialog* custom_move_dialog_;
  PatrolDialog* patrol_dialog_;
  ROSNode* ros_node_;
  bool admin_profile_, threat_window_open;
  QPixmap loaded_map_;
  QProcess* process_;
  QLabel* video_stream_label_;
  QShortcut* shortcut3, *shortcut4;
  QSignalMapper* map;
  QSqlTableModel* model_;
  QString status_var_text_;
  QString horus_path_, map_name_;

  bool eventFilter(QObject *object, QEvent *event);
  void horusRVizSetup();
  void updateThreatsComboBox();
  void clearRouteSelection();
};

} // namespace horus_patrol

#endif
