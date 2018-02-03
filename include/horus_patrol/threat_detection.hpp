#ifndef THREATDETECTION_H
#define THREATDETECTION_H

#include <QThread>
#include <QDir>
#include <QSqlQuery>
#include <QMessageBox>
#include <QGridLayout>
#include <QPushButton>
#include <QLabel>
#include <QSoundEffect>
#include <QSettings>
#include "horus_logger.hpp"
#include "ros_node.hpp"

namespace HorusPatrol
{

class ThreatDetection : public QThread
{
  Q_OBJECT

public:
  explicit ThreatDetection(ROSNode* node);
  ~ThreatDetection();
  bool accepted_;

  void run();

private:
  static bool dialog_openned_;
  ROSNode* ros_node_;
  short int threat_counter_;
  QString threat_place_;
  QDateTime time_detected_;

  void calculateThreatPosition();
};
}
#endif // THREATDETECTION_H
