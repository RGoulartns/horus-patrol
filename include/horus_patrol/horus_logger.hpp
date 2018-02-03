#ifndef HORUSLOGGER_HPP
#define HORUSLOGGER_HPP

#include <QThread>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QSettings>
#include <QDir>

namespace HorusPatrol
{

class HorusLogger : public QThread
{
  Q_OBJECT

public:
  explicit HorusLogger(QObject* parent, QString file_path, QString message);
  ~HorusLogger();

  void run();

private:
  QFile* log_file_;
  QString file_;
  QString message_;

};
}
#endif
