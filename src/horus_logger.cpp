#include "../include/horus_patrol/horus_logger.hpp"

namespace HorusPatrol
{

HorusLogger::HorusLogger(QObject* parent, QString file_, QString message)
    : QThread(parent)
{
  QString horus_log_path =
      QSettings("Horus Settings", "horus_patrol_settings")
          .value("horus_path")
          .toString() + "/Log/";

  if(!QDir(horus_log_path).exists())
    QDir().mkdir(horus_log_path);

  message_ = message;
  log_file_ = new QFile;
  log_file_->setFileName(horus_log_path + file_);
  log_file_->open(QIODevice::Append | QIODevice::Text);

  connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
}

HorusLogger::~HorusLogger()
{
  if (log_file_ != 0)
    log_file_->close();
}

void HorusLogger::run()
{
  QString text = "[" +
                 QDateTime::currentDateTime().toString("dd.MM.yyyy hh:mm:ss") +
                 "] " + message_ + "\n";
  QTextStream out(log_file_);
  out.setCodec("UTF-8");
  if (log_file_ != 0)
    out << text;
}
}
