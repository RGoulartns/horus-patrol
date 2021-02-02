#ifndef USER_QUERIES_HPP
#define USER_QUERIES_HPP

#include <QObject>
#include <QSqlDatabase>
#include <QSqlQuery>
#include <QtQml>

class UserQueries : public QObject
{
  Q_OBJECT
  Q_PROPERTY(bool dbConnected READ dbConnected)
  Q_PROPERTY(bool dbExist READ dbExist)
  QML_ELEMENT

public:
  explicit UserQueries(QObject *parent = nullptr);
  virtual ~UserQueries();

  bool dbConnected();
  bool dbExist();

signals:
  void queryResult(bool result);

public slots:
  void login(QString username, QString password);

private:
  QSqlDatabase db_;
  bool m_dbConnected;
  bool m_dbExist;

};

#endif // USER_QUERIES_HPP
