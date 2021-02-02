#include <include/horus_patrol/user_queries.hpp>
#include <iostream>

UserQueries::UserQueries(QObject *parent) : QObject(parent)
{
  db_ = QSqlDatabase::addDatabase("QMYSQL");
  db_.setHostName("localhost");
  db_.setUserName("root");
  db_.setPassword("123456789");

  m_dbConnected = db_.open();
  if(m_dbConnected)
  {
    QSqlQuery query;
    query.exec("SELECT SCHEMA_NAME FROM INFORMATION_SCHEMA.SCHEMATA WHERE "
                   "SCHEMA_NAME = 'horusdb'");
    m_dbExist = query.next();
    if (!m_dbExist)
    {
      query.exec("CREATE DATABASE IF NOT EXISTS horusdb DEFAULT CHARACTER SET latin1;");
      query.exec("USE horusdb;");
      query.exec("CREATE TABLE IF NOT EXISTS user ( "
                    "username varchar(20) NOT NULL,"
                    "`password` varchar(20) NOT NULL,"
                    "first_name varchar(30) NOT NULL,"
                    "surname varchar(30) NOT NULL,"
                    "level_access ENUM('user','administrator') NOT NULL,"
                    "PRIMARY KEY (username));"
      );
      query.exec("INSERT INTO `horusdb`.`user` (`username`, `password`, `first_name`, `surname`, `level_access`) "
                    "VALUES ('rgns', '123', 'rafael', 'goulart', 'Administrator');");
     }

    query.exec("USE horusdb");
  }

}

UserQueries::~UserQueries()
{
  db_.close();
}

void UserQueries::login(QString username, QString password)
{
  QSqlQuery query;
  if (query.exec("SELECT * FROM `user` WHERE username='" +
                 username +
                 "' AND `password`='" +
                 password + "'") )
  {
    bool result = false;
    if(query.next())
      result = true;

    emit queryResult(result);
  }
}

bool UserQueries::dbConnected()
{
  return m_dbConnected;
}

bool UserQueries::dbExist()
{
  return m_dbExist;
}
