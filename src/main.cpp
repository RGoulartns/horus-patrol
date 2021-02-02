#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <horus_patrol/ros_node.hpp>
#include <horus_patrol/user_queries.hpp>

int main(int argc, char *argv[])
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

  QGuiApplication app(argc, argv);
  QQmlApplicationEngine engine;
  const QUrl url(QStringLiteral("qrc:/main.qml"));

  qmlRegisterType<UserQueries>("Horus.Queries.User",1,0,"UserQueries");

  // Initialise ROS2 node:
  char ns[255];
  gethostname(ns, 255);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ROSNode>(ns);
  node->start();

  QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                   &app, [url](QObject *obj, const QUrl &objUrl) {
    if (!obj && url == objUrl)
      QCoreApplication::exit(-1);
  }, Qt::QueuedConnection);
  engine.load(url);

  return app.exec();
}
