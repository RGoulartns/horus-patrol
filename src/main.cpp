#include <unistd.h>
#include <QGuiApplication>
#include <QQmlApplicationEngine>

#include <rclcpp/rclcpp.hpp>
#include <include/horus_patrol/ros_node.hpp>

int main(int argc, char *argv[])
{
  // Start app
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QGuiApplication app(argc, argv);

  // Initiate QML engine:
  QQmlApplicationEngine engine;
  const QUrl url(QStringLiteral("qrc:/main.qml"));
  engine.load(url);

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

  return app.exec();
}
