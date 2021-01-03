#ifndef ROS_NODE_HPP
#define ROS_NODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>

class ROSNode : public rclcpp::Node, public QThread
{

public:
  ROSNode(char ns[]);
  virtual ~ROSNode();

  void run() override;

private:
  void updateTimerCB();

  float loopFreq_;

  std::string ns_;
  rclcpp::Node::SharedPtr node_;

  rclcpp::TimerBase::SharedPtr update_tm_;

};

#endif // ROS_NODE_HPP
