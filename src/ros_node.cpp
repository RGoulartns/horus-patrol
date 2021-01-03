#include <include/horus_patrol/ros_node.hpp>

ROSNode::ROSNode(char ns[]) :
  Node("horus_patrol_node", ns),
  loopFreq_(1)
{
  //ROS2 practice: clear stdout buffer to ensure sync of prints
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  ns_ = ns;

  update_tm_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / loopFreq_)),
        std::bind(&ROSNode::updateTimerCB, this));

}


ROSNode::~ROSNode()
{
  rclcpp::shutdown();
  while(rclcpp::ok()){usleep(100);}
  this->quit();
  this->wait();
}

void ROSNode::run()
{
  rclcpp::spin(this->get_node_base_interface());
  rclcpp::shutdown();
}


void ROSNode::updateTimerCB()
{
}
