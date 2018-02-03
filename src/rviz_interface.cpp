#include "../include/horus_patrol/rviz_interface.hpp"

namespace HorusPatrol
{

horusRViz::horusRViz(QWidget* parent) : RenderPanel(parent)
{
  manager_ = new rviz::VisualizationManager(this);
  this->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  manager_->startUpdate();
  manager_->getFrameManager()->update();
  // manager_->getViewManager()->getCurrent()->getCamera()->setPosition(-10,-7,10);
  // manager_->getViewManager()->getCurrent()->getCamera()->setNearClipDistance(
  // 5 );
  this->getCamera()->setNearClipDistance(5);
  manager_->getViewManager()->setCurrentViewControllerType("rviz/TopDownOrtho");

  map_ = manager_->createDisplay("rviz/Map", "rviz::MapDisplay", true);
  map_->subProp("Topic")->setValue("/map");
  robot_ = manager_->createDisplay("rviz/RobotModel", "rviz::RobotModel", true);
  robot_->setEnabled(false);
  threat_ =
      manager_->createDisplay("rviz/PointStamped", "rviz::PointStamped", true);
  threat_->subProp("Topic")->setValue("/clicked_point");
  threat_->subProp("Radius")->setValue("1");
  threat_->subProp("Color")->setValue("255; 0; 0");
  threat_->subProp("Alpha")->setValue("0.5");
  threat_->setEnabled(false);
}

horusRViz::~horusRViz()
{
/*  this->getRenderWindow()->_endUpdate();
  this->getRenderWindow()->setActive(false);
  this->getRenderWindow()->setDeactivateOnFocusChange(true);
  this->getRenderWindow()->setHidden(true);
  this->getRenderWindow()->detachDepthBuffer();
  this->getRenderWindow()->resetStatistics();
  this->getRenderWindow()->destroy();
  this->getRenderWindow()->removeAllListeners();
  this->getRenderWindow()->removeAllViewports();
  this->getRenderWindow()->~RenderTarget();


  manager_->stopUpdate();
  manager_->resetTime();
  manager_->unlockRender();
  manager_->removeAllDisplays();
  manager_->deleteLater();*/
  delete manager_;
}

void horusRViz::modeSelection(int mode)
{
  switch (mode)
  {
  case 0: // none
    robot_->setEnabled(false);
    threat_->setEnabled(false);
    break;
  case 1: // threat
    robot_->setEnabled(false);
    threat_->setEnabled(true);
    break;
  case 2: // robot
    robot_->setEnabled(true);
    threat_->setEnabled(false);
    break;
  default: // both
    robot_->setEnabled(true);
    threat_->setEnabled(true);
    break;
  }
}
}
