#ifndef RVIZINTERFACE_H
#define RVIZINTERFACE_H

#include "/usr/include/OGRE/OgreCamera.h"
#include "rviz/view_manager.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include <OGRE/OgreRenderWindow.h>
#include "rviz/tool_manager.h"
#include "rviz/tool.h"



namespace HorusPatrol {

class horusRViz: public rviz::RenderPanel
{
    Q_OBJECT
public:
    horusRViz( QWidget* parent = 0 );
    ~horusRViz();

    rviz::Display *map_, *robot_, *threat_;
    rviz::Tool *mytool_;
    rviz::ToolManager *toolmanager_;
    rviz::VisualizationManager *manager_;

    void modeSelection(int mode);
};
}

#endif // RVIZINTERFACE_H
