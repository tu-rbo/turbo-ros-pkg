#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"

#include "selected_points_topic.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <QVariant>

namespace rviz_plugin_selected_points_topic
{
SelectedPointsTopic::SelectedPointsTopic()
{
  updateTopic();
}

SelectedPointsTopic::~SelectedPointsTopic()
{
}

void SelectedPointsTopic::updateTopic()
{
  nh_.param("frame_id", tf_frame_, std::string("/base_link"));
  cloud_topic_ = "/selected_points";
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>( cloud_topic_.c_str(), 1 );
  ROS_INFO( "Publishing data on topic %s with frame_id %s.",
            nh_.resolveName (cloud_topic_).c_str (),
            tf_frame_.c_str() );
}

int SelectedPointsTopic::processMouseEvent( rviz::ViewportMouseEvent& event )
{
  int flags = rviz::SelectionTool::processMouseEvent( event );

  // determine current selection mode
  if( event.alt() )
  {
    selecting_ = false;
  }
  else
  {
    if( event.leftDown() )
    {
      selecting_ = true;
    }
  }

  if( selecting_ )
  {
    if( event.leftUp() )
    {
      rviz::SelectionManager* sel_manager = context_->getSelectionManager();
      rviz::M_Picked selection = sel_manager->getSelection();
      rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();
      int num_points = model->rowCount();
      if( selection.empty() || num_points <= 0 )
      {
        return flags;
      }

      sensor_msgs::PointCloud2 msg_pc;
      msg_pc.header.frame_id = context_->getFixedFrame().toStdString();
      msg_pc.height = 1;
      msg_pc.width = num_points;
      msg_pc.point_step = 3 * 4;
      msg_pc.row_step = num_points * msg_pc.point_step;
      msg_pc.is_dense = false;
      msg_pc.is_bigendian = false;

      msg_pc.data.resize( msg_pc.row_step );
      msg_pc.fields.resize( 3 );

      msg_pc.fields[0].name = "x";
      msg_pc.fields[0].offset = 0;
      msg_pc.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
      msg_pc.fields[0].count = 1;

      msg_pc.fields[1].name = "y";
      msg_pc.fields[1].offset = 4;
      msg_pc.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
      msg_pc.fields[1].count = 1;

      msg_pc.fields[2].name = "z";
      msg_pc.fields[2].offset = 8;
      msg_pc.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
      msg_pc.fields[2].count = 1;

      for( int i = 0; i < num_points; i++ )
      {
        QModelIndex child_index = model->index( i, 0 );
        rviz::Property* child = model->getProp( child_index );
        rviz::VectorProperty* subchild = (rviz::VectorProperty*) child->childAt( 0 );
        Ogre::Vector3 vec = subchild->getVector();

        uint8_t* ptr = &msg_pc.data[0] + i * msg_pc.point_step;
        *(float*)ptr = vec.x;
        ptr += 4;
        *(float*)ptr = vec.y;
        ptr += 4;
        *(float*)ptr = vec.z;
        ptr += 4;
      }

      msg_pc.header.stamp = ros::Time::now();
      pub_.publish( msg_pc );
    }
  }

  return flags;
}

} // end namespace rviz_plugin_selected_points_topic

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_plugin_selected_points_topic::SelectedPointsTopic, rviz::Tool )
