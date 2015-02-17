#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/view_manager.h"
#include "rviz/view_controller.h"
#include "OGRE/OgreCamera.h"


#include "selected_points_publisher/SelectedPointsPublisher.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <QVariant>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>

#include <pcl/filters/impl/box_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>

#include <visualization_msgs/Marker.h>

#include <pcl/filters/crop_box.h>
#include <pcl/common/angles.h>

namespace rviz_plugin_selected_points_publisher
{
SelectedPointsPublisher::SelectedPointsPublisher()
{
    updateTopic();
}

SelectedPointsPublisher::~SelectedPointsPublisher()
{
}

void SelectedPointsPublisher::updateTopic()
{
    nh_.param("frame_id", tf_frame_, std::string("/base_link"));
    rviz_cloud_topic_ = std::string("/rviz_selected_points");
    real_cloud_topic_ = std::string("/real_selected_points");
    subs_cloud_topic_ = std::string("/camera/depth_registered/points");
    bb_marker_topic_ = std::string("visualization_marker");

    rviz_selected_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( rviz_cloud_topic_.c_str(), 1 );
    real_selected_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( real_cloud_topic_.c_str(), 1 );
    bb_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(bb_marker_topic_.c_str(), 1);
    pc_subs_ =  nh_.subscribe(subs_cloud_topic_.c_str(),1,&SelectedPointsPublisher::PointCloudsCallback, this);

    ROS_INFO_STREAM_NAMED("SelectedPointsPublisher.updateTopic", "Publishing rviz selected points on topic " <<  nh_.resolveName (rviz_cloud_topic_) );//<< " with frame_id " << context_->getFixedFrame().toStdString() );
    ROS_INFO_STREAM_NAMED("SelectedPointsPublisher.updateTopic", "Publishing real selected points on topic " <<  nh_.resolveName (real_cloud_topic_) );//<< " with frame_id " << context_->getFixedFrame().toStdString() );
    ROS_INFO_STREAM_NAMED("SelectedPointsPublisher.updateTopic", "Publishing bounding box marker on topic " <<  nh_.resolveName (bb_marker_topic_) );//<< " with frame_id " << context_->getFixedFrame().toStdString() );

    current_pc_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
}

void SelectedPointsPublisher::PointCloudsCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
    ROS_INFO_STREAM_NAMED( "SelectedPointsPublisher::PointCloudsCallback", "Received PC");
    // Convert ROS PC message into a pcl point cloud
    pcl::fromROSMsg(*pc_msg, *this->current_pc_);
}

int SelectedPointsPublisher::processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel )
{
        if(event->type() == QKeyEvent::KeyPress)
        {
            if(event->key() == 'c' || event->key() == 'C')
            {
                ROS_INFO_STREAM_NAMED( "SelectedPointsPublisher::processKeyEvent", "Cleaning a previous selection");
                rviz::SelectionManager* sel_manager = context_->getSelectionManager();
                rviz::M_Picked selection = sel_manager->getSelection();
                sel_manager->removeSelection(selection);
                visualization_msgs::Marker marker;
                // Set the frame ID and timestamp.  See the TF tutorials for information on these.
                marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
                marker.header.stamp = ros::Time::now();
                marker.ns = "basic_shapes";
                marker.id = 0;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::DELETE;
                marker.lifetime = ros::Duration();

                bb_marker_pub_.publish(marker);
            }
            else if(event->key() == 'r' || event->key() == 'R')
            {
                ROS_INFO_STREAM_NAMED( "SelectedPointsPublisher.processKeyEvent", "Reusing the previously selected area to find a new bounding box and publish the points inside of it");
                this->_processSelectedAreaAndPublishPoints();
            }
        }
}

int SelectedPointsPublisher::processMouseEvent( rviz::ViewportMouseEvent& event )
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
            ROS_INFO_STREAM_NAMED( "SelectedPointsPublisher.processKeyEvent", "Using selected area to find a new bounding box and publish the points inside of it");
            this->_processSelectedAreaAndPublishPoints();
        }
    }
    return flags;
}

int SelectedPointsPublisher::_processSelectedAreaAndPublishPoints()
{
    rviz::SelectionManager* sel_manager = context_->getSelectionManager();
    rviz::M_Picked selection = sel_manager->getSelection();
    rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();
    int num_points = model->rowCount();
    ROS_INFO_STREAM_NAMED( "_processSelectedArea.processMouseEvent", "Number of points in the selected area: " << num_points);

    // Generate a ros point cloud message with the selected points in rviz
    sensor_msgs::PointCloud2 selected_points_ros;
    selected_points_ros.header.frame_id = context_->getFixedFrame().toStdString();
    selected_points_ros.height = 1;
    selected_points_ros.width = num_points;
    selected_points_ros.point_step = 3 * 4;
    selected_points_ros.row_step = num_points * selected_points_ros.point_step;
    selected_points_ros.is_dense = false;
    selected_points_ros.is_bigendian = false;

    selected_points_ros.data.resize( selected_points_ros.row_step );
    selected_points_ros.fields.resize( 3 );

    selected_points_ros.fields[0].name = "x";
    selected_points_ros.fields[0].offset = 0;
    selected_points_ros.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[0].count = 1;

    selected_points_ros.fields[1].name = "y";
    selected_points_ros.fields[1].offset = 4;
    selected_points_ros.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[1].count = 1;

    selected_points_ros.fields[2].name = "z";
    selected_points_ros.fields[2].offset = 8;
    selected_points_ros.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    selected_points_ros.fields[2].count = 1;

    for( int i = 0; i < num_points; i++ )
    {
        QModelIndex child_index = model->index( i, 0 );
        rviz::Property* child = model->getProp( child_index );
        rviz::VectorProperty* subchild = (rviz::VectorProperty*) child->childAt( 0 );
        Ogre::Vector3 vec = subchild->getVector();

        uint8_t* ptr = &selected_points_ros.data[0] + i * selected_points_ros.point_step;
        *(float*)ptr = vec.x;
        ptr += 4;
        *(float*)ptr = vec.y;
        ptr += 4;
        *(float*)ptr = vec.z;
        ptr += 4;
    }
    selected_points_ros.header.stamp = ros::Time::now();
    rviz_selected_pub_.publish( selected_points_ros );

    // Convert the ros point cloud message with the selected points into a pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr selected_points_pcl(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(selected_points_ros, *selected_points_pcl);

    // Generate an oriented bounding box around the selected points in RVIZ
    // Compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*selected_points_pcl, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*selected_points_pcl, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

    // Move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*selected_points_pcl, cPoints, p2w);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // Final transform and bounding box size
    const Eigen::Quaternionf qfinal(eigDx);
    const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();
    double bb_size_x = max_pt.x - min_pt.x;
    double bb_size_y = max_pt.y - min_pt.y;
    double bb_size_z = max_pt.z - min_pt.z;

    // NOTE: Use these two lines and change the following code (templates on PointXYZ instead of PointXYZRGB)
    // if your input cloud is not colored
    // Convert the point cloud from the callback into a xyz point cloud
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::copyPointCloud(*this->current_pc_, *cloud_xyz);

    // Vectors for the size of the croping box
    Eigen::Vector4f cb_min(-bb_size_x/2.0, -bb_size_y/2.0, -bb_size_z/2.0, 1.0);
    Eigen::Vector4f cb_max(bb_size_x/2.0, bb_size_y/2.0, bb_size_z/2.0, 1.0);

    // We apply the inverse of the transformation of the bounding box to the whole point cloud to boxcrop it (then we do not move the box)
    Eigen::Affine3f transform = Eigen::Translation3f(tfinal)*qfinal;
    Eigen::Affine3f transform_inverse = transform.inverse();

    pcl::CropBox<pcl::PointXYZRGB> crop_filter;
    crop_filter.setTransform(transform_inverse);
    crop_filter.setMax(cb_max);
    crop_filter.setMin(cb_min);
    crop_filter.setKeepOrganized(true);
    crop_filter.setInputCloud(this->current_pc_);

    pcl::PointIndices::Ptr inliers( new pcl::PointIndices() );
    crop_filter.filter(inliers->indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_segment_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices_filter;
    extract_indices_filter.setIndices(inliers);
    extract_indices_filter.setKeepOrganized(true);
    extract_indices_filter.setInputCloud(this->current_pc_);
    selected_segment_pc->header = this->current_pc_->header;
    extract_indices_filter.filter(*selected_segment_pc);

    ROS_INFO_STREAM_NAMED("_processSelectedArea.processMouseEvent", "Real number of points of the point cloud in the selected area: "<< inliers->indices.size());

    sensor_msgs::PointCloud2 selected_segment_ros;
    pcl::toROSMsg(*selected_segment_pc, selected_segment_ros);
    this->real_selected_pub_.publish(selected_segment_ros);


    // Publish the bounding box as a rectangular marker
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = tfinal.x();
    marker.pose.position.y = tfinal.y();
    marker.pose.position.z = tfinal.z();
    marker.pose.orientation.x = qfinal.x();
    marker.pose.orientation.y = qfinal.y();
    marker.pose.orientation.z = qfinal.z();
    marker.pose.orientation.w = qfinal.w();
    marker.scale.x = max_pt.x - min_pt.x;
    marker.scale.y = max_pt.y - min_pt.y;
    marker.scale.z = max_pt.z - min_pt.z;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration();

    bb_marker_pub_.publish(marker);
}

} // end namespace rviz_plugin_selected_points_topic

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_plugin_selected_points_publisher::SelectedPointsPublisher, rviz::Tool )
