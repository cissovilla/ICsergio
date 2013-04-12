/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
  * \author Dejan Pangercic
  *
  * @b drawer_handles_detector detects furtniture doors and drawers and
  correponding handles
  */
// ROS core
#include <ros/ros.h>
// Messages
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
// PCL stuff
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"
#include "pcl/common/common.h"
#include <pcl/io/pcd_io.h>
#include "pcl/segmentation/extract_clusters.h"
#include <pcl/features/normal_3d.h>
#include <pcl/common/angles.h>
#include "pcl/common/common.h"
#include <pcl_ros/publisher.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

const tf::Vector3 wp_normal(1, 0, 0);
const double wp_offset = -1.45;

// Waits for a point cloud with pan-tilt close to (0,0) and deduces the position of ptu_base
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class DrawerHandlesDetector 
{
public:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  DrawerHandlesDetector (const ros::NodeHandle &nh) : nh_ (nh)
  {
    nh_.param("z_min_limit", z_min_limit_, 0.7);
    nh_.param("z_max_limit", z_max_limit_, 0.9);
    nh_.param("y_min_limit", y_min_limit_, -0.5);
    nh_.param("y_max_limit", y_max_limit_, 0.5);
    nh_.param("x_min_limit", x_min_limit_, 0.0);
    nh_.param("x_max_limit", x_max_limit_, 1.0);
    nh_.param("publish_largest_handle_pose", publish_largest_handle_pose_, false);
    
    nh_.param("k", k_, 30);
    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
    n3d_.setKSearch (k_);
    n3d_.setSearchMethod (normals_tree_);
    
    nh_.param("sac_distance", sac_distance_, 0.02);
    nh_.param("normal_distance_weight", normal_distance_weight_, 0.05);
    nh_.param("max_iter", max_iter_, 500);
    nh_.param("eps_angle", eps_angle_, 20.0);
    nh_.param("seg_prob", seg_prob_, 0.99);
    seg_.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg_.setMethodType (pcl::SAC_RANSAC);
    seg_.setDistanceThreshold (sac_distance_);
    seg_.setNormalDistanceWeight (normal_distance_weight_);
    seg_.setOptimizeCoefficients (true);
    btVector3 axis(0.0, 0.0, 1.0);
    seg_.setAxis (Eigen::Vector3f(fabs(axis.getX()), fabs(axis.getY()), fabs(axis.getZ())));
    seg_.setEpsAngle(pcl::deg2rad(eps_angle_));
    seg_.setMaxIterations (max_iter_);
    seg_.setProbability (seg_prob_);

    nh_.param("object_cluster_tolerance", object_cluster_tolerance_, 0.03);
    nh_.param("object_cluster_min_size", object_cluster_min_size_, 200);
    cluster_.setClusterTolerance (object_cluster_tolerance_);
    cluster_.setSpatialLocator(0);
    cluster_.setMinClusterSize (object_cluster_min_size_);
    //    cluster_.setMaxClusterSize (object_cluster_max_size_);
    clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
    clusters_tree_->setEpsilon (1);
    cluster_.setSearchMethod (clusters_tree_);

    nh_.param("nr_cluster", nr_cluster_, 1);

    nh_.param("cluster_min_height", cluster_min_height_, 0.03);
    nh_.param("cluster_max_height", cluster_max_height_, 0.1);

    nh_.param("handle_cluster_tolerance", handle_cluster_tolerance_, 0.02);
    nh_.param("handle_cluster_min_size", handle_cluster_min_size_,40);
    nh_.param("handle_cluster_max_size", handle_cluster_max_size_, 500);
    handle_cluster_.setClusterTolerance (handle_cluster_tolerance_);
    handle_cluster_.setSpatialLocator(0);
    handle_cluster_.setMinClusterSize (handle_cluster_min_size_);
    handle_cluster_.setMaxClusterSize (handle_cluster_max_size_);
    cluster_.setSearchMethod (clusters_tree_);
    
    seg_line_.setModelType (pcl::SACMODEL_LINE);
    seg_line_.setMethodType (pcl::SAC_RANSAC);
    seg_line_.setDistanceThreshold (0.05);
//    seg_line_.setNormalDistanceWeight (0.0);
    seg_line_.setOptimizeCoefficients (true);
    seg_line_.setMaxIterations (max_iter_);
    seg_line_.setProbability (seg_prob_);

    nh_.param("min_table_inliers", min_table_inliers_, 100);
    nh_.param("voxel_size", voxel_size_, 0.01);
    nh_.param("point_cloud_topic", point_cloud_topic_, std::string("/shoulder_cloud2"));
    nh_.param("output_handle_topic", output_handle_topic_,
    std::string("handle_projected_inliers/output"));
    nh_.param("handle_pose_topic", handle_pose_topic_, std::string("handle_pose"));
    cloud_pub_.advertise (nh_, "debug_cloud", 1);
    //cloud_extracted_pub_.advertise (nh_, "cloud_extracted", 1);
    cloud_handle_pub_.advertise (nh_, output_handle_topic_, 10);
    handle_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> (handle_pose_topic_, 1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual ~DrawerHandlesDetector () 
  {
  }

    
  void 
  init ()  // tolerance: how close to (0,0) is good enough?
  {
    ROS_INFO ("[DrawerHandlesDetector:] Listening for incoming data on topic %s", nh_.resolveName (point_cloud_topic_).c_str ());
    point_cloud_sub_ = nh_.subscribe (point_cloud_topic_, 1,  &DrawerHandlesDetector::ptuFinderCallback, this);
    //object_name_ = object_name;
  }
    
private:
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void 
  ptuFinderCallback (const sensor_msgs::PointCloud2ConstPtr &cloud_in)
    {
      ROS_INFO_STREAM ("[" << getName ().c_str () << "] Received cloud: cloud time " << cloud_in->header.stamp);
      PointCloud cloud_raw, cloud;
      // Downsample + filter the input dataser
      pcl::fromROSMsg (*cloud_in, cloud_raw);

      PointCloudPtr cloud_raw_ptr (new PointCloud(cloud_raw));
      PointCloudPtr cloud_x_ptr (new PointCloud());
      PointCloudPtr cloud_y_ptr (new PointCloud());
      PointCloudPtr cloud_z_ptr (new PointCloud());

      //Downsample
      vgrid_.setInputCloud (cloud_raw_ptr);
      vgrid_.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
      //Filter x
      vgrid_.setFilterFieldName ("x");
      vgrid_.setFilterLimits (x_min_limit_, x_max_limit_);
      vgrid_.filter (*cloud_x_ptr);
      //Filter y
      vgrid_.setInputCloud (cloud_x_ptr);
      vgrid_.setFilterFieldName ("y");
      vgrid_.setFilterLimits (y_min_limit_, y_max_limit_);
      vgrid_.filter (*cloud_y_ptr);
      //Filter z
      vgrid_.setInputCloud (cloud_y_ptr);
      vgrid_.setFilterFieldName ("z");
      vgrid_.setFilterLimits (z_min_limit_, z_max_limit_);
      vgrid_.filter (*cloud_z_ptr);
      //For Debug
      //cloud_pub_.publish(cloud_z_ptr);
      //return;

      //Estimate Point Normals
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
      n3d_.setInputCloud (cloud_z_ptr);
      n3d_.compute (*cloud_normals);
      
      //Segment the biggest furniture_face plane
      pcl::ModelCoefficients::Ptr table_coeff (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices ());
      seg_.setInputCloud (cloud_z_ptr);
      seg_.setInputNormals (cloud_normals);
      seg_.segment (*table_inliers, *table_coeff);
      ROS_INFO ("[%s] Table model: [%f, %f, %f, %f] with %d inliers.", getName ().c_str (), 
                table_coeff->values[0], table_coeff->values[1], table_coeff->values[2], table_coeff->values[3], 
                (int)table_inliers->indices.size ());

      if ((int)table_inliers->indices.size () <= min_table_inliers_)
      {
        ROS_ERROR ("table has to few inliers");
        return;
      }

      //Extract the biggest cluster correponding to above inliers
      std::vector<pcl::PointIndices> clusters;
      cluster_.setInputCloud (cloud_z_ptr);
      cluster_.setIndices(table_inliers);
      cluster_.extract (clusters);

      
      PointCloudPtr biggest_face (new PointCloud());
      if (int(clusters.size()) >= nr_cluster_)
      {
        for (int i = 0; i < nr_cluster_; i++)
        {
          pcl::copyPointCloud (*cloud_z_ptr, clusters[i], *biggest_face);
        }
      }
      else
      {
        ROS_ERROR("Only %ld clusters found with size > %d points", clusters.size(), object_cluster_min_size_);
        return;
      }
      ROS_INFO("Found biggest face with %ld points", biggest_face->points.size());
      //For Debug
      //cloud_pub_.publish(*biggest_face);
      //return;

      //Project Points into the Perfect plane
      PointCloudPtr cloud_projected (new PointCloud());
      proj_.setInputCloud (biggest_face);
      proj_.setModelCoefficients (table_coeff);
      proj_.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
      proj_.filter (*cloud_projected);
      //For Debug
      //cloud_pub_.publish(*cloud_projected);
      //return;

      PointCloudPtr cloud_hull (new PointCloud());
      // Create a Convex Hull representation of the projected inliers
      chull_.setInputCloud (cloud_projected);
      chull_.reconstruct (*cloud_hull);
      ROS_INFO ("Convex hull has: %d data points.", (int)cloud_hull->points.size ());
      if ((int)cloud_hull->points.size () == 0)
      {
        ROS_WARN("Convex hull has: %d data points. Returning.", (int)cloud_hull->points.size ());
        return;
      }
      //For Debug
      cloud_pub_.publish(*cloud_hull);
      pcl::PointXYZ hull_min;
      pcl::PointXYZ hull_max;
      pcl::PointXYZ hull_center;
      pcl::getMinMax3D (*cloud_hull, hull_min, hull_max);
      hull_center.x = (hull_max.x + hull_min.x)/2;
      hull_center.y = (hull_max.y + hull_min.y)/2;
      hull_center.z = (hull_max.z + hull_min.z)/2;
      //return;

      // Extract the handle clusters using a polygonal prism 
      pcl::PointIndices::Ptr handles_indices (new pcl::PointIndices ());
      ROS_INFO_STREAM("min, max height" << cluster_min_height_ << " " << cluster_max_height_);
      prism_.setHeightLimits (cluster_min_height_, cluster_max_height_);
      //prism_.setInputCloud (cloud_z_ptr);
      prism_.setInputCloud (cloud_raw_ptr);
      prism_.setInputPlanarHull (cloud_hull);
      prism_.segment (*handles_indices);
      ROS_INFO ("[%s] Number of handle point indices: %d.", getName ().c_str (), (int)handles_indices->indices.size ());

      //Cluster handles
      PointCloudPtr handles (new PointCloud());
      pcl::copyPointCloud (*cloud_raw_ptr, *handles_indices, *handles);
      //For Debug
      //cloud_pub_.publish(*handles);
      //return;

      std::vector<pcl::PointIndices> handle_clusters;
      handle_cluster_.setInputCloud (handles);
      handle_cluster_.extract (handle_clusters);
      ROS_INFO ("[%s] Found handle clusters: %d.", getName ().c_str (), (int)handle_clusters.size ());
      if ((int)handle_clusters.size () == 0)
        return;

      PointCloudPtr handle_final (new PointCloud());
      PointCloudPtr handle_final_final (new PointCloud());
      pcl::ModelCoefficients::Ptr line_coeff (new pcl::ModelCoefficients ());
      pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices ());

      uint handle_clusters_size;
      // if (publish_largest_handle_pose_)
      //   handle_clusters_size = 1;
      // else
      handle_clusters_size = handle_clusters.size();

      double dist = 1000.0;
      geometry_msgs::PoseStamped handle_pose_final;
      //fit lines, project points into perfect lines
      for (uint i = 0; i < handle_clusters_size; i++)
      {
        pcl::copyPointCloud (*handles, handle_clusters[i], *handle_final);
        // seg_line_.setInputCloud (handle_final);
        // seg_line_.segment (*line_inliers, *line_coeff);
        // ROS_INFO("line_inliers %ld", line_inliers->indices.size());
        // //Project Points into the Perfect plane
        // PointCloudPtr line_projected (new PointCloud());
        // proj_.setInputCloud (handle_final);
        // proj_.setModelCoefficients (line_coeff);
        // proj_.setModelType (pcl::SACMODEL_LINE);
        // proj_.filter (*line_projected);
        //For Debug
        
        //TODO: get centroid, get orientation and publish pose
        //stamped, change topic name
        pcl::PointXYZ point_center;
        geometry_msgs::PoseStamped handle_pose;
        std::string frame (cloud_in->header.frame_id);
	getHandlePose(handle_final, table_coeff, frame,
                       handle_pose);
	//	handle_pose_pub_.publish(handle_pose);
	double dist_tmp = sqrt((hull_center.x - handle_pose.pose.position.x)*(hull_center.x - handle_pose.pose.position.x) + 
	  (hull_center.y - handle_pose.pose.position.y)*(hull_center.y - handle_pose.pose.position.y));
	std::cerr << "dist: " << i << " " << dist_tmp << "cluster size: " << handle_final->points.size () << std::endl;
	if (dist_tmp < dist)
	  {
	    dist = dist_tmp;
	    handle_pose_final = handle_pose;
	    *handle_final_final = *handle_final;
	  }
      //   ROS_INFO("Handle pose published: x %f, y %f, z %f, ox %f, oy 
      // %f, oz %f, ow %f", handle_pose.pose.position.x,
      // handle_pose.pose.position.y, handle_pose.pose.position.z,
      // handle_pose.pose.orientation.x, handle_pose.pose.orientation.y,
      // handle_pose.pose.orientation.z, handle_pose.pose.orientation.w);
      }
      if (dist < 999.0)
	{
	  handle_pose_pub_.publish(handle_pose_final);
	  cloud_handle_pub_.publish(*handle_final_final);
	}
    }

  void getHandlePose(pcl::PointCloud<Point>::Ptr line_projected,
                     pcl::ModelCoefficients::Ptr table_coeff,
                     std::string & frame,
                     geometry_msgs::PoseStamped & pose)
  {
    //Calculate the centroid of the line
    pcl::PointXYZ point_min;
    pcl::PointXYZ point_max;
    pcl::getMinMax3D (*line_projected, point_min, point_max);
    // point_min = line_projected->points[line_projected->points.size() - 1];
    // point_max = line_projected->points[0];
    pose.pose.position.x = (point_max.x + point_min.x)/2;
    pose.pose.position.y = (point_max.y + point_min.y)/2;
    pose.pose.position.z = (point_max.z + point_min.z)/2;

    ROS_INFO("min  %f %f %f", point_min.x, point_min.y, point_min.z);
    ROS_INFO("max  %f %f %f", point_max.x, point_max.y, point_max.z);

    //Calculate orientation of the line
    //x = orientation of the normal of the plane
    //y = cross product of x and y
    //z = orientation of the handle line
    // btVector3 x_axis(table_coeff->values[0],  table_coeff->values[1],  table_coeff->values[2]);
    // btVector3 z_axis(point_max.x - point_min.x, point_max.y - point_min.y, point_max.z - point_min.z);
    // x_axis.normalize();
    // z_axis.normalize();
    // btVector3 y_axis = (z_axis.cross(x_axis)).normalize();
    // x_axis =  (y_axis.cross(z_axis)).normalize();
    // z_axis =  (x_axis.cross(y_axis)).normalize();
    // y_axis = (z_axis.cross(x_axis)).normalize();

    // ROS_DEBUG("x_AXIS %f %f %f len %f", x_axis.x(), x_axis.y(), x_axis.z(), x_axis.length());
    // ROS_DEBUG("y_AXIS %f %f %f len %f", y_axis.x(), y_axis.y(), y_axis.z(), y_axis.length());
    // ROS_DEBUG("z_AXIS %f %f %f len %f", z_axis.x(), z_axis.y(), z_axis.z(), z_axis.length());
  
    // btMatrix3x3 rot(x_axis.x(), y_axis.x(), z_axis.x(),
    // 	                 x_axis.y(), y_axis.y(), z_axis.y(),
    //                 x_axis.z(), y_axis.z(), z_axis.z());
    // btQuaternion rot_quat;
    // rot.getRotation(rot_quat);
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.frame_id = frame;
    pose.header.stamp = ros::Time::now();
    pose.header.seq = 0;
  }

  ros::NodeHandle nh_;
  double voxel_size_;

  std::string point_cloud_topic_, output_handle_topic_, handle_pose_topic_;
  double object_cluster_tolerance_, handle_cluster_tolerance_, cluster_min_height_, cluster_max_height_;
  int object_cluster_min_size_, object_cluster_max_size_, handle_cluster_min_size_, handle_cluster_max_size_;

  double sac_distance_, normal_distance_weight_, z_min_limit_, z_max_limit_;
  double y_min_limit_, y_max_limit_, x_min_limit_, x_max_limit_;
  double eps_angle_, seg_prob_;
  int k_, max_iter_, min_table_inliers_, nr_cluster_;
  //whether to publish the pose of the largest handle found or all of them
  bool publish_largest_handle_pose_;

  ros::Subscriber point_cloud_sub_;
  pcl_ros::Publisher<Point> cloud_pub_;
  pcl_ros::Publisher<Point> cloud_handle_pub_;
  ros::Publisher handle_pose_pub_;

  // PCL objects
  //pcl::PassThrough<Point> vgrid_;                   // Filtering + downsampling object
  pcl::VoxelGrid<Point> vgrid_;                   // Filtering + downsampling object
  pcl::NormalEstimation<Point, pcl::Normal> n3d_;   //Normal estimation
  // The resultant estimated point cloud normals for \a cloud_filtered_
  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_;
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg_;               // Planar segmentation object
  pcl::SACSegmentation<Point> seg_line_;               // Planar segmentation object
  pcl::ProjectInliers<Point> proj_;               // Inlier projection object
  pcl::ExtractIndices<Point> extract_;            // Extract (too) big tables
  pcl::ConvexHull<Point> chull_;  
  pcl::ExtractPolygonalPrismData<Point> prism_;
  pcl::PointCloud<Point> cloud_objects_;
  pcl::EuclideanClusterExtraction<Point> cluster_, handle_cluster_;
  KdTreePtr clusters_tree_, normals_tree_;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get a string representation of the name of this class. */
  std::string getName () const { return ("DrawerHandlesDetector"); }
};

/* ---[ */
int
main (int argc, char** argv)
{
  ros::init (argc, argv, "drawer_cluster_detector");
  ros::NodeHandle nh("~");
  DrawerHandlesDetector dhd (nh);
  dhd.init ();  // 5 degrees tolerance
  ros::spin ();
}
/* ]--- */
