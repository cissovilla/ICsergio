
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>


int
main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final;
    pcl::visualization::CloudViewer viewer ("Cloud Viewer");

    pcl::io::loadPCDFile (argv [1], *cloud_1);
    //pcl::io::loadPLYFile (argv [2], *cloud_2);
    cloud_2 =cloud_1;
    for (size_t i = 0; i < cloud_1->points.size (); ++i)
    cloud_2->points[i].x = cloud_1->points[i].x+0.7f;

    icp.setInputCloud(cloud_1);
    icp.setInputTarget(cloud_2);
    icp.align(*Final);



   viewer.showCloud (Final);
   while (!viewer.wasStopped ())
     {
     }

  return (0);
}
