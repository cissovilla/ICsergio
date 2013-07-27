#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};




class BoundingBox
{
public:
    PointT pontos[8];
    PointCloud::Ptr box;
    float Distancia;
    float menor_x,menor_y,menor_z,maior_x,maior_y,maior_z,Maior,xmedio, ymedio,zmedio;
    BoundingBox();
    PointCloud::Ptr Twisting(float grau, PointCloud::Ptr cloud_source);
    PointCloud::Ptr TwistingBox(float teta);
    PointCloud::Ptr Normaliza( PointCloud::Ptr cloud_source);
    PointCloud::Ptr DefineBoundingBox(PointCloud::Ptr cloud);
    void definePonto(float x, float y, float z, int indice);
    PointCloud::Ptr TransformaBox(float teta, char tipo);
    PointCloud::Ptr Transforma(float teta,PointCloud::Ptr cloud_source, char tipo);
    PointCloud::Ptr BendingBox(float teta);
    PointCloud::Ptr Bending(float grau, PointCloud::Ptr cloud_source);
    PointCloud::Ptr TaperingBox(float teta);
    PointCloud::Ptr Tapering(float grau, PointCloud::Ptr cloud_source);
    float dist_pontos(PointT p1,PointT p2);
};

#endif // BOUNDINGBOX_H
