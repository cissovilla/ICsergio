/*
 * scan.cpp
 *
 *  Created on: Apr 17, 2012
 *      Author: vsu
 */

#include <pcl17/console/parse.h>
#include <pcl17/common/transforms.h>

#include <pcl17/ModelCoefficients.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/point_types.h>
#include <pcl17/sample_consensus/method_types.h>
#include <pcl17/sample_consensus/model_types.h>
#include <pcl17/segmentation/sac_segmentation.h>
#include <pcl17/filters/extract_indices.h>

#include <pcl17/filters/voxel_grid.h>

int main(int argc, char** argv)
{

  if (argc < 5)
  {
    PCL17_INFO ("Usage %s -input_file /in_file -output_file /out_file [options]\n", argv[0]);
    PCL17_INFO (" * where options are:\n"
        "         -tilt <X>  : tilt. Default : 30\n"
        "");
    return -1;
  }

  int tilt = 30;
  std::string input_file;
  std::string output_file;

  pcl17::console::parse_argument(argc, argv, "-input_file", input_file);
  pcl17::console::parse_argument(argc, argv, "-output_file", output_file);
  pcl17::console::parse_argument(argc, argv, "-tilt", tilt);

  pcl17::PointCloud<pcl17::PointXYZRGB> cloud, cloud_transformed, cloud_aligned, cloud_filtered;

  pcl17::io::loadPCDFile(input_file, cloud);

  Eigen::Affine3f view_transform;
  view_transform.matrix() << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;

  Eigen::AngleAxis<float> rot(tilt * M_PI / 180, Eigen::Vector3f(0, 1, 0));

  view_transform.prerotate(rot);

  pcl17::transformPointCloud(cloud, cloud_transformed, view_transform);

  pcl17::ModelCoefficients::Ptr coefficients(new pcl17::ModelCoefficients);
  pcl17::PointIndices::Ptr inliers(new pcl17::PointIndices);

  pcl17::SACSegmentation<pcl17::PointXYZRGB> seg;

  seg.setOptimizeCoefficients(true);

  seg.setModelType(pcl17::SACMODEL_PLANE);
  seg.setMethodType(pcl17::SAC_RANSAC);
  seg.setDistanceThreshold(0.05);
  seg.setProbability(0.99);

  seg.setInputCloud(cloud_transformed.makeShared());
  seg.segment(*inliers, *coefficients);

  pcl17::ExtractIndices<pcl17::PointXYZRGB> extract;

  extract.setInputCloud(cloud_transformed.makeShared());
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(cloud_transformed);

  std::cout << "Z vector: " << coefficients->values[0] << " " << coefficients->values[1] << " "
      << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

  Eigen::Vector3f z_current(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  Eigen::Vector3f y(0, 1, 0);

  Eigen::Affine3f rotation;
  rotation = pcl17::getTransFromUnitVectorsZY(z_current, y);
  rotation.translate(Eigen::Vector3f(0, 0, coefficients->values[3]));

  pcl17::transformPointCloud(cloud_transformed, cloud_aligned, rotation);

  Eigen::Affine3f res = (rotation * view_transform);

  cloud_aligned.sensor_origin_ = res * Eigen::Vector4f(0, 0, 0, 1);
  cloud_aligned.sensor_orientation_ = res.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f(0, 0, 1))).rotation();

  seg.setInputCloud(cloud_aligned.makeShared());
  seg.segment(*inliers, *coefficients);

  std::cout << "Z vector: " << coefficients->values[0] << " " << coefficients->values[1] << " "
      << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

  pcl17::io::savePCDFile(output_file, cloud_aligned);

  return 0;
}

