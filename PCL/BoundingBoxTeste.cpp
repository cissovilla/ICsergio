/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id$
 *
 */

/* \author Radu Bogdan Rusu
 * adaptation Raphael Favier*/

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
#include <math.h>

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

// This is a tutorial so we can afford having global variables
    //our visualizer
    pcl::visualization::PCLVisualizer *p;
    //its left and right viewports
    int vp_1, vp_2;

//convenient structure to handle our pointcloud



class BoundingBox
{
    public:PointT pontos[8];
    public:PointCloud::Ptr box ;
    public:float Distancia;

    void definePonto(float x, float y, float z, int indice){
        pontos[indice].x=x;
        pontos[indice].y=y;
        pontos[indice].z=z;
    }

    PointCloud::Ptr DefineBoundingBox(PointCloud::Ptr cloud){

        //definindo margens
            float menor_x,menor_y,menor_z,maior_x,maior_y,maior_z;
            menor_x=maior_x=cloud->points[1].x;
            menor_y=maior_y=cloud->points[1].y;
            menor_z=maior_z=cloud->points[1].z;
        //

            //procurando máximos e mínimos
             for (size_t i = 0; i < cloud->points.size (); ++i){
                 if(cloud->points[i].x<menor_x) menor_x=cloud->points[i].x;
                 if(cloud->points[i].y<menor_y) menor_y=cloud->points[i].y;
                 if(cloud->points[i].z<menor_z) menor_z=cloud->points[i].z;
                 if(cloud->points[i].x>maior_x) maior_x=cloud->points[i].x;
                 if(cloud->points[i].y>maior_y) maior_y=cloud->points[i].y;
                 if(cloud->points[i].z>maior_z) maior_z=cloud->points[i].z;
             }

        //definindo os pontos da BoundingBox

             definePonto(menor_x-0.2f,menor_y-0.2f,menor_z-0.2f,0);
             definePonto(menor_x-0.2f,menor_y-0.2f,maior_z+0.2f,1);
             definePonto(maior_x+0.2f,menor_y-0.2f,maior_z+0.2f,2);
             definePonto(maior_x+0.2f,menor_y-0.2f,menor_z-0.2f,3);

             definePonto(menor_x-0.2f,maior_y+0.2f,menor_z-0.2f,4);
             definePonto(menor_x-0.2f,maior_y+0.2f,maior_z+0.2f,5);
             definePonto(maior_x+0.2f,maior_y+0.2f,maior_z+0.2f,6);
             definePonto(maior_x+0.2f,maior_y+0.2f,menor_z-0.2f,7);

            //retornando
             PointCloud::Ptr box (new PointCloud);
             box->width    = 8;
             box->height   = 1;
             box->points.resize(8);
            for (int i = 0; i < 8; i++){
                box->points[i].x = pontos[i].x;
                box->points[i].y = pontos[i].y;
                box->points[i].z = pontos[i].z;
           }
            this->box=box;
            return this->box;

    }



    PointCloud::Ptr TwistingBox(float teta){
        float X,Z;
        teta=teta*M_PI/180;
        for(int i=4; i<8; i++){
            X=(box->points[i].z * sin(teta)) + (box->points[i].x * cos(teta));
            Z=(box->points[i].z * cos(teta)) - (box->points[i].x * sin(teta));
            box->points[i].x=X;
            box->points[i].z=Z;
        }
        return box;
    }

    PointCloud::Ptr Twisting(float grau, PointCloud::Ptr cloud_source){
        float X,Z,D1,teta;
        PointCloud::Ptr cloud =cloud_source;

        for(size_t i=0; i < cloud->points.size (); i++){

            D1=dist_pontos(cloud->points[i],box->points[0]);
            Distancia=dist_pontos(this->box->points[0],this->box->points[4]);
            D1=(float)D1/Distancia;
            teta=grau*M_PI/180;
            teta=(float)teta*D1;
            X=(cloud->points[i].z * sin(teta)) + (cloud->points[i].x * cos(teta));
            Z=(cloud->points[i].z * cos(teta)) - (cloud->points[i].x * sin(teta));
            cloud->points[i].x=X;
            cloud->points[i].z=Z;
        }
        return cloud;
    }

    float dist_pontos(PointT p1,PointT p2){
        return sqrt((p1.y-p2.y)*(p1.y-p2.y));
    }



};



////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);

  p->addPointCloud (cloud_target, tgt_h, "target", vp_2);
  p->addPointCloud (cloud_source, src_h, "source", vp_2);

  p->spin();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".ply");
  // Suppose the first argument is the actual test model


    std::string fname = std::string (argv[1]);
    // Needs to be at least 5: .plot


    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      PCD m;
      m.f_name = argv[1];
      pcl::io::loadPLYFile (argv[1], *m.cloud);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
      models.push_back (m);
    }else{
        // Load the cloud and saves it into the global list of models
        PCD m;
        m.f_name = argv[1];
        pcl::io::loadPCDFile (argv[1], *m.cloud);
        //remove NAN points from the cloud
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
        models.push_back (m);
    }
  }




////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */


int main (int argc, char** argv)
{
  // Load data
  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
  loadData (argc, argv, data);
  BoundingBox box;
  // Check user input
  if (data.empty ())
  {
    PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
    PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
    return (-1);
  }
  PCL_INFO ("Loaded %d datasets.", (int)data.size ());

  // Create a PCLVisualizer object
  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");

  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);

  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);


  PointCloud::Ptr target , source;


  source = data[0].cloud;

  target = box.DefineBoundingBox(source);


    // Add visualization data


    for(int i=0; i<=20;i++){
            showCloudsLeft(source, box.TwistingBox((atof(argv[2]))/20));
            showCloudsRight(box.Twisting((atof(argv[2]))/20,source), box.TwistingBox((atof(argv[2]))/20));
    }



}
/* ]--- */

