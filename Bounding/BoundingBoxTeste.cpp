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
#include "myLib.hpp"
#include "boundingbox.h"

#define AMOSTRAS 40


// This is a tutorial so we can afford having global variables
//our visualizer
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2;

void showCloudsLeft(const PointCloud::Ptr cloud_target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_source);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, rgb, "vp1_source", vp_1);
  p-> spin();
}

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

float distancia(PointT p1,PointT p2){
    return sqrt(((p1.x-p2.x)*(p1.x-p2.x))+((p1.y-p2.y)*(p1.y-p2.y))+((p1.z-p2.z)*(p1.z-p2.z)));
}

float derivada(PointT x,PointT y){

    float fxhfx = x.z - y.z;
    float h = x.x -y.x;

    return fxhfx/h;
}


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

int main (int argc, char** argv)
{

    // Load data
    std::vector<PCD, Eigen::aligned_allocator<PCD> > data;
    loadData (argc, argv, data);
    // Check user input
    if (data.empty ())
    {
      PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
      PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
      return (-1);
    }
    // Create BoundignBox
    BoundingBox::BoundingBox box;
    // Create a PCLVisualizer object
    p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);


    //Definindo nuvens
      PointCloud::Ptr target , source;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr target2 (new pcl::PointCloud<pcl::PointXYZRGB>);
      source = data[0].cloud;
      target = box.DefineBoundingBox(source);

      int tamMarks=0;
      float *landMarksX,*landMarksY,*landMarksZ,*landMarks,*splinedX,*splinedY,*splinedZ,*Derivadas,
              *landMarksX2,*landMarksY2,*landMarksZ2;

      landMarksX = (float*)malloc((sizeof(float)*box.box->points.size())*AMOSTRAS);
      landMarksY = (float*)malloc((sizeof(float)*box.box->points.size())*AMOSTRAS);
      landMarksZ = (float*)malloc((sizeof(float)*box.box->points.size())*AMOSTRAS);
      Derivadas = (float*)malloc((sizeof(float)*box.box->points.size())*3);
      splinedX = (float*)malloc((sizeof(float)*box.box->points.size())*AMOSTRAS);
      splinedY = (float*)malloc((sizeof(float)*box.box->points.size())*AMOSTRAS);
      splinedZ = (float*)malloc((sizeof(float)*box.box->points.size())*AMOSTRAS);
      landMarksX2 = (float*)malloc((sizeof(float)*box.box->points.size())*2);
      landMarksY2 = (float*)malloc((sizeof(float)*box.box->points.size())*2);
      landMarksZ2 = (float*)malloc((sizeof(float)*box.box->points.size())*2);


    //Definindo Splines
    tamMarks=0;
    for(int init=0; init<AMOSTRAS;init++){

        for(size_t i=0; i < box.box->points.size (); i++){
            landMarksX[tamMarks]=box.box->points[i].x;
            landMarksY[tamMarks]=box.box->points[i].y;
            landMarksZ[tamMarks]=box.box->points[i].z;
            tamMarks++;
        }
        box.TransformaBox((float)((atof(argv[3]))/AMOSTRAS),argv[2][0]);
    }
    splinedX = aplicaSpline(landMarksX,AMOSTRAS);
    splinedY = aplicaSpline(landMarksY,AMOSTRAS);
    splinedZ = aplicaSpline(landMarksZ,AMOSTRAS);


   //reiniciando nuvens para comparação
    source = data[0].cloud;
    target = box.DefineBoundingBox(source);
    source = box.Normaliza(source);

    float* pointCloud;
    int TamCloud=0;
    pointCloud = (float*)malloc (sizeof(float)*source->points.size ()*3);
    for(size_t i=0; i < source->points.size (); i++){
        pointCloud[TamCloud]=source->points[i].x;
        TamCloud++;
        pointCloud[TamCloud]=source->points[i].y;
        TamCloud++;
        pointCloud[TamCloud]=source->points[i].z;
        TamCloud++;
    }

    target2->width  = TamCloud/3;
    target2->height = 1;
    target2->points.resize (target2->width * target2->height);

    for (size_t j = 0; j < target2->points.size (); ++j)
     {

        target2->points[j].x = source->points[j].x;
        target2->points[j].y = source->points[j].y;
        target2->points[j].z = source->points[j].z;


     }


for(int init=0; init<AMOSTRAS;init++){


        for(int i= 0; i<8;i++){
            Derivadas[i]=splinedX[((i)*AMOSTRAS)+init];
            Derivadas[i+8]=splinedY[((i)*AMOSTRAS)+init];
            Derivadas[i+16]=splinedZ[((i)*AMOSTRAS)+init];
        }
        tamMarks=0;

          for(size_t i=0; i < box.box->points.size (); i++){
              landMarksX2[tamMarks]=box.box->points[i].x;
              landMarksY2[tamMarks]=box.box->points[i].y;
              landMarksZ2[tamMarks]=box.box->points[i].z;
              tamMarks++;
          }
          box.TransformaBox((float)((atof(argv[3]))/AMOSTRAS),argv[2][0]);
          landMarks = mainKekel(Derivadas,landMarksX2,landMarksY2,landMarksZ2,tamMarks,pointCloud,TamCloud,atof(argv[4]));

          box.Transforma((float)((atof(argv[3]))/AMOSTRAS),source, argv[2][0]);
         for (size_t j = 0; j < target2->points.size (); ++j)
         {
            for(int i =0; i<8 ;i++){
            float T =atof(argv[4]);
            float dist = sqrt(((box.box->points[i].x-target2->points[j].x)*(box.box->points[i].x-target2->points[j].x))
                             +((box.box->points[i].y-target2->points[j].y)*(box.box->points[i].y-target2->points[j].y))
                             +((box.box->points[i].z-target2->points[j].z)*(box.box->points[i].z-target2->points[j].z)));


            dist=(1/pow((4*M_PI*T),(3/2)))*exp(-((dist*dist)/(4*T)));


            target2->points[j].x += (landMarks[i]*dist);
            target2->points[j].y += (landMarks[i+8]*dist);
            target2->points[j].z += (landMarks[i+16]*dist);

            float distClouds = ((source->points[j].x-target2->points[j].x));
            if(strcmp(argv[5],"y") == 0 )distClouds = ((source->points[j].y-target2->points[j].y));
            if(strcmp(argv[5],"z")== 0 )distClouds = ((source->points[j].z-target2->points[j].z));

            distClouds=(distClouds/0.1f);
            distClouds=(distClouds*255);
            if(distClouds>255)distClouds=255;
            if(distClouds<-255)distClouds=-255;
            int r,g,b;
            if(distClouds==0){
                r=0;g=255;b=0;
            }
            if(distClouds<0){
              r=(-1*distClouds);g=(255+distClouds);b=(0);
            }
            if(distClouds>0){
                r=(0); g=(255-distClouds); b=(distClouds);
            }
            uint8_t r1(r), g1(g), b1(b);
            uint32_t rgb = (static_cast<uint32_t>(r1) << 16 |
                            static_cast<uint32_t>(g1) << 8  |
                            static_cast<uint32_t>(b1));
            target2->points[j].rgb = *reinterpret_cast<float*>(&rgb);

           }

         }
            showCloudsLeft(target,target2);
            showCloudsRight(source, box.box);





    }
free(Derivadas);
free(pointCloud);
free(landMarks);
free(landMarksX);
free(landMarksY);
free(landMarksZ);

}


