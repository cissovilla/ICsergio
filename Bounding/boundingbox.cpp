#include "boundingbox.h"






    BoundingBox::BoundingBox(){}

    void BoundingBox::definePonto(float x, float y, float z, int indice){
        pontos[indice].x=x;
        pontos[indice].y=y;
        pontos[indice].z=z;
    }

    PointCloud::Ptr BoundingBox::DefineBoundingBox(PointCloud::Ptr cloud){

            //definindo margens
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

             menor_x = sqrt(menor_x*menor_x);
             menor_y = sqrt(menor_y*menor_y);
             menor_z = sqrt(menor_z*menor_z);
             maior_x = sqrt(maior_x*maior_x);
             maior_y = sqrt(maior_y*maior_y);
             maior_z = sqrt(maior_z*maior_z);
             Maior=maior_x;
             if(Maior<maior_y)Maior=maior_y;
             if(Maior<maior_z)Maior=maior_z;

             //definindo os pontos da BoundingBox e normalizando
             definePonto(0,0,0,0);
             definePonto(0,0,(maior_z+0.2f+menor_z)/(Maior+0.2f),1);
             definePonto((maior_x+0.2f+menor_x)/(Maior+0.2f),0,(maior_z+0.2f+menor_z)/(Maior+0.2f),2);
             definePonto((maior_x+0.2f+menor_x)/(Maior+0.2f),0,0,3);

             definePonto(0,(maior_y+0.2f+menor_y)/(Maior+0.2f),0,4);
             definePonto(0,(maior_y+0.2f+menor_y)/(Maior+0.2f),(maior_z+0.2f+menor_z)/(Maior+0.2f),5);
             definePonto((maior_x+0.2f+menor_x)/(Maior+0.2f),(maior_y+0.2f+menor_y)/(Maior+0.2f),(maior_z+0.2f+menor_z)/(Maior+0.2f),6);
             definePonto((maior_x+0.2f+menor_x)/(Maior+0.2f),(maior_y+0.2f+menor_y)/(Maior+0.2f),0,7);

                xmedio=((maior_x+0.2f+menor_x)/(Maior+0.2f))/2;
                ymedio=((maior_y+0.2f+menor_y)/(Maior+0.2f))/2;
                zmedio=((maior_z+0.2f+menor_z)/(Maior+0.2f))/2;

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

    PointCloud::Ptr BoundingBox::Normaliza( PointCloud::Ptr cloud_source){
        PointCloud::Ptr cloud =cloud_source;
        for(size_t i=0; i < cloud->points.size (); i++){

            cloud->points[i].x=(cloud->points[i].x+menor_x+0.2f)/(Maior+0.2f);
            cloud->points[i].y=(cloud->points[i].y+menor_y+0.2f)/(Maior+0.2f);
            cloud->points[i].z=(cloud->points[i].z+menor_z+0.2f)/(Maior+0.2f);
        }
        return cloud;
    }


    PointCloud::Ptr BoundingBox::TransformaBox(float teta, char tipo){
        switch (tipo){
        case 'B':
            return this->BendingBox(-teta);
        case 'T':
            return this->TaperingBox(teta*40);
        case 'W':
            return this->TwistingBox(teta);
        }
    }

    PointCloud::Ptr BoundingBox::Transforma(float teta,PointCloud::Ptr cloud_source, char tipo){
        switch (tipo){
        case 'B':
            return this->Bending(-teta, cloud_source);
        case 'T':
            return this->Tapering(teta*40, cloud_source);
        case 'W':
            return this->Twisting(teta, cloud_source);
        }
    }

    PointCloud::Ptr BoundingBox::TwistingBox(float teta){
        float X,Z;
        teta=teta*M_PI/180;
        for(int i=4; i<8; i++){
            X=((box->points[i].z-zmedio) * sin(teta)) + ((box->points[i].x-xmedio) * cos(teta));
            Z=((box->points[i].z-zmedio) * cos(teta)) - ((box->points[i].x-xmedio) * sin(teta));
            box->points[i].x=X+xmedio;
            box->points[i].z=Z+zmedio;
        }
        return box;
    }


    PointCloud::Ptr BoundingBox::Twisting(float grau, PointCloud::Ptr cloud_source){
        float X,Z,D1,teta;
        PointCloud::Ptr cloud =cloud_source;
        for(size_t i=0; i < cloud->points.size (); i++){

            D1=dist_pontos(cloud->points[i],box->points[0]);
            Distancia=dist_pontos(this->box->points[0],this->box->points[4]);
            D1=(float)D1/Distancia;
            teta=grau*M_PI/180;
            teta=(float)teta*D1;
            X=((cloud->points[i].z-zmedio) * sin(teta)) + ((cloud->points[i].x-xmedio) * cos(teta));
            Z=((cloud->points[i].z-zmedio) * cos(teta)) - ((cloud->points[i].x-xmedio) * sin(teta));
            cloud->points[i].x=X+xmedio;
            cloud->points[i].z=Z+zmedio;
        }
        return cloud;
    }
    PointCloud::Ptr BoundingBox::BendingBox(float teta){
        float X,Z;
        teta=teta*M_PI/180;
        for(int i=4; i<8; i++){
            X=((box->points[i].y-ymedio) * sin(teta)) + ((box->points[i].x+xmedio) * cos(teta));
            Z=((box->points[i].y-ymedio) * cos(teta)) - ((box->points[i].x+xmedio) * sin(teta));
            box->points[i].x=X-xmedio;
            box->points[i].y=Z+ymedio;
        }
        return box;
    }
    PointCloud::Ptr BoundingBox::Bending(float grau, PointCloud::Ptr cloud_source){
        float X,Z,D1,teta;
        PointCloud::Ptr cloud =cloud_source;
        for(size_t i=0; i < cloud->points.size (); i++){

            D1=dist_pontos(cloud->points[i],box->points[7]);
            Distancia=dist_pontos(this->box->points[0],this->box->points[7]);
            D1=(float)D1/Distancia;
            teta=grau*M_PI/180;
            teta=(float)teta*(1-D1);
            X=((cloud->points[i].y-ymedio) * sin(teta)) + ((cloud->points[i].x+xmedio) * cos(teta));
            Z=((cloud->points[i].y-ymedio) * cos(teta)) - ((cloud->points[i].x+xmedio) * sin(teta));
            cloud->points[i].x=X-xmedio;
            cloud->points[i].y=Z+ymedio;
        }
        return cloud;
    }
    PointCloud::Ptr BoundingBox::TaperingBox(float teta){
        float X,Z;
        for(int i=4; i<8; i++){
            X=(box->points[i].x-xmedio)*teta;
            Z=(box->points[i].z-zmedio)*teta;
            box->points[i].x=X+xmedio;
            box->points[i].z=Z+zmedio;
        }
        return box;
    }
    PointCloud::Ptr BoundingBox::Tapering(float grau, PointCloud::Ptr cloud_source){
        float X,Z,D1,teta;
        PointCloud::Ptr cloud =cloud_source;
        for(size_t i=0; i < cloud->points.size (); i++){

            D1=dist_pontos(cloud->points[i],box->points[0]);
            Distancia=dist_pontos(this->box->points[0],this->box->points[4]);
            D1=(float)D1/Distancia;
            teta=grau-1;
            teta=(float)teta*D1;
            X=((cloud->points[i].x-xmedio) * (teta+1));
            Z=((cloud->points[i].z-zmedio) * (teta+1));
            cloud->points[i].x=X+xmedio;
            cloud->points[i].z=Z+zmedio;
        }
        return cloud;
    }

    float BoundingBox::dist_pontos(PointT p1,PointT p2){
        return sqrt((p1.y-p2.y)*(p1.y-p2.y));
    }




