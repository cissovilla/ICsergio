#include <stdio.h>
#include "myLib.hpp"
// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>
#include <cula_lapack.h>
#include <cula_lapack_device.h>

#include "./src/stdafx.h"
#include "./src/ap.h"
#include "./src/statistics.h"
#include "./src/alglibmisc.h"
#include "./src/linalg.h"
#include "./src/specialfunctions.h"
#include "./src/alglibinternal.h"
#include <stdlib.h>
#include <math.h>
#include "./src/interpolation.h"
#include <string>

#define _USE_MATH_DEFINES
#define LANDS_NUMBER 8



__global__ void criaMatrizDerivadas(float *x,float *y,float *z,float *d){

        float h = sqrt( ((x[blockIdx.x]-x[blockIdx.x+LANDS_NUMBER])*(x[blockIdx.x]-x[blockIdx.x+LANDS_NUMBER]))
                       +((y[blockIdx.x]-y[blockIdx.x+LANDS_NUMBER])*(y[blockIdx.x]-y[blockIdx.x+LANDS_NUMBER]))
                       +((z[blockIdx.x]-z[blockIdx.x+LANDS_NUMBER])*(z[blockIdx.x]-z[blockIdx.x+LANDS_NUMBER])));
        if(h==0){
            d[blockIdx.x]     = 0;
            d[(blockIdx.x)+LANDS_NUMBER] = 0;
            d[(blockIdx.x)+LANDS_NUMBER*2] = 0;
        }else{
            d[blockIdx.x]     = (x[blockIdx.x+LANDS_NUMBER] - x[blockIdx.x]);
            d[(blockIdx.x)+LANDS_NUMBER] = (y[blockIdx.x+LANDS_NUMBER] - y[blockIdx.x]);
            d[(blockIdx.x)+LANDS_NUMBER*2] = (z[blockIdx.x+LANDS_NUMBER] - z[blockIdx.x]);
        }
}
__global__ void criaMatrizDistancias(float *x,float *y,float *z,float *d, float T){

    int index = threadIdx.x + blockIdx.x * LANDS_NUMBER;

     d[index] = sqrt(((x[blockIdx.x]-x[threadIdx.x])*(x[blockIdx.x]-x[threadIdx.x]))
                     +((y[blockIdx.x]-y[threadIdx.x])*(y[blockIdx.x]-y[threadIdx.x]))
                     +((z[blockIdx.x]-z[threadIdx.x])*(z[blockIdx.x]-z[threadIdx.x])));

     d[index] = (1/pow((4*M_PI*T),(3/2)))*exp(-((d[index]*d[index])/(4*T)));
}
__global__ void colocaPesos(float *pesos, float *cloud, float *landsX,float *landsY,float *landsZ, float T){
    int index = threadIdx.x + blockIdx.x * LANDS_NUMBER;
    float dist;
    __shared__ float *X,*Y,*Z;
    X=landsX;
    Y=landsY;
    Z=landsZ;
    for(int i =0; i<LANDS_NUMBER ;i++){
    dist = sqrt(  ((X[i]-cloud[index*3])*(X[i]-cloud[index*3]))
                       +((Y[i]-cloud[(index*3)+1])*(Y[i]-cloud[(index*3)+1]))
                       +((Z[i]-cloud[(index*3)+2])*(Z[i]-cloud[(index*3)+2])));

    dist = (1/pow((4*M_PI*T),4))*exp(-((dist*dist)/(4*T)));
    cloud[index*3]     += pesos[i]*dist;
    cloud[(index*3)+1] += pesos[i+LANDS_NUMBER]*dist;
    cloud[(index*3)+2] += pesos[i+LANDS_NUMBER*2]*dist;
    }



    __syncthreads();
}

void checkStatus(culaStatus status)
{
    char buf[256];

    if(!status)
        return;

    culaGetErrorInfoString(status, culaGetErrorInfo(), buf, sizeof(buf));
    printf("seu erro foi:%s\n", buf);

    culaShutdown();
    exit(EXIT_FAILURE);
}
void culaSolveProblem(culaFloat *Gaussianas, int N, culaFloat *Derivadas)
{
    culaInt* IPIV = NULL;

    cudaMalloc((void**)&IPIV,N*sizeof(culaInt));
    culaStatus status;
    status = culaInitialize();
    checkStatus(status);
    cudaMemset(IPIV, 0, N*sizeof(culaInt));
    status = culaDeviceSgesv(N, 3, Gaussianas, N, IPIV, Derivadas, N);
    checkStatus(status);
    cudaFree(IPIV);
    culaShutdown();


}

float* aplicaSpline(float *landMarks,int AMOSTRAS){
     float* splined = (float*)malloc((sizeof(float)*LANDS_NUMBER)*AMOSTRAS);

     using namespace alglib;
     real_1d_array Pontos [LANDS_NUMBER];
     std::string pontos[LANDS_NUMBER];
     std::string pontosX;
     int index=0;
     for(int i =0; i< LANDS_NUMBER; i++){
         pontos[i]="[";
         pontosX="[";
         for(int j=0;j<AMOSTRAS;j++){
                char spi[30];

                 snprintf(spi, 7, "%f", landMarks[(j*LANDS_NUMBER)+i]);
             pontos[i]+=spi;

             snprintf(spi, 7, "%f", ((j+1)/40.0f)*60);
             pontosX+=spi;
             if(j<(AMOSTRAS-1)){
                 pontos[i]+=",";
                  pontosX+=",";
             }

         }
         pontos[i]+="]";
         pontosX+="]";
         real_1d_array y = pontos[i].c_str();
         real_1d_array x = pontosX.c_str();
         real_1d_array d1;

         //
         // We calculate first derivatives: they must be equal to 2*x
         //
         spline1dgriddiffcubic(x, y, d1);
         for (int ig=0; ig<AMOSTRAS; ++ig){
              splined[index] = (float)d1.operator [](ig);
              index++;
         }
    }

     return splined;

}

float* mainKekel(float *Derivadas,float *landmarksX,float *landmarksY,float *landmarksZ,int tamMarks, float *pointCloud,int tamCloud,float T) {
culaFloat *cudaMarksX,
      *cudaMarksY,
      *cudaMarksZ,
      *cudaCloud,
      *cudaDerivadas,
      *cudaDistancias,
      *landmarks;

        cudaMalloc((void**)&cudaMarksX, sizeof(float)*tamMarks);
        cudaMalloc((void**)&cudaMarksY, sizeof(float)*tamMarks);
        cudaMalloc((void**)&cudaMarksZ, sizeof(float)*tamMarks);
        cudaMalloc((void**)&cudaDerivadas, sizeof(float)*LANDS_NUMBER*3);
        cudaMalloc((void**)&cudaDistancias, sizeof(float)*LANDS_NUMBER*LANDS_NUMBER);
        cudaMalloc((void**)&cudaCloud, sizeof(float)*tamCloud);


        cudaMemcpy(cudaMarksX, landmarksX , sizeof(float)*tamMarks,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaMarksY, landmarksY , sizeof(float)*tamMarks,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaMarksZ, landmarksZ , sizeof(float)*tamMarks,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaCloud, pointCloud, sizeof(float)*tamCloud,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaDerivadas, Derivadas, sizeof(float)*LANDS_NUMBER*3,cudaMemcpyHostToDevice);
        criaMatrizDistancias<<<LANDS_NUMBER,LANDS_NUMBER>>>(cudaMarksX,
                                      cudaMarksY,
                                      cudaMarksZ,
                                      cudaDistancias,T);
        culaSolveProblem(cudaDistancias,LANDS_NUMBER,cudaDerivadas);

        landmarks = (float*)malloc (sizeof(float)*LANDS_NUMBER*3);
        cudaMemcpy(landmarks,cudaDerivadas, sizeof(float)*LANDS_NUMBER*3,cudaMemcpyDeviceToHost);
        cudaFree(cudaCloud);
        cudaFree(cudaDerivadas);
        cudaFree(cudaDistancias);
        cudaFree(cudaMarksX);
        cudaFree(cudaMarksY);
        cudaFree(cudaMarksZ);

return landmarks;
}
