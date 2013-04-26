#include <stdio.h>
#include "myLib.hpp"

// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>
#include <cula_lapack.h>
#include <cula_lapack_device.h>

__global__ void add(int *a,int *b,int *c){
    c[blockIdx.x] = a[blockIdx.x] + b[blockIdx.x];

}

__global__ void criaMatrizDerivadas(float *a,float *b){
    if(blockIdx.x%3==0){
        float h = sqrt(((a[blockIdx.x]-a[blockIdx.x+24])*(a[blockIdx.x]-a[blockIdx.x+24]))+((a[blockIdx.x+1]-a[blockIdx.x+25])*(a[blockIdx.x+1]-a[blockIdx.x+25]))+((a[blockIdx.x+2]-a[blockIdx.x+26])*(a[blockIdx.x+2]-a[blockIdx.x+26])));
        if(h==0){
            b[blockIdx.x] = 0;
            b[blockIdx.x+1] = 0;
            b[blockIdx.x+2] =0;
        }else{
            b[blockIdx.x] = (a[blockIdx.x+24]-a[blockIdx.x])/h;
            b[blockIdx.x+1] = (a[blockIdx.x+25]-a[blockIdx.x+1])/h;
            b[blockIdx.x+2] = (a[blockIdx.x+26]-a[blockIdx.x+2])/h;
        }
    }
}

void checkStatus(culaStatus status)
{
    char buf[256];

    if(!status)
        return;

    culaGetErrorInfoString(status, culaGetErrorInfo(), buf, sizeof(buf));
    printf("%s\n", buf);

    culaShutdown();
    exit(EXIT_FAILURE);
}
void culaFloatExample()
{
#ifdef NDEBUG
    int N = 8192;
#else
    int N = 1024;
#endif
    int NRHS = 1;
    int i;

    culaStatus status;

    culaFloat* A = NULL;
    culaFloat* B = NULL;
    culaFloat* X = NULL;
    culaInt* IPIV = NULL;

    culaFloat one = 1.0f;
    culaFloat thresh = 1e-6f;
    culaFloat diff;

    printf("-------------------\n");
    printf("       SGESV\n");
    printf("-------------------\n");

    printf("Allocating Matrices\n");
    A = (culaFloat*)malloc(N*N*sizeof(culaFloat));
    B = (culaFloat*)malloc(N*sizeof(culaFloat));
    X = (culaFloat*)malloc(N*sizeof(culaFloat));
    IPIV = (culaInt*)malloc(N*sizeof(culaInt));
    if(!A || !B || !IPIV)
        exit(EXIT_FAILURE);

    printf("Initializing CULA\n");
    status = culaInitialize();
    checkStatus(status);

    // Set A to the identity matrix
    memset(A, 0, N*N*sizeof(culaFloat));
    for(i = 0; i < N; ++i)
        A[i*N+i] = one;

    // Set B to a random matrix (see note at top)
    for(i = 0; i < N; ++i)
        B[i] = (culaFloat)rand();
    memcpy(X, B, N*sizeof(culaFloat));

    memset(IPIV, 0, N*sizeof(culaInt));

    printf("Calling culaSgesv\n");
    status = culaSgesv(N, NRHS, A, N, IPIV, X, N);
    checkStatus(status);

    printf("Verifying Result\n");
    for(i = 0; i < N; ++i)
    {
        diff = X[i] - B[i];
        if(diff < 0.0f)
            diff = -diff;
        if(diff > thresh)
            printf("Result check failed:  i=%d  X[i]=%f  B[i]=%f", i, X[i], B[i]);
    }

    printf("Shutting down CULA\n\n");
    culaShutdown();

    free(A);
    free(B);
    free(IPIV);
}
void random_ints(int* a, int N)
{
   int i;
   for (i = 0; i < N; ++i)
   a[i] = rand()% 100;
}


float* mainKekel(float *landmarks,int tamMarks, float *pointCloud,int tamCloud) {
float *cudaMarks, *cudaCloud, *cudaDerivadas;

        cudaMalloc((void**)&cudaMarks, sizeof(float)*tamMarks);
        cudaMalloc((void**)&cudaDerivadas, sizeof(float)*tamMarks);
        cudaMalloc((void**)&cudaCloud, sizeof(float)*tamCloud);


        cudaMemcpy(cudaMarks, landmarks , sizeof(float)*tamMarks,cudaMemcpyHostToDevice);
        cudaMemcpy(cudaCloud, pointCloud, sizeof(float)*tamCloud,cudaMemcpyHostToDevice);

        criaMatrizDerivadas<<<(tamMarks-24),1>>>(cudaMarks,cudaDerivadas);

        cudaMemcpy(landmarks,cudaDerivadas, sizeof(float)*tamMarks,cudaMemcpyDeviceToHost);

        printf("\nItens Copiados com sucesso!!\n");
  culaFloatExample();
return landmarks;
}
