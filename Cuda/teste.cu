#include <stdio.h>

// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>

__global__ void mykernel(void) {
}

__global__ void add(int *a,int *b,int *c){
    c[blockIdx.x] = a[blockIdx.x] + b[blockIdx.x];

}
__global__ void add2(int *a,int *b,int *c){
    c[threadIdx] = a[threadIdx] + b[threadIdx];

}
__global__ void add3(int *a,int *b,int *c){
    int index =threadIdx.x+blockIdx.x*blockDim.x;
    c[index]=a[index]+b[index];

}


void random_ints(int* a, int N)
{
   int i;
   for (i = 0; i < N; ++i)
    a[i] = rand()% 100;
}

#define N 12
int main(void) {
        int *a, *b, *c;
        // host copies of a, b, c
        int *d_a, *d_b, *d_c;
        // device copies of a, b, c
        int size = N* sizeof(int);
        // Allocate space for device copies of a, b, c
        cudaMalloc((void**)&d_a, size);
        cudaMalloc((void**)&d_b, size);
        cudaMalloc((void**)&d_c, size);
        // Setup input values
        a = (int*)malloc (size);random_ints(a, N);
        b = (int*)malloc (size);random_ints(b, N);
        c = (int*)malloc (size);

                cudaMemcpy(d_a, a, size,cudaMemcpyHostToDevice);
                cudaMemcpy(d_b, b, size,cudaMemcpyHostToDevice);
                // Launch add() kernel on GPU
                add<<<N,1>>>( d_a,d_b,d_c);
                // Copy result back to host
                cudaMemcpy(c,d_c, size,cudaMemcpyDeviceToHost);
                for (int i =0 ;i <N;i++ ){
                    printf("Hello World! A: %d\n",a[i]);
                    printf("Hello World! B: %d\n",b[i]);
                    printf("Hello World! C: %d\n",c[i]);
                }
                //Cleanup
                free(a); free(b); free(c);
                cudaFree(d_a);
                cudaFree(d_b);
                cudaFree(d_c);
printf("Hello World! C: %d\n",c);
return 0;
}
