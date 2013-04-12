#include <stdio.h>

// For the CUDA runtime routines (prefixed with "cuda_")
#include <cuda_runtime.h>

__global__ void mykernel(void) {
}

int main(void) {
mykernel<<<1,1>>>();
printf("Hello World!\n");
return
0;
}
