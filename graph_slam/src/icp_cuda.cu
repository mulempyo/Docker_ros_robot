#include <cuda_runtime.h>

__global__ void findNearestNeighbors(
    const float* src, const float* tgt, int N, int M, int* indices) {
    
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= N) return;

    float min_dist = 1e10;
    int best_j = 0;

    float sx = src[3 * i];
    float sy = src[3 * i + 1];
    float sz = src[3 * i + 2];

    for (int j = 0; j < M; ++j) {
        float dx = sx - tgt[3 * j];
        float dy = sy - tgt[3 * j + 1];
        float dz = sz - tgt[3 * j + 2];
        float dist = dx * dx + dy * dy + dz * dz;
        if (dist < min_dist) {
            min_dist = dist;
            best_j = j;
        }
    }

    indices[i] = best_j;
}

// CUDA kernel wrapper

extern "C" void launchNearestNeighborKernel(
    const float* d_src, const float* d_tgt, int N, int M, int* d_indices) {
    int threads = 256;
    int blocks = (N + threads - 1) / threads;
    findNearestNeighbors<<<blocks, threads>>>(d_src, d_tgt, N, M, d_indices);
}

