#include <cuda_runtime.h>
#include <cmath>

extern "C" __global__
void boundingBoxKernel(float *obj_x, float *obj_y, float *obj_z,
                       float *probabilities, int *class_ids,
                       float *distances, int *person_detected,
                       int num_boxes) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < num_boxes) {
        distances[i] = sqrtf(obj_x[i]*obj_x[i] + obj_y[i]*obj_y[i] + obj_z[i]*obj_z[i]);
        person_detected[i] = (class_ids[i] == 0 && probabilities[i] >= 0.5f && distances[i] < 0.5f) ? 1 : 0;
    }
}

extern "C" void runBoundingBoxKernel(float *d_obj_x, float *d_obj_y, float *d_obj_z,
                                     float *d_probabilities, int *d_class_ids,
                                     float *d_distances, int *d_person_detected,
                                     int num_boxes) {
    int threadsPerBlock = 32;
    int blocksPerGrid = (num_boxes + threadsPerBlock - 1) / threadsPerBlock;

    boundingBoxKernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_obj_x, d_obj_y, d_obj_z,
        d_probabilities, d_class_ids,
        d_distances, d_person_detected,
        num_boxes
    );

    cudaDeviceSynchronize();
}

