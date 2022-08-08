#include <cuda_runtime.h>
#include <Eigen/Dense>
#include <device_launch_parameters.h>
__global__ void cu_dot(Eigen::Vector3d *v1, Eigen::Vector3d *v2, double *out, size_t N)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if(idx < N)
    {
        out[idx] = v1[idx].dot(v2[idx]);
    }
    return;
}