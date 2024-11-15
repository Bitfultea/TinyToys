// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "kernel/NonZero.h"

#include "Device.h"
#include "Logging.h"
#include "Tensor.h"

namespace open3d {
namespace core {
namespace kernel {

Tensor NonZero(const Tensor& src) {
    if (src.IsCPU()) {
        return NonZeroCPU(src);
    } else if (src.IsCUDA()) {
#ifdef BUILD_CUDA_MODULE
        return NonZeroCUDA(src);
#else
        utility::LogError("Not compiled with CUDA, but CUDA device is used.");
#endif
    } else {
        utility::LogError("NonZero: Unimplemented device");
    }
}

}  // namespace kernel
}  // namespace core
}  // namespace open3d
