// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "linalg/LUImpl.h"
#include "linalg/LapackWrapper.h"
#include "linalg/LinalgUtils.h"

namespace open3d {
namespace core {

void LUCPU(void* A_data,
           void* ipiv_data,
           int64_t rows,
           int64_t cols,
           Dtype dtype,
           const Device& device) {
    DISPATCH_LINALG_DTYPE_TO_TEMPLATE(dtype, [&]() {
        OPEN3D_LAPACK_CHECK(
                getrf_cpu<scalar_t>(
                        LAPACK_COL_MAJOR, rows, cols,
                        static_cast<scalar_t*>(A_data), rows,
                        static_cast<OPEN3D_CPU_LINALG_INT*>(ipiv_data)),
                "getrf failed in LUCPU");
    });
}

}  // namespace core
}  // namespace open3d
