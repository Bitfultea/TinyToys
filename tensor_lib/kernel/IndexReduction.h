// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include "Logging.h"
#include "Tensor.h"

namespace open3d {
namespace core {
namespace kernel {

void IndexAdd_(int64_t dim,
               const Tensor& index,
               const Tensor& src,
               Tensor& dst);

void IndexAddCPU_(int64_t dim,
                  const Tensor& index,
                  const Tensor& src,
                  Tensor& dst);

#ifdef BUILD_CUDA_MODULE
void IndexAddCUDA_(int64_t dim,
                   const Tensor& index,
                   const Tensor& src,
                   Tensor& dst);
#endif

}  // namespace kernel
}  // namespace core
}  // namespace open3d
