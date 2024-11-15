// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#pragma once

#include "Tensor.h"

namespace open3d { namespace core { namespace kernel {

Tensor Arange(const Tensor &start, const Tensor &stop, const Tensor &step);

void ArangeCPU(const Tensor &start, const Tensor &stop, const Tensor &step, Tensor &dst);

#ifdef BUILD_CUDA_MODULE
void ArangeCUDA(const Tensor &start, const Tensor &stop, const Tensor &step, Tensor &dst);
#endif

}}} // namespace open3d::core::kernel
