// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

#include "Device.h"

#include <string>
#include <vector>

#include "CUDAUtils.h"
#include "Logging.h"
#include "SYCLUtils.h"

namespace open3d {
namespace core {

std::vector<std::string> SplitString(const std::string &str,
                                     const std::string &delimiters /* = " "*/,
                                     bool trim_empty_str /* = true*/) {
    std::vector<std::string> tokens;
    std::string::size_type pos = 0, new_pos = 0, last_pos = 0;
    while (pos != std::string::npos) {
        pos = str.find_first_of(delimiters, last_pos);
        new_pos = (pos == std::string::npos ? str.length() : pos);
        if (new_pos != last_pos || !trim_empty_str) {
            tokens.push_back(str.substr(last_pos, new_pos - last_pos));
        }
        last_pos = new_pos + 1;
    }
    return tokens;
}

std::string ToLower(const std::string &str) {
    std::string out = str;
    std::transform(str.begin(), str.end(), out.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return out;
}

static Device::DeviceType StringToDeviceType(const std::string &type_colon_id) {
    const std::vector<std::string> tokens =
            SplitString(type_colon_id, ":", true);
    if (tokens.size() == 2) {
        std::string device_type_lower = ToLower(tokens[0]);
        if (device_type_lower == "cpu") {
            return Device::DeviceType::CPU;
        } else if (device_type_lower == "cuda") {
            return Device::DeviceType::CUDA;
        } else if (device_type_lower == "sycl") {
            return Device::DeviceType::SYCL;
        } else {
            utility::LogError(
                    "Invalid device string {}. Valid device strings are like "
                    "\"CPU:0\" or \"CUDA:1\"",
                    type_colon_id);
        }
    } else {
        utility::LogError(
                "Invalid device string {}. Valid device strings are like "
                "\"CPU:0\" or \"CUDA:1\"",
                type_colon_id);
    }
}

static int StringToDeviceId(const std::string &type_colon_id) {
    const std::vector<std::string> tokens =
            SplitString(type_colon_id, ":", true);
    if (tokens.size() == 2) {
        return std::stoi(tokens[1]);
    } else {
        utility::LogError(
                "Invalid device string {}. Valid device strings are like "
                "\"CPU:0\" or \"CUDA:1\"",
                type_colon_id);
    }
}

Device::Device(DeviceType device_type, int device_id)
    : device_type_(device_type), device_id_(device_id) {
    // Sanity checks.
    if (device_type_ == DeviceType::CPU && device_id_ != 0) {
        utility::LogError("CPU has device_id {}, but it must be 0.",
                          device_id_);
    }
}

Device::Device(const std::string &device_type, int device_id)
    : Device(device_type + ":" + std::to_string(device_id)) {}

Device::Device(const std::string &type_colon_id)
    : Device(StringToDeviceType(type_colon_id),
             StringToDeviceId(type_colon_id)) {}

bool Device::operator==(const Device &other) const {
    return this->device_type_ == other.device_type_ &&
           this->device_id_ == other.device_id_;
}

bool Device::operator!=(const Device &other) const {
    return !operator==(other);
}

bool Device::operator<(const Device &other) const {
    return ToString() < other.ToString();
}

std::string Device::ToString() const {
    std::string str = "";
    switch (device_type_) {
        case DeviceType::CPU:
            str += "CPU";
            break;
        case DeviceType::CUDA:
            str += "CUDA";
            break;
        case DeviceType::SYCL:
            str += "SYCL";
            break;
        default:
            utility::LogError("Unsupported device type");
    }
    str += ":" + std::to_string(device_id_);
    return str;
}

bool Device::IsAvailable() const {
    for (const Device &device : GetAvailableDevices()) {
        if (device == *this) {
            return true;
        }
    }
    return false;
}

std::vector<Device> Device::GetAvailableDevices() {
    const std::vector<Device> cpu_devices = GetAvailableCPUDevices();
    const std::vector<Device> cuda_devices = GetAvailableCUDADevices();
    const std::vector<Device> sycl_devices = GetAvailableSYCLDevices();
    std::vector<Device> devices;
    devices.insert(devices.end(), cpu_devices.begin(), cpu_devices.end());
    devices.insert(devices.end(), cuda_devices.begin(), cuda_devices.end());
    devices.insert(devices.end(), sycl_devices.begin(), sycl_devices.end());
    return devices;
}

std::vector<Device> Device::GetAvailableCPUDevices() {
    return {Device(DeviceType::CPU, 0)};
}

std::vector<Device> Device::GetAvailableCUDADevices() {
    std::vector<Device> devices;
    for (int i = 0; i < cuda::DeviceCount(); i++) {
        devices.push_back(Device(DeviceType::CUDA, i));
    }
    return devices;
}

std::vector<Device> Device::GetAvailableSYCLDevices() {
    return sycl::GetAvailableSYCLDevices();
}

void Device::PrintAvailableDevices() {
    for (const auto &device : GetAvailableDevices()) {
        utility::LogInfo("Device(\"{}\")", device.ToString());
    }
}

}  // namespace core
}  // namespace open3d
