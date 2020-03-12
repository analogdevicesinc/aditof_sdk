/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "pybind11/functional.h"
#include "pybind11/numpy.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include <aditof/aditof.h>
#include <aditof/camera_96tof1_specifics.h>
#include <aditof/camera_chicony_specifics.h>

namespace py = pybind11;

PYBIND11_MODULE(aditofpython, m) {

    m.doc() = "ADI Time Of Flight python extensions";

    // General declarations

    py::enum_<aditof::Status>(m, "Status")
        .value("Ok", aditof::Status::OK)
        .value("Busy", aditof::Status::BUSY)
        .value("Unreachable", aditof::Status::UNREACHABLE)
        .value("InvalidArgument", aditof::Status::INVALID_ARGUMENT)
        .value("GenericError", aditof::Status::GENERIC_ERROR);

    // Frame declarations

    py::enum_<aditof::FrameDataType>(m, "FrameDataType")
        .value("Raw", aditof::FrameDataType::RAW)
        .value("Depth", aditof::FrameDataType::DEPTH)
        .value("IR", aditof::FrameDataType::IR);

    py::class_<aditof::FrameCalData>(m, "FrameCalData")
        .def(py::init<>())
        .def_readwrite("offset", &aditof::FrameCalData::offset)
        .def_readwrite("gain", &aditof::FrameCalData::gain);

    py::class_<aditof::FrameDetails>(m, "FrameDetails")
        .def(py::init<>())
        .def_readwrite("width", &aditof::FrameDetails::width)
        .def_readwrite("height", &aditof::FrameDetails::height)
        .def_readwrite("type", &aditof::FrameDetails::type)
        .def_readwrite("cal_data", &aditof::FrameDetails::cal_data);

    // Camera declarations

    py::enum_<aditof::ConnectionType>(m, "ConnectionType")
        .value("Usb", aditof::ConnectionType::USB)
        .value("Ethernet", aditof::ConnectionType::ETHERNET)
        .value("local", aditof::ConnectionType::LOCAL);

    py::enum_<aditof::Revision>(m, "Revision")
        .value("RevB", aditof::Revision::RevB)
        .value("RevC", aditof::Revision::RevC);

    py::class_<aditof::IntrinsicParameters>(m, "IntrinsicParameters")
        .def(py::init<>())
        .def_readwrite("cameraMatrix",
                       &aditof::IntrinsicParameters::cameraMatrix)
        .def_readwrite("distCoeffs", &aditof::IntrinsicParameters::distCoeffs)
        .def_readwrite("pixelWidth", &aditof::IntrinsicParameters::pixelWidth)
        .def_readwrite("pixelHeight",
                       &aditof::IntrinsicParameters::pixelHeight);

    py::class_<aditof::CameraDetails>(m, "CameraDetails")
        .def(py::init<>())
        .def_readwrite("cameraId", &aditof::CameraDetails::cameraId)
        .def_readwrite("mode", &aditof::CameraDetails::mode)
        .def_readwrite("frameType", &aditof::CameraDetails::frameType)
        .def_readwrite("connection", &aditof::CameraDetails::connection)
        .def_readwrite("intrinsics", &aditof::CameraDetails::intrinsics)
        .def_readwrite("minDepth", &aditof::CameraDetails::minDepth)
        .def_readwrite("maxDepth", &aditof::CameraDetails::maxDepth)
        .def_readwrite("bitCount", &aditof::CameraDetails::bitCount);

    // Helpers

    struct frameData {
        uint16_t *pData;
        aditof::FrameDetails details;
    };

    py::class_<frameData>(m, "frameData", py::buffer_protocol())
        .def(py::init<>())
        .def_buffer([](const frameData &f) -> py::buffer_info {
            return py::buffer_info(
                f.pData, sizeof(uint16_t),
                py::format_descriptor<uint16_t>::format(), 2,
                {f.details.height, f.details.width},
                {sizeof(uint16_t) * f.details.width, sizeof(uint16_t)});
        });

    // ADI Time of Flight API

    py::class_<aditof::System>(m, "System")
        .def(py::init<>())
        .def("initialize", &aditof::System::initialize)
        .def("getCameraList",
             [](aditof::System &system, py::list cameras) {
                 std::vector<std::shared_ptr<aditof::Camera>> cameraList;
                 aditof::Status status = system.getCameraList(cameraList);

                 for (const auto &cam : cameraList) {
                     cameras.append(cam);
                 }

                 return status;
             },
             py::arg("cameras"))
        .def("getCameraListAtIp",
             [](aditof::System &system, py::list cameras, py::str ip) {
                 std::vector<std::shared_ptr<aditof::Camera>> cameraList;
                 aditof::Status status =
                     system.getCameraListAtIp(cameraList, ip);

                 for (const auto &cam : cameraList) {
                     cameras.append(cam);
                 }

                 return status;
             },
             py::arg("cameras"), py::arg("ip"));

    py::class_<aditof::Camera, std::shared_ptr<aditof::Camera>>(m, "Camera")
        .def("initialize", &aditof::Camera::initialize)
        .def("start", &aditof::Camera::start)
        .def("stop", &aditof::Camera::stop)
        .def("setMode", &aditof::Camera::setMode, py::arg("mode"),
             py::arg("modeFilename") = "")
        .def("getAvailableModes",
             [](const aditof::Camera &camera, py::list modes) {
                 std::vector<std::string> modeList;
                 aditof::Status status = camera.getAvailableModes(modeList);

                 for (const auto &mode : modeList)
                     modes.append(mode);

                 return status;
             },
             py::arg("availableModes"))
        .def("setFrameType", &aditof::Camera::setFrameType,
             py::arg("frameType"))
        .def("getAvailableFrameTypes",
             [](const aditof::Camera &camera, py::list types) {
                 std::vector<std::string> typeList;
                 aditof::Status status =
                     camera.getAvailableFrameTypes(typeList);

                 for (const auto &type : typeList)
                     types.append(type);

                 return status;
             },
             py::arg("availableFrameTypes"))
        .def("requestFrame", &aditof::Camera::requestFrame, py::arg("frame"),
             py::arg("cb") = nullptr)
        .def("getDetails", &aditof::Camera::getDetails, py::arg("details"))
        .def("getDevice", &aditof::Camera::getDevice)
        .def("getCamera96Tof1Specifics",
             [](aditof::Camera &camera) {
                 using namespace aditof;
                 std::shared_ptr<CameraSpecifics> specifics =
                     camera.getSpecifics();

                 return std::dynamic_pointer_cast<Camera96Tof1Specifics>(
                     specifics);
             })
        .def("getCameraChiconySpecifics", [](aditof::Camera &camera) {
            using namespace aditof;
            std::shared_ptr<CameraSpecifics> specifics = camera.getSpecifics();

            return std::dynamic_pointer_cast<CameraChiconySpecifics>(specifics);
        });

    py::class_<aditof::Frame>(m, "Frame")
        .def(py::init<>())
        .def("setDetails", &aditof::Frame::setDetails, py::arg("details"))
        .def("getDetails", &aditof::Frame::getDetails, py::arg("details"))
        .def("getData",
             [](aditof::Frame &frame,
                aditof::FrameDataType dataType) -> frameData {
                 frameData f;

                 frame.getData(dataType, &f.pData);
                 frame.getDetails(f.details);

                 return f;
             },
             py::arg("dataType"));

    py::class_<aditof::DeviceInterface,
               std::shared_ptr<aditof::DeviceInterface>>(m, "DeviceInterface")
        .def("open", &aditof::DeviceInterface::open)
        .def("start", &aditof::DeviceInterface::start)
        .def("stop", &aditof::DeviceInterface::stop)
        .def("getAvailableFrameTypes",
             [](aditof::DeviceInterface &device, py::list types) {
                 std::vector<aditof::FrameDetails> typeList;
                 aditof::Status status =
                     device.getAvailableFrameTypes(typeList);

                 for (const auto &type : typeList)
                     types.append(type);

                 return status;
             },
             py::arg("types"))
        .def("setFrameType", &aditof::DeviceInterface::setFrameType,
             py::arg("details"))
        .def("program",
             [](aditof::DeviceInterface &device, py::array_t<uint8_t> firmware,
                size_t size) {
                 py::buffer_info buffInfo = firmware.request();
                 uint8_t *ptr = static_cast<uint8_t *>(buffInfo.ptr);

                 return device.program(ptr, size);
             },
             py::arg("firmware"), py::arg("size"))
        .def("getFrame",
             [](aditof::DeviceInterface &device, py::array_t<uint16_t> buffer) {
                 py::buffer_info buffInfo = buffer.request(true);
                 uint16_t *ptr = static_cast<uint16_t *>(buffInfo.ptr);

                 return device.getFrame(ptr);
             },
             py::arg("buffer"))
        .def("readEeprom",
             [](aditof::DeviceInterface &device, uint32_t address,
                py::array_t<uint8_t> data, size_t length) {
                 py::buffer_info buffInfo = data.request(true);
                 uint8_t *ptr = static_cast<uint8_t *>(buffInfo.ptr);

                 return device.readEeprom(address, ptr, length);
             },
             py::arg("address"), py::arg("data"), py::arg("length"))
        .def("writeEeprom",
             [](aditof::DeviceInterface &device, uint32_t address,
                py::array_t<uint8_t> data, size_t length) {
                 py::buffer_info buffInfo = data.request();
                 uint8_t *ptr = static_cast<uint8_t *>(buffInfo.ptr);

                 return device.writeEeprom(address, ptr, length);
             },
             py::arg("address"), py::arg("data"), py::arg("length"))
        .def("readAfeRegisters",
             [](aditof::DeviceInterface &device, py::array_t<uint16_t> address,
                py::array_t<uint16_t> data, size_t length) {
                 py::buffer_info addrBuffInfo = address.request();
                 uint16_t *addrPtr = static_cast<uint16_t *>(addrBuffInfo.ptr);

                 py::buffer_info dataBuffInfo = data.request(true);
                 uint16_t *dataPtr = static_cast<uint16_t *>(dataBuffInfo.ptr);

                 return device.readAfeRegisters(addrPtr, dataPtr, length);
             },
             py::arg("address"), py::arg("data"), py::arg("length"))
        .def("writeAfeRegisters",
             [](aditof::DeviceInterface &device, py::array_t<uint16_t> address,
                py::array_t<uint16_t> data, size_t length) {
                 py::buffer_info addrBuffInfo = address.request();
                 uint16_t *addrPtr = static_cast<uint16_t *>(addrBuffInfo.ptr);

                 py::buffer_info dataBuffInfo = data.request();
                 uint16_t *dataPtr = static_cast<uint16_t *>(dataBuffInfo.ptr);

                 return device.writeAfeRegisters(addrPtr, dataPtr, length);
             },
             py::arg("address"), py::arg("data"), py::arg("length"))
        .def("readAfeTemp",
             [](aditof::DeviceInterface &device, py::list temperature) {
                 float temp;
                 aditof::Status status = device.readAfeTemp(temp);
                 temperature.append(temp);
                 return status;
             })
        .def("readLaserTemp",
             [](aditof::DeviceInterface &device, py::list temperature) {
                 float temp;
                 aditof::Status status = device.readLaserTemp(temp);
                 temperature.append(temp);
                 return status;
             })
        .def("setCalibrationParams",
             &aditof::DeviceInterface::setCalibrationParams, py::arg("mode"),
             py::arg("gain"), py::arg("offset"), py::arg("range"))
        .def("applyCalibrationToFrame",
             [](aditof::DeviceInterface &device, py::array_t<uint16_t> frame,
                const std::string &mode) {
                 py::buffer_info buffInfo = frame.request(true);
                 uint16_t *ptr = static_cast<uint16_t *>(buffInfo.ptr);

                 return device.applyCalibrationToFrame(ptr, mode);
             },
             py::arg("frame"), py::arg("mode"));

    py::class_<aditof::Camera96Tof1Specifics,
               std::shared_ptr<aditof::Camera96Tof1Specifics>>(
        m, "Camera96Tof1Specifics")
        .def("enableNoiseReduction",
             &aditof::Camera96Tof1Specifics::enableNoiseReduction,
             py::arg("en"))
        .def("noiseReductionEnabled",
             &aditof::Camera96Tof1Specifics::noiseReductionEnabled)
        .def("setNoiseReductionThreshold",
             &aditof::Camera96Tof1Specifics::setNoiseReductionThreshold,
             py::arg("threshold"))
        .def("noiseReductionThreshold",
             &aditof::Camera96Tof1Specifics::noiseReductionThreshold)
        .def("setCameraRevision",
             &aditof::Camera96Tof1Specifics::setCameraRevision,
             py::arg("revision"))
        .def("getRevision", &aditof::Camera96Tof1Specifics::getRevision);

    py::class_<aditof::CameraChiconySpecifics,
               std::shared_ptr<aditof::CameraChiconySpecifics>>(
        m, "CameraChiconySpecifics")
        .def("enableNoiseReduction",
             &aditof::CameraChiconySpecifics::enableNoiseReduction,
             py::arg("en"))
        .def("noiseReductionEnabled",
             &aditof::CameraChiconySpecifics::noiseReductionEnabled)
        .def("setNoiseReductionThreshold",
             &aditof::CameraChiconySpecifics::setNoiseReductionThreshold,
             py::arg("threshold"))
        .def("noiseReductionThreshold",
             &aditof::CameraChiconySpecifics::noiseReductionThreshold);
}
