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


#include <emscripten/bind.h>
#include <aditof/aditof.h>
// #include "../../sdk/include/aditof/aditof.h"




using namespace emscripten;

EMSCRIPTEN_BINDINGS(Module) {
    // General declarations

    emscripten::enum_<aditof::Status>("Status")
        .value("Ok", aditof::Status::OK)
        .value("Busy", aditof::Status::BUSY)
        .value("Unreachable", aditof::Status::UNREACHABLE)
        .value("InvalidArgument", aditof::Status::INVALID_ARGUMENT)
        .value("Unavailable", aditof::Status::UNAVAILABLE)
        .value("GenericError", aditof::Status::GENERIC_ERROR)
        ;

    // Frame declarations

    emscripten::enum_<aditof::FrameDataType>("FrameDataType")
        .value("FullData", aditof::FrameDataType::FULL_DATA)
        .value("Depth", aditof::FrameDataType::DEPTH)
        .value("IR", aditof::FrameDataType::IR)
        ;

    emscripten::class_<aditof::FrameDetails>("FrameDetails")
        .constructor<>()
        .property("width", &aditof::FrameDetails::width)
        .property("height", &aditof::FrameDetails::height)
        .property("fullDataWidth", &aditof::FrameDetails::fullDataWidth)
        .property("fullDataHeight", &aditof::FrameDetails::fullDataHeight)
        .property("rgbWidth", &aditof::FrameDetails::rgbWidth)
        .property("rgbHeight", &aditof::FrameDetails::rgbHeight)
        .property("type", &aditof::FrameDetails::type)
        ;

    // Camera declarations

    emscripten::enum_<aditof::ConnectionType>("ConnectionType")
        .value("Usb", aditof::ConnectionType::USB)
        .value("Network", aditof::ConnectionType::NETWORK)
        .value("OnTarget", aditof::ConnectionType::ON_TARGET)
        ;

    emscripten::class_<aditof::IntrinsicParameters>("IntrinsicParameters")
        .constructor<>()
        .property("cameraMatrix", &aditof::IntrinsicParameters::cameraMatrix)
        .property("distCoeffs", &aditof::IntrinsicParameters::distCoeffs)
        .property("pixelWidth", &aditof::IntrinsicParameters::pixelWidth)
        .property("pixelHeight", &aditof::IntrinsicParameters::pixelHeight)
        ;

    emscripten::class_<aditof::DepthParameters>("DepthParameters")
        .constructor<>()
        .property("depthGain", &aditof::DepthParameters::depthGain)
        .property("depthOffset", &aditof::DepthParameters::depthOffset)
        .property("minDepth", &aditof::DepthParameters::minDepth)
        .property("maxDepth", &aditof::DepthParameters::maxDepth)
        ;

    emscripten::class_<aditof::CameraDetails>("CameraDetails")
        .constructor<>()
        .property("cameraId", &aditof::CameraDetails::cameraId)
        .property("mode", &aditof::CameraDetails::mode)
        .property("frameType", &aditof::CameraDetails::frameType)
        .property("connection", &aditof::CameraDetails::connection)
        .property("intrinsics", &aditof::CameraDetails::intrinsics)
        .property("depthParameters", &aditof::CameraDetails::depthParameters)
        .property("bitCount", &aditof::CameraDetails::bitCount)
        ;

    // // Helpers

    // // struct frameData {
    // //     uint16_t *pData;
    // //     aditof::FrameDetails details;
    // // };

    // // em::class_<frameData>("frameData", buffer_protocol())
    // //     .constructor<>()
    // //     .property("pData", pData)
    // //     .property("details", details)
    // //     // .def_buffer([](const frameData &f) -> buffer_info {
    //     //     return buffer_info(
    //     //         f.pData, sizeof(uint16_t),
    //     //         format_descriptor<uint16_t>::format(), 2,
    //     //         {f.details.height + f.details.rgbHeight,
    //     //          f.details.width + f.details.rgbWidth},
    //     //         {sizeof(uint16_t) * f.details.width, sizeof(uint16_t)});
    //     // })
    //     ;

    // // ADI Time of Flight API

    // System
    // emscripten::class_<aditof::System>("System")
    //     .constructor<>()
    //     .function("getCameraList", &aditof::System::getCameraList)
    //     .function("getCameraListAtIp", &aditof::System::getCameraListAtIp)
    //     ;

    // Camera
    // emscripten::class_<aditof::Camera, std::shared_ptr<aditof::Camera>>("Camera")
    //     .function("initialize", &aditof::Camera::initialize)
    //     .function("start", &aditof::Camera::start)
    //     .function("stop", &aditof::Camera::stop)
    //     .function("setMode", &aditof::Camera::setMode)
    //     .function("getAvailableModes", &aditof::Camera::getAvailableModes)
    //     .function("setFrameType", &aditof::Camera::setFrameType)
    //     .function("getAvailableFrameTypes", &aditof::Camera::getAvailableFrameTypes)
    //     .function("requestFrame", &aditof::Camera::requestFrame)
    //     .function("getDetails", &aditof::Camera::getDetails)
    //     .function("getAvailableControls", &aditof::Camera::getAvailableControls)
    //     .function("setControl", &aditof::Camera::setControl)
    //     .function("getControl", &aditof::Camera::getControl)
    //     .function("getImageSensors", &aditof::Camera::getImageSensors)
    //     .function("getEeproms", &aditof::Camera::getEeproms)
    //     .function("getTemperatureSensors", &aditof::Camera::getTemperatureSensors)
    //     ;

    // // Frame
    // emscripten::class_<aditof::Frame>("Frame")
    //     .constructor<>()
    //     .function("setDetails", &aditof::Frame::setDetails)
    //     .function("getDetails", &aditof::Frame::getDetails)
    //     .function("getData", &aditof::Frame::getData)
    //     ;

    // // DepthSensorInterface
    // emscripten::class_<aditof::DepthSensorInterface, std::shared_ptr<aditof::DepthSensorInterface>>("DepthSensorInterface")
    //     .function("open", &aditof::DepthSensorInterface::open)
    //     .function("start", &aditof::DepthSensorInterface::start)
    //     .function("stop", &aditof::DepthSensorInterface::stop)
    //     .function("getAvailableFrameTypes", &aditof::DepthSensorInterface::getAvailableFrameTypes)
    //     .function("setFrameType", &aditof::DepthSensorInterface::setFrameType)
    //     .function("program", &aditof::DepthSensorInterface::program)
    //     .function("getFrame", &aditof::DepthSensorInterface::getFrame)
    //     .function("readAfeRegisters", &aditof::DepthSensorInterface::readAfeRegisters)
    //     .function("writeAfeRegisters", &aditof::DepthSensorInterface::writeAfeRegisters)
    //     ;

    // // StorageInterface
    // emscripten::class_<aditof::StorageInterface, std::shared_ptr<aditof::StorageInterface>>("StorageInterface")
    //     .function("open", &aditof::StorageInterface::open)
    //     .function("read", &aditof::StorageInterface::read)
    //     .function("write", &aditof::StorageInterface::write)
    //     .function("close", &aditof::StorageInterface::close)
    //     .function("getName", &aditof::StorageInterface::getName)
    //     ;

    // // TemperatureSensorInterface
    // emscripten::class_<aditof::TemperatureSensorInterface, std::shared_ptr<aditof::TemperatureSensorInterface>>("TemperatureSensorInterface")
    //     .function("open", &aditof::TemperatureSensorInterface::open)
    //     .function("read", &aditof::TemperatureSensorInterface::read)
    //     .function("close", &aditof::TemperatureSensorInterface::close)
    //     .function("getName", &aditof::TemperatureSensorInterface::getName)
    //     ;

}