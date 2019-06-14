#include "pybind11/functional.h"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include <aditof/aditof.h>

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

    py::class_<aditof::CameraDetails>(m, "CameraDetails")
        .def(py::init<>())
        .def_readwrite("cameraId", &aditof::CameraDetails::cameraId)
        .def_readwrite("mode", &aditof::CameraDetails::mode)
        .def_readwrite("frameType", &aditof::CameraDetails::frameType)
        .def_readwrite("connection", &aditof::CameraDetails::connection);

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
                {f.details.width, f.details.height},
                {sizeof(uint16_t) + f.details.width, sizeof(uint16_t)});
        });

    // ADI Time of Flight API

    py::class_<aditof::System>(m, "System")
        .def(py::init<>())
        .def("initialize", &aditof::System::initialize)
        .def("getCameraList",
             [](aditof::System &system, py::list cameras) {
                 std::vector<aditof::Camera *> cameraList;
                 aditof::Status status = system.getCameraList(cameraList);

                 for (const auto &cam : cameraList) {
                     cameras.append(cam);
                 }

                 return status;
             },
             py::arg("cameras"));

    py::class_<aditof::Camera>(m, "Camera")
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
        .def("getDevice", &aditof::Camera::getDevice,
             py::return_value_policy::reference_internal);

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

    // TO DO: class DeviceInterface
}
