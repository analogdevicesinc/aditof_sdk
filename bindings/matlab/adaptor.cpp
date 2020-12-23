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
#include "aditofimaq.h"
#include "device_format.h"
#include "source_adaptor.h"
#include <mwadaptorimaq.h>

// *******************************************************************
// EXPORTED FUNCTIONS
// *******************************************************************

aditof::System *m_system = nullptr;

/**
 * initializeAdaptor: Exported function to initialize the adaptor.
 * This function is called directly after the adaptor DLL is loaded into
 * memory and gives the adaptor a chance to perform initializations before
 * any adaptor object is created.
 */

void initializeAdaptor() {
    m_system = new aditof::System();
}

/**
 * uninitializeAdaptor: Exported function to uninitialize the adaptor.
 * This function is called just before the adaptor DLL is unloaded when the
 * state of the toolbox is reset using IMAQRESET or when MATLAB exits.
 * This function gives the adaptor a chance to perform any clean-up tasks
 * such as deleting any dynamically allocated memory not covered in the
 * adaptor instance's destructor. This function will be called after the
 * destructor for all existing adaptor objects have been invoked.
 */

void uninitializeAdaptor() { delete m_system; }

/**
 * getAvailHW: Exported function to enumerate all the hardware to which
 * the adaptor interfaces. The imaqkit::IEngine calls this function when the
 * IMAQHWINFO function is called on the MATLAB command line.
 *
 * Adaptors can query the device's SDK to find out what devices are available.
 * Adaptors can also query the SDK for information about the formats supported
 * by each device. If this format information is available in advance, an
 * adaptor writer can store it in an IMDF file.
 *
 * @param imaqkit:IHardwareInfo*: The imaqkit::IEngine passes a handle to a
 * hardware information container. Adaptors use this container to communicate
 * all the hardware supported by the adaptor.
 */

void getAvailHW(imaqkit::IHardwareInfo *hardwareContainer) {
    std::vector<std::shared_ptr<aditof::Camera>> foundCameras;

    if (m_system) {
        m_system->getCameraList(foundCameras);
    }

    int currentCount = 0;

    for (size_t i = 0; i < foundCameras.size(); ++i) {

        imaqkit::IDeviceInfo *deviceInfo = hardwareContainer->createDeviceInfo(
            aditof::USB_DEVICE_ID + currentCount, aditof::USB_DEVICE_STR);

        deviceInfo->setDeviceFileSupport(false);

        imaqkit::IDeviceFormat *deviceFormat = deviceInfo->createDeviceFormat(
            aditof::BGR_FORMAT_ID, aditof::BGR_FORMAT_STR);

        DeviceFormat *bgrFormatInfo = new DeviceFormat();

        bgrFormatInfo->setFormatWidth(aditof::BGR_FORMAT_WIDTH);
        bgrFormatInfo->setFormatHeight(aditof::BGR_FORMAT_HEIGHT);
        bgrFormatInfo->setFormatNumBands(aditof::BGR_FORMAT_BANDS);
        bgrFormatInfo->setFormatFrameType(imaqkit::frametypes::BGR24_PACKED);

        deviceFormat->setAdaptorData(bgrFormatInfo);

        deviceInfo->addDeviceFormat(deviceFormat, true);

        deviceFormat = deviceInfo->createDeviceFormat(aditof::MONO_FORMAT_ID,
                                                      aditof::MONO_FORMAT_STR);

        DeviceFormat *monoFormatInfo = new DeviceFormat();

        monoFormatInfo->setFormatWidth(aditof::MONO_FORMAT_WIDTH);
        monoFormatInfo->setFormatHeight(aditof::MONO_FORMAT_WIDTH);
        monoFormatInfo->setFormatNumBands(aditof::MONO_FORMAT_BANDS);
        monoFormatInfo->setFormatFrameType(imaqkit::frametypes::MONO8);

        deviceFormat->setAdaptorData(monoFormatInfo);

        deviceInfo->addDeviceFormat(deviceFormat);

        hardwareContainer->addDevice(deviceInfo);

        currentCount++;
    }

    // network device with configurable ip (FORMAT)
    imaqkit::IDeviceInfo *deviceInfo = hardwareContainer->createDeviceInfo(
        currentCount + 1, aditof::NETWORK_DEVICE_STR);

    deviceInfo->setDeviceFileSupport(true);

    hardwareContainer->addDevice(deviceInfo);
}

/**
 * getDeviceAttributes() -- Exported function used to dynamically add
 * device-specific properties. The imaqkit::IEngine calls this function when a
 * user creates a video input object by calling the VIDEOINPUT function at the
 * MATLAB command line.
 *
 * Adaptors can query the device's SDK to determine these properties or, if the
 * information is available in advance, store it in an IMDF file and read the
 * information from the file.
 *
 * Adaptors create property objects and use methods of the imaqkit::IPropFactory
 * object to store the properties in the device-specific property container.
 *
 * @param deviceInfo: The imaqkit::IEngine passes a handle to an
 * imaqkit::IDeviceInfo object identifying the target device. The handle will be
 * one of the IDeviceInfo objects created in getAvailHW().
 *
 * @param formatName: The imaqkit::IEngine passes a text string that specifies
 * either one of the fixed format names specified for the device in getAvailHW()
 * or a filename. If it's a file name, it is the name of a device configuration
 *                    file (also known as a camera file).
 *                    The imaqkit::IEngine performs no processing on device
 * configuration files. Your adaptor should just pass this file to your device.
 *
 * @param devicePropFact: The imaqkit::IEngine passes a handle to an
 * imaqkit::IPropFactory object. This object supports methods you use to create
 * and add device-specific properties.
 *
 * @param sourceContainer: The imaqkit::IEngine passes a handle to an
 * imaqkit::IVideoSourceInfo object. You use this object to identify the
 * device-specific video sources.
 *
 *                         NOTE: To be able to create a videoinput object in
 * MATLAB, your adaptor must identify at least one video source.
 *
 * @param hwTriggerInfo: The imaqkit::IEngine passes a handle to an
 * imaqkit::ITriggerInfo object. You use this object to create and add valid
 * hardware trigger configurations. Manual and immediate trigger configurations
 * are handled by the IMAQ engine automatically.
 *
 */
void getDeviceAttributes(const imaqkit::IDeviceInfo *deviceInfo,
                         const char *formatName,
                         imaqkit::IPropFactory *devicePropFact,
                         imaqkit::IVideoSourceInfo *sourceContainer,
                         imaqkit::ITriggerInfo *hwTriggerInfo) {

    void *hProp; // Declare a handle to a property object.

    // Get the device ID from the imaqkit::IDeviceInfo object.
    const char *devName = deviceInfo->getDeviceName();

    // Create properties based on which device was selected.
    // Network and usb devices have the same properties.
    if (strcmp(devName, aditof::USB_DEVICE_STR) == 0 ||
        strcmp(devName, aditof::NETWORK_DEVICE_STR) == 0) {
        // ******************************************************
        // IDENTIFY THE VIDEO SOURCE
        // ********************************************************
        // Use the imaqkit::IVideoSourceInfo object's addAdaptorSource() method
        // to specify the video source. The addAdaptorSource method
        // accepts two arguments:
        //      - Source name (text string)
        //      - Numeric ID

        sourceContainer->addAdaptorSource(aditof::SOURCE_INPUT_1_STR,
                                          aditof::SOURCE_INPUT_1_ID);

        // Property for settings the cameras mode
        // Options are: near, medium, far
        hProp = devicePropFact->createEnumProperty(
            aditof::MODE_STR, aditof::MODE_NEAR_STR, aditof::MODE_NEAR_ID);
        devicePropFact->addEnumValue(hProp, aditof::MODE_MEDIUM_STR,
                                     aditof::MODE_MEDIUM_ID);
        devicePropFact->addEnumValue(hProp, aditof::MODE_FAR_STR,
                                     aditof::MODE_FAR_ID);
        devicePropFact->setPropReadOnly(hProp,
                                        imaqkit::propreadonly::WHILE_RUNNING);
        devicePropFact->addProperty(hProp, aditof::ADITOF_PROPERTY_MODE);

        // Property for setting the displayed frame type
        // Options are: Depth, Ir
        hProp = devicePropFact->createEnumProperty(aditof::FRAME_TYPE_STR,
                                                   aditof::FRAME_TYPE_DEPTH_STR,
                                                   aditof::FRAME_TYPE_DEPTH_ID);
        devicePropFact->addEnumValue(hProp, aditof::FRAME_TYPE_IR_STR,
                                     aditof::FRAME_TYPE_IR_ID);

        devicePropFact->setPropReadOnly(hProp,
                                        imaqkit::propreadonly::WHILE_RUNNING);
        devicePropFact->addProperty(hProp, aditof::ADITOF_PROPERTY_FRAME_TYPE);

        // Property for setting the small signal removal value
        // Range is 0 - 16383
        hProp = devicePropFact->createIntProperty(
            aditof::SMALL_SIGNAL_STR, aditof::SMALL_SIGNAL_LOWER_LIMIT,
            aditof::SMALL_SIGNAL_UPPER_LIMIT, aditof::SMALL_SIGNAL_DEFAULT);
        devicePropFact->setPropReadOnly(hProp,
                                        imaqkit::propreadonly::WHILE_RUNNING);
        devicePropFact->addProperty(hProp,
                                    aditof::ADITOF_PROPERTY_SMALL_SIGNAL);

        // Temp readings
        hProp = devicePropFact->createStringProperty(
            aditof::TEMPERATURE_STR, "No temperature sensor found!");
        devicePropFact->setPropReadOnly(hProp, imaqkit::propreadonly::ALWAYS);
        devicePropFact->addProperty(hProp, aditof::ADITOF_PROPERTY_TEMP);

    } else {
        // Throw an error in MATLAB.
        imaqkit::adaptorError(NULL, aditof::ERRID_DEVICE_NOT_FOUND,
                              aditof::ERRMSG_DEVICE_NOT_FOUND);
    }
}

/**
 * createInstance() -- Exported function to return a new instance of an adaptor
 * object. The imaqkit::IEngine calls this function when a user attempts to
 * create a video input object by calling the VIDEOINPUT function at the MATLAB
 * command line.
 *
 * @param engine: The imaqkit::IEngine passes a handle to an imaqkit::IEngine
 * object to which the adaptor will interface.
 *
 * @param deviceInfo: The imaqkit::IEngine passes a handle to an
 * imaqkit::IDeviceInfo object identifying the target device. The handle will be
 * one of the IDeviceInfo objects created in getAvailHW().
 *
 * @param formatName: The imaqkit::IEngine passes a text string that specifies
 * either one of the fixed format names specified for the device in getAvailHW()
 * or a filename. If it's a file name, it is the name of a device configuration
 *                    file (also known as a camera file).
 *                    The imaqkit::IEngine performs no processing on device
 * configuration files. Your adaptor should just pass this file to your device.
 *
 */

imaqkit::IAdaptor *createInstance(imaqkit::IEngine *engine,
                                  const imaqkit::IDeviceInfo *deviceInfo,
                                  const char *formatName) {

    // Call your adaptor's constructor to create an instance, passing the
    // the same three arguments to the constructor.

    std::vector<std::shared_ptr<aditof::Camera>> cameraList;
    m_system->getCameraList(cameraList);

    std::shared_ptr<aditof::Camera> camera = nullptr;

    int nrCameras = cameraList.size();

    if (deviceInfo->getDeviceID() - 1 < nrCameras) {
        camera = cameraList.at(deviceInfo->getDeviceID() - 1);
    } else if (deviceInfo->getDeviceID() - 1 == nrCameras) {
        std::vector<std::shared_ptr<aditof::Camera>> cameraListAtIp;
        m_system->getCameraListAtIp(cameraListAtIp, formatName);
        if (cameraListAtIp.size() > 0) {
            camera = cameraListAtIp.at(0);
        }
    }

    imaqkit::IAdaptor *adaptor =
        new SourceAdaptor(engine, deviceInfo, formatName, camera);

    if (!camera) {
        // Throw an error in MATLAB.
        imaqkit::adaptorError(NULL, aditof::ERRID_DEVICE_NOT_FOUND,
                              aditof::ERRMSG_DEVICE_NOT_FOUND);
    }

    return adaptor;
}
