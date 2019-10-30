#include "AditofAdaptor.h"
#include "AditofDeviceFormat.h"
#include "AditofPropListener.h"
#include "AditofSourceListener.h"
#include "AditofTemperatureGetFcn.h"
#include "aditofimaq.h"
#include <string>

#include <aditof/device_interface.h>

#include <exception>
#ifndef _WIN32
#include <unistd.h> /* usleep */
#endif

static const uint8_t colormap[3 * 256] = {
#include "colormap.txt"
};

//******************************************
//  CONSTRUCTOR/DESTRUCTOR
//******************************************
AditofAdaptor::AditofAdaptor(imaqkit::IEngine *engine,
                             const imaqkit::IDeviceInfo *deviceInfo,
                             const char *formatName, aditof::Camera *camera)
    : imaqkit::IAdaptor(engine), m_di(deviceInfo),
      m_sendThread(static_cast<Thread>(NULL)), m_formatName(formatName),
      m_acquisitionActive(false), m_camera(camera),
      m_currentMode(aditof::MODE_NEAR_ID),
      m_currentDisplayedType(aditof::FRAME_TYPE_DEPTH_ID) {

    // Perform any necessary device initialization and create listeners for
    // device-specific properties.
    initDevice();

    // Create a critical section handle.
    m_driverGuard = imaqkit::createCriticalSection();
    m_acquisitionActiveGuard = imaqkit::createCriticalSection();
}

// The destructor calls the imaqkit::IAdaptor::close() method, not the
// closeDevice() method. The close() method calls the closeDevice() method.
AditofAdaptor::~AditofAdaptor() {

    // Stop the device.
    stop();

    // Close the device.
    close();

    // Cleanup - deallocate the critical section handle.
    delete m_driverGuard;
}

//  Device Initialization Method
//  Setup property listeners to get notification of changes to their values.
void AditofAdaptor::initDevice() {

    // Get the engine property container.
    m_enginePropContainer = getEngine()->getEnginePropContainer();

    m_enginePropContainer->addListener("SelectedSourceName",
                                       new AditofSourceListener(this));

    // Get a handle to the property container for device-specific properties
    // associated with the imaqkit::IEngine object.
    imaqkit::IPropContainer *adaptorPropContainer =
        getEngine()->getAdaptorPropContainer();

    // Determine the number of properties in the container.
    int numDeviceProps = adaptorPropContainer->getNumberProps();

    // Allocate a string array for storing property names.
    const char **devicePropNames = new const char *[numDeviceProps];

    // Get the property names and store them in the string array.
    adaptorPropContainer->getPropNames(devicePropNames);

    // Cycle through each property and add listeners to those that are device
    // specific.
    for (int i = 0; i < numDeviceProps; i++) {

        // Add custom get functions to the timestamp properties.  For the other
        // properties add post-set listeners.
        if ((std::string(aditof::AFE_TEMPERATURE_STR) == devicePropNames[i]) ||
            (std::string(aditof::LASER_TEMPERATURE_STR) ==
             devicePropNames[i])) {
            adaptorPropContainer->setCustomGetFcn(
                devicePropNames[i], new AditofTemperatureGetFcn(this));
        } else {
            // Get the property information object.
            imaqkit::IPropInfo *propInfo =
                adaptorPropContainer->getIPropInfo(devicePropNames[i]);

            // Check to see whether the property is device-specific. Do not
            // create create property listeners for non device-specific
            // properties such as 'Parent' and 'Tag'.
            if (propInfo->isPropertyDeviceSpecific()) {
                adaptorPropContainer->addListener(devicePropNames[i],
                                                  new AditofPropListener(this));
            }
        }
    }

    // Clean up the array of property names.
    delete[] devicePropNames;

    if (!m_camera) {
        // If no camera is found, we have nothing to configure
        return;
    }

    aditof::Status status = m_camera->initialize();
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not initialize camera!";
        return;
    }

    std::vector<std::string> frameTypes;
    m_camera->getAvailableFrameTypes(frameTypes);
    if (frameTypes.empty()) {
        LOG(ERROR) << "no frame type avaialble!";
        return;
    }
    status = m_camera->setFrameType(frameTypes.front());
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not set camera frame type!";
        return;
    }

    std::vector<std::string> modes;
    m_camera->getAvailableModes(modes);
    if (modes.empty()) {
        LOG(ERROR) << "no camera modes available!";
        return;
    }
    status = m_camera->setMode(modes.front());
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Could not set camera mode!";
        return;
    }
}

// Return a string identifying the name of the vendor's device driver.
const char *AditofAdaptor::getDriverDescription() const {
    return aditof::DRIVER_DESCRIPTION_STR;
}

// Return a string identifying the version of the vendor's device driver.
const char *AditofAdaptor::getDriverVersion() const {
    return aditof::DRIVER_VERSION_STR;
}

// Return the width of the frame, in pixels, as defined by the video format
// specified.
int AditofAdaptor::getMaxWidth() const {

    // Get the format information object.
    AditofDeviceFormat *formatInfo = getFormatInfo();

    if (formatInfo) {
        // Return the image width information stored in the AditofDeviceFormat
        // object.
        return formatInfo->getFormatWidth();
    } else {
        // If no format information is found because the device is configured to
        // use the camera file, query the SDK to determine the image width.
        // Since no SDK is used in this demo, it just returns a made-up
        // number.
        return aditof::DEMO_DEFAULT_WIDTH;
    }
}

// Return the height of the frame, in pixels, as defined by the video format
// specified.
int AditofAdaptor::getMaxHeight() const {

    // Get the format information object.
    AditofDeviceFormat *formatInfo = getFormatInfo();

    if (formatInfo) {
        // Return the image height information stored in the AditofDeviceFormat
        // object.
        return formatInfo->getFormatHeight();
    } else {
        // If no format information is found because the device is configured to
        // use the camera file, query the SDK to determine the image height.
        // Since no SDK is used in this demo, it just returns a made-up
        // number.
        return aditof::DEMO_DEFAULT_HEIGHT;
    }
}

imaqkit::frametypes::FRAMETYPE AditofAdaptor::getFrameType() const {

    /*    // Get the format information object.
        AditofDeviceFormat *formatInfo = getFormatInfo();

        if (formatInfo) {
            // Return the frame type stored in the AditofDeviceFormat object.
            return formatInfo->getFormatFrameType();
        } else {If*/
    // no format information is found because the device is configured to
    // use the camera file, query the SDK to determine the image frame type.
    // Since no SDK is used in this demo, it just returns a common frame
    // type.
    return imaqkit::frametypes::BGR24_PACKED;
    //    }
}

bool AditofAdaptor::isAcquisitionActive(void) const {
    std::unique_ptr<imaqkit::IAutoCriticalSection> acquisitionActiveSection(
        imaqkit::createAutoCriticalSection(m_acquisitionActiveGuard, true));
    return m_acquisitionActive;
}

void AditofAdaptor::setMode(int16_t mode) {

    if (m_currentMode == mode) {
        return;
    }

    if (!m_camera) {
        return;
    }

    switch (mode) {
    case aditof::MODE_NEAR_ID:
        m_camera->setMode("near");
        break;
    case aditof::MODE_MEDIUM_ID:
        m_camera->setMode("medium");
        break;
    case aditof::MODE_FAR_ID:
        m_camera->setMode("far");
        break;
    }

    m_currentMode = mode;
}

void AditofAdaptor::setDisplayedFrameType(int16_t type) {
    if (m_currentDisplayedType == type) {
        return;
    }

    switch (type) {
    case aditof::FRAME_TYPE_DEPTH_ID:
        break;
    case aditof::FRAME_TYPE_IR_ID:
        break;
    }

    m_currentDisplayedType = type;
}

void AditofAdaptor::setSmallSignalValue(int16_t value) {
    const size_t REGS_CNT = 5;
    uint16_t afeRegsAddr[REGS_CNT] = {0x4001, 0x7c22, 0xc34a, 0x4001, 0x7c22};
    uint16_t afeRegsVal[REGS_CNT] = {0x0006, 0x0004, 0, 0x0007, 0x0004};

    afeRegsVal[2] |= value;

    //    if (m_smallSignal) {
    afeRegsVal[2] |= 0x8000; // enable disable property
    //    }
    // TO DO: This breaks things over USB. Works well on the target and
    // over ethernet.
    if (m_camera) {
        m_camera->getDevice()->writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
    }
}

int AditofAdaptor::getCurrentHwRange() {
    aditof::CameraDetails cameraDetails;

    if (!m_camera) {
        return 0;
    }

    m_camera->getDetails(cameraDetails);

    return cameraDetails.range;
}

std::shared_ptr<aditof::Frame> AditofAdaptor::getFrameFromHwDevice() {
    if (!m_camera) {
        return nullptr;
    }

    auto frame = std::make_shared<aditof::Frame>();
    m_camera->requestFrame(frame.get());

    return frame;
}

float AditofAdaptor::readAfeTemp() {
    float afeTemp = 0.0;

    if (!m_camera) {
        return 0.0;
    }

    m_camera->getDevice()->readAfeTemp(afeTemp);
    return afeTemp;
}

float AditofAdaptor::readLaserTemp() {
    float laserTemp = 0.0;

    if (!m_camera) {
        return 0.0;
    }

    m_camera->getDevice()->readLaserTemp(laserTemp);
    return laserTemp;
}

void AditofAdaptor::setAcquisitionActive(bool state) {
    std::unique_ptr<imaqkit::IAutoCriticalSection> acquisitionActiveSection(
        imaqkit::createAutoCriticalSection(m_acquisitionActiveGuard, true));
    m_acquisitionActive = state;
}

// Return the number of bands of data returned: RGB is 3, monochrome is 1.
int AditofAdaptor::getNumberOfBands() const {

    // Get the format information object.
    AditofDeviceFormat *formatInfo = getFormatInfo();

    if (formatInfo) {
        // Return the number of color bands stored in the AditofDeviceFormat
        // object.
        return formatInfo->getFormatNumBands();
    } else {
        // If no format information is found because the device is configured to
        // use the camera file, query the SDK to determine the image width.
        // Since no SDK is used in this demo, it just returns a made-up
        // number.
        return aditof::DEMO_DEFAULT_NUMBER_OF_BANDS;
    }
}

// Set up the device for acquisition.
bool AditofAdaptor::openDevice() {
    // Check if the device is already opened.
    // If it is, then nothing else needs to be done.
    if (isOpen())
        return true;

#ifdef _WIN32
    // Create a thread for capturing images from the image acquisition
    // device and sending those images back to the IMAQ engine.
    _sendThread = CreateThread(NULL, 0, sendThread, this, 0, &_sendThreadID);

    // Check that the thread was successfully created.
    if (_sendThread == NULL) {
        return false;
    }

    // Post a "dummy" message to ensure that the thread has been created.
    // PostThreadMessage returns 0 if it fails to successfully post a message.
    // If it fails, the thread probably hasn't been created yet, so
    // keep trying until the message is successfully posted.
    while (PostThreadMessage(_sendThreadID, WM_USER + 1, 0, 0) == 0) {
        Sleep(1);
    }
#endif

    return true;
}

//  Engine calls this method to start an acquisition.
bool AditofAdaptor::startCapture() {

    // Check if the device is opened.
    // Acquisition is not possible if the device is not already opened.
    if (!isOpen())
        return false;

    // ************************************************
    // * CONFIGURE THE IMAGE ACQUISITION DEVICE PROPERTIES.
    // Add code here to call into the image acquisition device's SDK
    // to configure the device settings (ie. video format, frame rate, et
    // cetera).
    // ************************************************
    if (m_camera) {
        m_camera->start();
    }
    // Configure device-specific properties by calling notifyAllListeners()
    // to invoke all property listener objects. The property listeners
    // notify() method is responsible for configuring the image acquisition
    // device properties.
    getEngine()->getAdaptorPropContainer()->notifyAllListeners();

    // ************************************************
    // * IF NECESSARY, TURN ON THE IMAGE ACQUISITION DEVICE FOR ACQUISITION
    // Add code here to initialize device to start getting data or initialize
    // acquisition.
    // ************************************************

    // Set the acquiring frames flag.
    setAcquisitionActive(true);

#ifdef _WIN32
    // Post message to sendThread() to tell it to begin capturing images and
    // sending those images back to the IMAQ engine.
    PostThreadMessage(_sendThreadID, WM_USER, 0, 0);
    return true;
#else
    int errcode = pthread_create(&m_sendThread, NULL, this->sendThread, this);
    return (errcode == 0);
#endif
}

// The startCapture() method posts a message to this ThreadProc to see if
// the acquisition is complete. This method calls the
// imaqkit::IAdaptor::isAcquisitionNotComplete method to see if the requested
// number of frames have been acquired.
ThreadReturnType CALLING_CONVENTION AditofAdaptor::sendThread(void *param) {

    AditofAdaptor *adaptor = reinterpret_cast<AditofAdaptor *>(param);

#ifdef _WIN32
    // Set the thread priority.
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    // If message is not WM_QUIT, then it is a non-zero value.
    // Continue the while loop until a WM_QUIT message is received.
    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0) > 0) {

        switch (msg.message) {
        case WM_USER:
            sendFrame(adaptor);
            break;
        } // switch-case WM_USER

    } // while message is not WM_QUIT
    return 0;
#else
    while (adaptor->isAcquisitionActive()) {
        sendFrame(adaptor);
    }

    return NULL;
#endif
}

void AditofAdaptor::sendFrame(AditofAdaptor *adaptor) {

    std::unique_ptr<imaqkit::IAutoCriticalSection> driverSection(
        imaqkit::createAutoCriticalSection(adaptor->m_driverGuard, false));

    // Initialize the offset for the image data generated by this
    // demo adaptor to 0. Increasing the offset makes the image
    // stream from right to left.
    int offset = 0;

    // Since the demo adaptor generates images inside the acquisition
    // while loop below, save the image format information
    // here to make it more efficient.
    int imageWidth = adaptor->getMaxWidth();
    int imageHeight = adaptor->getMaxHeight();

    while (adaptor->isAcquisitionNotComplete() &&
           adaptor->isAcquisitionActive()) {
        driverSection->enter();
        // **************************************************************
        // * IF NECESSARY, CONFIGURE THE HARDWARE TRIGGER.
        // If the adaptor supports external triggering, add the
        // following code to check the trigger mode requested by the
        // user.

        // Check if the trigger mode is set to 'hardware'.
        if (adaptor->useHardwareTrigger()) {
            // Add code here to configure the image acquisition
            // device for hardware triggering.
        }
        // **************************************************************

        // **************************************************************
        // * GRAB IMAGE DATA FROM THE IMAGE ACQUISITION DEVICE
        // Add code here to grab a frame from the image
        // acquisition device.
        //
        // Normally the adaptor queries the SDK to return image
        // data from the imaging device. However, since this demo
        // adaptor only simulates what an adaptor would normally do
        // and no imaging device is used, it has to generate
        // the image data.
        // The code to generate image data is demo adaptor specific
        // and shouldn't used by your adaptor. Instead, adaptors
        // should be able to query their SDK for image data.

        // Create a buffer for storing the frame.
        size_t bufferSize = imageWidth * imageHeight;
        unsigned char *imBuffer = NULL;
        try {
            imBuffer = new unsigned char[bufferSize];
        } catch (const std::bad_alloc &) {
            imaqkit::adaptorError(adaptor, "imaq:demo:badFrameAlloc",
                                  "Unable to allocate memory for frame.");
            // Because we are not on the MATLAB thread, we need to return to
            // avoid continuing further.
            return;
        }

        auto frame = adaptor->getFrameFromHwDevice();
        int currentRange = adaptor->getCurrentHwRange();

        imaqkit::frametypes::FRAMETYPE frameType;

        if (frame) {
            if (adaptor->m_currentDisplayedType == aditof::FRAME_TYPE_IR_ID) {
                uint16_t *data = nullptr;
                frame->getData(aditof::FrameDataType::IR, &data);
                for (int i = 0; i < imageHeight * imageWidth; ++i) {
                    uint16_t value = (data[i] * (255.0 / currentRange));
                    imBuffer[i] =
                        static_cast<uint8_t>(value <= 255 ? value : 255);
                }

                frameType = imaqkit::frametypes::MONO8;
            } else {
                uint16_t *data = nullptr;
                frame->getData(aditof::FrameDataType::DEPTH, &data);
                for (int i = 0; i < imageHeight * imageWidth; ++i) {
                    uint16_t value = (data[i] * (255.0 / currentRange));
                    imBuffer[i] =
                        static_cast<uint8_t>(value <= 255 ? value : 255);
                }

                uint8_t *displayedImage =
                    new uint8_t[3 * imageWidth * imageHeight];

                /*
                 * Copy 3 bytes corresponding to the colormaped value of
                 * imBuffer at index i for each value in imBuffer
                 * */
                for (size_t i = 0; i < bufferSize; ++i) {
                    memcpy(displayedImage + i * 3, &colormap[imBuffer[i] * 3],
                           3);
                }

                delete[] imBuffer;

                imBuffer = displayedImage;

                frameType = imaqkit::frametypes::BGR24_PACKED;
            }
        } else {
            for (int i = 0; i < imageHeight * imageWidth; ++i) {
                imBuffer[i] = 123;
            }
        }

        offset += aditof::IMAGE_DATA_OFFSET;

        // **************************************************************

        // Check if the adaptor needs to send the frame back to the IMAQ engine.
        if (adaptor->isSendFrame()) {

            // Get the region of interest information.
            int roiOriginX, roiOriginY, roiWidth, roiHeight;
            adaptor->getROI(roiOriginX, roiOriginY, roiWidth, roiHeight);

            // Create an imaqkit::IAdaptorFrame object for storing the image
            // data.
            imaqkit::IAdaptorFrame *frame =
                adaptor->getEngine()->makeFrame(frameType,  // frametype
                                                roiWidth,   // width
                                                roiHeight); // height

            // Put the image data and format information into the IAdaptorFrame
            // object.
            frame->setImage(imBuffer, imageWidth, imageHeight, roiOriginX,
                            roiOriginY);
            // frame->setImage(imBuffer, roiWidth, roiHeight, 0, 0);

            // Set the image's timestamp to be the current time.
            frame->setTime(imaqkit::getCurrentTime());

            // Send the IAdaptorFrame object back to the IMAQ engine.
            adaptor->getEngine()->receiveFrame(frame);
        } // if isSendFrame()

        // Increment the frame count.
        adaptor->incrementFrameCount();

        // Cleanup. Deallocate imBuffer.
        delete[] imBuffer;
        //        delete[] displayedImage;
        driverSection->leave();
    } // while(isAcquisitionNotComplete() && adaptor->isAcquisitionActive())
}

//  Engine calls this method to stop an acquisition.
bool AditofAdaptor::stopCapture() {

    // Check if the device is acquiring data.
    // If the device is not acquiring data then nothing else needs to be done.
    if (!isAcquiring())
        return true;

    // Set the acquire frame flag to false to stop the while loop in
    // sendThread().
    setAcquisitionActive(false);

#ifndef _WIN32
    void *status;
    int errcode = pthread_join(m_sendThread, &status);
#endif

    // Enter the critical section to ensure that you exit the sendThread()'s
    // while loop before continuing.
    std::unique_ptr<imaqkit::IAutoCriticalSection> driverSection(
        imaqkit::createAutoCriticalSection(m_driverGuard, true));

    // *********************************************************************
    // * IF NECESSARY, STOP THE DEVICE FROM ACQUIRING DATA.
    // *********************************************************************
    // Add code here to stop the image acquisition device from acquiring data.
    // *********************************************************************

    if (m_camera) {
        m_camera->stop();
    }

    // Leave the critical section.
    driverSection->leave();

    return true;
}

// Terminate the threads used for acquisition
bool AditofAdaptor::closeDevice() {

    // Check if the device is opened.
    // If it is not opened, then nothing more needs to be done.
    if (!isOpen())
        return true;

#ifdef _WIN32
    // Terminate and close image the send thread.
    if (_sendThread) {
        // Post an WM_QUIT message to sendThread() thread.
        PostThreadMessage(_sendThreadID, WM_QUIT, 0, 0);

        // Give the thread a chance to finish.
        WaitForSingleObject(_sendThread, demo::SINGLE_OBJECT_WAIT_TIME);

        // Close sendThread() thread handle.
        CloseHandle(_sendThread);
        _sendThread = NULL;
    }
#else
#endif

    return true;
}

// Utility function used to get the device format object.
AditofDeviceFormat *AditofAdaptor::getFormatInfo() const {

    // First get the specified format's imaqkit::IDeviceFormat object from the
    // imaqkit::IDeviceInfo object, using the getDeviceFormat() method.
    imaqkit::IDeviceFormat *selectedFormat =
        m_di->getDeviceFormat(m_formatName);

    // Return the specified format's AditofDeviceFormat object. This object
    // is stored in the imaqkit::IDeviceFormat object. Use the getAdaptorData()
    // method of the imaqkit::IDeviceFormat object.
    // If the user specified a camera file,  no format has been saved.
    if (selectedFormat) {
        return dynamic_cast<AditofDeviceFormat *>(
            selectedFormat->getAdaptorData());
    } else {
        return NULL;
    }
}
