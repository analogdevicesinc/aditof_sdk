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
#ifndef __DEMO_ADAPTOR_HEADER__
#define __DEMO_ADAPTOR_HEADER__
#include "device_format.h"
#include <mwadaptorimaq.h>

#ifdef _WIN32
#include <windows.h>
#define CALLING_CONVENTION WINAPI
typedef DWORD ThreadReturnType;
typedef HANDLE Thread;
#else
#include <pthread.h>
#define CALLING_CONVENTION
typedef void *ThreadReturnType;
typedef pthread_t Thread;
#endif

#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/system.h>

/**
 * Class SourceAdaptor
 *
 * @brief:  Defines adaptor methods expected by engine.
 *
 */
class SourceAdaptor : public imaqkit::IAdaptor {

  public:
    // *******************************************************************
    // CONSTRUCTOR/DESTRUCTOR
    // *******************************************************************
    // Constructor accepts three arguments:
    //  -  Handle to the imaqkit::IEngine object
    //  -  Handle to one of the imaqkit::IDeviceInfo objects you created in
    //  getAvailHW()
    //  -  Character string specifying the device format or the full path
    //     of a device configuration file (camera file).
    //
    //  The engine passes these arguments to the adaptor in the
    //  createInstance() routine.
    SourceAdaptor(imaqkit::IEngine *engine,
                  const imaqkit::IDeviceInfo *deviceInfo,
                  const char *formatName,
                  std::shared_ptr<aditof::Camera> camera);
    virtual ~SourceAdaptor();

    // *******************************************************************
    // IMAGE ACQUISITION/CAPTURE METHODS
    // *******************************************************************
    /**
     * openDevice: Opens the device and prepares it for acquisition.
     *
     * Adaptors typically check to make sure the device isn't already in use.
     * You cannot create a video input object for a device that is in use.
     * This method also creates the ProcThread used to perform the actual
     * acquiring of image data.
     *
     * @return bool: TRUE if successful, otherwise FALSE.
     */
    virtual bool openDevice();

    /**
     * startCapture: Start image acquisition and capture.
     *
     * Adaptors typically configure the video format, mode, and frame rate
     * settings on the device, if they are not already configured. This method
     * also notifies all property listeners to make sure properties have
     * the correct values. If a property is not correctly configured, the
     * property listener should automatically configure it.
     *
     * If necessary, this method should call the SDK to startup the device for
     * acquisition. After verifying that the device is setup properly and
     * acquiring data, this method posts a WM_USER message to the acquisition
     * ProcThread to begin grabbing image data from the device.
     *
     * startCapture() is called after openDevice() finishes.
     *
     * @return bool: TRUE if successful, otherwise FALSE.
     */
    virtual bool startCapture();

    /**
     * stopCapture: Stop image acquisition and capture.
     *
     * Stop the while loop in the ProcThread, sendThread(), from grabbing and
     * returning any more data. One way to do this, illustrated by the demo
     * adaptor, is to set the acquisitionActive state to false.
     * Once the ProcThread stops grabbing image data, you should, if you can,
     * call the SDK to stop the device from acquiring frames.
     *
     * After stopCapture() finishes, closeDevice() is called.
     *
     * @return bool: TRUE if successful, otherwise FALSE.
     */
    virtual bool stopCapture();

    /**
     * closeDevice: Close the device.
     *
     * The ProcThread should be at an idle state and be able to recieve thread
     * messages. Terminate the acquisition ProcThread by posting a
     * WM_QUIT message to that thread.
     *
     * @return bool: TRUE if successful, otherwise FALSE.
     */
    virtual bool closeDevice();

    // *******************************************************************
    // ADAPTOR-SPECIFIC INFORMATION
    // *******************************************************************
    /**
     * getDriverDescription: Return a text string that identifies the device
     * driver used by the device.
     *
     * @return const char*: Driver description string.
     */
    virtual const char *getDriverDescription() const;

    /**
     * getDriverVersion: Return a text string that identifies the version
     * number of the device driver being used.
     *
     * @return const char*: Device driver version number string.
     */
    virtual const char *getDriverVersion() const;

    // *******************************************************************
    // RESOLUTION INFORMATION
    // *******************************************************************
    /**
     * getWidth: Get the image frame width in pixels. This does not return the
     * adjusted ROI image width.
     *
     * @return int: The number of pixels in each line for the current image
     * setting.
     */
    virtual int getMaxWidth() const;

    /**
     * getHeight: Get the number of lines in the current image. This does not
     * return the adjusted ROI image height.
     *
     * @return int: The number of lines for the current image setting.
     */
    virtual int getMaxHeight() const;

    /**
     * getNumberOfBands: Get the number of color bands in the current image
     * setting. Monochrome video will return 1 band; all others will
     * return 3 bands.
     *
     * @return int: The number of color bands for the current image setting.
     */
    virtual int getNumberOfBands() const;

    /**
     * getFrameType: Get the frame type of the current image setting.
     *
     * @return FRAMETYPE: Frame type of the current image setting.
     */
    virtual imaqkit::frametypes::FRAMETYPE getFrameType() const;

    /**
     * isAcquisitionActive: Indicates if startCapture() has been called.
     *
     * This function indicates to the acquisition thread, which is started
     * in openDevice(), that startCapture() has been called.  This is the
     * flag to indicate when to start sending frames to the engine.
     *
     * @return bool: True is startCapture() has been called, false once
     * stopCapture() is called.
     */
    bool isAcquisitionActive(void) const;

    void setMode(int16_t mode);

    std::pair<int, int> getCurrentHwRange() const;

    int getCurrentBitCount() const;

    void setDisplayedFrameType(int16_t type);

    void setSmallSignalValue(int16_t value);

    std::string readTemp() const;

  private:
    std::shared_ptr<aditof::Frame> getFrameFromHwDevice();

    /**
     * setAcquisitionActive: Indicate whether or not to acquire images.
     *
     * This function is called by startCapture() with an argument of true to
     * indicate to the acquisition thread that frames should be sent to the
     * engine.  It is called by stopCapture() to indicate that no more frames
     * should be acquired and that the acquisition thread can exit.
     *
     * @param state: True if acquisition should take place, otherwise false.
     */
    void setAcquisitionActive(bool state);

    // *******************************************************************
    // CAMERA SETUP AND INITIALIZATION METHOD
    // *******************************************************************
    /**
     * initDevice(): Perform any initialization or configuration required by
     * the device. This could include selecting the device, performing any
     * necessary initialization of the selected camera, and setting up property
     * listeners.
     * initDevice() is invoked in the SourceAdaptor constructor.
     *
     * @return void:
     */
    void initDevice();

    /**
     * getFormatInfo: Return the image acquisition device's DeviceFormat
     * object. The DeviceFormat object contains image width, image height,
     * number of color bands, and colorspace information.
     *
     * @return DeviceFormat: The device format information object.
     */
    DeviceFormat *getFormatInfo() const;

    /**
     * sendThread: The ThreadProc responsible for acquiring frames from the
     * device and sending those frames back to the engine.
     *
     * @param param: Parameter value to be cast to DeviceFormat.
     *
     * @return ThreadReturnType:
     */
    static ThreadReturnType CALLING_CONVENTION sendThread(void *param);

    static void sendFrame(SourceAdaptor *adaptor);

    // ************************************************************************
    // CLASS DATA MEMBERS
    // ************************************************************************

    /// Handle to the engine property container.
    imaqkit::IEnginePropContainer *m_enginePropContainer;

    /// Handle to imaqkit::IDeviceFormat object.
    const imaqkit::IDeviceInfo *m_di;

    /// The driver critical section.
    imaqkit::ICriticalSection *m_driverGuard;

    /// The acquisition active critical section.
    imaqkit::ICriticalSection *m_acquisitionActiveGuard;

    /// Handle to the thread.
    Thread m_sendThread;

#ifdef _WIN32
    /// Thread ID returned by Windows.
    DWORD _sendThreadID;
#endif

    /// Current video format name.
    const char *m_formatName;

    /// Flag to indicate whether the image acquisition engine is acquiring data
    /// from the device.
    bool m_acquisitionActive;

    std::shared_ptr<aditof::Camera> m_camera;

    int16_t m_currentMode;
    int16_t m_currentDisplayedType;
};

#endif
