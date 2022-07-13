/*
class Frame;
class DepthSensorInterface;
class StorageInterface;
class TemperatureSensorInterface;
*/

/*

class Camera3D_Smart : public aditof::Camera {
  public:
    Camera3D_Smart(
        std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
        std::shared_ptr<aditof::DepthSensorInterface> rgbSensor,
        std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms,
        std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
            &tSensors);

    Camera3D_Smart(
        std::shared_ptr<aditof::DepthSensorInterface> depthSensor,
        std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms,
        std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
            &tSensors);
    ~Camera3D_Smart();

  public: // implements Camera
    aditof::Status initialize() override;
    aditof::Status start() override;
    aditof::Status stop() override;
    aditof::Status setMode(const std::string &mode,
                           const std::string &modeFilename) override;
    aditof::Status
    getAvailableModes(std::vector<std::string> &availableModes) const override;
    aditof::Status setFrameType(const std::string &frameType) override;
    aditof::Status getAvailableFrameTypes(
        std::vector<std::string> &availableFrameTypes) const override;
    aditof::Status requestFrame(aditof::Frame *frame,
                                aditof::FrameUpdateCallback cb) override;
    aditof::Status getDetails(aditof::CameraDetails &details) const override;
    aditof::Status
    getAvailableControls(std::vector<std::string> &controls) const override;
    aditof::Status setControl(const std::string &control,
                              const std::string &value) override;
    aditof::Status getControl(const std::string &control,
                              std::string &value) const override;
    aditof::Status getImageSensors(
        std::vector<std::shared_ptr<aditof::DepthSensorInterface>> &sensors)
        override;
    aditof::Status
    getEeproms(std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms)
        override;
    aditof::Status getTemperatureSensors(
        std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>>
            &sensors) override;

  private:
    aditof::Status setNoiseReductionTreshold(uint16_t treshold);
    aditof::Status setIrGammaCorrection(float gamma);

#ifdef BAYER_CONVERSION
    float verticalKernel(uint8_t *pData, int x, int y, int width, int height);
    float horizontalKernel(uint8_t *pData, int x, int y, int width, int height);
    float plusKernel(uint8_t *pData, int x, int y, int width, int height);
    float crossKernel(uint8_t *pData, int x, int y, int width, int height);
    float directCopy(uint8_t *buffer, int x, int y, int width, int height);
    float getValueFromData(uint8_t *pData, int x, int y, int width, int height);
    void bayer2RGB(uint16_t *buffer, uint8_t *pData, int width, int height);
#endif

  private:
    aditof::CameraDetails m_details;
    std::shared_ptr<aditof::DepthSensorInterface> m_depthSensor;
    std::shared_ptr<aditof::DepthSensorInterface> m_rgbSensor;
    std::shared_ptr<aditof::DepthSensorInterface> m_rgbdSensor;
    std::shared_ptr<aditof::StorageInterface> m_eeprom;
    std::shared_ptr<aditof::TemperatureSensorInterface> m_temperatureSensor;
    bool m_devStarted;
    bool m_devProgrammed;
    bool m_eepromInitialized;
    bool m_tempSensorsInitialized;
    std::vector<std::string> m_availableControls;
    Calibration3D_Smart m_calibration;
    uint16_t m_noiseReductionThreshold;
    float m_irGammaCorrection;
    bool m_depthCorrection;
    bool m_cameraGeometryCorrection;
    bool m_cameraBayerRgbConversion;
    bool m_cameraDistortionCorrection;
    bool m_irDistorsionCorrection;
    std::string m_revision;
    std::vector<aditof::FrameDetails> m_rgbdFrameTypes;
    float m_Rw;
    float m_Gw;
    float m_Bw;
};

*/


class CameraDetails {
    connection

}

const EEPROM_NAME = '24c1024';

class Camera3D_Smart {
    // aditof::CameraDetails 
    m_details = new CameraDetails();

    // std::shared_ptr<aditof::DepthSensorInterface> 
    m_depthSensor;

    // std::shared_ptr<aditof::DepthSensorInterface> 
    m_rgbSensor;

    // std::shared_ptr<aditof::DepthSensorInterface> 
    m_rgbdSensor;

    // std::shared_ptr<aditof::StorageInterface> 
    m_eeprom;

    // std::shared_ptr<aditof::TemperatureSensorInterface> 
    m_temperatureSensor;

    // bool 
    m_devStarted;

    // bool 
    m_devProgrammed;

    // bool 
    m_eepromInitialized;

    // bool 
    m_tempSensorsInitialized;

    // std::vector<std::string> 
    m_availableControls;

    // Calibration3D_Smart 
    m_calibration;

    // uint16_t 
    m_noiseReductionThreshold;

    // float 
    m_irGammaCorrection;

    // bool 
    m_depthCorrection;

    // bool 
    m_cameraGeometryCorrection;

    // bool 
    m_cameraBayerRgbConversion;

    // bool 
    m_cameraDistortionCorrection;

    // bool 
    m_irDistorsionCorrection;

    // std::string 
    m_revision;

    // std::vector<aditof::FrameDetails> 
    m_rgbdFrameTypes;

    // float 
    m_Rw;

    // float 
    m_Gw;

    // float 
    m_Bw;



    // constructor(depthSensor, rgbSensor, eeproms, tSensors){
    constructor(rgbdSensor, eeproms, tSensors) {
        this.m_rgbdSensor = rgbdSensor;
        this.m_devStarted = false;
        this.m_devProgrammed = false;
        this.m_eepromInitialized = false;
        this.m_tempSensorsInitialized = false;
        // this.m_availableControls = availableControls;
        this.m_depthCorrection = true;
        this.m_cameraGeometryCorrection = true;
        this.m_cameraDistortionCorrection = true;
        this.m_irDistorsionCorrection = false;
        this.m_revision = "RevA";


        this.m_Rw = 255.0 * 0.25;
        this.m_Gw = 255.0 * 0.35;
        this.m_Bw = 255.0 * 0.25;

        // Check Depth Sensor
        if (!rgbdSensor) {
            console.log('WARNING: Invalid instance of a depth sensor');
            return;
        }
        // aditof::SensorDetails 
        let sDetails = this.m_rgbdSensor.getDetails();
        this.m_details.connection = sDetails.connectionType;

        // Look for EEPROM
        let eeprom_exists = false;
        console.log(typeof eeproms[0]) //NetworkStorage
        for (let e in eeproms) {
            console.log('getName(): ', e.m_name);
            if (e.getName() === EEPROM_NAME) {
                eeprom_exists = true;
                this.m_eeprom = e;
            }
        }

        if (eeprom_exists === false) {
            console.log('WARNING: Could not find ', EEPROM_NAME, ' while looking for storage for camera '); //skCameraName
            return;
        }

        // Look for the temperature sensor
        let tSensor_exists = false;
        for (let t in tSensors) {
            if (t.getName() == TEMPERATURE_SENSOR_NAME) {
                tSensor_exists = true;
                this.m_temperatureSensor = t;
            }
        }
        if (tSensor_exists == false) {
            console.log('WARNING: Could not find ', TEMPERATURE_SENSOR_NAME, ' while looking for temperature sensors for camera ', skCameraName);
            return;
        }
    }

    // aditof::Status initialize() override;
    initialize() {
        let status;

        console.log('INFO: Initializing camera');

        if (!this.m_rgbdSensor || !this.m_eeprom || !this.m_temperatureSensor) {

            console.log('WARNING: Failed to initialize! Not all sensors are available');
            return Status.GENERIC_ERROR;
        }

        // Open communication with the depth sensor
        status = this.m_rgbdSensor.open();
        if (status != Status.OK) {
            console.log('WARNING: Failed to open device');
            return status;
        }

        let handle;
        [status, handle] = this.m_rgbdSensor.getHandle();
        if (status != Status.OK) {
            console.log('ERROR: Failed to obtain the handle');
            return status;
        }

        this.m_details.bitCount = 12;

        // Open communication with EEPROM
        [status, handle] = this.m_eeprom.open();
        if (status != Status.OK) {
            let name = this.m_eeprom.getName();
            console.log('ERROR: Failed to open EEPROM with name ', name);
            return status;
        }
        this.m_eepromInitialized = true;

        // Open communication with temperature sensor
        status = this.m_temperatureSensor.open(handle);
        if (status != Status.OK) {
            let name = this.m_temperatureSensor.getName();
            console.log('ERROR: Failed to open temperature sensor with name', name);
            return status;
        }
        this.m_tempSensorsInitialized = true;

        // Read the camera's serial number from eeprom
        let eepromSerial = Array(EEPROM_SERIAL_LENGHT);
        status = this.m_eeprom.read(EEPROM_SERIAL_ADDR, eepromSerial, EEPROM_SERIAL_LENGHT);
        if (status != Status.OK) {
            console.log('ERROR: Failed to read serial from eeprom');
            return status;
        }

        // this.m_details.cameraId = std::string((char *)(eepromSerial), EEPROM_SERIAL_LENGHT);
        this.m_details.cameraId = eepromSerial;
        console.log('INFO: Camera ID: ', this.m_details.cameraId);

        status = this.m_calibration.readCalMap(this.m_eeprom);
        if (status != Status.OK) {
            console.log('WARNING: Failed to read calibration data from eeprom');
            return status;
        }

        this.m_calibration.getIntrinsic(INTRINSIC, m_details.intrinsics.cameraMatrix);
        this.m_calibration.getIntrinsic(DISTORTION_COEFFICIENTS, m_details.intrinsics.distCoeffs);

        // For now we use the unit cell size values specified in the datasheet
        this.m_details.intrinsics.pixelWidth = 0.0056;
        this.m_details.intrinsics.pixelHeight = 0.0056;

        // Cache the frame types provided by Depth and RGB sensors
        status = this.m_rgbdSensor.getAvailableFrameTypes(this.m_rgbdFrameTypes);
        if (status != Status.OK) {
            console.log('WARNING: Failed to get the depth sensor frame types');
            return status;
        }

        console.log('INFO: Camera initialized');

        return Status.OK;
    }

    // aditof::Status start() override;
    start() {
        let status;

        status = this.m_rgbdSensor.start();
        if (status != Status.OK) {
            console.log('ERROR: Failed to start the rgbd sensor');
            return status;
        }
        this.m_devStarted = true;
        return Status.OK;
    }

    // aditof::Status stop() override;
    stop() {
            let status;
            status = this.m_rgbdSensor.stop();
            if (status != Status.OK) {
                console.log('ERROR: Failed to stop the rgbd sensor');
                return status;
            }
            this.m_devStarted = false;

            return Status.OK;
        }
        // aditof::Status setMode(const std::string &mode, const std::string &modeFilename) override;
    setMode(mode, modeFilename) {
        let status = Status.OK;

        // Set the values specific to the Revision requested
        // std::array<rangeStruct, 3> rangeValues = RangeValuesForRevision.at(m_revision);
        let rangeValues;

        console.log('INFO: Chosen mode: ', mode);

        let mode_found = false;
        for (let rm in rangeValues) {
            if (rm.mode == mode) {
                mode_found = true;
                this.m_details.depthParameters.maxDepth = rm.maxDepth;
                this.m_details.depthParameters.minDepth = rm.minDepth;
                break;
            }
        }
        if (!mode_found) {
            this.m_details.depthParameters.maxDepth = 1;
        }


        console.log('INFO: Camera range for mode: ', mode, ' is: ', this.m_details.depthParameters.minDepth, ' mm and ', this.m_details.depthParameters.maxDepth, ' mm');

        if (!this.m_devProgrammed) {
            let firmwareData;
            status = this.m_calibration.getAfeFirmware(mode, firmwareData);
            if (status != Status.OK) {
                console.log('WARNING: Failed to read firmware from eeprom');
                return Status.UNREACHABLE;
            } else {
                console.log('INFO: Found firmware for mode: ', mode);
            }

            console.log('INFO: Firmware size: ', firmwareData.size() * 2, ' bytes');
            status = this.m_rgbdSensor.program(firmwareData.data(),
                2 * firmwareData.size());
            if (status != Status.OK) {
                console.log('WARNING: Failed to program AFE');
                return Status.UNREACHABLE;
            }
            this.m_devProgrammed = true;
        }

        status = this.m_calibration.setMode(this.m_rgbdSensor, mode,
            this.m_details.depthParameters.maxDepth,
            this.m_details.frameType.width,
            this.m_details.frameType.height);

        if (status != Status.OK) {
            console.log('WARNING: Failed to set calibration mode');
            return status;
        }

        // register writes for enabling only one video stream (depth/ ir)
        // must be done here after programming the camera in order for them to
        // work properly. Setting the mode of the camera, programming it
        // with a different firmware would reset the value in the 0xc3da register
        if (this.m_details.frameType.type == 'depth_rgb') {
            let afeRegsAddr = [0x4001, 0x7c22, 0xc3da, 0x4001, 0x7c22];
            let afeRegsVal = [0x0006, 0x0004, 0x03, 0x0007, 0x0004];
            this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
        } else if (this.m_details.frameType.type == 'ir_rgb') {
            let afeRegsAddr = [0x4001, 0x7c22, 0xc3da, 0x4001, 0x7c22];
            let afeRegsVal = [0x0006, 0x0004, 0x05, 0x0007, 0x0004];
            this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
        } else if (this.m_details.frameType.type == 'depth_ir_rgb') {
            let afeRegsAddr = [0x4001, 0x7c22, 0xc3da, 0x4001, 0x7c22];
            let afeRegsVal = [0x0006, 0x0004, 0x03, 0x0007, 0x0004];

            this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
        }

        if (mode == 'near') {
            this.m_details.depthParameters.depthGain = 0.5;
        } else {
            this.m_details.depthParameters.depthGain = 1.15;
        }
        this.m_details.depthParameters.depthOffset = 0.0;

        this.m_details.mode = mode;

        return status;
    }


    // aditof::Status getAvailableModes(std::vector<std::string> &availableModes) const override;
    getAvailableModes() {
        let status = Status.OK;
        let availableModes = [];
        // Dummy data. To remove when implementig this method
        availableModes.push('near');
        availableModes.push('medium');
        return [status, availableModes];
    }


    // aditof::Status setFrameType(const std::string &frameType) override;
    setFrameType(frameType) {
        let status = Status.OK;

        if (this.m_devStarted) {
            status = this.m_rgbdSensor.stop();
            if (status != Status.OK) {
                return status;
            }
            this.m_devStarted = false;
        }

        // std::vector<FrameDetails> 
        let detailsList = [];
        status = this.m_rgbdSensor.getAvailableFrameTypes(detailsList);
        if (status != Status.OK) {
            console.log('WARNING: Failed to get available frame types');
            return status;
        }

        let typeFound = false;
        let frameDetailsIt;
        for (let d in detailsList) {
            if (d.type == frameType) {
                typeFound = true;
                frameDetailsIt = d;
            }
        }

        if (typeFound == false) {
            console.log('WARNING: Frame type: ', frameType, ' not supported by camera');
            return Status.INVALID_ARGUMENT;
        }

        if (this.m_details.frameType != frameDetailsIt) {

            //Turn off the streaming of data to modify parameters in the driver.
            let afeRegsAddr = [0x4001, 0x7c22];
            let afeRegsVal = [0x0006, 0x0004];
            this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 2);

            status = this.m_rgbdSensor.setFrameType(frameDetailsIt);

            if (status != Status.OK) {
                console.log('WARNING: Failed to set frame type of the depth sensor');
                return status;
            }
            this.m_details.frameType.type = frameType;
            this.m_details.frameType.width = frameDetailsIt.width;
            this.m_details.frameType.height = frameDetailsIt.height;
            this.m_details.frameType.rgbWidth = frameDetailsIt.rgbWidth;
            this.m_details.frameType.rgbHeight = frameDetailsIt.rgbHeight;
            this.m_details.frameType.fullDataWidth = frameDetailsIt.fullDataWidth;
            this.m_details.frameType.fullDataHeight = frameDetailsIt.fullDataHeight;

            //Turn on the streaming of data.
            let afeRegsAddress = [0x4001, 0x7c22];
            let afeRegsValue = [0x0007, 0x0004];
            this.m_rgbdSensor.writeAfeRegisters(afeRegsAddress, afeRegsValue, 2);
        }

        if (!this.m_devStarted) {
            status = this.m_rgbdSensor.start();
            if (status != Status.OK) {
                return status;
            }
            this.m_devStarted = true;
        }

        return status;
    }

    // aditof::Status getAvailableFrameTypes(std::vector<std::string> &availableFrameTypes) const override;
    getAvailableFrameTypes() {
        let status = Status.OK;

        let availableFrameTypes = [];
        // std::vector<FrameDetails> 
        let frameDetailsList = [];
        status = this.m_rgbdSensor.getAvailableFrameTypes(frameDetailsList);
        if (status != Status.OK) {
            console.log('WARNING: Failed to get available frame types');
            return status;
        }

        for (let item in frameDetailsList) {
            availableFrameTypes.push(item.type);
        }

        return [status, availableFrameTypes];
    }

    // aditof::Status requestFrame(aditof::Frame *frame, aditof::FrameUpdateCallback cb) override;
    requestFrame(frame, cb) {
        let status = Status.OK;

        if (frame == null) {
            console.log('ERROR: Received pointer frame is null');
            return Status.INVALID_ARGUMENT;
        }

        // FrameDetails 
        let frameDetails;
        frameDetails = frame.getDetails();

        if (this.m_details.frameType.type != frameDetails.type) {
            frame.setDetails(this.m_details.frameType);
        }

        let fullDataLocation;
        frame.getData(FrameDataType.FULL_DATA, fullDataLocation);

        // aditof::BufferInfo 
        let rgbdBufferInfo;
        status = this.m_rgbdSensor.getFrame(fullDataLocation, rgbdBufferInfo);
        if (status != Status.OK) {
            console.log('WARNING: Failed to get frame from depth sensor');
            return status;
        }

        /*
            // #ifdef BAYER_CONVERSION
          
            //conversion for bayer to rgb
            let copyOfRgbData;
                // =(uint16_t *)malloc(m_details.frameType.rgbWidth *
                //                   m_details.frameType.rgbHeight * sizeof(uint16_t));

            std::memcpy(copyOfRgbData, rgbDataLocation,
                        m_details.frameType.rgbWidth * m_details.frameType.rgbHeight *
                            2);

            bayer2RGB(rgbDataLocation, (uint8_t *)copyOfRgbData,
                      m_details.frameType.rgbWidth, m_details.frameType.rgbHeight);
            free(copyOfRgbData);
            }
            // #endif
            */

        if (this.m_details.mode != skCustomMode && (this.m_details.frameType.type == 'depth_ir_rgb' || this.m_details.frameType.type == 'depth_rgb')) {

            let depthDataLocation;
            frame.getData(FrameDataType.DEPTH, depthDataLocation);

            let irDataLocation;
            frame.getData(FrameDataType.IR, irDataLocation);

            if (this.m_depthCorrection) {
                this.m_calibration.calibrateDepth(depthDataLocation, this.m_details.frameType.width * this.m_details.frameType.height);
            }
            if (this.m_cameraGeometryCorrection) {
                this.m_calibration.calibrateCameraGeometry(depthDataLocation, this.m_details.frameType.width * this.m_details.frameType.height);
            }
            if (this.m_cameraDistortionCorrection) {
                this.m_calibration.distortionCorrection(depthDataLocation, this.m_details.frameType.width, this.m_details.frameType.height);
            }
            if (this.m_irDistorsionCorrection) {
                this.m_calibration.distortionCorrection(irDataLocation, this.m_details.frameType.width, this.m_details.frameType.height);
            }
        }

        return [Status.OK, frame];
    }

    // aditof::Status getDetails(aditof::CameraDetails &details) const override;
    getDetails() {
        return [Status.OK, this.m_details];
    }

    // aditof::Status Camera3D_Smart::getImageSensors(
    getImageSensors() {
        let sensors = [this.m_rgbdSensor];
        return [Status.OK, sensors];
    }

    // aditof::Status Camera3D_Smart::getEeproms(std::vector<std::shared_ptr<aditof::StorageInterface>> &eeproms) {
    getEeproms() {
        let eeproms = [this.m_eeprom];
        return [Status.OK, eeproms];
    }

    // aditof::Status Camera3D_Smart::getTemperatureSensors(std::vector<std::shared_ptr<aditof::TemperatureSensorInterface>> &sensors) {
    getTemperatureSensors() {
        let sensors = [this.m_temperatureSensor];
        return [Status.OK, sensors];
    }

    // aditof::Status getAvailableControls(std::vector<std::string> &controls) const override;
    getAvailableControls(controls) {
        return [Status.OK, this.m_availableControls];
    }

    // aditof::Status setControl(const std::string &control, const std::string &value) override;
    setControl(control, value) {
        let status = Status.OK;

        let controlExists = false;
        for (let c in this.m_availableControls) {
            if (c == control) {
                controlExists = true;
            }
        }
        if (!controlExists) {
            console.log('WARNING: Unsupported control');
            return Status.INVALID_ARGUMENT;
        }

        if (control == 'noise_reduction_threshold') {
            return setNoiseReductionTreshold(value);
        }

        if (control == 'ir_gamma_correction') {
            return setIrGammaCorrection(value);
        }

        if (control == 'depth_correction') {
            this.m_depthCorrection = (value != 0);
        }

        if (control == 'camera_geometry_correction') {
            this.m_cameraGeometryCorrection = (value != 0);
        }

        if (control == 'bayer_rgb_conversion') {
            this.m_cameraBayerRgbConversion = (value != 0);
        }

        if (control == 'revision') {
            this.m_revision = value;
        }
        if (control == 'camera_distortion_correction') {
            this.m_cameraDistortionCorrection = (value != 0);
        }
        if (control == 'ir_distortion_correction') {
            this.m_irDistorsionCorrection = (value != 0);
        }
        return status;
    }

    // aditof::Status getControl(const std::string &control, std::string &value) const override;
    getControl(control, value) {
        let status = Status.OK;

        let controlExists = false;
        for (let c in this.m_availableControls) {
            if (c == control) {
                controlExists = true;
            }
        }
        if (!controlExists) {
            console.log('WARNING: Unsupported control');
            return Status.INVALID_ARGUMENT;
        }

        if (control == 'noise_reduction_threshold') {
            value = this.m_noiseReductionThreshold;
        }

        if (control == 'ir_gamma_correction') {
            value = this.m_irGammaCorrection;
        }

        if (control == 'depth_correction') {
            value = this.m_depthCorrection ? '1' : '0';
        }

        if (control == 'camera_geometry_correction') {
            value = this.m_cameraGeometryCorrection ? '1' : '0';
        }

        if (control == 'bayer_rgb_conversion') {
            value = this.m_cameraBayerRgbConversion ? '1' : '0';
        }
        if (control == 'revision') {
            value = this.m_revision;
        }

        if (control == 'camera_distortion_correction') {
            value = this.m_cameraDistortionCorrection ? '1' : '0';
        }

        if (control == 'ir_distorsion_correction') {
            value = this.m_irDistorsionCorrection ? '1' : '0';
        }

        return status;
    }

    // aditof::Status setNoiseReductionTreshold(uint16_t treshold);
    setNoiseReductionTreshold(treshold) {
        // if (this.m_details.mode.compare('far') == 0) {
        if (this.m_details.mode == 'far') {
            console.log('WARNING: Far mode does not support noise reduction!');
            return Status.UNAVAILABLE;
        }

        let REGS_CNT = 5;
        let afeRegsAddr = [0x4001, 0x7c22, 0xc34a, 0x4001, 0x7c22];
        let afeRegsVal = [0x0006, 0x0004, 0x8000, 0x0007, 0x0004];

        afeRegsVal[2] |= treshold;
        this.m_noiseReductionThreshold = treshold;

        return this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
    }

    // aditof::Status setIrGammaCorrection(float gamma);
    setIrGammaCorrection(gamma) {
        let status = Status.OK;
        let x_val = [256, 512, 768, 896, 1024, 1536, 2048, 3072, 4096];
        let y_val = Array(9);

        for (let i = 0; i < 9; i++) {
            y_val[i] = ((x_val[i] / 4096.0) ** gamma) * 1024.0;
        }

        let afeRegsAddr = [0x4001, 0x7c22, 0xc372, 0xc373, 0xc374, 0xc375, 0xc376, 0xc377, 0xc378, 0xc379, 0xc37a, 0xc37b, 0xc37c, 0xc37d, 0x4001, 0x7c22];
        let afeRegsVal = [0x0006, 0x0004, 0x7888, 0xa997, 0x000a, y_val[0], y_val[1], y_val[2], y_val[3], y_val[4], y_val[5], y_val[6], y_val[7], y_val[8], 0x0007, 0x0004];

        status = this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 8);
        if (status != Status.OK) {
            return status;
        }
        status = this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr + 8, afeRegsVal + 8, 8);
        if (status != Status.OK) {
            return status;
        }

        this.m_irGammaCorrection = gamma;

        return status;
    }

    // float verticalKernel(uint8_t *pData, int x, int y, int width, int height);
    verticalKernel(pData, x, y, width, height) {
            let sum = 0;
            let nr = 0;
            if ((x - 1) >= 0 && (x - 1) < height) {
                sum += getValueFromData(pData, x - 1, y, width, height);
                nr++;
            }
            if ((x + 1) >= 0 && (x + 1) < height) {
                sum += getValueFromData(pData, x + 1, y, width, height);
                nr++;
            }
            return (sum / (nr > 0 ? nr : 1));
        }
        // float horizontalKernel(uint8_t *pData, int x, int y, int width, int height);
    horizontalKernel(pData, x, y, width, height) {
            let sum = 0;
            let nr = 0;
            if ((y - 1) >= 0 && (y - 1) < width) {
                sum += getValueFromData(pData, x, y - 1, width, height);
                nr++;
            }
            if ((y + 1) >= 0 && (y + 1) < width) {
                sum += getValueFromData(pData, x, y + 1, width, height);
                nr++;
            }
            return (sum / (nr > 0 ? nr : 1));
        }
        // float plusKernel(uint8_t *pData, int x, int y, int width, int height);
    plusKernel(pData, x, y, width, height) {
            let sum = 0;
            let nr = 0;
            if ((x - 1) >= 0 && (x - 1) < height) {
                sum += getValueFromData(pData, x - 1, y, width, height);
                nr++;
            }
            if ((x + 1) >= 0 && (x + 1) < height) {
                sum += getValueFromData(pData, x + 1, y, width, height);
                nr++;
            }
            if ((y - 1) >= 0 && (y - 1) < width) {
                sum += getValueFromData(pData, x, y - 1, width, height);
                nr++;
            }
            if ((y + 1) >= 0 && (y + 1) < width) {
                sum += getValueFromData(pData, x, y + 1, width, height);
                nr++;
            }
            return (sum / (nr > 0 ? nr : 1));
        }
        // float crossKernel(uint8_t *pData, int x, int y, int width, int height);
    crossKernel(pData, x, y, width, height) {
            let sum = 0;
            let nr = 0;
            if ((x - 1) >= 0 && (x - 1) < height && (y - 1) >= 0 && (y - 1) < width) {
                sum += getValueFromData(pData, x - 1, y - 1, width, height);
                nr++;
            }
            if ((x + 1) >= 0 && (x + 1) < height && (y - 1) >= 0 && (y - 1) < width) {
                sum += getValueFromData(pData, x + 1, y - 1, width, height);
                nr++;
            }
            if ((x - 1) >= 0 && (x - 1) < height && (y + 1) >= 0 && (y + 1) < width) {
                sum += getValueFromData(pData, x - 1, y + 1, width, height);
                nr++;
            }
            if ((x + 1) >= 0 && (x + 1) < height && (y + 1) >= 0 && (y + 1) < width) {
                sum += getValueFromData(pData, x + 1, y + 1, width, height);
                nr++;
            }
            return (sum / (nr > 0 ? nr : 1));
        }
        // float directCopy(uint8_t *buffer, int x, int y, int width, int height);
    directCopy(buffer, x, y, width, height) {
            return getValueFromData(pData, x, y, width, height);
        }
        // float getValueFromData(uint8_t *pData, int x, int y, int width, int height);
    getValueFromData(pData, x, y, width, height) {
            return ((pData[x * width * 2 + y * 2 + 1] << 8) +
                    pData[x * width * 2 + y * 2]) /
                4095.0 * 255.0;
        }
        // void bayer2RGB(uint16_t *buffer, uint8_t *pData, int width, int height);
    bayer2RGB(buffer, pData, width, height) {
        //casting into 8 in order to work with it
        rgb = buffer;

        for (let i = 0; i < height; i++) {
            for (let j = 0; j < width; j++) {
                if (i % 2 == RED_START_POZ_Y && j % 2 == RED_START_POZ_X) { //red square
                    rgb[(i * width + j) * 3 + RED] = (directCopy(pData, i, j, width, height) * 255.0 / this.m_Rw);
                    rgb[(i * width + j) * 3 + GREEN] = (plusKernel(pData, i, j, width, height) * 255.0 / this.m_Gw);
                    rgb[(i * width + j) * 3 + BLUE] = (crossKernel(pData, i, j, width, height) * 255.0 / this.m_Bw);
                } else if (i % 2 == (RED_START_POZ_Y ^ 1) && j % 2 == (RED_START_POZ_X ^ 1)) { //blue square
                    rgb[(i * width + j) * 3 + RED] = (crossKernel(pData, i, j, width, height) * 255.0 / this.m_Rw);
                    rgb[(i * width + j) * 3 + GREEN] = (plusKernel(pData, i, j, width, height) * 255.0 / this.m_Gw);
                    rgb[(i * width + j) * 3 + BLUE] = (directCopy(pData, i, j, width, height) * 255.0 / this.m_Bw);
                } else if (i % 2 == (RED_START_POZ_Y ^ 1) && j % 2 == RED_START_POZ_X) { //green pixel, blue row
                    rgb[(i * width + j) * 3 + RED] = (verticalKernel(pData, i, j, width, height) * 255.0 / this.m_Rw);
                    rgb[(i * width + j) * 3 + GREEN] = (directCopy(pData, i, j, width, height) * 255.0 / this.m_Gw);
                    rgb[(i * width + j) * 3 + BLUE] = (horizontalKernel(pData, i, j, width, height) * 255.0 / this.m_Bw);
                } else if (i % 2 == RED_START_POZ_Y && j % 2 == (RED_START_POZ_X ^ 1)) { // green pixel, red row
                    rgb[(i * width + j) * 3 + RED] = (horizontalKernel(pData, i, j, width, height) * 255.0 / this.m_Rw);
                    rgb[(i * width + j) * 3 + GREEN] = (directCopy(pData, i, j, width, height) * 255.0 / this.m_Gw);
                    rgb[(i * width + j) * 3 + BLUE] = (verticalKernel(pData, i, j, width, height) * 255.0 / this.m_Bw);
                }
            }
        }
    }


}









//CAMERA_DEFINITIONS
/*

struct IntrinsicParameters {
    
     std::vector<float> cameraMatrix;

     
     std::vector<float> distCoeffs;
 
     
     float pixelWidth;
 
     
     float pixelHeight;
 };
 
 
 struct DepthParameters {
     
     float depthGain;
 
     
     float depthOffset;
 
     
     int maxDepth;
 
     
     int minDepth;
 };
 
 
 struct CameraDetails {
     
     std::string cameraId;
 
     
     std::string mode;
 
     
     FrameDetails frameType;
 
     
     ConnectionType connection;
 
     
     IntrinsicParameters intrinsics;
 
     
     DepthParameters depthParameters;
 
     
     int bitCount;
 };

*/






// Calibration3D_Smart
/*
class Calibration3D_Smart {
  public:
    Calibration3D_Smart();
    ~Calibration3D_Smart();

  public:
    aditof::Status readCalMap(std::shared_ptr<aditof::StorageInterface> eeprom);
    aditof::Status getAfeFirmware(const std::string &mode,
                                  std::vector<uint16_t> &data) const;
    aditof::Status getIntrinsic(float key, std::vector<float> &data) const;
    aditof::Status setMode(std::shared_ptr<aditof::DepthSensorInterface> sensor,
                           const std::string &mode, int range,
                           unsigned int frameWidth, unsigned int frameheight);
    aditof::Status calibrateDepth(uint16_t *frame, uint32_t frame_size);
    aditof::Status calibrateCameraGeometry(uint16_t *frame,
                                           uint32_t frame_size);
    aditof::Status distortionCorrection(uint16_t *frame, unsigned int width,
                                        unsigned int height);

  private:
    void buildDepthCalibrationCache(float gain, float offset,
                                    int16_t maxPixelValue, int range);
    void buildGeometryCalibrationCache(const std::vector<float> &cameraMatrix,
                                       unsigned int width, unsigned int height);
    void buildDistortionCorrectionCache(unsigned int width,
                                        unsigned int height);

  private:
    //! mode_struct - Structure to hold the packet consisting of map of parameters
   
      struct mode_struct {
        uint16_t pulse_cnt;
        uint8_t depth_pwr[48];
        uint16_t depth_x0;
        uint16_t depth_offset[49];
        uint16_t depth3;
        uint16_t depth2;
    };

  private:
    uint16_t *m_depth_cache;
    double *m_geometry_cache;
    double *m_distortion_cache;
    int m_range;
    mode_struct m_mode_settings[2];
    std::vector<float> m_intrinsics;
    std::vector<uint16_t> m_afe_code;
    bool m_cal_valid;
};
*/

class Calibration3D_Smart {

    // private:
    //   //! mode_struct - Structure to hold the packet consisting of map of parameters
    //   /*!
    //     mode_struct provides structure to hold the packet(sub map) of parameters
    // */
    //   struct mode_struct {
    //       uint16_t pulse_cnt;
    //       uint8_t depth_pwr[48];
    //       uint16_t depth_x0;
    //       uint16_t depth_offset[49];
    //       uint16_t depth3;
    //       uint16_t depth2;
    //   };
    mode_struct;

    // private:
    // uint16_t *
    m_depth_cache;
    // double *
    m_geometry_cache;
    // double *
    m_distortion_cache;
    // int 
    m_range;
    // mode_struct 
    m_mode_settings = Array(2);
    // std::vector<float> 
    m_intrinsics = [];
    // std::vector<uint16_t> 
    m_afe_code = [];
    // bool 
    m_cal_valid;

    constructor() {
        this.m_depth_cache = nullptr;
        this.m_geometry_cache = nullptr;
        this.m_distortion_cache = nullptr;
        this.m_range = 16000;
        this.m_cal_valid = false;
        // this.m_afe_code.insert(m_afe_code.end(), &basecode[0],&basecode[ARRAY_SIZE(basecode)]);
        this.m_afe_code.push(basecode);
    }

    // aditof::Status Calibration3D_Smart::readCalMap(std::shared_ptr<aditof::StorageInterface> eeprom) {
    readCalMap(eeprom) {
        let status = Status.OK;
        let mode_data = Array(MODE_CFG_SIZE);
        let float_size = 4;
        let intrinsic_data = Array((COMMON_SIZE + 1) / float_size);

        /*Read the mode data*/
        for (let i = 0; i < 2; i++) {
            eeprom.read(ROMADDR_CFG_BASE[i], mode_data, MODE_CFG_SIZE);

            if (mode_data[0] == 0xFF) {
                console.log('WARNING: Invalid calibration in EEPROM, using default settings');
                return Status.OK;
            }

            this.m_mode_settings[i].pulse_cnt = mode_data[PULSE_CNT_OFFSET] | (mode_data[PULSE_CNT_OFFSET + 1] << 8);
            // memcpy(m_mode_settings[i].depth_pwr, &mode_data[DEPTH_PWR_OFFSET], DEPTH_PWR_CNT);

            this.m_mode_settings[i].depth_x0 = mode_data[DEPTH_X0_OFFSET] | (mode_data[DEPTH_X0_OFFSET + 1] << 8);

            for (let j = DEPTH_OFFSET_OFFSET; j < DEPTH_OFFSET_OFFSET + DEPTH_OFST_CNT * 2; j += 2) {
                this.m_mode_settings[i].depth_offset[(j - DEPTH_OFFSET_OFFSET) / 2] = mode_data[j] | (mode_data[j + 1] << 8);
            }
            this.m_mode_settings[i].depth3 = mode_data[DEPTH_3_OFFSET] | (mode_data[DEPTH_3_OFFSET + 1] << 8);
            this.m_mode_settings[i].depth2 = mode_data[DEPTH_2_OFFSET] | (mode_data[DEPTH_2_OFFSET + 1] << 8);
        }

        /*Replace the settings in the base code with the eeprom values*/
        for (let i = 0; i < 2; i++) {
            let afeCodeFound = false;
            for (let afeCode in this.m_afe_code) {
                if (afeCode == MODE_REG_BASE_ADDR[i] + R_MODE_PSPACE) {
                    afeCodeFound = true;
                    break;
                }
            }

            if (!afeCodeFound) {
                console.log('WARNING: Could not find R_MODE_PSPACE');
                return Status.INVALID_ARGUMENT;
            }
            // let pulse_space = *(it + 1);
            let pulse_space;
            let pulse_cnt = this.m_mode_settings[i].pulse_cnt;
            let pulse_hd = (((pulse_cnt - 1) * pulse_space + 90) / 928 + 3) * 36 + 57;

            afeCodeFound = false;
            for (let afeCode in this.m_afe_code) {
                if (afeCode == MODE_REG_BASE_ADDR[i] + R_PULSECNT) {
                    afeCodeFound = true;
                    break;
                }
            }
            if (!afeCodeFound) {
                console.log('WARNING: Could not find R_PULSECNT');
                return Status.INVALID_ARGUMENT;
            }

            // *(it + 1) = pulse_cnt;


            afeCodeFound = false;
            for (let afeCode in this.m_afe_code) {
                if (afeCode == MODE_REG_BASE_ADDR[i] + R_PULSEHD) {
                    afeCodeFound = true;
                    break;
                }
            }
            if (!afeCodeFound) {
                console.log('WARNING: Could not find R_PULSEHD');
                return Status.INVALID_ARGUMENT;
            }

            // *(it + 1) = pulse_hd;
            afeCodeFound = false;
            for (let afeCode in this.m_afe_code) {
                if (afeCode == MODE_LCORR_BASE_ADDR[i] + R_DEPTH_2) {
                    afeCodeFound = true;
                    break;
                }
            }
            if (!afeCodeFound) {
                console.log('WARNING: Could not find R_DEPTH_2');
                return Status.INVALID_ARGUMENT;
            }


            // *(it + 1) = m_mode_settings[i].depth2;
            afeCodeFound = false;
            for (let afeCode in this.m_afe_code) {
                if (afeCode == MODE_LCORR_BASE_ADDR[i] + R_DEPTH_3) {
                    afeCodeFound = true;
                    break;
                }
            }
            if (!afeCodeFound) {
                console.log('WARNING: Could not find R_DEPTH_3');
                return Status.INVALID_ARGUMENT;
            }

            // *(it + 1) = m_mode_settings[i].depth3;
            afeCodeFound = false;
            for (let afeCode in this.m_afe_code) {
                if (afeCode == MODE_LCORR_BASE_ADDR[i] + R_DEPTH_OFST) {
                    afeCodeFound = true;
                    break;
                }
            }
            if (!afeCodeFound) {
                console.log('WARNING: Could not find R_DEPTH_OFST');
                return Status.INVALID_ARGUMENT;
            }

            for (let j = 0; j < DEPTH_OFST_CNT; j++) {
                // *it = m_mode_settings[i].depth_offset[j];
                it += 2;
            }
        }

        /*Read the intrinsics and distortion params*/

        let intrinsics_byte = Array(28);
        let intrinsics_full = Array(7);

        eeprom.read(ROMADDR_COMMOM_BASE, intrinsics_byte, 28);

        for (let i = 0; i < 28; i = i + 4) {
            intrinsics_full[i / 4] = intrinsics_byte[i] | (intrinsics_byte[i + 1] << 8) | (intrinsics_byte[i + 2] << 16) | (intrinsics_byte[i + 3] << 24);
        }

        // memcpy(intrinsic_data, &intrinsics_full, 28);
        // m_intrinsics.insert(m_intrinsics.end(), &intrinsic_data[0], &intrinsic_data[7]);
        this.m_intrinsics.push(intrinsic_data);
        this.m_cal_valid = true;

        return status;
    }


    // aditof::Status Calibration3D_Smart::getAfeFirmware(const std::string &mode, std::vector<uint16_t> &data) const {
    getAfeFirmware(mode) {
        return [Status.OK, this.m_afe_code];
    }


    // aditof::Status Calibration3D_Smart::getIntrinsic(float key, std::vector<float> &data) const {
    getIntrinsic(key) {
        let data = [];
        let validParam = (INTRINSIC == key) || (DISTORTION_COEFFICIENTS == key);

        if (!validParam || !this.m_cal_valid) {
            console.log('WARNING: Invalid intrinsic ', key);
            return Status.INVALID_ARGUMENT;
        }

        if (key == INTRINSIC) {
            let intrinsic = [m_intrinsics[2], 0, m_intrinsics[0], 0, m_intrinsics[3], m_intrinsics[1], 0, 0, 1];
            // data.insert(data.end(), intrinsic.begin(), intrinsic.end());
            data.push(intrinsic);
        } else {
            let dist_coeff = [m_intrinsics[4], m_intrinsics[5], 0, 0, m_intrinsics[6]];
            // data.insert(data.end(), dist_coeff.begin(), dist_coeff.end());
            data.push(dist_coeff);
        }

        return [Status.OK, data];
    }

    // aditof::Status Calibration3D_Smart::setMode(std::shared_ptr<aditof::DepthSensorInterface> depthSensor, const std::string &mode, int range, unsigned int frameWidth, unsigned int frameheight) {
    setMode(depthSensor, mode, range, frameWidth, frameheight) {
        let status = Status.OK;
        let cameraMatrix = [];
        let distortionCoeffs = [];
        let mode_id = (mode == 'near' ? 0 : 1);
        let pixelMaxValue = (1 << 12) - 1; // 4095
        let gain = (mode == 'near' ? 0.5 : 1.15);
        let offset = 0.0;

        buildDepthCalibrationCache(gain, offset, pixelMaxValue, range);
        this.m_range = range;

        status = getIntrinsic(INTRINSIC, cameraMatrix);
        if (status != Status.OK) {
            console.log('WARNING: Failed to read intrinsic from eeprom');
        } else {
            console.log('INFO: Camera intrinsic parameters:\n\tfx: ', cameraMatrix[0], '\n\tfy: ', cameraMatrix[4], '\n\tcx: ', cameraMatrix[2], '\n\tcy: ', cameraMatrix[5]);
            buildGeometryCalibrationCache(cameraMatrix, frameWidth, frameheight);
        }

        status = getIntrinsic(DISTORTION_COEFFICIENTS, distortionCoeffs);
        if (status != Status.OK) {
            console.log('WARNING: Failed to read distortion coefficients from eeprom');
        } else {
            buildDistortionCorrectionCache(frameWidth, frameheight);
        }

        /*Execute the mode change command*/
        let afeRegsAddr = [0x4000, 0x4001, 0x7c22];
        let afeRegsVal = [mode_id, 0x0004, 0x0004];
        depthSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);

        return Status.OK;
    }


    // aditof::Status Calibration3D_Smart::calibrateDepth(uint16_t *frame, uint32_t frame_size) {
    calibrateDepth(frame, frame_size) {
        let cache = this.m_depth_cache;
        let end = frame + (frame_size - frame_size % 8);
        let framePtr = frame;

        for (; framePtr < end; framePtr += 8) {
            // *framePtr = *(cache + *framePtr);
            // *(framePtr + 1) = *(cache + *(framePtr + 1));
            // *(framePtr + 2) = *(cache + *(framePtr + 2));
            // *(framePtr + 3) = *(cache + *(framePtr + 3));
            // *(framePtr + 4) = *(cache + *(framePtr + 4));
            // *(framePtr + 5) = *(cache + *(framePtr + 5));
            // *(framePtr + 6) = *(cache + *(framePtr + 6));
            // *(framePtr + 7) = *(cache + *(framePtr + 7));
        }

        end += (frame_size % 8);

        for (; framePtr < end; framePtr++) {
            // *framePtr = *(cache + *framePtr);
        }

        return Status.OK;
    }


    // aditof::Status Calibration3D_Smart::calibrateCameraGeometry(uint16_t *frame, uint32_t frame_size) {
    calibrateCameraGeometry(frame, frame_size) {
        if (!this.m_cal_valid) {
            return Status.OK;
        }

        for (let i = 0; i < frame_size; i++) {
            if (frame[i] != this.m_range) {
                // frame[i] = static_cast<uint16_t>(frame[i] * m_geometry_cache[i]);
                frame[i] = frame[i] * m_geometry_cache[i];
            }
            if (frame[i] > this.m_range) {
                frame[i] = this.m_range;
            }
        }

        return Status.OK;
    }

    // Create a cache to speed up depth calibration computation
    // void Calibration3D_Smart::buildDepthCalibrationCache(float gain, float offset, int16_t maxPixelValue, int range) {
    buildDepthCalibrationCache(gain, offset, maxPixelValue, range) {
        if (this.m_depth_cache) {
            // delete[] m_depth_cache;
        }

        //new uint16_t[maxPixelValue + 1];
        this.m_depth_cache = Array(maxPixelValue + 1);
        for (let current = 0; current <= maxPixelValue; ++current) {
            let currentValue = current * gain + offset;
            this.m_depth_cache[current] = currentValue <= range ? currentValue : range;
        }
    }

    // Create a cache to speed up depth geometric camera calibration computation
    // void Calibration3D_Smart::buildGeometryCalibrationCache(const std::vector<float> &cameraMatrix, unsigned int width, unsigned int height) {
    buildGeometryCalibrationCache(cameraMatrix, width, height) {
        let fx = cameraMatrix[0];
        let fy = cameraMatrix[4];
        let x0 = cameraMatrix[2];
        let y0 = cameraMatrix[5];

        let validParameters = (fx != 0 && fy != 0);

        if (this.m_geometry_cache) {
            // delete[] m_geometry_cache;
        }

        //new double[width * height];
        this.m_geometry_cache = Array(width * height);
        for (let i = 0; i < height; i++) {
            for (let j = 0; j < width; j++) {

                if (validParameters) {
                    let tanXAngle = (x0 - j) / fx;
                    let tanYAngle = (y0 - i) / fy;

                    this.m_geometry_cache[i * width + j] = 1.0 / sqrt(1 + tanXAngle * tanXAngle + tanYAngle * tanYAngle);
                } else {
                    this.m_geometry_cache[i * width + j] = 1;
                }
            }
        }
    }

    // void Calibration3D_Smart::buildDistortionCorrectionCache(unsigned int width, unsigned int height) {
    buildDistortionCorrectionCache(width, height) {
        let fx = this.m_intrinsics[2];
        let fy = this.m_intrinsics[3];
        let cx = this.m_intrinsics[0];
        let cy = this.m_intrinsics[1];

        // std::vector<double> 
        let k = [m_intrinsics[4], m_intrinsics[5], 0.0, 0.0, m_intrinsics[6]];
        if (this.m_distortion_cache) {
            // delete[] m_distortion_cache;
        }

        // new double[width * height];
        this.m_distortion_cache = Array(width * height);
        for (let i = 0; i < width; i++) {
            for (let j = 0; j < height; j++) {
                let x = (i - cx) / fx;
                let y = (j - cy) / fy;

                //DISTORTION_COEFFICIENTS for [k1, k2, p1, p2, k3]
                let r2 = x * x + y * y;
                let k_calc = 1 + k[0] * r2 + k[1] * r2 * r2 + k[4] * r2 * r2 * r2;
                this.m_distortion_cache[j * width + i] = k_calc;
            }
        }
    }

    // aditof::Status Calibration3D_Smart::distortionCorrection(uint16_t *frame, unsigned int width, unsigned int height) {
    distortionCorrection(frame, width, height) {
        let fx = this.m_intrinsics[2];
        let fy = this.m_intrinsics[3];
        let cx = this.m_intrinsics[0];
        let cy = this.m_intrinsics[1];

        //new uint16_t[width * height];
        let buff = Array(width * height);

        for (let i = 0; i < width; i++) {
            for (let j = 0; j < height; j++) {
                //transform in dimensionless space
                let x = (i - cx) / fx;
                let y = (j - cy) / fy;

                //apply correction
                let x_dist_adim = x * m_distortion_cache[j * width + i];
                let y_dist_adim = y * m_distortion_cache[j * width + i];

                //back to original space
                let x_dist = (x_dist_adim * fx + cx);
                let y_dist = (y_dist_adim * fy + cy);

                if (x_dist >= 0 && x_dist < width && y_dist >= 0 && y_dist < height) {
                    buff[j * width + i] = frame[y_dist * width + x_dist];
                } else {
                    buff[j * width + i] = frame[j * width + i];
                }
            }
        }
        // memcpy(frame, buff, width * height * 2);
        // delete[] buff;
        return Status.OK;
    }




}




window.Camera3D_Smart = Camera3D_Smart;
window.Calibration3D_Smart = Calibration3D_Smart;
window.Calibration3D_Smart = Calibration3D_Smart;