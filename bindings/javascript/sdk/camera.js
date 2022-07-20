class SensorDetails {
    connectionType; // ConnectionType 
    constructor(){
        this.connectionType = ConnectionType.NETWORK;
    }
};

class BufferInfo {
    timestamp; // long long unsigned int 
    constructor(){
        this.timestamp = 0;
    }
};

class DepthParameters {
    depthGain; // float 
    depthOffset; // float 
    maxDepth; // int 
    minDepth; // int 
    // constructor(depthGain, depthOffset, maxDepth, minDepth){
    //     this.depthGain = depthGain;
    //     this.depthOffset = depthOffset;
    //     this.maxDepth = maxDepth;
    //     this.minDepth = minDepth;
    // }
}

class IntrinsicParameters {
    cameraMatrix = []; // float array
    distCoeffs = []; // float array
    pixelWidth; // float 
    pixelHeight; // float 
    // constructor(cameraMatrix, distCoeffs, pixelWidth, pixelHeight){
    //     this.cameraMatrix = cameraMatrix;
    //     this.distCoeffs = distCoeffs;
    //     this.pixelWidth = pixelWidth;
    //     this.pixelHeight = pixelHeight;
    // }
}

class FrameDetails {
    width; // unsigned int 
    height; // unsigned int 
    fullDataWidth; // unsigned int 
    fullDataHeight; // unsigned int 
    rgbWidth; // unsigned int 
    rgbHeight; // unsigned int 
    type; // string 
    // constructor(width, height, fullDataWidth, fullDataHeight, rgbWidth, rgbHeight, type){
    //     this.width = width;
    //     this.height = height;
    //     this.fullDataWidth = fullDataWidth;
    //     this.fullDataHeight = fullDataHeight;
    //     this.rgbWidth = rgbWidth;
    //     this.rgbHeight = rgbHeight;
    //     this.type = type;
    // }
}

class CameraDetails {
    cameraId; // string 
    mode; // string 
    frameType = new FrameDetails(); // FrameDetails 
    connection = ConnectionType.NETWORK; // ConnectionType.NETWORK 
    intrinsics = new IntrinsicParameters(); // IntrinsicParameters 
    depthParameters = new DepthParameters(); // DepthParameters 
    bitCount; // int 
    // constructor(cameraId, mode, connection, bitCount){
    //     this.cameraId = cameraId;
    //     this.mode = mode;
    //     this.frameType = new FrameDetails();
    //     this.connection = ConnectionType.NETWORK ;
    //     this.intrinsics = new IntrinsicParameters();
    //     this.depthParameters = new DepthParameters();
    //     this.bitCount = bitCount;
    // }
}

const FrameDataType = {
    FULL_DATA: 0, // Raw information
    DEPTH: 1,     // Depth information
    IR: 2,        // Infrared information
    RGB: 3        // RGB information
};
Object.freeze(FrameDataType);


// An array that contains the specific values for each revision
const RangeValuesForRevision = [
    {
        revision: "RevA", 
        ranges: [
                    {mode: "near", minDepth: 250, maxDepth: 800}, 
                    {mode: "medium", minDepth: 300, maxDepth: 3000}
                ]
    }];



const EEPROM_NAME = '24c1024';
const TEMPERATURE_SENSOR_NAME = 'TMP10X';
const skCameraName = '3D-Smart-Camera';
const skCustomMode = "custom";
const INTRINSIC = 5;
const DISTORTION_COEFFICIENTS = 6;
const EEPROM_SERIAL_LENGHT = 12;
const EEPROM_SERIAL_ADDR = 0x00010014;

const availableControls = ["noise_reduction_threshold", "ir_gamma_correction",
"depth_correction",          "camera_geometry_correction",
"bayer_rgb_conversion",      "camera_distortion_correction",
"ir_distortion_correction",  "revision"];

class Camera3D_Smart {
    m_details; // CameraDetails
    // m_depthSensor;
    // m_rgbSensor;
    m_rgbdSensor; // NetworkDepthSensor array
    m_eeprom; // NetworkStorage array
    m_temperatureSensor; // NetworkTemperatureSensor array
    m_devStarted; // bool 
    m_devProgrammed; // bool 
    m_eepromInitialized; // bool 
    m_tempSensorsInitialized; // bool 
    m_availableControls; // string array
    m_calibration; // Calibration3D_Smart 
    m_noiseReductionThreshold; // uint16_t 
    m_irGammaCorrection; // float 
    m_depthCorrection; // bool 
    m_cameraGeometryCorrection; // bool 
    m_cameraBayerRgbConversion; // bool 
    m_cameraDistortionCorrection; // bool 
    m_irDistorsionCorrection; // bool 
    m_revision; // string 
    m_rgbdFrameTypes; // FrameDetails array
    m_Rw; // float 
    m_Gw; // float 
    m_Bw; // float 
    
    constructor(rgbdSensor, eeproms, tSensors) {
        this.m_details = new CameraDetails();
        this.m_rgbdSensor = rgbdSensor;
        this.m_devStarted = false;
        this.m_devProgrammed = false;
        this.m_eepromInitialized = false;
        this.m_tempSensorsInitialized = false;
        this.m_availableControls = availableControls;
        this.m_calibration = new Calibration3D_Smart();
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
        let sDetails = this.m_rgbdSensor.getDetails();
        this.m_details.connection = sDetails.connectionType;

        // Look for EEPROM
        let eeprom_exists = false;
        for(const e of eeproms){
            if(e.getName() === EEPROM_NAME){
                eeprom_exists = true;
                this.m_eeprom = e;
                break;
            }
        }
        if (!eeprom_exists) {
            console.log('WARNING: Could not find ', EEPROM_NAME, ' while looking for storage for camera ', skCameraName);
            return;
        }
        // Look for the temperature sensor
        let tSensor_exists = false;
        for (const t of tSensors) {
            if (t.getName() === TEMPERATURE_SENSOR_NAME) {
                tSensor_exists = true;
                this.m_temperatureSensor = t;
                break;
            }
        }
        if (!tSensor_exists) {
            console.log('WARNING: Could not find ', TEMPERATURE_SENSOR_NAME, ' while looking for temperature sensors for camera ', skCameraName);
            return;
        }
    }

    async initialize() {
        let status;
        console.log('INFO: Initializing camera');

        if (!this.m_rgbdSensor || !this.m_eeprom || !this.m_temperatureSensor) {
            console.log('WARNING: Failed to initialize! Not all sensors are available');
            return Status.GENERIC_ERROR;
        }

        // Open communication with the depth sensor
        status = await this.m_rgbdSensor.open();
        if (status !== Status.OK) {
            console.log('WARNING: Failed to open device');
            return status;
        }

        this.m_details.bitCount = 12;

        // Open communication with EEPROM
        status = await this.m_eeprom.open();
        if (status !== Status.OK) {
            console.log('ERROR: Failed to open EEPROM with name ', this.m_eeprom.getName());
            return status;
        }
        this.m_eepromInitialized = true;

        // Open communication with temperature sensor
        status = await this.m_temperatureSensor.open();
        if (status !== Status.OK) {
            console.log('ERROR: Failed to open temperature sensor with name', this.m_temperatureSensor.getName());
            return status;
        }
        this.m_tempSensorsInitialized = true;

        // Read the camera's serial number from eeprom
        let eepromSerial;
        [status, eepromSerial] = await this.m_eeprom.read(EEPROM_SERIAL_ADDR, EEPROM_SERIAL_LENGHT);
        if (status !== Status.OK) {
            console.log('ERROR: Failed to read serial from eeprom');
            return status;
        }

        this.m_details.cameraId = new TextDecoder().decode(eepromSerial);
        console.log('INFO: Camera ID: ', this.m_details.cameraId);

        status = await this.m_calibration.readCalMap(this.m_eeprom);
        if (status !== Status.OK) {
            console.log('WARNING: Failed to read calibration data from eeprom');
            return status;
        }

        [status, this.m_details.intrinsics.cameraMatrix] = this.m_calibration.getIntrinsic(INTRINSIC);
        [status, this.m_details.intrinsics.distCoeffs] = this.m_calibration.getIntrinsic(DISTORTION_COEFFICIENTS);

        // For now we use the unit cell size values specified in the datasheet
        this.m_details.intrinsics.pixelWidth = 0.0056;
        this.m_details.intrinsics.pixelHeight = 0.0056;

        // Cache the frame types provided by Depth and RGB sensors
        [status, this.m_rgbdFrameTypes] = await this.m_rgbdSensor.getAvailableFrameTypes();
        if (status !== Status.OK) {
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
    async setMode(mode, modeFilename) {
        let status = Status.OK;

        // Set the values specific to the Revision requested
        // std::array<rangeStruct, 3> rangeValues = RangeValuesForRevision.at(m_revision);
        
        let rangeValues = [];
        for (const rvv of RangeValuesForRevision){
            if (rvv.revision === this.m_revision){
                rangeValues = rvv.ranges;
                break;
            }
        }
        console.log('rangeValues: ', rangeValues);

        console.log('INFO: Chosen mode: ', mode);

        let mode_found = false;
        for (const val of rangeValues) {
            console.log(val.mode, val.maxDepth, val.minDepth);
            if (val.mode === mode) {
                mode_found = true;                
                this.m_details.depthParameters.maxDepth = val.maxDepth;
                this.m_details.depthParameters.minDepth = val.minDepth;
                break;
            }
        }
        if (!mode_found) {
            this.m_details.depthParameters.maxDepth = 1;
        }

        console.log('INFO: Camera range for mode: ', mode, ' is: ', this.m_details.depthParameters.minDepth, ' mm and ', this.m_details.depthParameters.maxDepth, ' mm');

        if (!this.m_devProgrammed) {
            let firmwareData;
            [status, firmwareData] = this.m_calibration.getAfeFirmware();
            console.log(firmwareData);
            if (status !== Status.OK) {
                console.log('WARNING: Failed to read firmware from eeprom');
                return Status.UNREACHABLE;
            } else {
                console.log('INFO: Found firmware for mode: ', mode);
            }

            console.log('INFO: Firmware size: ', firmwareData.length * 2, ' bytes');
            status = await this.m_rgbdSensor.program(firmwareData,
                2 * firmwareData.length);
            if (status !== Status.OK) {
                console.log('WARNING: Failed to program AFE');
                return Status.UNREACHABLE;
            }
            this.m_devProgrammed = true;
        }

        status = await this.m_calibration.setMode(this.m_rgbdSensor, mode,
            this.m_details.depthParameters.maxDepth,
            this.m_details.frameType.width,
            this.m_details.frameType.height);

        if (status !== Status.OK) {
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
            await this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
        } else if (this.m_details.frameType.type == 'ir_rgb') {
            let afeRegsAddr = [0x4001, 0x7c22, 0xc3da, 0x4001, 0x7c22];
            let afeRegsVal = [0x0006, 0x0004, 0x05, 0x0007, 0x0004];
            await this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
        } else if (this.m_details.frameType.type == 'depth_ir_rgb') {
            let afeRegsAddr = [0x4001, 0x7c22, 0xc3da, 0x4001, 0x7c22];
            let afeRegsVal = [0x0006, 0x0004, 0x03, 0x0007, 0x0004];

            await this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 5);
        }

        if (mode === 'near') {
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
    async setFrameType(frameType) {
        let status = Status.OK;

        if (this.m_devStarted) {
            status = this.m_rgbdSensor.stop();
            if (status !== Status.OK) {
                return status;
            }
            this.m_devStarted = false;
        }

        
        let frameDetailsList;
        [status, frameDetailsList] = await this.m_rgbdSensor.getAvailableFrameTypes();
        if (status !== Status.OK) {
            console.log('WARNING: Failed to get available frame types');
            return status;
        }

        let typeFound = false;
        let frameDetails;
        for (const d of frameDetailsList) {
            if (d.getType() === frameType) {
                typeFound = true;
                frameDetails = d;
                
                console.log('found: ', frameDetails);
                break;

            }
        }

        if (!typeFound) {
            console.log('WARNING: Frame type: ', frameType, ' not supported by camera');
            return Status.INVALID_ARGUMENT;
        }

        if (this.m_details.frameType !== frameDetails) {

            
            //Turn off the streaming of data to modify parameters in the driver.
            let afeRegsAddr = [0x4001, 0x7c22];
            let afeRegsVal = [0x0006, 0x0004];
            status = await this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 2);
            console.log('writeAfeRegisters status:', status);

            let details = new FrameDetails(frameDetails.getWidth(), 
                frameDetails.getHeight(),
                frameDetails.getFullDataWidth(),
                frameDetails.getFullDataHeight(),
                frameDetails.getRgbWidth(),
                frameDetails.getRgbHeight(),
                frameDetails.getType());
            status = await this.m_rgbdSensor.setFrameType(details);

            if (status !== Status.OK) {
                console.log('WARNING: Failed to set frame type of the depth sensor');
                return status;
            }
            console.log('frameDetails: ', frameDetails);
            this.m_details.frameType = details;
            // this.m_details.frameType.type = frameType;
            // this.m_details.frameType.width = frameDetailsIt.getWidth();
            // this.m_details.frameType.height = frameDetailsIt.getHeight();
            // this.m_details.frameType.rgbWidth = frameDetailsIt.getRgbWidth();
            // this.m_details.frameType.rgbHeight = frameDetailsIt.getRgbHeight();
            // this.m_details.frameType.fullDataWidth = frameDetailsIt.getFullDataWidth();
            // this.m_details.frameType.fullDataHeight = frameDetailsIt.getFullDataHeight();

            //Turn on the streaming of data.
            afeRegsAddr = [0x4001, 0x7c22];
            afeRegsVal = [0x0007, 0x0004];
            await this.m_rgbdSensor.writeAfeRegisters(afeRegsAddr, afeRegsVal, 2);
        }

        if (!this.m_devStarted) {
            status = await this.m_rgbdSensor.start();
            if (status !== Status.OK) {
                return status;
            }
            this.m_devStarted = true;
        }

        return status;
    }

    // aditof::Status getAvailableFrameTypes(std::vector<std::string> &availableFrameTypes) const override;
    async getAvailableFrameTypes() {
        let status = Status.OK;

        let availableFrameTypes = [];
        // std::vector<FrameDetails> 
        let frameDetailsList;
        [status, frameDetailsList] = await this.m_rgbdSensor.getAvailableFrameTypes();
        if (status !== Status.OK) {
            console.log('WARNING: Failed to get available frame types');
            return status;
        }

        for (let i = 0; i < frameDetailsList.length; i++) {
            // console.log(frameDetailsList[i].getType());
            availableFrameTypes.push(frameDetailsList[i].getType());
        }

        return [status, availableFrameTypes];
    }

    // aditof::Status requestFrame(aditof::Frame *frame, aditof::FrameUpdateCallback cb) override;
    async requestFrame() {
        let status = Status.OK;
        let frame = new Frame();
        // if (frame == null) {
        //     console.log('ERROR: Received pointer frame is null');
        //     return Status.INVALID_ARGUMENT;
        // }

        // FrameDetails 
        let frameDetails = new FrameDetails();
        [status, frameDetails] = frame.getDetails();

        if (this.m_details.frameType.type != frameDetails.type) {
            frame.setDetails(this.m_details.frameType);
        }

        let fullDataLocation;
        [status, fullDataLocation] = frame.getData(FrameDataType.FULL_DATA);

        // aditof::BufferInfo 
        let rgbdBufferInfo;
        [status, fullDataLocation] = await this.m_rgbdSensor.getFrame();
        if (status != Status.OK) {
            console.log('WARNING: Failed to get frame from depth sensor');
            return [status, null];
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
            [status, depthDataLocation] = frame.getData(FrameDataType.DEPTH);

            let irDataLocation;
            [status, irDataLocation] = frame.getData(FrameDataType.IR);

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

window.INTRINSIC = INTRINSIC;
window.DISTORTION_COEFFICIENTS = DISTORTION_COEFFICIENTS;
window.Camera3D_Smart = Camera3D_Smart;
window.CameraDetails = CameraDetails;
window.DepthParameters = DepthParameters;
window.IntrinsicParameters = IntrinsicParameters;
window.FrameDataType = FrameDataType;
window.FrameDetails = FrameDetails;
window.BufferInfo = BufferInfo; 
window.SensorDetails = SensorDetails;
