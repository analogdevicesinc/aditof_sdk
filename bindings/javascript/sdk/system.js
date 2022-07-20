function buildCamera(networkSensorEnumerator) {    
    console.assert(networkSensorEnumerator instanceof NetworkSensorEnumerator, "Parameter is not of type NetworkSensorEnumerator");

    let status;
    let depthSensors; //DepthSensorInterface
    let storages; //StorageInterface 
    let temperatureSensors; //TemperatureSensorInterface
    let camera; //Camera
    let cameraType; //CameraType

    [status, depthSensors] = networkSensorEnumerator.getDepthSensors();
    if (depthSensors.length < 1) {
        console.log('ERROR: No imagers found');
        return camera;
    }

    [status, storages] = networkSensorEnumerator.getStorages();
    [status, temperatureSensors] = networkSensorEnumerator.getTemperatureSensors();
    cameraType = networkSensorEnumerator.getCameraTypeOnTarget();

    switch (cameraType) {
        case CameraType.AD_96TOF1_EBZ:
            console.log('AD_96TOF1_EBZ');
            camera = new Camera96Tof1(depthSensors[0], storages, temperatureSensors);
            break;
        case CameraType.AD_FXTOF1_EBZ:
            console.log('AD_FXTOF1_EBZ');
            camera = new CameraFxTof1(depthSensors[0], storages, temperatureSensors);
            break;
        case CameraType.SMART_3D_CAMERA:
            console.log('SMART_3D_CAMERA');
            camera = new Camera3D_Smart(depthSensors[0], storages, temperatureSensors);
            break;
        default:
            console.log('ERROR: cameraType unknown.');
    }
    return camera;
}




class SystemImpl {
    static sdkRevisionLogged = false;

    constructor() {
        if (!this.sdkRevisionLogged) {
            this.sdkRevisionLogged = true;
            // right now, these macros can't be taken from C++
            console.log('INFO: SDK version: ADITOF_API_VERSION | branch: ADITOFSDK_GIT_BRANCH | commit: ADITOFSDK_GIT_COMMIT');
            console.log('INFO: SDK built with websockets version: LWS_LIBRARY_VERSION ');
        }
    }

    async getCameraListAtIp(ip) {
        let cameraList = [];
        let status = Status.OK;
        let camera;
        let network = new Network(ip);
        
        let networkSensorEnumerator = new NetworkSensorEnumerator(network);

        status = await networkSensorEnumerator.searchSensors();

        if (status === Status.OK) {
            camera = buildCamera(networkSensorEnumerator);
            console.log(camera);
            if (camera) {
                cameraList.push(camera);
            }
        }
        return [status, cameraList];
    }
}



class System extends SystemImpl {
    constructor() {
        super();
    }
}


window.System = System;
window.SystemImpl = SystemImpl;