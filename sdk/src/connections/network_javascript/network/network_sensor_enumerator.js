class SensorEnumeratorInterface {}

class NetworkSensorEnumerator extends SensorEnumeratorInterface {
    m_network;
    m_cameraType;
    m_imageSensorsInfo;
    m_storagesInfo;
    m_temperatureSensorsInfo;

    constructor(network) {
        super();
        this.m_network = network;
        this.m_cameraType = -1;
        this.m_imageSensorsInfo = [];
        this.m_storagesInfo = [];
        this.m_temperatureSensorsInfo = [];
    }

    async getVersionString() {
        let status = Status.OK;
        let connectionString = "";

        this.m_network.send_buff = new BufferProtobuf.ClientRequest();
        this.m_network.send_buff.setFuncName("GetVersionString");
        this.m_network.send_buff.setExpectReply(true);

        if ((await this.m_network.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            status = Status.INVALID_ARGUMENT
            return [status, connectionString];
        }

        if (this.m_network.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            status = Status.GENERIC_ERROR;
            return [status, connectionString];
        }
        status = this.m_network.recv_buff.getStatus();
        if (status === Status.OK) {
            connectionString = this.m_network.recv_buff.getMessage();
        }

        console.log("status: " + status + "\nconnectionString: " + connectionString);

        return [status, connectionString];
    }

    async getCameraType() {
        let status = Status.OK;
        let cameraType = -1;

        this.m_network.send_buff = new BufferProtobuf.ClientRequest();
        this.m_network.send_buff.setFuncName("GetCameraType");
        this.m_network.send_buff.setExpectReply(true);

        if ((await this.m_network.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            status = Status.INVALID_ARGUMENT;
            return [status, cameraType];
        }

        if (this.m_network.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            status = Status.GENERIC_ERROR;
            return [status, cameraType];
        }
        status = this.m_network.recv_buff.getStatus();
        if (status === Status.OK) {
            cameraType = this.m_network.recv_buff.getCameraType();
        }

        console.log("status: " + status + "\ncameraType: " + cameraType);

        return [status, cameraType];
    }

    async searchSensors() {
        let status = Status.OK;
        console.log("INFO: Looking for sensors over network");
        let connectionString = "";

        if ((await this.m_network.ServerConnect()) !== 0) {
            console.log("WARNING: Server Connect Failed");
            return Status.UNREACHABLE;
        }
        [status, connectionString] = await this.getVersionString();
        if (!isValidConnection(connectionString)) {
            console.log(`ERROR: invalid connection string: ${connectionString}`);
            return Status.GENERIC_ERROR;
        }
        [status, this.m_cameraType] = await this.getCameraType();
        if (status !== Status.OK) {
            console.log("WARNING: Failed to find out the camera type on target. Assumming it's camera: AD-96TOF1-EBZ");
            this.m_cameraType = CameraType.AD_96TOF1_EBZ;
        }

        this.m_network.send_buff = new BufferProtobuf.ClientRequest();
        this.m_network.send_buff.setFuncName("FindSensors");
        this.m_network.send_buff.setExpectReply(true);

        if ((await this.m_network.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        if (this.m_network.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        const sensorsInfo = this.m_network.recv_buff.getSensorsInfo();
        let name, id;
        for (let i = 0; i < sensorsInfo.getImageSensorsList().length; i++) {
            name = sensorsInfo.getImageSensorsList()[i].getName();
            id = sensorsInfo.getImageSensorsList()[i].getId();
            this.m_imageSensorsInfo.push({ name, id });
        }

        for (let i = 0; i < sensorsInfo.getStoragesList().length; i++) {
            name = sensorsInfo.getStoragesList()[i].getName();
            id = sensorsInfo.getStoragesList()[i].getId();
            this.m_storagesInfo.push({ name, id });
        }

        for (let i = 0; i < sensorsInfo.getTempSensorsList().length; i++) {
            name = sensorsInfo.getTempSensorsList()[i].getName();
            id = sensorsInfo.getTempSensorsList()[i].getId();
            this.m_temperatureSensorsInfo.push({ name, id });
        }

        return this.m_network.recv_buff.getStatus();
    }

    getDepthSensors() {
        let depthSensors = [];
        let sensor;
        let status = Status.OK;
        // if (this.m_imageSensorsInfo.length > 0) {
        //     let { name, id } = this.m_imageSensorsInfo[0]; //front
        //     sensor = new NetworkDepthSensor(name, id, this.m_network);
        //     depthSensors.push(sensor);

        //     let communicationHandle;
        //     status = sensor.getHandle(communicationHandle);
        //     if (status !== Status.OK) {
        //         console.log("ERROR: Failed to obtain the handle");
        //         return [status, depthSensors];
        //     }
        //     for (let i = 1; i < this.m_imageSensorsInfo.length; i++) {
        //         let { name, id } = this.m_imageSensorsInfo[i];
        //         sensor = new NetworkDepthSensor(name, id, communicationHandle);
        //         depthSensors.push(sensor);
        //     }
        // }
        for (const nameAndId of this.m_imageSensorsInfo) {
            let { name, id } = nameAndId;
            sensor = new NetworkDepthSensor(name, id, this.m_network);
            depthSensors.push(sensor);
        }


        return [status, depthSensors];
    }

    getStorages() {
        let storages = [];
        let storage;
        let status = Status.OK;
        for (const nameAndId of this.m_storagesInfo) {
            let { name, id } = nameAndId;
            storage = new NetworkStorage(name, id, this.m_network);
            storages.push(storage);
        }

        return [status, storages];
    }

    getTemperatureSensors() {
        let temperatureSensors = [];
        let tSensor;
        let status = Status.OK;
        for (const nameAndId of this.m_temperatureSensorsInfo) {
            let { name, id } = nameAndId;
            tSensor = new NetworkTemperatureSensor(name, id, this.m_network);
            temperatureSensors.push(tSensor);
        }

        return [status, temperatureSensors];
    }

    getCameraTypeOnTarget() {
        return [Status.OK, this.m_cameraType];
    }
}
window.NetworkSensorEnumerator = NetworkSensorEnumerator;

