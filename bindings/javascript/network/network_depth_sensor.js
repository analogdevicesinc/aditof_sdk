class NetworkDepthSensor {
    m_name; // string
    m_id; // int
    m_network; // Network
    m_opened; // bool
    frameDetails_cache; // FrameDetails 
    m_sensorDetails; // SensorDetails
    m_bufferInfo; // BufferInfo

    constructor(name, id, network) {
        this.m_name = name;
        this.m_id = id;
        this.m_network = network;
        this.m_opened = false;
        this.m_sensorDetails = new SensorDetails();
        this.m_bufferInfo = new BufferInfo();
    }

    async open() {
        let net = this.m_network;

        if ((await net.ServerConnect()) !== 0) {
            console.log("WARNING: Server Connect Failed");
            return Status.UNREACHABLE;
        }

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("Open");
        net.send_buff.setSensorsInfo(new BufferProtobuf.SensorsInfo());
        net.send_buff.getSensorsInfo().addImageSensors().setId(this.m_id);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();

        if (status === Status.OK) {
            this.m_opened = true;
        }

        return status;
    }
    async start() {
        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("Start");
        net.send_buff.setSensorsInfo(new BufferProtobuf.SensorsInfo());
        net.send_buff.getSensorsInfo().addImageSensors().setId(this.m_id);
        net.send_buff.setExpectReply(true);


        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();

        return status;
    }
    async stop() {
        let net = this.m_network;

        console.log("INFO: Stopping device");

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("Stop");
        net.send_buff.setSensorsInfo(new BufferProtobuf.SensorsInfo());
        net.send_buff.getSensorsInfo().addImageSensors().setId(this.m_id);
        net.send_buff.setExpectReply(true);


        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();

        return status;
    }

    async getAvailableFrameTypes() {
        let net = this.m_network;
        let types = [];

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return [Status.UNREACHABLE, types];
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("GetAvailableFrameTypes");
        net.send_buff.setSensorsInfo(new BufferProtobuf.SensorsInfo());
        net.send_buff.getSensorsInfo().addImageSensors().setId(this.m_id);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return [Status.INVALID_ARGUMENT, types];
        }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return [Status.GENERIC_ERROR, types];
        }

        for (const frameType of net.recv_buff.getAvailableFrameTypesList()) {
            let frameDetails = new FrameDetails();
            frameDetails.width = frameType.getWidth();
            frameDetails.height = frameType.getHeight();
            frameDetails.type = frameType.getType();
            frameDetails.fullDataWidth = frameType.getFullDataWidth();
            frameDetails.fullDataHeight = frameType.getFullDataHeight();
            frameDetails.rgbWidth = frameType.getRgbWidth();
            frameDetails.rgbHeight = frameType.getRgbHeight();
            types.push(frameDetails);
        }

        let status = net.recv_buff.getStatus();

        return [status, types];
    }
    async setFrameType(details) {
        console.assert(details instanceof FrameDetails, "Parameter is not of type FrameDetails.");
        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("SetFrameType");
        net.send_buff.setSensorsInfo(new BufferProtobuf.SensorsInfo());
        net.send_buff.getSensorsInfo().addImageSensors().setId(this.m_id);
        net.send_buff.setFrameType(new BufferProtobuf.FrameDetails());
        net.send_buff.getFrameType().setWidth(details.width);
        net.send_buff.getFrameType().setHeight(details.height);
        net.send_buff.getFrameType().setType(details.type);
        net.send_buff.getFrameType().setFullDataWidth(details.fullDataWidth);
        net.send_buff.getFrameType().setFullDataHeight(details.fullDataHeight);
        net.send_buff.getFrameType().setRgbWidth(details.rgbWidth);
        net.send_buff.getFrameType().setRgbHeight(details.rgbHeight);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();

        if (status === Status.OK) {
            this.frameDetails_cache = details;
        }

        return status;
    }
    async program(firmware, size) {
        if (firmware === null) {
            console.log("ERROR: Received firmware null pointer");
            return Status.INVALID_ARGUMENT;
        }
        
        console.assert(size > 0, "Size parameter is negative.");

        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        let firmware8 = new Uint8Array(new Uint16Array(firmware).buffer);

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("Program");
        net.send_buff.addFuncBytesParam(firmware8, size);
        net.send_buff.addFuncInt32Param(size);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();

        return status;
    }
    async getFrame() {
        let buffer;
        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return [Status.UNREACHABLE, buffer];
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("GetFrame");
        net.send_buff.setSensorsInfo(new BufferProtobuf.SensorsInfo());
        net.send_buff.getSensorsInfo().addImageSensors().setId(this.m_id);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return [Status.INVALID_ARGUMENT, buffer];
        }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return [Status.GENERIC_ERROR, buffer];
        }

        let status = net.recv_buff.getStatus();
        if (status !== Status.OK) {
            console.log("WARNING: getFrame() failed on target");
            return [status, buffer];
        }

        
        buffer = net.recv_buff.getBytesPayloadList();
        console.log('buffer: ', buffer);

        

        console.log('getBufferDetails: ', net.recv_buff.getBufferDetails());
        // if(this.m_bufferInfo){
        //     this.m_bufferInfo.timestamp = net.recv_buff.getBufferDetails().getTimestamp();
        // }
        
        return [status, buffer];
    }

    async readAfeRegisters(address, length) {
        let address8, data;

        if (address === null) {
            console.log("ERROR: Received AfeRegisters address null pointer");
            return [Status.INVALID_ARGUMENT, data];
        }
        console.assert(length > 0, "Length parameter is negative.");

        address8 = new Uint8Array(new Uint16Array(address).buffer);
        data = new Uint8Array();

        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return [Status.UNREACHABLE, data];
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("ReadAfeRegisters");
        net.send_buff.addFuncBytesParam(address8, length);
        net.send_buff.addFuncBytesParam(data, length);
        net.send_buff.addFuncInt32Param(length);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return [Status.INVALID_ARGUMENT, data];
        }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return [Status.GENERIC_ERROR, data];
        }

        let status = net.recv_buff.getStatus();

        if (status === Status.OK) {
            data = net.recv_buff.getBytesPayloadList()[0];
        }

        return [status, data];
    }
    async writeAfeRegisters(address, data, length) {
        if (address === null) {
            console.log("ERROR: Received AfeRegisters address null pointer");
            return Status.INVALID_ARGUMENT;
        }

        if (data === null) {
            console.log("ERROR: Received AfeRegisters data null pointer");
            return Status.INVALID_ARGUMENT;
        }

        console.assert(length > 0, "Length parameter is negative.");

        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        let address8 = new Uint8Array(new Uint16Array(address).buffer);
        let data8 = new Uint8Array(new Uint16Array(data).buffer);
        // console.log(address, address8);
        // console.log(data, data8);

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("WriteAfeRegisters");
        net.send_buff.addFuncBytesParam(address8, length * 2);
        net.send_buff.addFuncBytesParam(data8, length * 2);
        net.send_buff.addFuncInt32Param(length);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();

        return status;
    }

    getDetails() {
        return this.m_sensorDetails;
    }
    getName() {
        return this.m_name;
    }

}

window.NetworkDepthSensor = NetworkDepthSensor;

