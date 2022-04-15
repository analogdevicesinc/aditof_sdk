class DepthSensorInterface {}

class NetworkDepthSensor extends DepthSensorInterface {
    m_name;
    m_id;
    m_network;
    m_opened;
    m_sensorDetails;
    m_bufferInfo;

    constructor(name, id, network) {
        super();
        this.m_name = name;
        this.m_id = id;
        this.m_network = network;
        this.m_opened = false;
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

        // if (net.recv_server_data() !== 0) {
        //     console.log("WARNING: Receive Data Failed");
        //     return Status.GENERIC_ERROR;
        // }

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

        // net.send_buff.setFuncName("Start");
        // // net.send_buff.mutable_sensors_info().add_image_sensors().setId(this.m_id);
        // net.send_buff.setSensorsInfo(new BufferProtobuf.SensorsInfo());
        // net.send_buff.getSensorsInfo().addImageSensors().setId(this.m_id);
        // console.log('id: ', net.send_buff.getSensorsInfo().getImageSensorsList()[0].getId());
        // net.send_buff.setExpectReply(true);


        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("Start");
        net.send_buff.setSensorsInfo(new BufferProtobuf.SensorsInfo());
        net.send_buff.getSensorsInfo().addImageSensors().setId(this.m_id);
        net.send_buff.setExpectReply(true);


        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        // if (net.recv_server_data() !== 0) {
        //     console.log("WARNING: Receive Data Failed");
        //     return Status.GENERIC_ERROR;
        // }

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

        // net.send_buff.setFuncName("Stop");
        // net.send_buff.mutable_sensors_info().add_image_sensors().setId(this.m_id);
        // net.send_buff.setExpectReply(true);

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("Stop");
        net.send_buff.setSensorsInfo(new BufferProtobuf.SensorsInfo());
        net.send_buff.getSensorsInfo().addImageSensors().setId(this.m_id);
        net.send_buff.setExpectReply(true);


        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        // if (net.recv_server_data() !== 0) {
        //     console.log("WARNING: Receive Data Failed");
        //     return Status.GENERIC_ERROR;
        // }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();

        return status;
    }

    async getAvailableFrameTypes() {
        let net = this.m_network;
        let details = [];

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        // net.send_buff.setFuncName("GetAvailableFrameTypes");
        // net.send_buff.mutable_sensors_info().add_image_sensors().setId(this.m_id);
        // net.send_buff.setExpectReply(true);

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("GetAvailableFrameTypes");
        net.send_buff.setSensorsInfo(new BufferProtobuf.SensorsInfo());
        net.send_buff.getSensorsInfo().addImageSensors().setId(this.m_id);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        // if (net.recv_server_data() !== 0) {
        //     console.log("WARNING: Receive Data Failed");
        //     return Status.GENERIC_ERROR;
        // }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }


        for (const frameType of net.recv_buff.getAvailableFrameTypesList()) {
            details.push(frameType);
        }

        let status = net.recv_buff.getStatus();

        return [status, details];
    }
    async setFrameType(details) {
        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        // net.send_buff.setFuncName("SetFrameType");
        // net.send_buff.mutable_sensors_info().add_image_sensors().setId(this.m_id);
        // net.send_buff.mutable_frame_type().setWidth(details.width);
        // net.send_buff.mutable_frame_type().setHeight(details.height);
        // net.send_buff.mutable_frame_type().setType(details.type);
        // net.send_buff.mutable_frame_type().setFullDataWidth(details.fullDataWidth);
        // net.send_buff.mutable_frame_type().setFullDataHeight(details.fullDataHeight);
        // net.send_buff.mutable_frame_type().setRgbWidth(details.rgbWidth);
        // net.send_buff.mutable_frame_type().setRgbHeight(details.rgbHeight);
        // net.send_buff.setExpectReply(true);

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

        // if (net.recv_server_data() !== 0) {
        //     console.log("WARNING: Receive Data Failed");
        //     return Status.GENERIC_ERROR;
        // }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();

        if (status === Status.OK) {
            this.m_sensorDetails = details;
        }

        return status;
    }
    async program(firmware, size) {
        if (firmware === null) { // nullptr
            console.log("ERROR: Received firmware null pointer");
            return Status.INVALID_ARGUMENT;
        }

        //assert(size > 0);

        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("Program");
        net.send_buff.addFuncInt32Param(size); // static_cast<::google::int32>(size)
        net.send_buff.addFuncBytesParam(firmware, size);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        // if (net.recv_server_data() !== 0) {
        //     console.log("WARNING: Receive Data Failed");
        //     return Status.GENERIC_ERROR;
        // }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();

        return status;
    }
    async getFrame() {
        // let buffer;
        // if (buffer === null) { //nullptr
        //     console.log("ERROR: Received buffer null pointer");
        //     return Status.INVALID_ARGUMENT;
        // }

        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("GetFrame");

        net.send_buff.setSensorsInfo(new BufferProtobuf.SensorsInfo());
        net.send_buff.getSensorsInfo().addImageSensors().setId(this.m_id);

        // net.send_buff.mutable_sensors_info().add_image_sensors().setId(this.m_id);

        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        // if (net.recv_server_data() !== 0) {
        //     console.log("WARNING: Receive Data Failed");
        //     return Status.GENERIC_ERROR;
        // }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();
        if (status !== Status.OK) {
            console.log("WARNING: getFrame() failed on target");
            return status;
        }

        // memcpy(buffer, net->recv_buff.bytes_payload(0).c_str(),
        //        net->recv_buff.bytes_payload(0).length());


        // let aha = (net.recv_buff.getBytesPayloadList()[0]);
        // console.log("aha: ", aha);

        let buffer = net.recv_buff.getBytesPayloadList()[0];


        // undefined
        // this.m_bufferInfo.timestamp = net.recv_buff.getBufferDetails().getTimestamp();



        return status;
    }

    async readAfeRegisters(address, data, length) {
        if (address === null) {
            console.log("ERROR: Received AfeRegisters address null pointer");
            return Status.INVALID_ARGUMENT;
        }

        if (data === null) {
            console.log("ERROR: Received AfeRegisters data null pointer");
            return Status.INVALID_ARGUMENT;
        }

        //assert(length > 0);

        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("ReadAfeRegisters");
        net.send_buff.addFuncInt32Param(length);
        net.send_buff.addFuncBytesParam(address, length);
        net.send_buff.addFuncBytesParam(data, length);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        // if (net.recv_server_data() !== 0) {
        //     console.log("WARNING: Receive Data Failed");
        //     return Status.GENERIC_ERROR;
        // }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();

        if (status === Status.OK) {
            // memcpy(data, net->recv_buff.bytes_payload(0).c_str(),
            //        net->recv_buff.bytes_payload(0).length());
            data = net.recv_buff.bytes_payload(0).c_str();
        }

        return status;
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

        // assert(length > 0);

        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("WriteAfeRegisters");
        net.send_buff.addFuncInt32Param(length);
        net.send_buff.addFuncBytesParam(address, length);
        net.send_buff.addFuncBytesParam(data, length);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return Status.INVALID_ARGUMENT;
        }

        // if (net.recv_server_data() !== 0) {
        //     console.log("WARNING: Receive Data Failed");
        //     return Status.GENERIC_ERROR;
        // }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return Status.GENERIC_ERROR;
        }

        let status = net.recv_buff.getStatus();

        return status;
    }

    getDetails() {
        return [Status.OK, this.m_sensorDetails];
    }
    getName() {
        return [Status.OK, this.m_name];
    }

}

window.NetworkDepthSensor = NetworkDepthSensor;

