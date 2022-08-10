class NetworkTemperatureSensor {
    m_name; // string
    m_id; // int
    m_network; // Network

    constructor(name, id, network) {
        this.m_name = name;
        this.m_id = id;
        this.m_network = network;
    }

    async open() {
        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("TemperatureSensorOpen");
        net.send_buff.addFuncInt32Param(this.m_id);
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
    async read() {
        let temperature;
        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return [Status.UNREACHABLE, temperature];
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("TemperatureSensorRead");
        net.send_buff.addFuncInt32Param(this.m_id);
        net.send_buff.setExpectReply(true);

        if ((await net.SendCommand()) !== 0) {
            console.log("WARNING: Send Command Failed");
            return [Status.INVALID_ARGUMENT, temperature];
        }

        if (net.recv_buff.getServerStatus() !== ServerStatus.REQUEST_ACCEPTED) {
            console.log("WARNING: API execution on Target Failed");
            return [Status.GENERIC_ERROR, temperature];
        }

        let status = net.recv_buff.getStatus();

        if (status === Status.OK) {
            temperature = net.recv_buff.getFloatPayloadList()[0];
        }

        return [status, temperature];
    }
    async close() {
        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("TemperatureSensorClose");
        net.send_buff.addFuncInt32Param(this.m_id);
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
    getName() {
        return this.m_name;
    }

}

window.NetworkTemperatureSensor = NetworkTemperatureSensor;

