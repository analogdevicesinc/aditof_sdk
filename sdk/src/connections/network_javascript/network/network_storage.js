class StorageInterface {}

class NetworkStorage extends StorageInterface {
    m_name;
    m_id;
    m_network;

    constructor(name, id, network) {
        super();
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
        net.send_buff.setFuncName("StorageOpen");
        net.send_buff.addFuncInt32Param(this.m_id);
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
    async read(address, bytesCount) {
        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("StorageRead");
        net.send_buff.addFuncInt32Param(this.m_id);
        net.send_buff.addFuncInt32Param(address);
        net.send_buff.addFuncInt32Param(bytesCount);
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

        let data;
        if (status === Status.OK) {
            // memcpy(data, net->recv_buff.bytes_payload(0).c_str(),
            //        net->recv_buff.bytes_payload(0).length());
            data = net.recv_buff.getBytesPayloadList()[0];
        }
        console.log('data: ', data.toLocaleString());

        return [status, data];
    }
    async write(address, data, bytesCount) {
        if (data === null) {
            console.log("ERROR: Received data null pointer");
            return Status.INVALID_ARGUMENT;
        }

        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("StorageWrite");
        net.send_buff.addFuncInt32Param(this.m_id);
        net.send_buff.addFuncInt32Param(address);
        net.send_buff.addFuncInt32Param(bytesCount);
        net.send_buff.addFuncBytesParam(data, bytesCount);
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
    async close() {
        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return Status.UNREACHABLE;
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("StorageClose");
        net.send_buff.addFuncInt32Param(this.m_id);
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
    getName() {
        return [Status.OK, this.m_name];
    }

}

window.NetworkStorage = NetworkStorage;

