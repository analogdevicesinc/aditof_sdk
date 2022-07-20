class NetworkStorage {
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
        net.send_buff.setFuncName("StorageOpen");
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
    async read(address, bytesCount) {
        let data;
        let net = this.m_network;

        if (!net.serverConnected) {
            console.log("WARNING: Not connected to server");
            return [Status.UNREACHABLE, data];
        }

        net.send_buff = new BufferProtobuf.ClientRequest();
        net.send_buff.setFuncName("StorageRead");
        net.send_buff.addFuncInt32Param(this.m_id);
        net.send_buff.addFuncInt32Param(address);
        net.send_buff.addFuncInt32Param(bytesCount);
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
        // console.log('data: ', data.toLocaleString());

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

window.NetworkStorage = NetworkStorage;