const WebSocketAsPromised = require('websocket-as-promised');
const BufferProtobuf = require('../buffer_pb');


// class NetworkHandle {
//     net;//network
// };


const Status = {
    OK: 0, // Success
    BUSY: 1, // Device or resource is busy
    UNREACHABLE: 2, // Device or resource is unreachable
    INVALID_ARGUMENT: 3, // Invalid arguments provided
    UNAVAILABLE: 4, // The requested action or resource is unavailable
    GENERIC_ERROR: 5
}
const CameraType = {
    AD_96TOF1_EBZ: 0, // AD-96TOF1-EBZ camera
    AD_FXTOF1_EBZ: 1, // AD-FXTOF1-EBZ camera
    SMART_3D_CAMERA: 2 // 3D Smart camera
};
const FrameDataType = {
    FULL_DATA: 0, // Raw information
    DEPTH: 1, // Depth information
    IR: 2, // Infrared information
    RGB: 3 // RGB information
};
const ConnectionType = {
    ON_TARGET: 0, // on the target, direct sysfs access
    USB: 1, // connects to target via USB
    NETWORK: 2 // connects to target via Network
};

const ServerStatus = {
    REQUEST_ACCEPTED: 0,
    REQUEST_UNKNOWN: 1,
};

Object.freeze(Status);
Object.freeze(CameraType);
Object.freeze(FrameDataType);
Object.freeze(ConnectionType);
Object.freeze(ServerStatus);

// const protocol_name = "network-protocol";


function isValidConnection(targetVersionString) {
    // getServerVersion(): functie din SDK - se va compila cu emscripten
    // targetVersionString.localeCompare(getServerVersion()) === 0;
    console.log("inside isValidConnection");
    return true;
}





class Network {
    ip_address;
    wsp;
    sendSuccessful; // bool 
    dataReceived; // bool 
    serverConnected; // bool 
    send_buff; // static payload::ClientRequest
    recv_buff; // static payload::ServerResponse

    constructor(ip_address) {
        this.ip_address = ip_address;
        try {
            this.wsp = new WebSocketAsPromised('ws://' + ip_address + ':5000 ', {
                packMessage: (data) => { // serialize object
                    // console.log("packMessage", +data);
                    return data.serializeBinary();
                },
                unpackMessage: async(data) => { // unpack the blob object to a protobuf ServerResponse
                    let buffer = await data.arrayBuffer();
                    let recv_buff = BufferProtobuf.ServerResponse.deserializeBinary(new Uint8Array(buffer));

                    // console.log("unpackMessage: ", recv_buff.getMessage());
                    return recv_buff;
                },
                attachRequestId: (data, id) => { // attach requestId to message as `id` field
                    data.setMessageId(id);
                    return data;
                },
                extractRequestId: (data) => { // extract messageId field
                    return data.getMessageId();
                },
            });

            this.wsp.onOpen.addListener(() => {
                console.log('onOpen: Connection established');
                this.serverConnected = true;
            });
            this.wsp.onSend.addListener(() => {
                this.sendSuccessful = true;
            });
            this.wsp.onResponse.addListener(() => {
                this.dataReceived = true;
            });
            this.wsp.onClose.addListener(() => {
                console.log('onClose: Connection terminated');
                this.serverConnected = false;
            });
        } catch (error) {
            console.log('constructor: ', error);
        }

        this.sendSuccessful = false;
        this.dataReceived = false;
        this.serverConnected = false;
        this.send_buff = new BufferProtobuf.ClientRequest();
        this.recv_buff = new BufferProtobuf.ServerResponse();
        // console.log("created network");
    }

    async ServerConnect() {
        try {
            await this.wsp.open();
        } catch (error) {
            console.log('ServerConnect: ', error);
            return -1;
        }
        return 0;
    }

    async SendCommand() {
        if (this.wsp == null) {
            console.log("Websocket was not initialized!");
            return -1;
        }
        try {
            this.recv_buff = await this.wsp.sendRequest(this.send_buff, {
                requestId: (Math.random() + 1).toString(36).substring(7),
            });
            console.log('recv_buff is: ', this.recv_buff);
        } catch (error) {
            console.log('SendCommand: ', error);
            return -1;
        }
        return 0;
    }

    async CloseConnection() {
        await wsp.close();
    }
}

window.WebSocketAsPromised = WebSocketAsPromised;
window.BufferProtobuf = BufferProtobuf;
// window.ip_address = ip_address;
window.Status = Status;
window.CameraType = CameraType;
window.FrameDataType = FrameDataType;
window.ConnectionType = ConnectionType;
window.ServerStatus = ServerStatus;
window.isValidConnection = isValidConnection;
window.Network = Network;

