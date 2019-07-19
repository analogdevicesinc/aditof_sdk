#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <libwebsockets.h>

#define RX_BUFFER_BYTES (1229500)

enum api_Values {
    API_NOT_DEFINED,
    FIND_DEVICES,
    INSTANTIATE_DEVICE,
    DESTROY_DEVICE,
    OPEN,
    START,
    STOP,
    GET_AVAILABLE_FRAME_TYPES,
    SET_FRAME_TYPE,
    PROGRAM,
    GET_FRAME,
    READ_EEPROM,
    WRITE_EEPROM,
    READ_AFE_REGISTERS,
    WRITE_AFE_REGISTERS,
    READ_AFE_TEMP,
    READ_LASER_TEMP,
};

enum protocols { PROTOCOL_EXAMPLE, PROTOCOL_COUNT };

class DeviceInterface;

class Network {
  public:
    struct lws_context *context;
    Network();
    static int callback_function(struct lws *wsi,
                                 enum lws_callback_reasons reason, void *user,
                                 void *in, size_t len);
};
