#include "metawear/core/module.h"
#include "metawear/core/status.h"
#include "metawear/core/cpp/datasignal_private.h"
#include "metawear/core/cpp/metawearboard_def.h"
#include "metawear/core/cpp/metawearboard_macro.h"
#include "metawear/core/cpp/register.h"
#include "metawear/core/cpp/responseheader.h"

#include "metawear/sensor/ambientlight_ltr329.h"
#include "ambientlight_ltr329_private.h"
#include "ambientlight_ltr329_register.h"
#include "utils.h"

#include <cstdlib>
#include <cstring>

using std::forward_as_tuple;
using std::malloc;
using std::memcpy;
using std::memset;
using std::piecewise_construct;
using std::unordered_map;
using std::stringstream;

const ResponseHeader LTR329_ILLUMINANCE_RESPONSE_HEADER(MBL_MW_MODULE_AMBIENT_LIGHT, ORDINAL(AmbientLightLtr329Register::OUTPUT));

struct Ltr329Config {
    uint8_t power_control:2;
    uint8_t als_gain:3;
    uint8_t:3;
    uint8_t als_measurement_rate:3;
    uint8_t als_integration_time:3;
    uint8_t:2;
};

struct AmbientLightState {
    MblMwFnBoardPtrInt read_config_completed;
    void *read_config_context;
};

static unordered_map<const MblMwMetaWearBoard*, AmbientLightState> states;

// Helper function - receive a config response
static int32_t received_config_response(MblMwMetaWearBoard *board, const uint8_t *response, uint8_t len) {
    memcpy(board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT), response + 2, sizeof(Ltr329Config));

    printf("Light Sensor Config Received\n");
    printf("Power Value: %d\n", ((Ltr329Config*) board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT))->power_control);

    auto callback = states[board].read_config_completed;
    auto context = states[board].read_config_context;
    states[board].read_config_completed = nullptr;
    states[board].read_config_context = nullptr;
    callback(context, board, MBL_MW_STATUS_OK);

    return MBL_MW_STATUS_OK;
}

// Helper function - init module
void init_ambient_light_module(MblMwMetaWearBoard *board) {
    if (board->module_info.count(MBL_MW_MODULE_AMBIENT_LIGHT) && board->module_info.at(MBL_MW_MODULE_AMBIENT_LIGHT).present) {
        if (!board->module_config.count(MBL_MW_MODULE_AMBIENT_LIGHT)) {
            Ltr329Config* new_config = (Ltr329Config*)malloc(sizeof(Ltr329Config));
            memset(new_config, 0, sizeof(Ltr329Config));
            new_config->als_measurement_rate = MBL_MW_ALS_LTR329_RATE_2000ms;
            board->module_config.emplace(MBL_MW_MODULE_AMBIENT_LIGHT, new_config);
        }

        if (!board->module_events.count(LTR329_ILLUMINANCE_RESPONSE_HEADER)) {
            board->module_events[LTR329_ILLUMINANCE_RESPONSE_HEADER] = new MblMwDataSignal(LTR329_ILLUMINANCE_RESPONSE_HEADER,
                board, DataInterpreter::UINT32, 1, 4, 0, 0);
        }
        board->responses[LTR329_ILLUMINANCE_RESPONSE_HEADER]= response_handler_data_no_id;

        board->responses.emplace(piecewise_construct, forward_as_tuple(MBL_MW_MODULE_AMBIENT_LIGHT, READ_REGISTER(ORDINAL(AmbientLightLtr329Register::CONFIG))),
            forward_as_tuple(received_config_response));

        AmbientLightState newState = {nullptr};
        states.insert({board, newState});
    }
}

// Helper function - serialize
void serialize_ambient_light_config(const MblMwMetaWearBoard *board, std::vector<uint8_t>& state) {
    SERIALIZE_MODULE_CONFIG(Ltr329Config, MBL_MW_MODULE_AMBIENT_LIGHT);
}

// Helper function - deserialize
void deserialize_ambient_light_config(MblMwMetaWearBoard *board, uint8_t** state_stream) {
    DESERIALIZE_MODULE_CONFIG(Ltr329Config, MBL_MW_MODULE_AMBIENT_LIGHT);
}

// Get ambient light data signal
MblMwDataSignal* mbl_mw_als_ltr329_get_illuminance_data_signal(const MblMwMetaWearBoard *board) {
    GET_DATA_SIGNAL(LTR329_ILLUMINANCE_RESPONSE_HEADER);
}


// Set gain
void mbl_mw_als_ltr329_set_gain(MblMwMetaWearBoard *board, MblMwAlsLtr329Gain gain) {
    switch(gain) {
        case MBL_MW_ALS_LTR329_GAIN_48X:
        case MBL_MW_ALS_LTR329_GAIN_96X:
            ((Ltr329Config*) board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT))->als_gain= gain + 2;
            break;
        default:
            ((Ltr329Config*) board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT))->als_gain= gain;
            break;
    }
}

// Set gain approximately
void mbl_mw_als_ltr329_set_gain_approximate(MblMwMetaWearBoard *board, int gain) {
    if (gain < 1.5) {
        mbl_mw_als_ltr329_set_gain(board, MBL_MW_ALS_LTR329_GAIN_1X);
    } else if (gain < 3) {
        mbl_mw_als_ltr329_set_gain(board, MBL_MW_ALS_LTR329_GAIN_2X);
    } else if (gain < 6) {
        mbl_mw_als_ltr329_set_gain(board, MBL_MW_ALS_LTR329_GAIN_4X);
    } else if (gain < 20) {
        mbl_mw_als_ltr329_set_gain(board, MBL_MW_ALS_LTR329_GAIN_8X);
    } else if (gain < 75) {
        mbl_mw_als_ltr329_set_gain(board, MBL_MW_ALS_LTR329_GAIN_48X);
    } else {
        mbl_mw_als_ltr329_set_gain(board, MBL_MW_ALS_LTR329_GAIN_96X);
    }
}

// Get gain
int mbl_mw_als_ltr329_get_gain(MblMwMetaWearBoard *board) {
    switch(((Ltr329Config*) board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT))->als_gain) {
    case MBL_MW_ALS_LTR329_GAIN_96X + 2:
        return 96;
    case MBL_MW_ALS_LTR329_GAIN_48X + 2:
        return 48;
    case MBL_MW_ALS_LTR329_GAIN_8X:
        return 8;
    case MBL_MW_ALS_LTR329_GAIN_4X:
        return 4;
    case MBL_MW_ALS_LTR329_GAIN_2X:
        return 2;
    case MBL_MW_ALS_LTR329_GAIN_1X:
        return 1;
    default:
        return 1;
    }
}

// Set integration time
void mbl_mw_als_ltr329_set_integration_time(MblMwMetaWearBoard *board, MblMwAlsLtr329IntegrationTime integration_time) {
    ((Ltr329Config*) board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT))->als_integration_time= integration_time;
}

// Set measurement rate
void mbl_mw_als_ltr329_set_measurement_rate(MblMwMetaWearBoard *board, MblMwAlsLtr329MeasurementRate measurement_rate) {
    ((Ltr329Config*) board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT))->als_measurement_rate= measurement_rate;
}

// Set measurement rate approximately
void mbl_mw_als_ltr329_set_measurement_rate_approximate(MblMwMetaWearBoard *board, float measurement_rate) {
    if (measurement_rate < 0.75) {
        mbl_mw_als_ltr329_set_measurement_rate(board, MBL_MW_ALS_LTR329_RATE_2000ms);
    } else if (measurement_rate < 1.5) {
        mbl_mw_als_ltr329_set_measurement_rate(board, MBL_MW_ALS_LTR329_RATE_1000ms);
    } else if (measurement_rate < 3) {
        mbl_mw_als_ltr329_set_measurement_rate(board, MBL_MW_ALS_LTR329_RATE_500ms);
    } else if (measurement_rate < 8) {
        mbl_mw_als_ltr329_set_measurement_rate(board, MBL_MW_ALS_LTR329_RATE_200ms);
    } else if (measurement_rate < 15) {
        mbl_mw_als_ltr329_set_measurement_rate(board, MBL_MW_ALS_LTR329_RATE_100ms);
    } else {
        mbl_mw_als_ltr329_set_measurement_rate(board, MBL_MW_ALS_LTR329_RATE_50ms);
    }
}

// Get gain
float mbl_mw_als_ltr329_get_measurement_rate(MblMwMetaWearBoard *board) {
    switch(((Ltr329Config*) board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT))->als_measurement_rate) {
    case MBL_MW_ALS_LTR329_RATE_2000ms:
        return 0.5f;
    case MBL_MW_ALS_LTR329_RATE_1000ms:
        return 1.0f;
    case MBL_MW_ALS_LTR329_RATE_500ms:
        return 2.0f;
    case MBL_MW_ALS_LTR329_RATE_200ms:
        return 5.0f;
    case MBL_MW_ALS_LTR329_RATE_100ms:
        return 10.0f;
    case MBL_MW_ALS_LTR329_RATE_50ms:
        return 20.0f;
    default:
        return 1.0f;
    }
}

// Write config
void mbl_mw_als_ltr329_write_config(const MblMwMetaWearBoard *board) {
    uint8_t command[4]= {MBL_MW_MODULE_AMBIENT_LIGHT, ORDINAL(AmbientLightLtr329Register::CONFIG)};
    memcpy(command + 2, board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT), sizeof(Ltr329Config));
    SEND_COMMAND;
}

void mbl_mw_als_ltr329_read_config(const MblMwMetaWearBoard *board, void *context, MblMwFnBoardPtrInt completed) {
    states[board].read_config_context = context;
    states[board].read_config_completed = completed;

    uint8_t command[2]= {MBL_MW_MODULE_AMBIENT_LIGHT, READ_REGISTER(ORDINAL(AmbientLightLtr329Register::CONFIG))};
    SEND_COMMAND;
}

// Get Power Mode of Light Sensor
uint8_t mbl_mw_als_ltr329_is_active(const MblMwMetaWearBoard *board) {
    return ((Ltr329Config*) board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT))->power_control & 0x01;
}

// Start ambient light
void mbl_mw_als_ltr329_start(const MblMwMetaWearBoard *board) {
    ((Ltr329Config*) board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT))->power_control= 0x01;
    uint8_t command[3]= {MBL_MW_MODULE_AMBIENT_LIGHT, ORDINAL(AmbientLightLtr329Register::ENABLE), 1};
    SEND_COMMAND;
}

// Stop ambient light
void mbl_mw_als_ltr329_stop(const MblMwMetaWearBoard *board) {
    ((Ltr329Config*) board->module_config.at(MBL_MW_MODULE_AMBIENT_LIGHT))->power_control= 0x00;
    uint8_t command[3]= {MBL_MW_MODULE_AMBIENT_LIGHT, ORDINAL(AmbientLightLtr329Register::ENABLE), 0};
    SEND_COMMAND;
}

// Name for the loggers
void create_als_uri(const MblMwDataSignal* signal, std::stringstream& uri) {
    switch(CLEAR_READ(signal->header.register_id)) {
    case ORDINAL(AmbientLightLtr329Register::OUTPUT):
        uri << "illuminance";
        break;
    }
}
