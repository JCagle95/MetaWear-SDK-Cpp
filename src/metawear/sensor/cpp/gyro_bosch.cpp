#include "metawear/core/module.h"
#include "metawear/core/status.h"
#include "metawear/core/cpp/datasignal_private.h"
#include "metawear/core/cpp/metawearboard_def.h"
#include "metawear/core/cpp/metawearboard_macro.h"
#include "metawear/core/cpp/register.h"
#include "metawear/core/cpp/responseheader.h"

#include "metawear/sensor/gyro_bosch.h"
#include "gyro_bosch_private.h"
#include "gyro_bosch_register.h"
#include "utils.h"

#include <cstdlib>
#include <cstring>
#include <unordered_map>

using std::forward_as_tuple;
using std::malloc;
using std::memcpy;
using std::memset;
using std::piecewise_construct;
using std::stringstream;
using std::unordered_map;
using std::vector;

#define CREATE_BMI160_ROT_SIGNAL_SINGLE(offset) CREATE_BMI160_ROT_SIGNAL(DataInterpreter::BOSCH_ROTATION_SINGLE_AXIS, 1, offset)
#define CREATE_BMI160_ROT_SIGNAL(interpreter, channels, offset) new MblMwDataSignal(GYRO_BMI160_ROT_RESPONSE_HEADER, board, interpreter, \
        FirmwareConverter::BOSCH_ROTATION, channels, 2, 1, offset)
#define CREATE_BMI270_ROT_SIGNAL_SINGLE(offset) CREATE_BMI270_ROT_SIGNAL(DataInterpreter::BOSCH_ROTATION_SINGLE_AXIS, 1, offset)
#define CREATE_BMI270_ROT_SIGNAL(interpreter, channels, offset) new MblMwDataSignal(GYRO_BMI270_ROT_RESPONSE_HEADER, board, interpreter, \
        FirmwareConverter::BOSCH_ROTATION, channels, 2, 1, offset)

const uint8_t MBL_MW_MODULE_GYRO_TYPE_BMI160 = 0;            ///< Constant identifying the BMI160 accelerometer module type
const uint8_t MBL_MW_MODULE_GYRO_TYPE_BMI270 = 1;            ///< Constant identifying the BMI270 accelerometer module type

const float FSR_SCALE[5]= {16.4f, 32.8f, 65.6f, 131.2f, 262.4f};
const uint8_t PACKED_ROT_REVISION= 1;
const ResponseHeader    GYRO_BMI160_ROT_RESPONSE_HEADER(MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi160Register::DATA)),
                        GYRO_BMI160_PACKED_ROT_RESPONSE_HEADER(MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi160Register::PACKED_GYRO_DATA)),
                        GYRO_BMI270_ROT_RESPONSE_HEADER(MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi270Register::DATA)),
                        GYRO_BMI270_PACKED_ROT_RESPONSE_HEADER(MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi270Register::PACKED_GYRO_DATA));

struct GyroBoschConfig {
    struct {
        uint8_t gyr_odr : 4;
        uint8_t gyr_bwp : 2;
        uint8_t:2;
        uint8_t gyr_range : 3;
        uint8_t:5;
    } config;
    struct {
        uint8_t enabled;
    } interrupt;
};

struct GyroBoschState {
    MblMwFnBoardPtrInt read_config_completed;
    void *read_config_context;
};

static unordered_map<const MblMwMetaWearBoard*, GyroBoschState> states;

// Wrapping Power State Sensing in Read Config
static void on_config_complete(void *context, MblMwMetaWearBoard* board, MblMwFnBoardPtrInt completed) {
    mbl_mw_gyro_read_interrupt(board, context, completed);
}

static int32_t received_power_response(MblMwMetaWearBoard *board, const uint8_t *response, uint8_t len) {
    auto config= &((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->interrupt;
    memcpy(config, response + 2, sizeof(*config));

    printf("Length of Interrupt: %d, Gyro Data Interrupt: %d\n", len, response[2]);

    auto callback = states[board].read_config_completed;
    auto context = states[board].read_config_context;
    states[board].read_config_completed = nullptr;
    states[board].read_config_context = nullptr;
    callback(context, board, MBL_MW_STATUS_OK);

    return MBL_MW_STATUS_OK;
}

// Helper function - receive a config response
static int32_t received_config_response(MblMwMetaWearBoard *board, const uint8_t *response, uint8_t len) {
    auto config= &((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config;
    memcpy(config, response + 2, sizeof(*config));

    auto callback = states[board].read_config_completed;
    auto context = states[board].read_config_context;
    states[board].read_config_completed = nullptr;
    states[board].read_config_context = nullptr;
    on_config_complete(context, board, callback);

    return MBL_MW_STATUS_OK;
}

// Helper function - get gyro scale
float bosch_gyro_get_data_scale(const MblMwMetaWearBoard *board) {
    auto config = ((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config;
    return FSR_SCALE[config.gyr_range];
}

// Helpfer function - init module
void init_gyro_module(MblMwMetaWearBoard *board) {
    switch(board->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        if (board->module_info.count(MBL_MW_MODULE_GYRO) && board->module_info.at(MBL_MW_MODULE_GYRO).present) {
            if (!board->module_config.count(MBL_MW_MODULE_GYRO)) {
                GyroBoschConfig *new_config = (GyroBoschConfig*)malloc(sizeof(GyroBoschConfig));

                memset(new_config, 0, sizeof(GyroBoschConfig));
                new_config->config.gyr_bwp = 2;
                new_config->config.gyr_odr = MBL_MW_GYRO_BOSCH_ODR_100Hz;
                new_config->config.gyr_range = MBL_MW_GYRO_BOSCH_RANGE_2000dps;
                board->module_config.emplace(MBL_MW_MODULE_GYRO, new_config);
            }

            MblMwDataSignal* rotation;
            if (board->module_events.count(GYRO_BMI160_ROT_RESPONSE_HEADER)) {
                rotation = dynamic_cast<MblMwDataSignal*>(board->module_events[GYRO_BMI160_ROT_RESPONSE_HEADER]);
            } else {
                rotation = CREATE_BMI160_ROT_SIGNAL(DataInterpreter::BOSCH_ROTATION, 3, 0);
                board->module_events[GYRO_BMI160_ROT_RESPONSE_HEADER] = rotation;
            }
            if (!rotation->components.size()) {
                rotation->components.push_back(CREATE_BMI160_ROT_SIGNAL_SINGLE(0));
                rotation->components.push_back(CREATE_BMI160_ROT_SIGNAL_SINGLE(2));
                rotation->components.push_back(CREATE_BMI160_ROT_SIGNAL_SINGLE(4));
            }

            board->responses[GYRO_BMI160_ROT_RESPONSE_HEADER]= response_handler_data_no_id;

            if (board->module_info.at(MBL_MW_MODULE_GYRO).revision >= PACKED_ROT_REVISION) {
                if (!board->module_events.count(GYRO_BMI160_PACKED_ROT_RESPONSE_HEADER)) {
                    board->module_events[GYRO_BMI160_PACKED_ROT_RESPONSE_HEADER]= new MblMwDataSignal(GYRO_BMI160_PACKED_ROT_RESPONSE_HEADER, board,
                        DataInterpreter::BOSCH_ROTATION, FirmwareConverter::BOSCH_ROTATION, 3, 2, 1, 0);
                }
                board->responses[GYRO_BMI160_PACKED_ROT_RESPONSE_HEADER]= response_handler_packed_data;
            }

            board->responses.emplace(piecewise_construct, forward_as_tuple(MBL_MW_MODULE_GYRO, READ_REGISTER(ORDINAL(GyroBmi160Register::CONFIG))),
                    forward_as_tuple(received_config_response));

            board->responses.emplace(piecewise_construct, forward_as_tuple(MBL_MW_MODULE_GYRO, READ_REGISTER(ORDINAL(GyroBmi160Register::DATA_INTERRUPT_ENABLE))),
                    forward_as_tuple(received_power_response));

            GyroBoschState newState = {nullptr};
            states.insert({board, newState});
        }
        break;
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        if (board->module_info.count(MBL_MW_MODULE_GYRO) && board->module_info.at(MBL_MW_MODULE_GYRO).present) {
            if (!board->module_config.count(MBL_MW_MODULE_GYRO)) {
                GyroBoschConfig *new_config = (GyroBoschConfig*)malloc(sizeof(GyroBoschConfig));

                memset(new_config, 0, sizeof(GyroBoschConfig));
                new_config->config.gyr_bwp = 2;
                new_config->config.gyr_odr = MBL_MW_GYRO_BOSCH_ODR_100Hz;
                new_config->config.gyr_range = MBL_MW_GYRO_BOSCH_RANGE_2000dps;
                board->module_config.emplace(MBL_MW_MODULE_GYRO, new_config);
            }

            MblMwDataSignal* rotation;
            if (board->module_events.count(GYRO_BMI270_ROT_RESPONSE_HEADER)) {
                rotation = dynamic_cast<MblMwDataSignal*>(board->module_events[GYRO_BMI270_ROT_RESPONSE_HEADER]);
            } else {
                rotation = CREATE_BMI270_ROT_SIGNAL(DataInterpreter::BOSCH_ROTATION, 3, 0);
                board->module_events[GYRO_BMI270_ROT_RESPONSE_HEADER] = rotation;
            }
            if (!rotation->components.size()) {
                rotation->components.push_back(CREATE_BMI270_ROT_SIGNAL_SINGLE(0));
                rotation->components.push_back(CREATE_BMI270_ROT_SIGNAL_SINGLE(2));
                rotation->components.push_back(CREATE_BMI270_ROT_SIGNAL_SINGLE(4));
            }

            board->responses[GYRO_BMI270_ROT_RESPONSE_HEADER]= response_handler_data_no_id;

            if (!board->module_events.count(GYRO_BMI270_PACKED_ROT_RESPONSE_HEADER)) {
                board->module_events[GYRO_BMI270_PACKED_ROT_RESPONSE_HEADER]= new MblMwDataSignal(GYRO_BMI270_PACKED_ROT_RESPONSE_HEADER, board,
                    DataInterpreter::BOSCH_ROTATION, FirmwareConverter::BOSCH_ROTATION, 3, 2, 1, 0);
            }
            board->responses[GYRO_BMI270_PACKED_ROT_RESPONSE_HEADER]= response_handler_packed_data;

            board->responses.emplace(piecewise_construct, forward_as_tuple(MBL_MW_MODULE_GYRO, READ_REGISTER(ORDINAL(GyroBmi270Register::CONFIG))),
                    forward_as_tuple(received_config_response));

            board->responses.emplace(piecewise_construct, forward_as_tuple(MBL_MW_MODULE_GYRO, READ_REGISTER(ORDINAL(GyroBmi270Register::DATA_INTERRUPT_ENABLE))),
                    forward_as_tuple(received_power_response));

            GyroBoschState newState = {nullptr};
            states.insert({board, newState});
        }
        break;
    default:
        return;
    }
}

// Helper function - delete module
void free_gyro_module(MblMwMetaWearBoard *board) {
    states.erase(board);
}

// Generic function for data signal
MblMwDataSignal* mbl_mw_gyro_get_rotation_data_signal(const MblMwMetaWearBoard *board) {
    switch(board->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        return mbl_mw_gyro_bmi160_get_rotation_data_signal(board);
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        return mbl_mw_gyro_bmi270_get_rotation_data_signal(board);
    default:
        return mbl_mw_gyro_bmi270_get_rotation_data_signal(board);
    }
}

// Generic command to receive gyro odr
float mbl_mw_gyro_get_odr(MblMwMetaWearBoard *board) {
    switch(board->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        return mbl_mw_gyro_bmi160_get_odr(board);
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        return mbl_mw_gyro_bmi270_get_odr(board);
    default:
        return -1;
    }
}

// Generic command to receive gyro odr
float mbl_mw_gyro_get_range(MblMwMetaWearBoard *board) {
    switch(board->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        return mbl_mw_gyro_bmi160_get_range(board);
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        return mbl_mw_gyro_bmi270_get_range(board);
    default:
        return -1;
    }
}

// Generic command to write configuration
void mbl_mw_gyro_write_config(const MblMwMetaWearBoard *board) {
    switch(board->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        mbl_mw_gyro_bmi160_write_config(board);
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        mbl_mw_gyro_bmi270_write_config(board);
    }
}

// Generic command to set gyro odr
void mbl_mw_gyro_set_odr(MblMwMetaWearBoard *board, float odr) {
    switch(board->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        if (odr > 2400) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_3200Hz);
        } else if (odr > 1200) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_1600Hz);
        } else if (odr > 600) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_800Hz);
        } else if (odr > 300) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_400Hz);
        } else if (odr > 150) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_200Hz);
        } else if (odr > 75) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_100Hz);
        } else if (odr > 37) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_50Hz);
        } else {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_25Hz);
        }
        break;
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        if (odr > 2400) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_3200Hz);
        } else if (odr > 1200) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_1600Hz);
        } else if (odr > 600) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_800Hz);
        } else if (odr > 300) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_400Hz);
        } else if (odr > 150) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_200Hz);
        } else if (odr > 75) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_100Hz);
        } else if (odr > 37) {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_50Hz);
        } else {
            mbl_mw_gyro_bmi160_set_odr(board, MBL_MW_GYRO_BOSCH_ODR_25Hz);
        }
    }
}

// Generic command to set gyro range
void mbl_mw_gyro_set_range(MblMwMetaWearBoard *board, float range) {
    switch(board->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        if (range > 1500) {
            mbl_mw_gyro_bmi160_set_range(board, MBL_MW_GYRO_BOSCH_RANGE_2000dps);
        } else if (range > 750) {
            mbl_mw_gyro_bmi160_set_range(board, MBL_MW_GYRO_BOSCH_RANGE_1000dps);
        } else if (range > 375) {
            mbl_mw_gyro_bmi160_set_range(board, MBL_MW_GYRO_BOSCH_RANGE_500dps);
        } else if (range > 180) {
            mbl_mw_gyro_bmi160_set_range(board, MBL_MW_GYRO_BOSCH_RANGE_250dps);
        } else {
            mbl_mw_gyro_bmi160_set_range(board, MBL_MW_GYRO_BOSCH_RANGE_125dps);
        }
        break;
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        if (range > 1500) {
            mbl_mw_gyro_bmi270_set_range(board, MBL_MW_GYRO_BOSCH_RANGE_2000dps);
        } else if (range > 750) {
            mbl_mw_gyro_bmi270_set_range(board, MBL_MW_GYRO_BOSCH_RANGE_1000dps);
        } else if (range > 375) {
            mbl_mw_gyro_bmi270_set_range(board, MBL_MW_GYRO_BOSCH_RANGE_500dps);
        } else if (range > 180) {
            mbl_mw_gyro_bmi270_set_range(board, MBL_MW_GYRO_BOSCH_RANGE_250dps);
        } else {
            mbl_mw_gyro_bmi270_set_range(board, MBL_MW_GYRO_BOSCH_RANGE_125dps);
        }
    }
}

// Generic command to start sensing
void mbl_mw_gyro_start(const MblMwMetaWearBoard *board) {
    switch(board->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        mbl_mw_gyro_bmi160_start(board);
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        mbl_mw_gyro_bmi270_start(board);
    }
}

// Generic command to stop sensing
void mbl_mw_gyro_stop(const MblMwMetaWearBoard *board) {
    switch(board->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        mbl_mw_gyro_bmi160_stop(board);
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        mbl_mw_gyro_bmi270_stop(board);
    }
}

// Generic command to enable sampling
void mbl_mw_gyro_enable_rotation_sampling(const MblMwMetaWearBoard *board) {
    switch(board->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        mbl_mw_gyro_bmi160_enable_rotation_sampling(board);
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        mbl_mw_gyro_bmi270_enable_rotation_sampling(board);
    }
}

// Generic command to disable sampling
void mbl_mw_gyro_disable_rotation_sampling(const MblMwMetaWearBoard *board) {
    switch(board->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        mbl_mw_gyro_bmi160_disable_rotation_sampling(board);
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        mbl_mw_gyro_bmi270_disable_rotation_sampling(board);
    }
}

// Get the bmi160 gyro signal
MblMwDataSignal* mbl_mw_gyro_bmi160_get_rotation_data_signal(const MblMwMetaWearBoard *board) {
    GET_DATA_SIGNAL(GYRO_BMI160_ROT_RESPONSE_HEADER);
}

// Get the bmi160 gyro signal
MblMwDataSignal* mbl_mw_gyro_bmi160_get_high_freq_rotation_data_signal(const MblMwMetaWearBoard *board) {
    return mbl_mw_gyro_bmi160_get_packed_rotation_data_signal(board);
}

// Get the bmi270 gyro signal
MblMwDataSignal* mbl_mw_gyro_bmi270_get_rotation_data_signal(const MblMwMetaWearBoard *board) {
    GET_DATA_SIGNAL(GYRO_BMI270_ROT_RESPONSE_HEADER);
}

// Get the packed bmi160 gyro signal
MblMwDataSignal* mbl_mw_gyro_bmi160_get_packed_rotation_data_signal(const MblMwMetaWearBoard *board) {
    GET_DATA_SIGNAL(GYRO_BMI160_PACKED_ROT_RESPONSE_HEADER);
}

// Get the packed bmi270 gyro signal
MblMwDataSignal* mbl_mw_gyro_bmi270_get_packed_rotation_data_signal(const MblMwMetaWearBoard *board) {
    GET_DATA_SIGNAL(GYRO_BMI270_PACKED_ROT_RESPONSE_HEADER);
}

// Helper function - serialize
void serialize_gyro_config(const MblMwMetaWearBoard *board, vector<uint8_t>& state) {
    SERIALIZE_MODULE_CONFIG(GyroBoschConfig, MBL_MW_MODULE_GYRO);
}

// Helper function - deserialize
void deserialize_gyro_config(MblMwMetaWearBoard *board, uint8_t** state_stream) {
    DESERIALIZE_MODULE_CONFIG(GyroBoschConfig, MBL_MW_MODULE_GYRO);
}

// Set the bmi160 odr
void mbl_mw_gyro_bmi160_set_odr(MblMwMetaWearBoard *board, MblMwGyroBoschOdr odr) {
    ((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config.gyr_odr= odr;
}

// Set the bmi160 range
void mbl_mw_gyro_bmi160_set_range(MblMwMetaWearBoard *board, MblMwGyroBoschRange range) {
    ((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config.gyr_range= range;
}

// Get the bmi160 odr
float mbl_mw_gyro_bmi160_get_odr(MblMwMetaWearBoard *board) {
    switch(((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config.gyr_odr) {
        case MBL_MW_GYRO_BOSCH_ODR_25Hz:
            return 25.0f;
        case MBL_MW_GYRO_BOSCH_ODR_50Hz:
            return 50.0f;
        case MBL_MW_GYRO_BOSCH_ODR_100Hz:
            return 100.0f;
        case MBL_MW_GYRO_BOSCH_ODR_200Hz:
            return 200.0f;
        case MBL_MW_GYRO_BOSCH_ODR_400Hz:
            return 400.0f;
        case MBL_MW_GYRO_BOSCH_ODR_800Hz:
            return 800.0f;
        case MBL_MW_GYRO_BOSCH_ODR_1600Hz:
            return 1600.0f;
        case MBL_MW_GYRO_BOSCH_ODR_3200Hz:
            return 3200.0f;
        default:
            return 25.0f;
    }
}

// Get the bmi160 range
float mbl_mw_gyro_bmi160_get_range(MblMwMetaWearBoard *board) {
    switch(((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config.gyr_range) {
        case MBL_MW_GYRO_BOSCH_RANGE_2000dps:
            return 2000.0f;
        case MBL_MW_GYRO_BOSCH_RANGE_1000dps:
            return 1000.0f;
        case MBL_MW_GYRO_BOSCH_RANGE_500dps:
            return 500.0f;
        case MBL_MW_GYRO_BOSCH_RANGE_250dps:
            return 250.0f;
        case MBL_MW_GYRO_BOSCH_RANGE_125dps:
            return 125.0f;
        default:
            return 250.0f;
    }
}

// Write the bmi160 config
void mbl_mw_gyro_bmi160_write_config(const MblMwMetaWearBoard *board) {
    uint8_t command[4]= {MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi160Register::CONFIG)};
    auto config= &((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config;
    memcpy(command + 2, config, sizeof(*config));
    SEND_COMMAND;
}

// Set the bmi270 odr
void mbl_mw_gyro_bmi270_set_odr(MblMwMetaWearBoard *board, MblMwGyroBoschOdr odr) {
    ((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config.gyr_odr= odr;
}

// Set the bmi270 range
void mbl_mw_gyro_bmi270_set_range(MblMwMetaWearBoard *board, MblMwGyroBoschRange range) {
    ((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config.gyr_range= range;
}

// Get the bmi160 odr
float mbl_mw_gyro_bmi270_get_odr(MblMwMetaWearBoard *board) {
    switch(((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config.gyr_odr) {
        case MBL_MW_GYRO_BOSCH_ODR_25Hz:
            return 25.0f;
        case MBL_MW_GYRO_BOSCH_ODR_50Hz:
            return 50.0f;
        case MBL_MW_GYRO_BOSCH_ODR_100Hz:
            return 100.0f;
        case MBL_MW_GYRO_BOSCH_ODR_200Hz:
            return 200.0f;
        case MBL_MW_GYRO_BOSCH_ODR_400Hz:
            return 400.0f;
        case MBL_MW_GYRO_BOSCH_ODR_800Hz:
            return 800.0f;
        case MBL_MW_GYRO_BOSCH_ODR_1600Hz:
            return 1600.0f;
        case MBL_MW_GYRO_BOSCH_ODR_3200Hz:
            return 3200.0f;
        default:
            return 25.0f;
    }
}

// Get the bmi160 range
float mbl_mw_gyro_bmi270_get_range(MblMwMetaWearBoard *board) {
    switch(((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config.gyr_range) {
        case MBL_MW_GYRO_BOSCH_RANGE_2000dps:
            return 2000.0f;
        case MBL_MW_GYRO_BOSCH_RANGE_1000dps:
            return 1000.0f;
        case MBL_MW_GYRO_BOSCH_RANGE_500dps:
            return 500.0f;
        case MBL_MW_GYRO_BOSCH_RANGE_250dps:
            return 250.0f;
        case MBL_MW_GYRO_BOSCH_RANGE_125dps:
            return 125.0f;
        default:
            return 250.0f;
    }
}

// Write the bmi270 config
void mbl_mw_gyro_bmi270_write_config(const MblMwMetaWearBoard *board) {
    uint8_t command[4]= {MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi270Register::CONFIG)};
    auto config= &((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->config;
    memcpy(command + 2, config, sizeof(*config));
    SEND_COMMAND;
}

uint8_t mbl_mw_gyro_is_active(const MblMwMetaWearBoard *board) {
    auto config = ((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->interrupt;
    printf("Interrupt for Gyro is %d\n", config.enabled);
    return ((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->interrupt.enabled;
}

// Read the bmi160 interrupt register
void mbl_mw_gyro_read_interrupt(const MblMwMetaWearBoard* board, void *context, MblMwFnBoardPtrInt completed) {
    states[board].read_config_context = context;
    states[board].read_config_completed = completed;

    uint8_t command[2]= {MBL_MW_MODULE_GYRO, READ_REGISTER(ORDINAL(GyroBmi160Register::DATA_INTERRUPT_ENABLE))};
    SEND_COMMAND;
}

// Read the bmi160 config
void mbl_mw_gyro_bmi160_read_config(const MblMwMetaWearBoard* board, void *context, MblMwFnBoardPtrInt completed) {
    states[board].read_config_context = context;
    states[board].read_config_completed = completed;

    uint8_t command[2]= {MBL_MW_MODULE_GYRO, READ_REGISTER(ORDINAL(GyroBmi160Register::CONFIG))};
    SEND_COMMAND;
}

// Read the bmi270 config
void mbl_mw_gyro_bmi270_read_config(const MblMwMetaWearBoard* board, void *context, MblMwFnBoardPtrInt completed) {
    states[board].read_config_context = context;
    states[board].read_config_completed = completed;

    uint8_t command[2]= {MBL_MW_MODULE_GYRO, READ_REGISTER(ORDINAL(GyroBmi270Register::CONFIG))};
    SEND_COMMAND;
}

// Start the bmi160 gyro
void mbl_mw_gyro_bmi160_start(const MblMwMetaWearBoard *board) {
    uint8_t command[3]= {MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi160Register::POWER_MODE), 1};
    SEND_COMMAND;
}

// Stop the bmi160 gyro
void mbl_mw_gyro_bmi160_stop(const MblMwMetaWearBoard *board) {
    uint8_t command[3]= {MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi160Register::POWER_MODE), 0};
    SEND_COMMAND;
}

// Start the bmi270 gyro
void mbl_mw_gyro_bmi270_start(const MblMwMetaWearBoard *board) {
    uint8_t command[3]= {MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi270Register::POWER_MODE), 1};
    SEND_COMMAND;
}

// Stop the bmi270 gyro
void mbl_mw_gyro_bmi270_stop(const MblMwMetaWearBoard *board) {
    uint8_t command[3]= {MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi270Register::POWER_MODE), 0};
    SEND_COMMAND;
}

// Start sampling the bmi160 gyro
void mbl_mw_gyro_bmi160_enable_rotation_sampling(const MblMwMetaWearBoard *board) {
    ((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->interrupt.enabled = 1;
    uint8_t command[4]= {MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi160Register::DATA_INTERRUPT_ENABLE), 0x1, 0x0};
    SEND_COMMAND;
}

// Stop sampling the bmi160 gyro
void mbl_mw_gyro_bmi160_disable_rotation_sampling(const MblMwMetaWearBoard *board) {
    ((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->interrupt.enabled = 0;
    uint8_t command[4]= {MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi160Register::DATA_INTERRUPT_ENABLE), 0x0, 0x1};
    SEND_COMMAND;
}

// Start sampling the bmi270 gyro
void mbl_mw_gyro_bmi270_enable_rotation_sampling(const MblMwMetaWearBoard *board) {
    ((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->interrupt.enabled = 1;
    uint8_t command[4]= {MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi270Register::DATA_INTERRUPT_ENABLE), 0x1, 0x0};
    SEND_COMMAND;
}

// Stop sampling the bmi270 gyro
void mbl_mw_gyro_bmi270_disable_rotation_sampling(const MblMwMetaWearBoard *board) {
    ((GyroBoschConfig*) board->module_config.at(MBL_MW_MODULE_GYRO))->interrupt.enabled = 0;
    uint8_t command[4]= {MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi270Register::DATA_INTERRUPT_ENABLE), 0x0, 0x1};
    SEND_COMMAND;
}

// Manual gyroscope offsets
void mbl_mw_gyro_bmi270_offsets(const MblMwMetaWearBoard* board, uint8_t x_offset, uint8_t y_offset, uint8_t z_offset) {
    uint8_t gyr_usr_off_x_7_0 = x_offset;
    uint8_t gyr_usr_off_y_7_0 = y_offset;
    uint8_t gyr_usr_off_z_7_0 = z_offset;

    uint8_t command[5]= {MBL_MW_MODULE_GYRO, ORDINAL(GyroBmi270Register::OFFSET), gyr_usr_off_x_7_0, gyr_usr_off_y_7_0, gyr_usr_off_z_7_0};
    SEND_COMMAND;
}

// Name for the loggers
void create_gyro_uri(const MblMwDataSignal* signal, std::stringstream& uri) {
    switch(signal->owner->module_info.at(MBL_MW_MODULE_GYRO).implementation) {
    case MBL_MW_MODULE_GYRO_TYPE_BMI160:
        switch(CLEAR_READ(signal->header.register_id)) {
        case ORDINAL(GyroBmi160Register::DATA):
            uri << "angular-velocity";
            if (signal->length() <= 2) {
                uri << "[" << (int) (signal->offset >> 1) << "]";
            }
        }
        break;
    case MBL_MW_MODULE_GYRO_TYPE_BMI270:
        switch(CLEAR_READ(signal->header.register_id)) {
        case ORDINAL(GyroBmi270Register::DATA):
            uri << "angular-velocity";
            if (signal->length() <= 2) {
                uri << "[" << (int) (signal->offset >> 1) << "]";
            }
        }
        break;

    }
}
