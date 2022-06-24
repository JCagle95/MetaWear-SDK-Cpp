#pragma once

#include <sstream>
#include <vector>

#include "metawear/core/metawearboard_fwd.h"

void init_ambient_light_module(MblMwMetaWearBoard *board);
void mbl_mw_als_ltr329_read_config(const MblMwMetaWearBoard *board, void *context, MblMwFnBoardPtrInt completed);
void serialize_ambient_light_config(const MblMwMetaWearBoard *board, std::vector<uint8_t>& state);
void deserialize_ambient_light_config(MblMwMetaWearBoard *board, uint8_t** state_stream);
void create_als_uri(const MblMwDataSignal* signal, std::stringstream& uri);
