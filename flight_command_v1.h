#pragma once

#include <stdint.h>

#include "flight_skills.h"

// Offline v1 wire decoder. It validates only the high-level skill payload;
// it does not start a flight skill or create a control target.
namespace flight_command_v1 {

bool isHighLevelSkill(uint16_t skill);

bool decode(uint16_t skill, uint32_t request_id, float param4, float param5,
	        float param6, float param7, flight_skills::Request& output);

}  // namespace flight_command_v1
