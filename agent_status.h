#pragma once

#include <stdint.h>

#include "agent_safety.h"
#include "flight_skills.h"

// Compact, versioned status carried by MAVLink common NAMED_VALUE_INT.
// All fields are unsigned and bits 30..31 are reserved as zero for v1.
namespace agent_status {

static const uint8_t kSchemaVersion = 1;

static const uint8_t kSchemaVersionShift = 0;
static const uint8_t kMissionStateShift = 3;
static const uint8_t kActiveSkillShift = 6;
static const uint8_t kCompletedSkillShift = 9;
static const uint8_t kFlightFaultShift = 12;
static const uint8_t kLastTargetActionShift = 17;
static const uint8_t kAgentFaultShift = 20;
static const uint8_t kAgentOwnsArmShift = 23;
static const uint8_t kArmedShift = 24;
static const uint8_t kManualControlActiveShift = 25;
static const uint8_t kHeartbeatFreshShift = 26;
static const uint8_t kGateStateShift = 27;
static const uint8_t kLandedShift = 29;
static const uint8_t kReservedShift = 30;

static const uint32_t kSchemaVersionMask = 0x00000007UL;
static const uint32_t kMissionStateMask = 0x00000038UL;
static const uint32_t kActiveSkillMask = 0x000001C0UL;
static const uint32_t kCompletedSkillMask = 0x00000E00UL;
static const uint32_t kFlightFaultMask = 0x0001F000UL;
static const uint32_t kLastTargetActionMask = 0x000E0000UL;
static const uint32_t kAgentFaultMask = 0x00700000UL;
static const uint32_t kAgentOwnsArmMask = 0x00800000UL;
static const uint32_t kArmedMask = 0x01000000UL;
static const uint32_t kManualControlActiveMask = 0x02000000UL;
static const uint32_t kHeartbeatFreshMask = 0x04000000UL;
static const uint32_t kGateStateMask = 0x18000000UL;
static const uint32_t kLandedMask = 0x20000000UL;
static const uint32_t kReservedMask = 0xC0000000UL;

struct Snapshot {
	uint8_t schema_version;
	flight_skills::MissionState mission_state;
	flight_skills::Skill active_skill;
	flight_skills::Skill completed_skill;
	flight_skills::Fault flight_fault;
	flight_skills::Action last_target_action;
	agent_safety::Fault agent_fault;
	agent_safety::State gate_state;
	bool agent_owns_arm;
	bool armed;
	bool manual_control_active;
	bool heartbeat_fresh;
	bool landed;
};

enum DecodeResult : uint8_t {
	DECODE_OK,
	DECODE_UNSUPPORTED_VERSION,
	DECODE_RESERVED_BITS,
	DECODE_INVALID_FIELD,
};

bool encode(const Snapshot& status, int32_t& value);
DecodeResult decode(int32_t value, Snapshot& status);

}  // namespace agent_status
