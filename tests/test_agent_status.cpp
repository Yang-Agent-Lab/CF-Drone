#include "agent_status.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

static agent_status::Snapshot status() {
	agent_status::Snapshot value = {};
	value.schema_version = agent_status::kSchemaVersion;
	value.mission_state = flight_skills::MISSION_COMPLETE;
	value.active_skill = flight_skills::SKILL_LAND;
	value.completed_skill = flight_skills::SKILL_TAKEOFF;
	value.flight_fault = flight_skills::FAULT_RANGE_LOSS;
	value.last_target_action = flight_skills::ACTION_REQUEST_LOCK;
	value.agent_fault = agent_safety::FAULT_MANUAL_TAKEOVER;
	value.gate_state = agent_safety::STATE_FAULT_LATCHED;
	value.agent_owns_arm = true;
	value.armed = false;
	value.manual_control_active = true;
	value.heartbeat_fresh = false;
	value.landed = true;
	return value;
}

static void testRoundTripAndFixedLayout() {
	const agent_status::Snapshot original = status();
	int32_t encoded = 0;
	assert(agent_status::encode(original, encoded));

	const uint32_t expected =
		(static_cast<uint32_t>(agent_status::kSchemaVersion)
		 << agent_status::kSchemaVersionShift) |
		(static_cast<uint32_t>(flight_skills::MISSION_COMPLETE)
		 << agent_status::kMissionStateShift) |
		(static_cast<uint32_t>(flight_skills::SKILL_LAND)
		 << agent_status::kActiveSkillShift) |
		(static_cast<uint32_t>(flight_skills::SKILL_TAKEOFF)
		 << agent_status::kCompletedSkillShift) |
		(static_cast<uint32_t>(flight_skills::FAULT_RANGE_LOSS)
		 << agent_status::kFlightFaultShift) |
		(static_cast<uint32_t>(flight_skills::ACTION_REQUEST_LOCK)
		 << agent_status::kLastTargetActionShift) |
		(static_cast<uint32_t>(agent_safety::FAULT_MANUAL_TAKEOVER)
		 << agent_status::kAgentFaultShift) |
		(static_cast<uint32_t>(agent_safety::STATE_FAULT_LATCHED)
		 << agent_status::kGateStateShift) |
		(1UL << agent_status::kAgentOwnsArmShift) |
		(1UL << agent_status::kManualControlActiveShift) |
		(1UL << agent_status::kLandedShift);
	assert(static_cast<uint32_t>(encoded) == expected);

	agent_status::Snapshot decoded = {};
	assert(agent_status::decode(encoded, decoded) == agent_status::DECODE_OK);
	assert(decoded.schema_version == original.schema_version);
	assert(decoded.mission_state == original.mission_state);
	assert(decoded.active_skill == original.active_skill);
	assert(decoded.completed_skill == original.completed_skill);
	assert(decoded.flight_fault == original.flight_fault);
	assert(decoded.last_target_action == original.last_target_action);
	assert(decoded.agent_fault == original.agent_fault);
	assert(decoded.gate_state == original.gate_state);
	assert(decoded.agent_owns_arm == original.agent_owns_arm);
	assert(decoded.armed == original.armed);
	assert(decoded.manual_control_active == original.manual_control_active);
	assert(decoded.heartbeat_fresh == original.heartbeat_fresh);
	assert(decoded.landed == original.landed);
}

static void testEncoderRejectsWrongVersionAndEnumRanges() {
	agent_status::Snapshot invalid = status();
	int32_t encoded = 0;
	invalid.schema_version = agent_status::kSchemaVersion + 1;
	assert(!agent_status::encode(invalid, encoded));
	invalid = status();
	invalid.mission_state = static_cast<flight_skills::MissionState>(5);
	assert(!agent_status::encode(invalid, encoded));
	invalid = status();
	invalid.active_skill = static_cast<flight_skills::Skill>(7);
	assert(!agent_status::encode(invalid, encoded));
	invalid = status();
	invalid.completed_skill = static_cast<flight_skills::Skill>(7);
	assert(!agent_status::encode(invalid, encoded));
	invalid = status();
	invalid.flight_fault = static_cast<flight_skills::Fault>(19);
	assert(!agent_status::encode(invalid, encoded));
	invalid = status();
	invalid.last_target_action = static_cast<flight_skills::Action>(7);
	assert(!agent_status::encode(invalid, encoded));
	invalid = status();
	invalid.agent_fault = static_cast<agent_safety::Fault>(8);
	assert(!agent_status::encode(invalid, encoded));
	invalid = status();
	invalid.gate_state = static_cast<agent_safety::State>(4);
	assert(!agent_status::encode(invalid, encoded));
}

static void testDecoderRejectsVersionReservedAndInvalidFields() {
	int32_t encoded = 0;
	assert(agent_status::encode(status(), encoded));
	agent_status::Snapshot decoded = {};

	const uint32_t bad_version =
		(static_cast<uint32_t>(encoded) & ~agent_status::kSchemaVersionMask) | 2U;
	assert(agent_status::decode(static_cast<int32_t>(bad_version), decoded) ==
	       agent_status::DECODE_UNSUPPORTED_VERSION);
	assert(decoded.schema_version == 0);

	const uint32_t reserved = static_cast<uint32_t>(encoded) |
		(1UL << agent_status::kReservedShift);
	assert(agent_status::decode(static_cast<int32_t>(reserved), decoded) ==
	       agent_status::DECODE_RESERVED_BITS);
	assert(decoded.schema_version == 0);

	const uint32_t bad_mission =
		(static_cast<uint32_t>(encoded) & ~agent_status::kMissionStateMask) |
		(5UL << agent_status::kMissionStateShift);
	assert(agent_status::decode(static_cast<int32_t>(bad_mission), decoded) ==
	       agent_status::DECODE_INVALID_FIELD);
	assert(decoded.schema_version == 0);
}

int main() {
	testRoundTripAndFixedLayout();
	testEncoderRejectsWrongVersionAndEnumRanges();
	testDecoderRejectsVersionReservedAndInvalidFields();
	puts("agent status tests: PASS");
}
