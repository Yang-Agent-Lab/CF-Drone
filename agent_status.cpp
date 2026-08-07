#include "agent_status.h"

namespace agent_status {

namespace {

bool valid(const Snapshot& status) {
	return status.schema_version == kSchemaVersion &&
		status.mission_state <= flight_skills::MISSION_FAILED &&
		status.active_skill <= flight_skills::SKILL_LAND &&
		status.completed_skill <= flight_skills::SKILL_LAND &&
		status.flight_fault <= flight_skills::FAULT_UPSTREAM_GATE &&
		status.last_target_action <= flight_skills::ACTION_ABORT &&
		status.agent_fault <= agent_safety::FAULT_EMERGENCY_STOP &&
		status.gate_state <= agent_safety::STATE_FAULT_LATCHED;
}

bool validDecoded(const Snapshot& status) {
	return valid(status);
}

uint32_t field(uint32_t value, uint8_t shift, uint32_t mask) {
	return (value & mask) >> shift;
}

}  // namespace

bool encode(const Snapshot& status, int32_t& value) {
	if (!valid(status)) return false;
	const uint32_t encoded =
		(static_cast<uint32_t>(status.schema_version) << kSchemaVersionShift) |
		(static_cast<uint32_t>(status.mission_state) << kMissionStateShift) |
		(static_cast<uint32_t>(status.active_skill) << kActiveSkillShift) |
		(static_cast<uint32_t>(status.completed_skill) << kCompletedSkillShift) |
		(static_cast<uint32_t>(status.flight_fault) << kFlightFaultShift) |
		(static_cast<uint32_t>(status.last_target_action) << kLastTargetActionShift) |
		(static_cast<uint32_t>(status.agent_fault) << kAgentFaultShift) |
		(status.agent_owns_arm ? kAgentOwnsArmMask : 0) |
		(status.armed ? kArmedMask : 0) |
		(status.manual_control_active ? kManualControlActiveMask : 0) |
		(status.heartbeat_fresh ? kHeartbeatFreshMask : 0) |
		(static_cast<uint32_t>(status.gate_state) << kGateStateShift) |
		(status.landed ? kLandedMask : 0);
	value = static_cast<int32_t>(encoded);
	return true;
}

DecodeResult decode(int32_t value, Snapshot& status) {
	status = Snapshot();
	const uint32_t encoded = static_cast<uint32_t>(value);
	if ((encoded & kSchemaVersionMask) != kSchemaVersion) {
		return DECODE_UNSUPPORTED_VERSION;
	}
	if ((encoded & kReservedMask) != 0) return DECODE_RESERVED_BITS;

	Snapshot decoded = {};
	decoded.schema_version = kSchemaVersion;
	decoded.mission_state = static_cast<flight_skills::MissionState>(
		field(encoded, kMissionStateShift, kMissionStateMask));
	decoded.active_skill = static_cast<flight_skills::Skill>(
		field(encoded, kActiveSkillShift, kActiveSkillMask));
	decoded.completed_skill = static_cast<flight_skills::Skill>(
		field(encoded, kCompletedSkillShift, kCompletedSkillMask));
	decoded.flight_fault = static_cast<flight_skills::Fault>(
		field(encoded, kFlightFaultShift, kFlightFaultMask));
	decoded.last_target_action = static_cast<flight_skills::Action>(
		field(encoded, kLastTargetActionShift, kLastTargetActionMask));
	decoded.agent_fault = static_cast<agent_safety::Fault>(
		field(encoded, kAgentFaultShift, kAgentFaultMask));
	decoded.agent_owns_arm = (encoded & kAgentOwnsArmMask) != 0;
	decoded.armed = (encoded & kArmedMask) != 0;
	decoded.manual_control_active = (encoded & kManualControlActiveMask) != 0;
	decoded.heartbeat_fresh = (encoded & kHeartbeatFreshMask) != 0;
	decoded.gate_state = static_cast<agent_safety::State>(
		field(encoded, kGateStateShift, kGateStateMask));
	decoded.landed = (encoded & kLandedMask) != 0;
	if (!validDecoded(decoded)) return DECODE_INVALID_FIELD;
	status = decoded;
	return DECODE_OK;
}

}  // namespace agent_status
