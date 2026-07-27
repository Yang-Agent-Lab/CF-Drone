#include "agent_safety.h"

namespace agent_safety {

namespace {

Decision decision(Result result, bool arm = false, bool disarm = false) {
	Decision value = {result, arm, disarm};
	return value;
}

bool knownSkill(Skill skill) {
	switch (skill) {
		case SKILL_HEARTBEAT:
		case SKILL_ARM:
		case SKILL_DISARM:
		case SKILL_EMERGENCY_STOP:
		case SKILL_TAKEOFF:
		case SKILL_LAND:
		case SKILL_HOLD:
		case SKILL_MOVE_BODY:
			return true;
	}
	return false;
}

bool unimplementedSkill(Skill skill) {
	return skill == SKILL_TAKEOFF || skill == SKILL_LAND ||
	       skill == SKILL_HOLD || skill == SKILL_MOVE_BODY;
}

}  // namespace

bool isAgentMessageAllowed(uint32_t message_id, uint16_t command) {
	return message_id == kCommandLongMessageId && command == kPrivateCommand;
}

Gate::Gate()
	: state_(STATE_IDLE),
	  fault_(FAULT_NONE),
	  agent_owns_arm_(false),
	  heartbeat_seen_(false),
	  last_heartbeat_ms_(0),
	  last_execution_valid_(false),
	  last_execution_request_(0),
	  last_execution_skill_(SKILL_HEARTBEAT) {}

bool Gate::heartbeatFresh(uint32_t now_ms) const {
	return heartbeat_seen_ &&
	       static_cast<uint32_t>(now_ms - last_heartbeat_ms_) <= kHeartbeatTimeoutMs;
}

Decision Gate::latch(Fault fault, Result result) {
	const bool disarm = agent_owns_arm_;
	state_ = STATE_FAULT_LATCHED;
	fault_ = fault;
	agent_owns_arm_ = false;
	return decision(result, false, disarm);
}

Decision Gate::handle(uint32_t now_ms, Skill skill, uint32_t request_id,
                      uint32_t confirmation_code, bool arguments_zero,
                      const Snapshot& snapshot) {
	// Emergency stop is deliberately honored even when the session is stale or
	// malformed. Stopping cannot grant the Agent additional authority.
	if (skill == SKILL_EMERGENCY_STOP) {
		Decision stopped = latch(FAULT_EMERGENCY_STOP, RESULT_ACCEPTED);
		stopped.disarm = true;
		return stopped;
	}

	if (!knownSkill(skill)) {
		return latch(FAULT_ILLEGAL_COMMAND, RESULT_UNKNOWN_SKILL);
	}

	if (request_id == 0 || confirmation_code != kConfirmationCode ||
	    !arguments_zero) {
		return latch(FAULT_ILLEGAL_COMMAND, RESULT_INVALID_REQUEST);
	}

	if (skill != SKILL_HEARTBEAT && last_execution_valid_ &&
	    request_id == last_execution_request_) {
		if (skill == last_execution_skill_) return decision(RESULT_DUPLICATE);
		return latch(FAULT_ILLEGAL_COMMAND, RESULT_INVALID_REQUEST);
	}

	if (skill == SKILL_DISARM) {
		const bool fault_latched = state_ == STATE_FAULT_LATCHED;
		agent_owns_arm_ = false;
		if (!fault_latched) {
			state_ = heartbeatFresh(now_ms) ? STATE_READY : STATE_IDLE;
		}
		last_execution_valid_ = true;
		last_execution_request_ = request_id;
		last_execution_skill_ = skill;
		return decision(RESULT_ACCEPTED, false, true);
	}

	if (state_ == STATE_FAULT_LATCHED) {
		return decision(RESULT_FAULT_LATCHED);
	}

	if (skill == SKILL_HEARTBEAT) {
		heartbeat_seen_ = true;
		last_heartbeat_ms_ = now_ms;
		if (state_ == STATE_IDLE) state_ = STATE_READY;
		return decision(RESULT_ACCEPTED);
	}

	if (unimplementedSkill(skill)) {
		return decision(RESULT_NOT_IMPLEMENTED);
	}

	if (!heartbeatFresh(now_ms)) {
		return latch(FAULT_HEARTBEAT_TIMEOUT, RESULT_HEARTBEAT_STALE);
	}
	if (snapshot.armed) return decision(RESULT_ALREADY_ARMED);
	if (!snapshot.throttle_low) return decision(RESULT_THROTTLE_NOT_LOW);
	if (snapshot.inverted) return decision(RESULT_INVERTED);
	if (!snapshot.battery_ok) return decision(RESULT_BATTERY_UNSAFE);
	if (!snapshot.attitude_ok) return decision(RESULT_ATTITUDE_INVALID);
	if (!snapshot.landed) return decision(RESULT_NOT_LANDED);
	if (snapshot.manual_control_active) {
		return decision(RESULT_MANUAL_CONTROL_ACTIVE);
	}

	state_ = STATE_AGENT_ARMED;
	agent_owns_arm_ = true;
	last_execution_valid_ = true;
	last_execution_request_ = request_id;
	last_execution_skill_ = skill;
	return decision(RESULT_ACCEPTED, true, false);
}

Decision Gate::update(uint32_t now_ms, const Snapshot& snapshot) {
	if (state_ == STATE_IDLE || state_ == STATE_FAULT_LATCHED) {
		return decision(RESULT_ACCEPTED);
	}
	if (!heartbeatFresh(now_ms)) {
		return latch(FAULT_HEARTBEAT_TIMEOUT, RESULT_FAULT_LATCHED);
	}

	// Manual input wins over every Agent-owned state.
	if (snapshot.manual_control_active ||
	    (state_ == STATE_READY && snapshot.armed) ||
	    (state_ == STATE_AGENT_ARMED && !snapshot.armed)) {
		return latch(FAULT_MANUAL_TAKEOVER, RESULT_FAULT_LATCHED);
	}
	if (!snapshot.battery_ok) {
		return latch(FAULT_LOW_BATTERY, RESULT_FAULT_LATCHED);
	}
	if (snapshot.inverted) {
		return latch(FAULT_INVERTED, RESULT_FAULT_LATCHED);
	}
	if (!snapshot.attitude_ok) {
		return latch(FAULT_ATTITUDE_INVALID, RESULT_FAULT_LATCHED);
	}
	return decision(RESULT_ACCEPTED);
}

Decision Gate::rejectIllegalMessage() {
	return latch(FAULT_ILLEGAL_COMMAND, RESULT_ILLEGAL_MESSAGE);
}

}  // namespace agent_safety
