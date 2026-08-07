#include "agent_safety.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

using namespace agent_safety;

static Snapshot safeSnapshot() {
	Snapshot snapshot = {};
	snapshot.throttle_low = true;
	snapshot.battery_ok = true;
	snapshot.attitude_ok = true;
	snapshot.landed = true;
	return snapshot;
}

static void startSession(Gate& gate, uint32_t now_ms = 100) {
	Decision decision = gate.handle(
		now_ms, SKILL_HEARTBEAT, 1, kConfirmationCode, true, safeSnapshot());
	assert(decision.result == RESULT_ACCEPTED);
	assert(gate.state() == STATE_READY);
}

static void armSession(Gate& gate, uint32_t now_ms = 200, uint32_t request_id = 2) {
	Decision decision = gate.handle(
		now_ms, SKILL_ARM, request_id, kConfirmationCode, true, safeSnapshot());
	assert(decision.result == RESULT_ACCEPTED);
	assert(decision.arm);
	assert(!decision.disarm);
	assert(gate.state() == STATE_AGENT_ARMED);
}

static void testAckResultParam2() {
	const uint32_t accepted = static_cast<uint32_t>(
		encodeAckResultParam2(42, RESULT_ACCEPTED));
	assert(accepted == 42);

	const uint32_t rejected = static_cast<uint32_t>(
		encodeAckResultParam2(42, RESULT_INVALID_REQUEST));
	assert(rejected == (static_cast<uint32_t>(RESULT_INVALID_REQUEST) <<
	                    kAckResultShift) + 42);

	const uint32_t maximum_request = static_cast<uint32_t>(
		encodeAckResultParam2(kAckRequestIdMax, RESULT_FLIGHT_REJECTED));
	assert((maximum_request & kAckRequestIdMax) == kAckRequestIdMax);
	assert((maximum_request & 0x80000000UL) == 0);
}

static void testLegalSessionAndDuplicate() {
	Gate gate;
	startSession(gate);
	armSession(gate);

	Snapshot armed = safeSnapshot();
	armed.armed = true;
	Decision duplicate = gate.handle(
		250, SKILL_ARM, 2, kConfirmationCode, true, armed);
	assert(duplicate.result == RESULT_DUPLICATE);
	assert(!duplicate.arm);
	assert(!duplicate.disarm);
	assert(gate.state() == STATE_AGENT_ARMED);

	Decision disarm = gate.handle(
		300, SKILL_DISARM, 3, kConfirmationCode, true, armed);
	assert(disarm.result == RESULT_ACCEPTED);
	assert(disarm.disarm);
	assert(gate.state() == STATE_READY);
}

static void testArmPreconditions() {
	struct Case {
		Snapshot snapshot;
		Result expected;
	};

	Snapshot already_armed = safeSnapshot();
	already_armed.armed = true;
	Snapshot throttle_high = safeSnapshot();
	throttle_high.throttle_low = false;
	Snapshot inverted = safeSnapshot();
	inverted.inverted = true;
	Snapshot low_battery = safeSnapshot();
	low_battery.battery_ok = false;
	Snapshot invalid_attitude = safeSnapshot();
	invalid_attitude.attitude_ok = false;
	Snapshot manual = safeSnapshot();
	manual.manual_control_active = true;
	Snapshot airborne = safeSnapshot();
	airborne.landed = false;

	const Case cases[] = {
		{already_armed, RESULT_ALREADY_ARMED},
		{throttle_high, RESULT_THROTTLE_NOT_LOW},
		{inverted, RESULT_INVERTED},
		{low_battery, RESULT_BATTERY_UNSAFE},
		{invalid_attitude, RESULT_ATTITUDE_INVALID},
		{airborne, RESULT_NOT_LANDED},
		{manual, RESULT_MANUAL_CONTROL_ACTIVE},
	};

	for (unsigned int i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
		Gate gate;
		startSession(gate);
		Decision decision = gate.handle(
			200, SKILL_ARM, 2, kConfirmationCode, true, cases[i].snapshot);
		assert(decision.result == cases[i].expected);
		assert(!decision.arm);
	}
}

static void testHeartbeatTimeout() {
	Gate gate;
	startSession(gate);
	armSession(gate);

	Snapshot armed = safeSnapshot();
	armed.armed = true;
	Decision decision = gate.update(100 + kHeartbeatTimeoutMs + 1, armed);
	assert(decision.result == RESULT_FAULT_LATCHED);
	assert(decision.disarm);
	assert(gate.state() == STATE_FAULT_LATCHED);
	assert(gate.fault() == FAULT_HEARTBEAT_TIMEOUT);
}

static void testRuntimeHazards() {
	struct Case {
		Snapshot snapshot;
		Fault expected;
	};

	Snapshot low_battery = safeSnapshot();
	low_battery.armed = true;
	low_battery.battery_ok = false;
	Snapshot inverted = safeSnapshot();
	inverted.armed = true;
	inverted.inverted = true;
	Snapshot invalid_attitude = safeSnapshot();
	invalid_attitude.armed = true;
	invalid_attitude.attitude_ok = false;
	Snapshot manual = safeSnapshot();
	manual.armed = true;
	manual.manual_control_active = true;

	const Case cases[] = {
		{low_battery, FAULT_LOW_BATTERY},
		{inverted, FAULT_INVERTED},
		{invalid_attitude, FAULT_ATTITUDE_INVALID},
		{manual, FAULT_MANUAL_TAKEOVER},
	};

	for (unsigned int i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
		Gate gate;
		startSession(gate);
		armSession(gate);
		Decision decision = gate.update(300, cases[i].snapshot);
		assert(decision.result == RESULT_FAULT_LATCHED);
		assert(decision.disarm);
		assert(gate.fault() == cases[i].expected);
	}

	Gate priority_gate;
	startSession(priority_gate);
	armSession(priority_gate);
	Snapshot simultaneous = safeSnapshot();
	simultaneous.armed = true;
	simultaneous.manual_control_active = true;
	simultaneous.battery_ok = false;
	Decision priority = priority_gate.update(300, simultaneous);
	assert(priority.result == RESULT_FAULT_LATCHED);
	assert(priority_gate.fault() == FAULT_MANUAL_TAKEOVER);

	Gate manual_gate;
	startSession(manual_gate);
	Snapshot manual_flight = safeSnapshot();
	manual_flight.armed = true;
	manual_flight.manual_control_active = true;
	Decision preserved = manual_gate.update(200, manual_flight);
	assert(preserved.result == RESULT_FAULT_LATCHED);
	assert(!preserved.disarm);
	assert(manual_gate.fault() == FAULT_MANUAL_TAKEOVER);
}

static void testUnknownSkillAndLatchedFault() {
	Gate gate;
	startSession(gate);
	Decision unknown = gate.handle(
		200, static_cast<Skill>(999), 2, kConfirmationCode, true, safeSnapshot());
	assert(unknown.result == RESULT_UNKNOWN_SKILL);
	assert(gate.state() == STATE_FAULT_LATCHED);
	assert(gate.fault() == FAULT_ILLEGAL_COMMAND);

	Decision heartbeat = gate.handle(
		300, SKILL_HEARTBEAT, 3, kConfirmationCode, true, safeSnapshot());
	assert(heartbeat.result == RESULT_FAULT_LATCHED);
	Decision arm = gate.handle(
		300, SKILL_ARM, 4, kConfirmationCode, true, safeSnapshot());
	assert(arm.result == RESULT_FAULT_LATCHED);
	assert(!arm.arm);
}

static void testInvalidRequestLatches() {
	Gate bad_confirmation;
	startSession(bad_confirmation);
	Decision confirmation = bad_confirmation.handle(
		200, SKILL_ARM, 2, kConfirmationCode + 1, true, safeSnapshot());
	assert(confirmation.result == RESULT_INVALID_REQUEST);
	assert(bad_confirmation.fault() == FAULT_ILLEGAL_COMMAND);

	Gate bad_arguments;
	startSession(bad_arguments);
	Decision arguments = bad_arguments.handle(
		200, SKILL_ARM, 2, kConfirmationCode, false, safeSnapshot());
	assert(arguments.result == RESULT_INVALID_REQUEST);
	assert(bad_arguments.fault() == FAULT_ILLEGAL_COMMAND);
}

static void testHighLevelSkillsNeedAgentArm() {
	const Skill skills[] = {
		SKILL_TAKEOFF,
		SKILL_LAND,
		SKILL_HOLD,
		SKILL_MOVE_BODY,
		SKILL_YAW,
		SKILL_RETURN_HOME,
	};
	for (unsigned int i = 0; i < sizeof(skills) / sizeof(skills[0]); ++i) {
		Gate gate;
		startSession(gate);
		Decision unarmed = gate.handle(
			200, skills[i], 2, kConfirmationCode, true, safeSnapshot());
		assert(unarmed.result == RESULT_AGENT_NOT_ARMED);
		armSession(gate);
		Snapshot armed = safeSnapshot();
		armed.armed = true;
		Decision decision = gate.handle(
			300, skills[i], 3, kConfirmationCode, true, armed);
		assert(decision.result == RESULT_ACCEPTED);
		assert(!decision.arm && !decision.disarm);
	}
}

static void testForbiddenDirectMessagesAndCommands() {
	const uint32_t prohibited_messages[] = {69, 23, 126, 82, 139};
	for (unsigned int i = 0;
	     i < sizeof(prohibited_messages) / sizeof(prohibited_messages[0]); ++i) {
		assert(!isAgentMessageAllowed(prohibited_messages[i], 0));
	}
	assert(!isAgentMessageAllowed(kCommandLongMessageId, 400));
	assert(!isAgentMessageAllowed(kCommandLongMessageId, 176));
	assert(isAgentMessageAllowed(kCommandLongMessageId, kPrivateCommand));

	Gate gate;
	startSession(gate);
	armSession(gate);
	Decision rejected = gate.rejectIllegalMessage();
	assert(rejected.result == RESULT_ILLEGAL_MESSAGE);
	assert(rejected.disarm);
	assert(gate.fault() == FAULT_ILLEGAL_COMMAND);
}

static void testEmergencyStopPriority() {
	Gate gate;
	Decision decision = gate.handle(
		10, SKILL_EMERGENCY_STOP, 0, 0, false, safeSnapshot());
	assert(decision.result == RESULT_ACCEPTED);
	assert(decision.disarm);
	assert(gate.fault() == FAULT_EMERGENCY_STOP);
}

int main() {
	testAckResultParam2();
	testLegalSessionAndDuplicate();
	testArmPreconditions();
	testHeartbeatTimeout();
	testRuntimeHazards();
	testUnknownSkillAndLatchedFault();
	testInvalidRequestLatches();
	testHighLevelSkillsNeedAgentArm();
	testForbiddenDirectMessagesAndCommands();
	testEmergencyStopPriority();
	puts("agent safety tests: PASS");
}
