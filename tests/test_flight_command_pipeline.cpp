#include "flight_command_v1.h"
#include "flight_command_pipeline.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

using namespace flight_skills;
using agent_safety::kConfirmationCode;
using sensor_telemetry::BarometerSample;
using sensor_telemetry::HealthySnapshot;
using sensor_telemetry::OpticalFlowSample;
using sensor_telemetry::RangeSample;
using sensor_telemetry::TelemetryAggregator;

static SafetyStatus safeStatus() {
	SafetyStatus status = {};
	status.upstream_gate_ok = true;
	status.battery_ok = true;
	status.heartbeat_fresh = true;
	return status;
}

static VehicleState vehicle(uint64_t now_ms, float x_m = 0.0f,
	                        float y_m = 0.0f, float altitude_m = 0.0f,
	                        float yaw_deg = 0.0f, bool landed = false) {
	VehicleState state = {};
	state.timestamp_ms = now_ms;
	state.armed = true;
	state.landed = landed;
	state.local_x_m = x_m;
	state.local_y_m = y_m;
	state.altitude_m = altitude_m;
	state.yaw_deg = yaw_deg;
	return state;
}

static HealthySnapshot sensors(uint64_t now_ms, float range_m = 1.0f) {
	TelemetryAggregator telemetry;
	const OpticalFlowSample flow = {now_ms, true, true, 0.0f, 0.0f, 200};
	const RangeSample range = {now_ms, true, true, range_m, 200};
	const BarometerSample barometer = {
		now_ms, true, true, 101325.0f, 20.0f, 1.0f, 200
	};
	telemetry.submitOpticalFlow(flow);
	telemetry.submitRange(range);
	telemetry.submitBarometer(barometer);
	return telemetry.snapshot(now_ms);
}

static Request decode(uint16_t skill, uint32_t request_id, float param4,
	                  float param5, float param6, Skill expected_skill) {
	Request request = {};
	assert(flight_command_v1::decode(skill, request_id, param4, param5,
	                                 param6, 1.0f, request));
	assert(request.request_id == request_id);
	assert(request.skill == expected_skill);
	return request;
}

static void testV1CommandsReachFlightMachine() {
	Machine machine;
	uint64_t now = 100;
	assert(machine.beginAtArm(now, vehicle(now, 0, 0, 0, 0, true),
	                          safeStatus()) == RESULT_ACCEPTED);

	Request takeoff = decode(100, 1, 0.5f, 0.0f, 0.0f, SKILL_TAKEOFF);
	assert(machine.start(now, takeoff, sensors(now), vehicle(now),
	                     safeStatus()) == RESULT_ACCEPTED);
	assert(machine.activeSkill() == SKILL_TAKEOFF);
	assert(machine.update(now, sensors(now), vehicle(now), safeStatus()).valid);
	now = 200;
	assert(!machine.update(now, sensors(now), vehicle(now, 0, 0, 0.5f),
	                       safeStatus()).valid);

	Request hold = decode(102, 2, 0.125f, 0.0f, 0.0f, SKILL_HOLD);
	assert(hold.duration_ms == 125);
	assert(machine.start(now, hold, sensors(now), vehicle(now, 0, 0, 0.5f),
	                     safeStatus()) == RESULT_ACCEPTED);
	now += hold.duration_ms;
	assert(!machine.update(now, sensors(now), vehicle(now, 0, 0, 0.5f),
	                       safeStatus()).valid);

	Request move = decode(103, 3, 0.3f, 0.4f, kMaxHorizontalSpeedMps,
	                      SKILL_MOVE_BODY);
	assert(move.speed_m_s == kMaxHorizontalSpeedMps);
	assert(machine.start(now, move, sensors(now), vehicle(now, 0, 0, 0.5f),
	                     safeStatus()) == RESULT_ACCEPTED);
	Target target = machine.update(now, sensors(now), vehicle(now, 0, 0, 0.5f),
	                               safeStatus());
	assert(target.horizontal_frame == FRAME_BODY);
	assert(fabsf(hypotf(target.velocity_x_m_s, target.velocity_y_m_s) -
	             move.speed_m_s) < 0.0001f);
	now += 100;
	assert(!machine.update(now, sensors(now), vehicle(now, 0.3f, 0.4f, 0.5f),
	                       safeStatus()).valid);

	Request yaw = decode(104, 4, 90.0f, kMaxYawRateDegS, 0.0f, SKILL_YAW);
	assert(yaw.yaw_rate_deg_s == kMaxYawRateDegS);
	assert(machine.start(now, yaw, sensors(now),
	                     vehicle(now, 0.3f, 0.4f, 0.5f), safeStatus()) ==
	       RESULT_ACCEPTED);
	target = machine.update(now, sensors(now), vehicle(now, 0.3f, 0.4f, 0.5f),
	                        safeStatus());
	assert(target.yaw_rate_deg_s == yaw.yaw_rate_deg_s);
	now += 100;
	assert(!machine.update(now, sensors(now),
	                       vehicle(now, 0.3f, 0.4f, 0.5f, 90.0f),
	                       safeStatus()).valid);

	Request return_home = decode(105, 5, kMaxHorizontalSpeedMps, 0.0f, 0.0f,
	                             SKILL_RETURN_HOME);
	assert(return_home.speed_m_s == kMaxHorizontalSpeedMps);
	assert(machine.start(now, return_home, sensors(now),
	                     vehicle(now, 0.3f, 0.4f, 0.5f, 90.0f), safeStatus()) ==
	       RESULT_ACCEPTED);
	target = machine.update(now, sensors(now),
	                        vehicle(now, 0.3f, 0.4f, 0.5f, 90.0f), safeStatus());
	assert(target.horizontal_frame == FRAME_LOCAL);
	assert(fabsf(hypotf(target.velocity_x_m_s, target.velocity_y_m_s) -
	             return_home.speed_m_s) < 0.0001f);
	now += 100;
	assert(!machine.update(now, sensors(now), vehicle(now, 0, 0, 0.5f, 90.0f),
	                       safeStatus()).valid);

	Request land = decode(101, 6, 0.0f, 0.0f, 0.0f, SKILL_LAND);
	assert(machine.start(now, land, sensors(now), vehicle(now, 0, 0, 0.5f, 90.0f),
	                     safeStatus()) == RESULT_ACCEPTED);
	target = machine.update(now, sensors(now), vehicle(now, 0, 0, 0.5f, 90.0f),
	                        safeStatus());
	assert(target.action == ACTION_LAND);
	now += 100;
	machine.update(now, sensors(now, 0.05f), vehicle(now, 0, 0, 0, 90.0f, true),
	               safeStatus());
	now += kGroundConfirmMs;
	target = machine.update(now, sensors(now, 0.05f),
	                        vehicle(now, 0, 0, 0, 90.0f, true), safeStatus());
	assert(target.action == ACTION_REQUEST_LOCK);
	assert(machine.missionState() == MISSION_COMPLETE);
}

static void testInvalidV1CommandDoesNotStartMachine() {
	Machine machine;
	const uint64_t now = 100;
	assert(machine.beginAtArm(now, vehicle(now, 0, 0, 0, 0, true),
	                          safeStatus()) == RESULT_ACCEPTED);
	Request request = {};
	assert(!flight_command_v1::decode(103, 1, 0.1f, 0.0f, 0.1f, 1.0f, request));
	assert(request.request_id == 0);
	assert(request.skill == SKILL_NONE);
	assert(machine.missionState() == MISSION_ACTIVE);
	assert(machine.activeSkill() == SKILL_NONE);
}

static void testGuardedPipelineNeedsBothAcceptances() {
	flight_command_pipeline::Pipeline pipeline;
	const uint64_t now = 100;
	const agent_safety::Snapshot safety_snapshot = [] {
		agent_safety::Snapshot snapshot = {};
		snapshot.throttle_low = true;
		snapshot.battery_ok = true;
		snapshot.attitude_ok = true;
		snapshot.landed = true;
		return snapshot;
	}();
	Request request = decode(100, 3, 0.5f, 0.0f, 0.0f, SKILL_TAKEOFF);

	flight_command_pipeline::Outcome heartbeat = pipeline.handle(
		now, agent_safety::SKILL_HEARTBEAT, 1, kConfirmationCode, true,
		Request(), safety_snapshot, vehicle(now, 0, 0, 0, 0, true),
		sensors(now));
	assert(heartbeat.result == agent_safety::RESULT_ACCEPTED);

	flight_command_pipeline::Outcome unarmed = pipeline.handle(
		now + 1, agent_safety::SKILL_TAKEOFF, request.request_id,
		kConfirmationCode, true, request, safety_snapshot,
		vehicle(now + 1, 0, 0, 0, 0, true), sensors(now + 1));
	assert(unarmed.result == agent_safety::RESULT_AGENT_NOT_ARMED);
	assert(pipeline.machine().activeSkill() == SKILL_NONE);

	flight_command_pipeline::Outcome arm = pipeline.handle(
		now + 2, agent_safety::SKILL_ARM, 2, kConfirmationCode, true,
		Request(), safety_snapshot, vehicle(now + 2, 0, 0, 0, 0, true),
		sensors(now + 2));
	assert(arm.result == agent_safety::RESULT_ACCEPTED);
	assert(arm.safety.arm);
	assert(pipeline.gate().state() == agent_safety::STATE_AGENT_ARMED);
	assert(pipeline.gate().agentOwnsArm());
	agent_safety::Snapshot active_snapshot = safety_snapshot;
	active_snapshot.armed = true;

	flight_command_pipeline::Outcome accepted = pipeline.handle(
		now + 3, agent_safety::SKILL_TAKEOFF, request.request_id,
		kConfirmationCode, true, request, active_snapshot,
		vehicle(now + 3), sensors(now + 3));
	assert(accepted.result == agent_safety::RESULT_ACCEPTED);
	assert(accepted.flight_result == RESULT_ACCEPTED);
	assert(pipeline.machine().activeSkill() == SKILL_TAKEOFF);

	flight_command_pipeline::Outcome duplicate = pipeline.handle(
		now + 4, agent_safety::SKILL_TAKEOFF, request.request_id,
		kConfirmationCode, true, request, active_snapshot,
		vehicle(now + 4), sensors(now + 4));
	assert(duplicate.result == agent_safety::RESULT_DUPLICATE);
	assert(!duplicate.flight_attempted);
}

static void testArmRollsBackWhenFlightMachineRejects() {
	flight_command_pipeline::Pipeline pipeline;
	const uint64_t now = 100;
	agent_safety::Snapshot snapshot = {};
	snapshot.throttle_low = true;
	snapshot.battery_ok = true;
	snapshot.attitude_ok = true;
	snapshot.landed = true;
	TelemetryAggregator telemetry;
	const HealthySnapshot unavailable = telemetry.snapshot(now);
	VehicleState unavailable_vehicle = {};
	unavailable_vehicle.timestamp_ms = now + 1;

	assert(pipeline.handle(now, agent_safety::SKILL_HEARTBEAT, 1,
	                       kConfirmationCode, true, Request(), snapshot,
	                       unavailable_vehicle, unavailable).result ==
	       agent_safety::RESULT_ACCEPTED);
	const flight_command_pipeline::Outcome rejected = pipeline.handle(
		now + 1, agent_safety::SKILL_ARM, 2, kConfirmationCode, true,
		Request(), snapshot, unavailable_vehicle, unavailable);
	assert(rejected.result == agent_safety::RESULT_FLIGHT_REJECTED);
	assert(!rejected.safety.arm);
	assert(pipeline.gate().state() != agent_safety::STATE_AGENT_ARMED);
	assert(!pipeline.gate().agentOwnsArm());
	assert(pipeline.machine().missionState() != MISSION_ACTIVE);

	const flight_command_pipeline::Outcome retry = pipeline.handle(
		now + 2, agent_safety::SKILL_ARM, 2, kConfirmationCode, true,
		Request(), snapshot, unavailable_vehicle, unavailable);
	assert(retry.result == agent_safety::RESULT_FLIGHT_REJECTED);
	assert(retry.result != agent_safety::RESULT_DUPLICATE);
	assert(!retry.safety.arm);
}

static void testGuardedPipelineFailsClosedForUnavailableSensors() {
	flight_command_pipeline::Pipeline pipeline;
	const uint64_t now = 100;
	agent_safety::Snapshot safety_snapshot = {};
	safety_snapshot.throttle_low = true;
	safety_snapshot.battery_ok = true;
	safety_snapshot.attitude_ok = true;
	safety_snapshot.landed = true;
	TelemetryAggregator telemetry;
	const HealthySnapshot unavailable = telemetry.snapshot(now);
	Request request = decode(100, 3, 0.5f, 0.0f, 0.0f, SKILL_TAKEOFF);

	assert(pipeline.handle(now, agent_safety::SKILL_HEARTBEAT, 1,
	                       kConfirmationCode, true, Request(), safety_snapshot,
	                       vehicle(now, 0, 0, 0, 0, true), unavailable).result ==
	       agent_safety::RESULT_ACCEPTED);
	assert(pipeline.handle(now + 1, agent_safety::SKILL_ARM, 2,
	                       kConfirmationCode, true, Request(), safety_snapshot,
	                       vehicle(now + 1, 0, 0, 0, 0, true), unavailable).result ==
	       agent_safety::RESULT_ACCEPTED);
	safety_snapshot.armed = true;
	flight_command_pipeline::Outcome rejected = pipeline.handle(
		now + 2, agent_safety::SKILL_TAKEOFF, request.request_id,
		kConfirmationCode, true, request, safety_snapshot, vehicle(now + 2),
		unavailable);
	assert(rejected.result == agent_safety::RESULT_FLIGHT_REJECTED);
	assert(rejected.flight_result == RESULT_SENSOR_UNHEALTHY);
	assert(pipeline.machine().fault() == FAULT_RANGE_LOSS);
}

static void testGuardedPipelineBlocksUnsafeRequests() {
	struct Case {
		bool valid_arguments;
		uint64_t command_time_ms;
		void (*change)(agent_safety::Snapshot&);
		agent_safety::Result expected;
	};
	const uint64_t now = 100;
	const Case cases[] = {
		{false, now + 3, 0, agent_safety::RESULT_INVALID_REQUEST},
		{true, now + agent_safety::kHeartbeatTimeoutMs + 2, 0,
		 agent_safety::RESULT_HEARTBEAT_STALE},
	};
	for (unsigned int i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
		flight_command_pipeline::Pipeline pipeline;
		agent_safety::Snapshot snapshot = {};
		snapshot.throttle_low = true;
		snapshot.battery_ok = true;
		snapshot.attitude_ok = true;
		snapshot.landed = true;
		assert(pipeline.handle(now, agent_safety::SKILL_HEARTBEAT, 1,
		                       kConfirmationCode, true, Request(), snapshot,
		                       vehicle(now, 0, 0, 0, 0, true), sensors(now)).result ==
		       agent_safety::RESULT_ACCEPTED);
		assert(pipeline.handle(now + 1, agent_safety::SKILL_ARM, 2,
		                       kConfirmationCode, true, Request(), snapshot,
		                       vehicle(now + 1, 0, 0, 0, 0, true), sensors(now + 1)).result ==
		       agent_safety::RESULT_ACCEPTED);
		snapshot.armed = true;
		const Request request = decode(100, 3, 0.5f, 0.0f, 0.0f, SKILL_TAKEOFF);
		const flight_command_pipeline::Outcome blocked = pipeline.handle(
			cases[i].command_time_ms, agent_safety::SKILL_TAKEOFF,
			request.request_id, kConfirmationCode, cases[i].valid_arguments,
			request, snapshot, vehicle(cases[i].command_time_ms),
			sensors(cases[i].command_time_ms));
		assert(blocked.result == cases[i].expected);
		assert(!blocked.flight_attempted);
		assert(pipeline.machine().activeSkill() == SKILL_NONE);
	}

	struct Hazard {
		void (*change)(agent_safety::Snapshot&);
		agent_safety::Result expected;
	};
	const Hazard hazards[] = {
		{[](agent_safety::Snapshot& s) { s.manual_control_active = true; },
		 agent_safety::RESULT_MANUAL_CONTROL_ACTIVE},
		{[](agent_safety::Snapshot& s) { s.battery_ok = false; },
		 agent_safety::RESULT_BATTERY_UNSAFE},
		{[](agent_safety::Snapshot& s) { s.inverted = true; },
		 agent_safety::RESULT_INVERTED},
		{[](agent_safety::Snapshot& s) { s.attitude_ok = false; },
		 agent_safety::RESULT_ATTITUDE_INVALID},
	};
	for (unsigned int i = 0; i < sizeof(hazards) / sizeof(hazards[0]); ++i) {
		flight_command_pipeline::Pipeline pipeline;
		agent_safety::Snapshot snapshot = {};
		snapshot.throttle_low = true;
		snapshot.battery_ok = true;
		snapshot.attitude_ok = true;
		snapshot.landed = true;
		assert(pipeline.handle(now, agent_safety::SKILL_HEARTBEAT, 1,
		                       kConfirmationCode, true, Request(), snapshot,
		                       vehicle(now, 0, 0, 0, 0, true), sensors(now)).result ==
		       agent_safety::RESULT_ACCEPTED);
		assert(pipeline.handle(now + 1, agent_safety::SKILL_ARM, 2,
		                       kConfirmationCode, true, Request(), snapshot,
		                       vehicle(now + 1, 0, 0, 0, 0, true), sensors(now + 1)).result ==
		       agent_safety::RESULT_ACCEPTED);
		snapshot.armed = true;
		hazards[i].change(snapshot);
		const Request request = decode(100, 3, 0.5f, 0.0f, 0.0f, SKILL_TAKEOFF);
		const flight_command_pipeline::Outcome blocked = pipeline.handle(
			now + 2, agent_safety::SKILL_TAKEOFF, request.request_id,
			kConfirmationCode, true, request, snapshot, vehicle(now + 2),
			sensors(now + 2));
		assert(blocked.result == hazards[i].expected);
		assert(!blocked.flight_attempted);
		assert(pipeline.machine().activeSkill() == SKILL_NONE);
		assert(pipeline.gate().fault() != agent_safety::FAULT_NONE);
	}
}

int main() {
	testV1CommandsReachFlightMachine();
	testInvalidV1CommandDoesNotStartMachine();
	testGuardedPipelineNeedsBothAcceptances();
	testArmRollsBackWhenFlightMachineRejects();
	testGuardedPipelineFailsClosedForUnavailableSensors();
	testGuardedPipelineBlocksUnsafeRequests();
	puts("flight command v1 pipeline tests: PASS");
}
