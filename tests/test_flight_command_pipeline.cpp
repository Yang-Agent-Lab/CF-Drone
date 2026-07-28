#include "flight_command_v1.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

using namespace flight_skills;
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

int main() {
	testV1CommandsReachFlightMachine();
	testInvalidV1CommandDoesNotStartMachine();
	puts("flight command v1 pipeline tests: PASS");
}
