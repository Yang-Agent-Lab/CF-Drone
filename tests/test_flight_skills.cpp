#include "flight_skills.h"

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

static HealthySnapshot sensors(uint64_t now_ms, bool flow_ok = true,
	                           bool range_ok = true, bool barometer_ok = true,
	                           float range_m = 1.0f,
	                           float relative_altitude_m = 1.0f) {
	TelemetryAggregator telemetry;
	OpticalFlowSample flow = {
		now_ms, flow_ok, flow_ok, 0.0f, 0.0f, 200
	};
	RangeSample range = {now_ms, range_ok, range_ok, range_m, 200};
	BarometerSample barometer = {
		now_ms, barometer_ok, barometer_ok, 101325.0f, 20.0f,
		relative_altitude_m, 200
	};
	telemetry.submitOpticalFlow(flow);
	telemetry.submitRange(range);
	telemetry.submitBarometer(barometer);
	return telemetry.snapshot(now_ms);
}

static HealthySnapshot staleFlowSensors(uint64_t now_ms) {
	TelemetryAggregator telemetry;
	OpticalFlowSample flow = {
		now_ms - TelemetryAggregator::kOpticalFlowMaxAgeMs - 1,
		true, true, 0.0f, 0.0f, 200
	};
	RangeSample range = {now_ms, true, true, 1.0f, 200};
	BarometerSample barometer = {
		now_ms, true, true, 101325.0f, 20.0f, 1.0f, 200
	};
	telemetry.submitOpticalFlow(flow);
	telemetry.submitRange(range);
	telemetry.submitBarometer(barometer);
	return telemetry.snapshot(now_ms);
}

static Request request(uint32_t request_id, Skill skill) {
	Request value = {};
	value.request_id = request_id;
	value.skill = skill;
	return value;
}

static void begin(Machine& machine, uint64_t now_ms = 100) {
	assert(machine.beginAtArm(now_ms, vehicle(now_ms, 0, 0, 0, 0, true),
	                          safeStatus()) == RESULT_ACCEPTED);
}

static void becomeAirborne(Machine& machine, uint64_t& now_ms) {
	now_ms = 100;
	begin(machine, now_ms);
	Request takeoff = request(1, SKILL_TAKEOFF);
	takeoff.height_m = 0.5f;
	assert(machine.start(now_ms, takeoff, sensors(now_ms), vehicle(now_ms),
	                     safeStatus()) == RESULT_ACCEPTED);
	now_ms = 200;
	Target completed = machine.update(
		now_ms, sensors(now_ms), vehicle(now_ms, 0, 0, 0.5f), safeStatus());
	assert(!completed.valid);
	assert(machine.activeSkill() == SKILL_NONE);
	assert(machine.missionState() == MISSION_ACTIVE);
}

static void testNormalSequenceAndTargets() {
	Machine machine;
	uint64_t now = 100;
	begin(machine, now);

	Request takeoff = request(1, SKILL_TAKEOFF);
	takeoff.height_m = 1.0f;
	assert(machine.start(now, takeoff, sensors(now), vehicle(now),
	                     safeStatus()) == RESULT_ACCEPTED);
	Target target = machine.update(
		now, sensors(now), vehicle(now), safeStatus());
	assert(target.valid);
	assert(target.action == ACTION_TRACK);
	assert(target.altitude_m == 1.0f);
	assert(target.velocity_x_m_s == 0.0f);
	assert(target.velocity_y_m_s == 0.0f);

	now = 200;
	machine.update(now, sensors(now), vehicle(now, 0, 0, 1.0f), safeStatus());
	assert(machine.activeSkill() == SKILL_NONE);

	Request hold = request(2, SKILL_HOLD);
	hold.duration_ms = 100;
	assert(machine.start(now, hold, sensors(now),
	                     vehicle(now, 0, 0, 1.0f, 15.0f),
	                     safeStatus()) == RESULT_ACCEPTED);
	target = machine.update(now, sensors(now),
	                        vehicle(now, 0, 0, 1.0f, 15.0f), safeStatus());
	assert(target.valid);
	assert(target.altitude_m == 1.0f);
	assert(target.yaw_deg == 15.0f);
	assert(target.velocity_x_m_s == 0.0f);
	assert(target.velocity_y_m_s == 0.0f);
	now += 100;
	assert(!machine.update(now, sensors(now),
	                       vehicle(now, 0, 0, 1.0f, 15.0f),
	                       safeStatus()).valid);

	Request move = request(3, SKILL_MOVE_BODY);
	move.body_x_m = 0.3f;
	move.body_y_m = 0.4f;
	move.speed_m_s = 0.25f;
	assert(machine.start(now, move, sensors(now),
	                     vehicle(now, 0, 0, 1.0f), safeStatus()) ==
	       RESULT_ACCEPTED);
	target = machine.update(now, sensors(now),
	                        vehicle(now, 0, 0, 1.0f), safeStatus());
	assert(target.valid);
	assert(target.horizontal_frame == FRAME_BODY);
	assert(fabsf(hypotf(target.velocity_x_m_s, target.velocity_y_m_s) -
	             0.25f) < 0.0001f);
	now += 100;
	assert(!machine.update(now, sensors(now),
	                       vehicle(now, 0.3f, 0.4f, 1.0f),
	                       safeStatus()).valid);

	Request yaw = request(4, SKILL_YAW);
	yaw.yaw_delta_deg = 180.0f;
	yaw.yaw_rate_deg_s = 30.0f;
	assert(machine.start(now, yaw, sensors(now),
	                     vehicle(now, 0.3f, 0.4f, 1.0f),
	                     safeStatus()) == RESULT_ACCEPTED);
	target = machine.update(now, sensors(now),
	                        vehicle(now, 0.3f, 0.4f, 1.0f),
	                        safeStatus());
	assert(target.valid);
	assert(target.yaw_deg == 180.0f);
	assert(target.yaw_rate_deg_s == 30.0f);
	assert(target.velocity_x_m_s == 0.0f);
	assert(target.velocity_y_m_s == 0.0f);
	now += 100;
	assert(!machine.update(now, sensors(now),
	                       vehicle(now, 0.3f, 0.4f, 1.0f, 180.0f),
	                       safeStatus()).valid);

	Request return_home = request(5, SKILL_RETURN_HOME);
	return_home.speed_m_s = 0.25f;
	assert(machine.start(now, return_home, sensors(now),
	                     vehicle(now, 0.3f, 0.4f, 1.0f, 180.0f),
	                     safeStatus()) == RESULT_ACCEPTED);
	target = machine.update(
		now, sensors(now), vehicle(now, 0.3f, 0.4f, 1.0f, 180.0f),
		safeStatus());
	assert(target.valid);
	assert(target.horizontal_frame == FRAME_LOCAL);
	assert(target.velocity_x_m_s < 0.0f);
	assert(target.velocity_y_m_s < 0.0f);
	now += 100;
	assert(!machine.update(now, sensors(now),
	                       vehicle(now, 0, 0, 1.0f, 180.0f),
	                       safeStatus()).valid);

	Request land = request(6, SKILL_LAND);
	assert(machine.start(now, land, sensors(now), vehicle(now, 0, 0, 1.0f),
	                     safeStatus()) == RESULT_ACCEPTED);
	target = machine.update(now, sensors(now), vehicle(now, 0, 0, 1.0f),
	                        safeStatus());
	assert(target.action == ACTION_LAND);
	assert(target.descent_rate_m_s == kLandingDescentRateMps);

	now += 100;
	target = machine.update(now, sensors(now, true, true, true, 0.05f, 0.0f),
	                        vehicle(now, 0, 0, 0, 0, true), safeStatus());
	assert(target.action == ACTION_LAND);
	now += kGroundConfirmMs - 1;
	assert(machine.update(now, sensors(now, true, true, true, 0.05f, 0.0f),
	                      vehicle(now, 0, 0, 0, 0, true),
	                      safeStatus()).action == ACTION_LAND);
	now += 1;
	target = machine.update(now, sensors(now, true, true, true, 0.05f, 0.0f),
	                        vehicle(now, 0, 0, 0, 0, true), safeStatus());
	assert(target.action == ACTION_REQUEST_LOCK);
	assert(machine.missionState() == MISSION_COMPLETE);
}

static void testLimits() {
	Machine takeoff_max;
	begin(takeoff_max);
	Request takeoff = request(1, SKILL_TAKEOFF);
	takeoff.height_m = kMaxTakeoffHeightM;
	assert(takeoff_max.start(100, takeoff, sensors(100), vehicle(100),
	                         safeStatus()) == RESULT_ACCEPTED);

	Machine takeoff_over;
	begin(takeoff_over);
	takeoff.height_m = kMaxTakeoffHeightM + 0.01f;
	assert(takeoff_over.start(100, takeoff, sensors(100), vehicle(100),
	                          safeStatus()) == RESULT_INVALID_ARGUMENT);

	Machine hold_max;
	uint64_t now = 0;
	becomeAirborne(hold_max, now);
	Request hold = request(2, SKILL_HOLD);
	hold.duration_ms = kMaxHoldDurationMs;
	assert(hold_max.start(now, hold, sensors(now),
	                      vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_ACCEPTED);

	Machine hold_zero;
	becomeAirborne(hold_zero, now);
	hold.duration_ms = 0;
	assert(hold_zero.start(now, hold, sensors(now),
	                       vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_INVALID_ARGUMENT);

	Machine hold_over;
	becomeAirborne(hold_over, now);
	hold.duration_ms = kMaxHoldDurationMs + 1;
	assert(hold_over.start(now, hold, sensors(now),
	                       vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_INVALID_ARGUMENT);

	Machine move_max;
	becomeAirborne(move_max, now);
	Request move = request(2, SKILL_MOVE_BODY);
	move.body_x_m = 0.3f;
	move.body_y_m = 0.4f;
	move.speed_m_s = kMaxHorizontalSpeedMps;
	assert(move_max.start(now, move, sensors(now),
	                      vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_ACCEPTED);

	Machine move_distance_over;
	becomeAirborne(move_distance_over, now);
	move.body_x_m = kMaxMoveDistanceM + 0.01f;
	move.body_y_m = 0;
	assert(move_distance_over.start(
		       now, move, sensors(now), vehicle(now, 0, 0, 0.5f),
		       safeStatus()) == RESULT_INVALID_ARGUMENT);

	Machine move_speed_over;
	becomeAirborne(move_speed_over, now);
	move.body_x_m = 0.1f;
	move.speed_m_s = kMaxHorizontalSpeedMps + 0.01f;
	assert(move_speed_over.start(
		       now, move, sensors(now), vehicle(now, 0, 0, 0.5f),
		       safeStatus()) == RESULT_INVALID_ARGUMENT);

	Machine yaw_max;
	becomeAirborne(yaw_max, now);
	Request yaw = request(2, SKILL_YAW);
	yaw.yaw_delta_deg = -kMaxYawDeltaDeg;
	yaw.yaw_rate_deg_s = kMaxYawRateDegS;
	assert(yaw_max.start(now, yaw, sensors(now),
	                     vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_ACCEPTED);

	Machine yaw_zero;
	becomeAirborne(yaw_zero, now);
	yaw.yaw_delta_deg = 0;
	assert(yaw_zero.start(now, yaw, sensors(now),
	                      vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_INVALID_ARGUMENT);

	Machine yaw_angle_over;
	becomeAirborne(yaw_angle_over, now);
	yaw.yaw_delta_deg = kMaxYawDeltaDeg + 0.01f;
	assert(yaw_angle_over.start(now, yaw, sensors(now),
	                            vehicle(now, 0, 0, 0.5f),
	                            safeStatus()) == RESULT_INVALID_ARGUMENT);

	Machine yaw_rate_over;
	becomeAirborne(yaw_rate_over, now);
	yaw.yaw_delta_deg = 10.0f;
	yaw.yaw_rate_deg_s = kMaxYawRateDegS + 0.01f;
	assert(yaw_rate_over.start(now, yaw, sensors(now),
	                           vehicle(now, 0, 0, 0.5f),
	                           safeStatus()) == RESULT_INVALID_ARGUMENT);

	Machine return_speed_over;
	becomeAirborne(return_speed_over, now);
	Request return_home = request(2, SKILL_RETURN_HOME);
	return_home.speed_m_s = kMaxHorizontalSpeedMps + 0.01f;
	assert(return_speed_over.start(
		       now, return_home, sensors(now),
		       vehicle(now, 0.1f, 0, 0.5f), safeStatus()) ==
	       RESULT_INVALID_ARGUMENT);

	Machine large_finite_yaw;
	assert(large_finite_yaw.beginAtArm(
		       100, vehicle(100, 0, 0, 0, 1.0e20f, true),
		       safeStatus()) == RESULT_ACCEPTED);
	takeoff.height_m = 0.5f;
	assert(large_finite_yaw.start(
		       100, takeoff, sensors(100), vehicle(100, 0, 0, 0, 1.0e20f),
		       safeStatus()) == RESULT_ACCEPTED);
	const Target wrapped = large_finite_yaw.update(
		100, sensors(100), vehicle(100, 0, 0, 0, 1.0e20f), safeStatus());
	assert(isfinite(wrapped.yaw_deg));
	assert(wrapped.yaw_deg >= -180.0f && wrapped.yaw_deg <= 180.0f);
}

static void testSequenceDuplicateAndTimeFailures() {
	Machine out_of_order;
	begin(out_of_order);
	Request move = request(1, SKILL_MOVE_BODY);
	move.body_x_m = 0.1f;
	move.speed_m_s = 0.1f;
	assert(out_of_order.start(100, move, sensors(100), vehicle(100),
	                          safeStatus()) == RESULT_SEQUENCE_ERROR);
	assert(out_of_order.missionState() == MISSION_FAILED);

	Machine unknown;
	uint64_t now = 0;
	becomeAirborne(unknown, now);
	Request invalid = request(2, static_cast<Skill>(99));
	assert(unknown.start(now, invalid, sensors(now),
	                     vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_INVALID_ARGUMENT);
	assert(unknown.missionState() == MISSION_FAILED);

	Machine overlapping;
	begin(overlapping);
	Request takeoff = request(1, SKILL_TAKEOFF);
	takeoff.height_m = 0.5f;
	assert(overlapping.start(100, takeoff, sensors(100), vehicle(100),
	                         safeStatus()) == RESULT_ACCEPTED);
	Request hold = request(2, SKILL_HOLD);
	hold.duration_ms = 100;
	assert(overlapping.start(101, hold, sensors(101), vehicle(101),
	                         safeStatus()) == RESULT_SEQUENCE_ERROR);

	Machine duplicate;
	becomeAirborne(duplicate, now);
	assert(duplicate.start(now, takeoff, sensors(now),
	                       vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_DUPLICATE_REQUEST);
	assert(duplicate.missionState() == MISSION_FAILED);

	Machine older_request;
	begin(older_request);
	takeoff.request_id = 5;
	assert(older_request.start(100, takeoff, sensors(100), vehicle(100),
	                           safeStatus()) == RESULT_ACCEPTED);
	older_request.update(200, sensors(200),
	                     vehicle(200, 0, 0, 0.5f), safeStatus());
	hold.request_id = 4;
	assert(older_request.start(200, hold, sensors(200),
	                           vehicle(200, 0, 0, 0.5f),
	                           safeStatus()) == RESULT_SEQUENCE_ERROR);

	Machine regressed;
	begin(regressed, 100);
	regressed.update(99, sensors(99), vehicle(99), safeStatus());
	assert(regressed.fault() == FAULT_TIME_REGRESSION);
	assert(regressed.missionState() == MISSION_FAILED);

	Machine stale_state;
	begin(stale_state, 100);
	stale_state.update(100 + kVehicleStateMaxAgeMs + 1,
	                   sensors(100 + kVehicleStateMaxAgeMs + 1),
	                   vehicle(100), safeStatus());
	assert(stale_state.fault() == FAULT_VEHICLE_STATE_TIMEOUT);

	Machine mission_timeout;
	begin(mission_timeout, 100);
	now = 100 + kMissionTimeoutMs + 1;
	mission_timeout.update(now, sensors(now), vehicle(now), safeStatus());
	assert(mission_timeout.fault() == FAULT_MISSION_TIMEOUT);
}

static void testSkillTimeouts() {
	Machine takeoff_timeout;
	begin(takeoff_timeout);
	Request takeoff = request(1, SKILL_TAKEOFF);
	takeoff.height_m = 0.5f;
	assert(takeoff_timeout.start(100, takeoff, sensors(100), vehicle(100),
	                             safeStatus()) == RESULT_ACCEPTED);
	uint64_t now = 100 + kTakeoffTimeoutMs + 1;
	takeoff_timeout.update(now, sensors(now), vehicle(now), safeStatus());
	assert(takeoff_timeout.fault() == FAULT_SKILL_TIMEOUT);

	Machine move_timeout;
	becomeAirborne(move_timeout, now);
	Request move = request(2, SKILL_MOVE_BODY);
	move.body_x_m = 0.5f;
	move.speed_m_s = 0.25f;
	assert(move_timeout.start(now, move, sensors(now),
	                          vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_ACCEPTED);
	now += kMoveTimeoutMs + 1;
	move_timeout.update(now, sensors(now),
	                    vehicle(now, 0, 0, 0.5f), safeStatus());
	assert(move_timeout.fault() == FAULT_SKILL_TIMEOUT);

	Machine yaw_timeout;
	becomeAirborne(yaw_timeout, now);
	Request yaw = request(2, SKILL_YAW);
	yaw.yaw_delta_deg = 10.0f;
	yaw.yaw_rate_deg_s = 10.0f;
	assert(yaw_timeout.start(now, yaw, sensors(now),
	                         vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_ACCEPTED);
	now += kYawTimeoutMs + 1;
	yaw_timeout.update(now, sensors(now),
	                   vehicle(now, 0, 0, 0.5f), safeStatus());
	assert(yaw_timeout.fault() == FAULT_SKILL_TIMEOUT);

	Machine return_timeout;
	becomeAirborne(return_timeout, now);
	Request return_home = request(2, SKILL_RETURN_HOME);
	return_home.speed_m_s = 0.25f;
	assert(return_timeout.start(
		       now, return_home, sensors(now),
		       vehicle(now, 0.5f, 0, 0.5f), safeStatus()) == RESULT_ACCEPTED);
	now += kReturnTimeoutMs + 1;
	return_timeout.update(now, sensors(now),
	                      vehicle(now, 0.5f, 0, 0.5f), safeStatus());
	assert(return_timeout.fault() == FAULT_SKILL_TIMEOUT);

	Machine land_timeout;
	becomeAirborne(land_timeout, now);
	Request land = request(2, SKILL_LAND);
	assert(land_timeout.start(now, land, sensors(now),
	                          vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_ACCEPTED);
	now += kLandTimeoutMs + 1;
	Target emergency = land_timeout.update(
		now, sensors(now), vehicle(now, 0, 0, 0.5f), safeStatus());
	assert(land_timeout.fault() == FAULT_SKILL_TIMEOUT);
	assert(emergency.action == ACTION_EMERGENCY_DESCENT);
}

static void testSensorFailsafesAndLandingContinuity() {
	uint64_t now = 0;
	Machine flow_loss;
	becomeAirborne(flow_loss, now);
	Target target = flow_loss.update(
		now + 1, sensors(now + 1, false, true, true),
		vehicle(now + 1, 0, 0, 0.5f), safeStatus());
	assert(flow_loss.missionState() == MISSION_FAILSAFE_LANDING);
	assert(flow_loss.fault() == FAULT_OPTICAL_FLOW_LOSS);
	assert(target.action == ACTION_LAND);
	assert(target.velocity_x_m_s == 0.0f);
	assert(target.velocity_y_m_s == 0.0f);

	Machine stale_flow;
	becomeAirborne(stale_flow, now);
	target = stale_flow.update(
		now + 1, staleFlowSensors(now + 1),
		vehicle(now + 1, 0, 0, 0.5f), safeStatus());
	assert(stale_flow.fault() == FAULT_OPTICAL_FLOW_LOSS);
	assert(target.action == ACTION_LAND);

	Machine range_loss;
	becomeAirborne(range_loss, now);
	target = range_loss.update(
		now + 1, sensors(now + 1, true, false, true),
		vehicle(now + 1, 0, 0, 0.5f), safeStatus());
	assert(range_loss.fault() == FAULT_RANGE_LOSS);
	assert(target.action == ACTION_REDUNDANT_LAND);
	assert(target.descent_rate_m_s == kRedundantDescentRateMps);

	Machine failsafe_timeout;
	becomeAirborne(failsafe_timeout, now);
	failsafe_timeout.update(
		now + 1, sensors(now + 1, false, true, true),
		vehicle(now + 1, 0, 0, 0.5f), safeStatus());
	now += kLandTimeoutMs + 2;
	target = failsafe_timeout.update(
		now, sensors(now), vehicle(now, 0, 0, 0.5f), safeStatus());
	assert(failsafe_timeout.fault() == FAULT_SKILL_TIMEOUT);
	assert(target.action == ACTION_EMERGENCY_DESCENT);

	Machine altitude_loss;
	becomeAirborne(altitude_loss, now);
	target = altitude_loss.update(
		now + 1, sensors(now + 1, true, false, false),
		vehicle(now + 1, 0, 0, 0.5f), safeStatus());
	assert(altitude_loss.missionState() == MISSION_FAILED);
	assert(altitude_loss.fault() == FAULT_ALTITUDE_SENSORS_LOST);
	assert(target.action == ACTION_EMERGENCY_DESCENT);
	assert(target.descent_rate_m_s == kEmergencyDescentRateMps);

	Request hold = request(2, SKILL_HOLD);
	hold.duration_ms = 100;
	assert(altitude_loss.start(now + 2, hold, sensors(now + 2),
	                           vehicle(now + 2, 0, 0, 0.5f),
	                           safeStatus()) == RESULT_FAILED_LATCHED);

	Machine takeoff_without_range;
	begin(takeoff_without_range);
	Request takeoff = request(1, SKILL_TAKEOFF);
	takeoff.height_m = 0.5f;
	assert(takeoff_without_range.start(
		       100, takeoff, sensors(100, true, false, true), vehicle(100),
		       safeStatus()) == RESULT_SENSOR_UNHEALTHY);

	Machine takeoff_without_barometer;
	begin(takeoff_without_barometer);
	assert(takeoff_without_barometer.start(
		       100, takeoff, sensors(100, true, true, false), vehicle(100),
		       safeStatus()) == RESULT_SENSOR_UNHEALTHY);

	Machine landing;
	becomeAirborne(landing, now);
	Request land = request(2, SKILL_LAND);
	assert(landing.start(now, land, sensors(now),
	                     vehicle(now, 0, 0, 0.5f), safeStatus()) ==
	       RESULT_ACCEPTED);
	now += 1;
	landing.update(now, sensors(now, true, true, true, 0.05f, 0.0f),
	               vehicle(now, 0, 0, 0, 0, true), safeStatus());
	now += kGroundConfirmMs - 1;
	landing.update(now, sensors(now, true, true, true, 0.2f, 0.0f),
	               vehicle(now, 0, 0, 0, 0, true), safeStatus());
	now += 1;
	landing.update(now, sensors(now, true, true, true, 0.05f, 0.0f),
	               vehicle(now, 0, 0, 0, 0, true), safeStatus());
	now += kGroundConfirmMs - 1;
	assert(landing.update(now, sensors(now, true, true, true, 0.05f, 0.0f),
	                      vehicle(now, 0, 0, 0, 0, true),
	                      safeStatus()).action == ACTION_LAND);
	now += 1;
	assert(landing.update(now, sensors(now, true, true, true, 0.05f, 0.0f),
	                      vehicle(now, 0, 0, 0, 0, true),
	                      safeStatus()).action == ACTION_REQUEST_LOCK);
}

static void testSafetyFailures() {
	enum Hazard {
		HAZARD_MANUAL,
		HAZARD_BATTERY,
		HAZARD_INVERTED,
		HAZARD_HEARTBEAT,
		HAZARD_GATE,
	};
	const Hazard hazards[] = {
		HAZARD_MANUAL,
		HAZARD_BATTERY,
		HAZARD_INVERTED,
		HAZARD_HEARTBEAT,
		HAZARD_GATE,
	};
	const Fault expected[] = {
		FAULT_MANUAL_TAKEOVER,
		FAULT_LOW_BATTERY,
		FAULT_INVERTED,
		FAULT_HEARTBEAT_TIMEOUT,
		FAULT_UPSTREAM_GATE,
	};

	for (unsigned int i = 0; i < sizeof(hazards) / sizeof(hazards[0]); ++i) {
		Machine machine;
		uint64_t now = 0;
		becomeAirborne(machine, now);
		SafetyStatus status = safeStatus();
		switch (hazards[i]) {
			case HAZARD_MANUAL: status.manual_takeover = true; break;
			case HAZARD_BATTERY: status.battery_ok = false; break;
			case HAZARD_INVERTED: status.inverted = true; break;
			case HAZARD_HEARTBEAT: status.heartbeat_fresh = false; break;
			case HAZARD_GATE: status.upstream_gate_ok = false; break;
		}
		Target target = machine.update(
			now + 1, sensors(now + 1), vehicle(now + 1, 0, 0, 0.5f),
			status);
		assert(machine.missionState() == MISSION_FAILED);
		assert(machine.fault() == expected[i]);
		assert(target.action == ACTION_ABORT);
	}
}

int main() {
	testNormalSequenceAndTargets();
	testLimits();
	testSequenceDuplicateAndTimeFailures();
	testSkillTimeouts();
	testSensorFailsafesAndLandingContinuity();
	testSafetyFailures();
	puts("flight skill state machine tests: PASS");
}
