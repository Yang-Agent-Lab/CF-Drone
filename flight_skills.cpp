#include "flight_skills.h"

#include <cmath>

namespace flight_skills {

namespace {

static const float kAltitudeToleranceM = 0.05f;
static const float kPositionToleranceM = 0.03f;
static const float kYawToleranceDeg = 2.0f;
static const float kGroundRangeM = 0.10f;
static const float kRedundantGroundAltitudeToleranceM = 0.15f;
static const float kPi = 3.14159265358979323846f;

bool finite(float value) {
	return std::isfinite(value);
}

float wrapDegrees(float value) {
	value = std::fmod(value, 360.0f);
	if (value > 180.0f) value -= 360.0f;
	if (value < -180.0f) value += 360.0f;
	return value;
}

Target emptyTarget() {
	Target target = {};
	target.horizontal_frame = FRAME_LOCAL;
	return target;
}

Target actionTarget(Action action) {
	Target target = emptyTarget();
	target.valid = true;
	target.action = action;
	return target;
}

}  // namespace

Machine::Machine()
	: mission_state_(MISSION_IDLE),
	  active_skill_(SKILL_NONE),
	  fault_(FAULT_NONE),
	  time_seen_(false),
	  last_now_ms_(0),
	  mission_start_ms_(0),
	  skill_start_ms_(0),
	  request_seen_(false),
	  last_request_id_(0),
	  took_off_(false),
	  redundant_landing_(false),
	  ground_seen_(false),
	  ground_since_ms_(0),
	  origin_x_m_(0.0f),
	  origin_y_m_(0.0f),
	  origin_altitude_m_(0.0f),
	  origin_yaw_deg_(0.0f),
	  target_altitude_m_(0.0f),
	  target_yaw_deg_(0.0f),
	  target_yaw_rate_deg_s_(0.0f),
	  command_speed_m_s_(0.0f),
	  move_target_x_m_(0.0f),
	  move_target_y_m_(0.0f),
	  hold_duration_ms_(0),
	  last_target_(emptyTarget()) {}

bool Machine::acceptTime(uint64_t now_ms) {
	if (time_seen_ && now_ms < last_now_ms_) {
		fail(FAULT_TIME_REGRESSION, ACTION_ABORT);
		return false;
	}
	time_seen_ = true;
	last_now_ms_ = now_ms;
	return true;
}

bool Machine::checkVehicle(uint64_t now_ms, const VehicleState& vehicle) {
	if (vehicle.timestamp_ms > now_ms) {
		fail(FAULT_TIME_REGRESSION, ACTION_ABORT);
		return false;
	}
	if (now_ms - vehicle.timestamp_ms > kVehicleStateMaxAgeMs) {
		fail(FAULT_VEHICLE_STATE_TIMEOUT, ACTION_ABORT);
		return false;
	}
	if (!vehicle.armed || !finite(vehicle.local_x_m) ||
	    !finite(vehicle.local_y_m) || !finite(vehicle.altitude_m) ||
	    !finite(vehicle.yaw_deg)) {
		fail(FAULT_UPSTREAM_GATE, ACTION_ABORT);
		return false;
	}
	return true;
}

bool Machine::checkSafety(const SafetyStatus& safety) {
	Fault fault = FAULT_NONE;
	if (safety.manual_takeover) fault = FAULT_MANUAL_TAKEOVER;
	else if (!safety.battery_ok) fault = FAULT_LOW_BATTERY;
	else if (safety.inverted) fault = FAULT_INVERTED;
	else if (!safety.heartbeat_fresh) fault = FAULT_HEARTBEAT_TIMEOUT;
	else if (!safety.upstream_gate_ok) fault = FAULT_UPSTREAM_GATE;
	if (fault == FAULT_NONE) return true;
	fail(fault, ACTION_ABORT);
	return false;
}

bool Machine::checkCommon(uint64_t now_ms, const VehicleState& vehicle,
	                      const SafetyStatus& safety) {
	if (!acceptTime(now_ms) || !checkVehicle(now_ms, vehicle) ||
	    !checkSafety(safety)) {
		return false;
	}
	if (mission_state_ != MISSION_IDLE &&
	    now_ms - mission_start_ms_ > kMissionTimeoutMs) {
		if (took_off_) {
			Target emergency = fail(FAULT_MISSION_TIMEOUT,
			                        ACTION_EMERGENCY_DESCENT);
			emergency.descent_rate_m_s = kEmergencyDescentRateMps;
			last_target_ = emergency;
		} else {
			fail(FAULT_MISSION_TIMEOUT, ACTION_ABORT);
		}
		return false;
	}
	return true;
}

Result Machine::reject(Fault fault, Result result) {
	fail(fault, ACTION_ABORT);
	return result;
}

Target Machine::fail(Fault fault, Action action) {
	fault_ = fault;
	mission_state_ = MISSION_FAILED;
	active_skill_ = SKILL_NONE;
	last_target_ = actionTarget(action);
	if (action == ACTION_EMERGENCY_DESCENT) {
		last_target_.descent_rate_m_s = kEmergencyDescentRateMps;
	}
	return last_target_;
}

Target Machine::startFailsafeLanding(Fault fault, bool redundant,
	                                 uint64_t now_ms,
	                                 const VehicleState& vehicle) {
	if (fault_ == FAULT_NONE) fault_ = fault;
	mission_state_ = MISSION_FAILSAFE_LANDING;
	active_skill_ = SKILL_LAND;
	redundant_landing_ = redundant;
	skill_start_ms_ = now_ms;
	target_altitude_m_ = origin_altitude_m_;
	target_yaw_deg_ = wrapDegrees(vehicle.yaw_deg);
	target_yaw_rate_deg_s_ = 0.0f;
	ground_seen_ = false;
	last_target_ = actionTarget(
		redundant ? ACTION_REDUNDANT_LAND : ACTION_LAND);
	last_target_.altitude_m = target_altitude_m_;
	last_target_.yaw_deg = target_yaw_deg_;
	last_target_.descent_rate_m_s =
		redundant ? kRedundantDescentRateMps : kLandingDescentRateMps;
	return last_target_;
}

Result Machine::beginAtArm(uint64_t now_ms, const VehicleState& vehicle,
	                       const SafetyStatus& safety) {
	if (mission_state_ != MISSION_IDLE) {
		return reject(FAULT_SEQUENCE, RESULT_SEQUENCE_ERROR);
	}
	if (!acceptTime(now_ms) || !checkVehicle(now_ms, vehicle) ||
	    !checkSafety(safety)) {
		return RESULT_SAFETY_FAULT;
	}
	if (!vehicle.landed) {
		return reject(FAULT_SEQUENCE, RESULT_SEQUENCE_ERROR);
	}

	mission_state_ = MISSION_ACTIVE;
	mission_start_ms_ = now_ms;
	origin_x_m_ = vehicle.local_x_m;
	origin_y_m_ = vehicle.local_y_m;
	origin_altitude_m_ = vehicle.altitude_m;
	origin_yaw_deg_ = wrapDegrees(vehicle.yaw_deg);
	last_target_ = emptyTarget();
	return RESULT_ACCEPTED;
}

Result Machine::start(
	uint64_t now_ms, const Request& request,
	const sensor_telemetry::HealthySnapshot& sensors,
	const VehicleState& vehicle, const SafetyStatus& safety) {
	if (mission_state_ == MISSION_FAILED ||
	    mission_state_ == MISSION_FAILSAFE_LANDING ||
	    mission_state_ == MISSION_COMPLETE) {
		return RESULT_FAILED_LATCHED;
	}
	if (mission_state_ != MISSION_ACTIVE ||
	    !checkCommon(now_ms, vehicle, safety)) {
		return RESULT_SAFETY_FAULT;
	}
	if (request.request_id == 0) {
		return reject(FAULT_INVALID_ARGUMENT, RESULT_INVALID_ARGUMENT);
	}
	if (request_seen_ && request.request_id == last_request_id_) {
		return reject(FAULT_DUPLICATE_REQUEST, RESULT_DUPLICATE_REQUEST);
	}
	if (request_seen_ && request.request_id < last_request_id_) {
		return reject(FAULT_SEQUENCE, RESULT_SEQUENCE_ERROR);
	}
	if (active_skill_ != SKILL_NONE) {
		return reject(FAULT_SEQUENCE, RESULT_SEQUENCE_ERROR);
	}
	if ((!took_off_ && request.skill != SKILL_TAKEOFF) ||
	    (took_off_ && (request.skill == SKILL_TAKEOFF ||
	                   request.skill == SKILL_NONE))) {
		return reject(FAULT_SEQUENCE, RESULT_SEQUENCE_ERROR);
	}

	sensor_telemetry::OpticalFlowReading flow = {};
	sensor_telemetry::RangeReading range = {};
	sensor_telemetry::BarometerReading barometer = {};
	const bool flow_ok = sensors.opticalFlow(&flow);
	const bool range_ok = sensors.range(&range);
	const bool barometer_ok = sensors.barometer(&barometer);

	if (took_off_) {
		if (!range_ok && !barometer_ok) {
			fail(FAULT_ALTITUDE_SENSORS_LOST, ACTION_EMERGENCY_DESCENT);
			return RESULT_SENSOR_UNHEALTHY;
		}
		if (!range_ok) {
			startFailsafeLanding(FAULT_RANGE_LOSS, true, now_ms, vehicle);
			return RESULT_SENSOR_UNHEALTHY;
		}
		if (!barometer_ok) {
			startFailsafeLanding(FAULT_BAROMETER_LOSS, false, now_ms, vehicle);
			return RESULT_SENSOR_UNHEALTHY;
		}
		if (!flow_ok) {
			startFailsafeLanding(
				FAULT_OPTICAL_FLOW_LOSS, false, now_ms, vehicle);
			return RESULT_SENSOR_UNHEALTHY;
		}
	}

	target_yaw_rate_deg_s_ = 0.0f;
	switch (request.skill) {
		case SKILL_TAKEOFF:
			if (!range_ok || !barometer_ok) {
				return reject(!range_ok ? FAULT_RANGE_LOSS :
				              FAULT_BAROMETER_LOSS,
				              RESULT_SENSOR_UNHEALTHY);
			}
			if (!finite(request.height_m) || request.height_m <= 0.0f ||
			    request.height_m > kMaxTakeoffHeightM) {
				return reject(FAULT_INVALID_ARGUMENT,
				              RESULT_INVALID_ARGUMENT);
			}
			target_altitude_m_ = origin_altitude_m_ + request.height_m;
			target_yaw_deg_ = origin_yaw_deg_;
			break;

		case SKILL_HOLD:
			if (!flow_ok || !range_ok || !barometer_ok) {
				return reject(FAULT_OPTICAL_FLOW_LOSS,
				              RESULT_SENSOR_UNHEALTHY);
			}
			if (request.duration_ms == 0 ||
			    request.duration_ms > kMaxHoldDurationMs) {
				return reject(FAULT_INVALID_ARGUMENT,
				              RESULT_INVALID_ARGUMENT);
			}
			hold_duration_ms_ = request.duration_ms;
			target_altitude_m_ = vehicle.altitude_m;
			target_yaw_deg_ = wrapDegrees(vehicle.yaw_deg);
			break;

		case SKILL_MOVE_BODY: {
			if (!flow_ok || !range_ok || !barometer_ok) {
				return reject(FAULT_OPTICAL_FLOW_LOSS,
				              RESULT_SENSOR_UNHEALTHY);
			}
			const float distance =
				std::hypot(request.body_x_m, request.body_y_m);
			if (!finite(request.body_x_m) || !finite(request.body_y_m) ||
			    !finite(request.speed_m_s) || distance <= 0.0f ||
			    distance > kMaxMoveDistanceM || request.speed_m_s <= 0.0f ||
			    request.speed_m_s > kMaxHorizontalSpeedMps ||
			    distance / request.speed_m_s * 1000.0f >
				    static_cast<float>(kMoveTimeoutMs)) {
				return reject(FAULT_INVALID_ARGUMENT,
				              RESULT_INVALID_ARGUMENT);
			}
			const float yaw_rad = vehicle.yaw_deg * kPi / 180.0f;
			move_target_x_m_ = vehicle.local_x_m +
				std::cos(yaw_rad) * request.body_x_m -
				std::sin(yaw_rad) * request.body_y_m;
			move_target_y_m_ = vehicle.local_y_m +
				std::sin(yaw_rad) * request.body_x_m +
				std::cos(yaw_rad) * request.body_y_m;
			command_speed_m_s_ = request.speed_m_s;
			target_altitude_m_ = vehicle.altitude_m;
			target_yaw_deg_ = wrapDegrees(vehicle.yaw_deg);
			break;
		}

		case SKILL_YAW:
			if (!flow_ok || !range_ok || !barometer_ok) {
				return reject(FAULT_OPTICAL_FLOW_LOSS,
				              RESULT_SENSOR_UNHEALTHY);
			}
			if (!finite(request.yaw_delta_deg) ||
			    !finite(request.yaw_rate_deg_s) ||
			    request.yaw_delta_deg == 0.0f ||
			    std::fabs(request.yaw_delta_deg) > kMaxYawDeltaDeg ||
			    request.yaw_rate_deg_s <= 0.0f ||
			    request.yaw_rate_deg_s > kMaxYawRateDegS ||
			    std::fabs(request.yaw_delta_deg) /
			            request.yaw_rate_deg_s * 1000.0f >
				    static_cast<float>(kYawTimeoutMs)) {
				return reject(FAULT_INVALID_ARGUMENT,
				              RESULT_INVALID_ARGUMENT);
			}
			target_altitude_m_ = vehicle.altitude_m;
			target_yaw_deg_ =
				wrapDegrees(vehicle.yaw_deg + request.yaw_delta_deg);
			target_yaw_rate_deg_s_ = request.yaw_rate_deg_s;
			break;

		case SKILL_RETURN_HOME: {
			if (!flow_ok || !range_ok || !barometer_ok) {
				return reject(FAULT_OPTICAL_FLOW_LOSS,
				              RESULT_SENSOR_UNHEALTHY);
			}
			const float distance = std::hypot(
				origin_x_m_ - vehicle.local_x_m,
				origin_y_m_ - vehicle.local_y_m);
			if (!finite(request.speed_m_s) || request.speed_m_s <= 0.0f ||
			    request.speed_m_s > kMaxHorizontalSpeedMps ||
			    distance / request.speed_m_s * 1000.0f >
				    static_cast<float>(kReturnTimeoutMs)) {
				return reject(FAULT_INVALID_ARGUMENT,
				              RESULT_INVALID_ARGUMENT);
			}
			command_speed_m_s_ = request.speed_m_s;
			target_altitude_m_ = vehicle.altitude_m;
			target_yaw_deg_ = wrapDegrees(vehicle.yaw_deg);
			break;
		}

		case SKILL_LAND:
			if (!range_ok && !barometer_ok) {
				return reject(FAULT_ALTITUDE_SENSORS_LOST,
				              RESULT_SENSOR_UNHEALTHY);
			}
			redundant_landing_ = !range_ok;
			target_altitude_m_ = origin_altitude_m_;
			target_yaw_deg_ = wrapDegrees(vehicle.yaw_deg);
			break;

		case SKILL_NONE:
			return reject(FAULT_INVALID_ARGUMENT, RESULT_INVALID_ARGUMENT);

		default:
			return reject(FAULT_INVALID_ARGUMENT, RESULT_INVALID_ARGUMENT);
	}

	request_seen_ = true;
	last_request_id_ = request.request_id;
	active_skill_ = request.skill;
	skill_start_ms_ = now_ms;
	ground_seen_ = false;
	last_target_ = emptyTarget();
	return RESULT_ACCEPTED;
}

uint64_t Machine::activeTimeoutMs() const {
	switch (active_skill_) {
		case SKILL_TAKEOFF: return kTakeoffTimeoutMs;
		case SKILL_HOLD: return kMaxHoldDurationMs;
		case SKILL_MOVE_BODY: return kMoveTimeoutMs;
		case SKILL_YAW: return kYawTimeoutMs;
		case SKILL_RETURN_HOME: return kReturnTimeoutMs;
		case SKILL_LAND: return kLandTimeoutMs;
		case SKILL_NONE: return 0;
	}
	return 0;
}

void Machine::completeSkill() {
	if (active_skill_ == SKILL_TAKEOFF) took_off_ = true;
	active_skill_ = SKILL_NONE;
	last_target_ = emptyTarget();
}

Target Machine::trackingTarget(const VehicleState& vehicle) {
	Target target = actionTarget(ACTION_TRACK);
	target.altitude_m = target_altitude_m_;
	target.yaw_deg = target_yaw_deg_;
	target.yaw_rate_deg_s = target_yaw_rate_deg_s_;

	if (active_skill_ == SKILL_MOVE_BODY) {
		const float remaining_x = move_target_x_m_ - vehicle.local_x_m;
		const float remaining_y = move_target_y_m_ - vehicle.local_y_m;
		const float remaining = std::hypot(remaining_x, remaining_y);
		if (remaining <= kPositionToleranceM) {
			completeSkill();
			return last_target_;
		}
		const float local_vx = remaining_x / remaining * command_speed_m_s_;
		const float local_vy = remaining_y / remaining * command_speed_m_s_;
		const float yaw_rad = vehicle.yaw_deg * kPi / 180.0f;
		target.horizontal_frame = FRAME_BODY;
		target.velocity_x_m_s =
			std::cos(yaw_rad) * local_vx + std::sin(yaw_rad) * local_vy;
		target.velocity_y_m_s =
			-std::sin(yaw_rad) * local_vx + std::cos(yaw_rad) * local_vy;
	} else if (active_skill_ == SKILL_RETURN_HOME) {
		const float remaining_x = origin_x_m_ - vehicle.local_x_m;
		const float remaining_y = origin_y_m_ - vehicle.local_y_m;
		const float remaining = std::hypot(remaining_x, remaining_y);
		if (remaining <= kPositionToleranceM) {
			completeSkill();
			return last_target_;
		}
		target.horizontal_frame = FRAME_LOCAL;
		target.velocity_x_m_s =
			remaining_x / remaining * command_speed_m_s_;
		target.velocity_y_m_s =
			remaining_y / remaining * command_speed_m_s_;
	}
	return target;
}

Target Machine::landingTarget(
	uint64_t now_ms, const sensor_telemetry::HealthySnapshot& sensors,
	const VehicleState& vehicle) {
	sensor_telemetry::RangeReading range = {};
	sensor_telemetry::BarometerReading barometer = {};
	const bool range_ok = sensors.range(&range);
	const bool barometer_ok = sensors.barometer(&barometer);
	const bool on_ground =
		vehicle.landed &&
		(redundant_landing_
		     ? barometer_ok &&
		           std::fabs(vehicle.altitude_m - origin_altitude_m_) <=
			           kRedundantGroundAltitudeToleranceM
		     : range_ok && range.distance_m <= kGroundRangeM);

	if (!on_ground) {
		ground_seen_ = false;
	} else if (!ground_seen_) {
		ground_seen_ = true;
		ground_since_ms_ = now_ms;
	} else if (now_ms - ground_since_ms_ >= kGroundConfirmMs) {
		Target lock = actionTarget(ACTION_REQUEST_LOCK);
		lock.altitude_m = origin_altitude_m_;
		lock.yaw_deg = target_yaw_deg_;
		last_target_ = lock;
		active_skill_ = SKILL_NONE;
		mission_state_ =
			fault_ == FAULT_NONE ? MISSION_COMPLETE : MISSION_FAILED;
		return last_target_;
	}

	Target target = actionTarget(
		redundant_landing_ ? ACTION_REDUNDANT_LAND : ACTION_LAND);
	target.altitude_m = origin_altitude_m_;
	target.yaw_deg = target_yaw_deg_;
	target.descent_rate_m_s = redundant_landing_
		? kRedundantDescentRateMps : kLandingDescentRateMps;
	last_target_ = target;
	return target;
}

Target Machine::update(
	uint64_t now_ms, const sensor_telemetry::HealthySnapshot& sensors,
	const VehicleState& vehicle, const SafetyStatus& safety) {
	if (mission_state_ == MISSION_FAILED ||
	    mission_state_ == MISSION_COMPLETE) {
		return last_target_;
	}
	if (mission_state_ == MISSION_IDLE ||
	    !checkCommon(now_ms, vehicle, safety)) {
		return last_target_;
	}

	sensor_telemetry::OpticalFlowReading flow = {};
	sensor_telemetry::RangeReading range = {};
	sensor_telemetry::BarometerReading barometer = {};
	const bool flow_ok = sensors.opticalFlow(&flow);
	const bool range_ok = sensors.range(&range);
	const bool barometer_ok = sensors.barometer(&barometer);

	if (!range_ok && !barometer_ok) {
		return fail(FAULT_ALTITUDE_SENSORS_LOST,
		            ACTION_EMERGENCY_DESCENT);
	}
	if (mission_state_ == MISSION_FAILSAFE_LANDING) {
		if (now_ms - skill_start_ms_ > kLandTimeoutMs) {
			return fail(FAULT_SKILL_TIMEOUT,
			            ACTION_EMERGENCY_DESCENT);
		}
		if (!range_ok) redundant_landing_ = true;
		return landingTarget(now_ms, sensors, vehicle);
	}
	if (!range_ok) {
		return startFailsafeLanding(
			FAULT_RANGE_LOSS, true, now_ms, vehicle);
	}
	if (!barometer_ok) {
		return startFailsafeLanding(
			FAULT_BAROMETER_LOSS, false, now_ms, vehicle);
	}
	if (!flow_ok && active_skill_ != SKILL_LAND) {
		return startFailsafeLanding(
			FAULT_OPTICAL_FLOW_LOSS, false, now_ms, vehicle);
	}

	if (active_skill_ == SKILL_NONE) return emptyTarget();

	const uint64_t elapsed_ms = now_ms - skill_start_ms_;
	if (active_skill_ == SKILL_HOLD &&
	    elapsed_ms >= hold_duration_ms_) {
		completeSkill();
		return last_target_;
	}
	if (elapsed_ms > activeTimeoutMs()) {
		if (active_skill_ == SKILL_LAND) {
			return fail(FAULT_SKILL_TIMEOUT,
			            ACTION_EMERGENCY_DESCENT);
		}
		return startFailsafeLanding(
			FAULT_SKILL_TIMEOUT, false, now_ms, vehicle);
	}

	switch (active_skill_) {
		case SKILL_TAKEOFF:
			if (vehicle.altitude_m >=
			    target_altitude_m_ - kAltitudeToleranceM) {
				completeSkill();
				return last_target_;
			}
			return trackingTarget(vehicle);

		case SKILL_YAW:
			if (std::fabs(wrapDegrees(
				    target_yaw_deg_ - vehicle.yaw_deg)) <=
			    kYawToleranceDeg) {
				completeSkill();
				return last_target_;
			}
			return trackingTarget(vehicle);

		case SKILL_LAND:
			return landingTarget(now_ms, sensors, vehicle);

		case SKILL_HOLD:
		case SKILL_MOVE_BODY:
		case SKILL_RETURN_HOME:
			return trackingTarget(vehicle);

		case SKILL_NONE:
			return emptyTarget();
	}
	return emptyTarget();
}

}  // namespace flight_skills
