#pragma once

#include <stdint.h>

#include "sensor_telemetry.h"

// Hardware-neutral flight-skill sequencing. This module emits bounded targets;
// it does not estimate state or write attitude, PID, mixer, or motor values.
namespace flight_skills {

static const float kMaxTakeoffHeightM = 1.0f;
static const uint32_t kMaxHoldDurationMs = 5000;
static const float kMaxMoveDistanceM = 0.5f;
static const float kMaxHorizontalSpeedMps = 0.25f;
static const float kMaxYawDeltaDeg = 180.0f;
static const float kMaxYawRateDegS = 30.0f;

static const uint64_t kVehicleStateMaxAgeMs = 100;
static const uint64_t kMissionTimeoutMs = 60000;
static const uint64_t kTakeoffTimeoutMs = 10000;
static const uint64_t kMoveTimeoutMs = 10000;
static const uint64_t kYawTimeoutMs = 10000;
static const uint64_t kReturnTimeoutMs = 20000;
static const uint64_t kLandTimeoutMs = 20000;
static const uint64_t kGroundConfirmMs = 500;

static const float kLandingDescentRateMps = 0.20f;
static const float kRedundantDescentRateMps = 0.15f;
static const float kEmergencyDescentRateMps = 0.30f;

enum Skill : uint8_t {
	SKILL_NONE,
	SKILL_TAKEOFF,
	SKILL_HOLD,
	SKILL_MOVE_BODY,
	SKILL_YAW,
	SKILL_RETURN_HOME,
	SKILL_LAND,
};

enum MissionState : uint8_t {
	MISSION_IDLE,
	MISSION_ACTIVE,
	MISSION_FAILSAFE_LANDING,
	MISSION_COMPLETE,
	MISSION_FAILED,
};

enum Fault : uint8_t {
	FAULT_NONE,
	FAULT_INVALID_ARGUMENT,
	FAULT_SEQUENCE,
	FAULT_DUPLICATE_REQUEST,
	FAULT_TIME_REGRESSION,
	FAULT_VEHICLE_STATE_TIMEOUT,
	FAULT_MISSION_TIMEOUT,
	FAULT_SKILL_TIMEOUT,
	FAULT_OPTICAL_FLOW_LOSS,
	FAULT_RANGE_LOSS,
	FAULT_BAROMETER_LOSS,
	FAULT_ALTITUDE_SENSORS_LOST,
	FAULT_MANUAL_TAKEOVER,
	FAULT_LOW_BATTERY,
	FAULT_INVERTED,
	FAULT_HEARTBEAT_TIMEOUT,
	FAULT_UPSTREAM_GATE,
};

enum Result : uint8_t {
	RESULT_ACCEPTED,
	RESULT_INVALID_ARGUMENT,
	RESULT_SEQUENCE_ERROR,
	RESULT_DUPLICATE_REQUEST,
	RESULT_SENSOR_UNHEALTHY,
	RESULT_SAFETY_FAULT,
	RESULT_FAILED_LATCHED,
};

enum Action : uint8_t {
	ACTION_NONE,
	ACTION_TRACK,
	ACTION_LAND,
	ACTION_REDUNDANT_LAND,
	ACTION_EMERGENCY_DESCENT,
	ACTION_REQUEST_LOCK,
	ACTION_ABORT,
};

enum HorizontalFrame : uint8_t {
	FRAME_LOCAL,
	FRAME_BODY,
};

struct VehicleState {
	uint64_t timestamp_ms;
	bool armed;
	bool landed;
	float local_x_m;
	float local_y_m;
	float altitude_m;
	float yaw_deg;
};

// These flags are decisions made by the existing upstream safety layer. The
// state machine consumes them and does not reproduce those checks.
struct SafetyStatus {
	bool upstream_gate_ok;
	bool manual_takeover;
	bool battery_ok;
	bool inverted;
	bool heartbeat_fresh;
};

struct Request {
	uint32_t request_id;
	Skill skill;
	float height_m;
	uint32_t duration_ms;
	float body_x_m;
	float body_y_m;
	float speed_m_s;
	float yaw_delta_deg;
	float yaw_rate_deg_s;
};

struct Target {
	bool valid;
	Action action;
	HorizontalFrame horizontal_frame;
	float altitude_m;
	float velocity_x_m_s;
	float velocity_y_m_s;
	float yaw_deg;
	float yaw_rate_deg_s;
	// Positive magnitude. A future controller decides how to realize it.
	float descent_rate_m_s;
};

class Machine {
public:
	Machine();

	Result beginAtArm(uint64_t now_ms, const VehicleState& vehicle,
	                  const SafetyStatus& safety);
	Result start(uint64_t now_ms, const Request& request,
	             const sensor_telemetry::HealthySnapshot& sensors,
	             const VehicleState& vehicle, const SafetyStatus& safety);
	Target update(uint64_t now_ms,
	              const sensor_telemetry::HealthySnapshot& sensors,
	              const VehicleState& vehicle, const SafetyStatus& safety);

	MissionState missionState() const { return mission_state_; }
	Skill activeSkill() const { return active_skill_; }
	Skill completedSkill() const { return completed_skill_; }
	Fault fault() const { return fault_; }

private:
	bool acceptTime(uint64_t now_ms);
	bool checkVehicle(uint64_t now_ms, const VehicleState& vehicle);
	bool checkSafety(const SafetyStatus& safety);
	bool checkCommon(uint64_t now_ms, const VehicleState& vehicle,
	                 const SafetyStatus& safety);
	Result reject(Fault fault, Result result);
	Target fail(Fault fault, Action action);
	Target startFailsafeLanding(Fault fault, bool redundant,
	                            uint64_t now_ms, const VehicleState& vehicle);
	Target landingTarget(uint64_t now_ms,
	                     const sensor_telemetry::HealthySnapshot& sensors,
	                     const VehicleState& vehicle);
	Target trackingTarget(const VehicleState& vehicle);
	uint64_t activeTimeoutMs() const;
	void completeSkill();

	MissionState mission_state_;
	Skill active_skill_;
	Skill completed_skill_;
	Fault fault_;
	bool time_seen_;
	uint64_t last_now_ms_;
	uint64_t mission_start_ms_;
	uint64_t skill_start_ms_;
	bool request_seen_;
	uint32_t last_request_id_;
	bool took_off_;
	bool redundant_landing_;
	bool ground_seen_;
	uint64_t ground_since_ms_;

	float origin_x_m_;
	float origin_y_m_;
	float origin_altitude_m_;
	float origin_yaw_deg_;
	float target_altitude_m_;
	float target_yaw_deg_;
	float target_yaw_rate_deg_s_;
	float command_speed_m_s_;
	float move_target_x_m_;
	float move_target_y_m_;
	uint32_t hold_duration_ms_;
	Target last_target_;
};

}  // namespace flight_skills
