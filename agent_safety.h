#pragma once

#include <stdint.h>

namespace agent_safety {

static const uint8_t kSourceSystem = 42;
static const uint8_t kSourceComponent = 191;
static const uint32_t kCommandLongMessageId = 76;
static const uint16_t kPrivateCommand = 31010;
static const uint32_t kConfirmationCode = 7319;
static const uint32_t kHeartbeatTimeoutMs = 1000;

enum Skill : uint16_t {
	SKILL_HEARTBEAT = 1,
	SKILL_ARM = 2,
	SKILL_DISARM = 3,
	SKILL_EMERGENCY_STOP = 4,
	SKILL_TAKEOFF = 100,
	SKILL_LAND = 101,
	SKILL_HOLD = 102,
	SKILL_MOVE_BODY = 103,
	SKILL_YAW = 104,
	SKILL_RETURN_HOME = 105,
};

enum State : uint8_t {
	STATE_IDLE,
	STATE_READY,
	STATE_AGENT_ARMED,
	STATE_FAULT_LATCHED,
};

enum Fault : uint8_t {
	FAULT_NONE,
	FAULT_HEARTBEAT_TIMEOUT,
	FAULT_MANUAL_TAKEOVER,
	FAULT_LOW_BATTERY,
	FAULT_INVERTED,
	FAULT_ATTITUDE_INVALID,
	FAULT_ILLEGAL_COMMAND,
	FAULT_EMERGENCY_STOP,
};

enum Result : uint8_t {
	RESULT_ACCEPTED = 0,
	RESULT_DUPLICATE = 1,
	RESULT_NOT_IMPLEMENTED = 10,
	RESULT_ALREADY_ARMED = 20,
	RESULT_THROTTLE_NOT_LOW = 21,
	RESULT_INVERTED = 22,
	RESULT_BATTERY_UNSAFE = 23,
	RESULT_ATTITUDE_INVALID = 24,
	RESULT_HEARTBEAT_STALE = 25,
	RESULT_MANUAL_CONTROL_ACTIVE = 26,
	RESULT_NOT_LANDED = 27,
	RESULT_FAULT_LATCHED = 30,
	RESULT_UNKNOWN_SKILL = 31,
	RESULT_INVALID_REQUEST = 32,
	RESULT_ILLEGAL_MESSAGE = 33,
};

struct Snapshot {
	bool armed;
	bool throttle_low;
	bool inverted;
	bool battery_ok;
	bool attitude_ok;
	bool landed;
	bool manual_control_active;
};

struct Decision {
	Result result;
	bool arm;
	bool disarm;
};

bool isAgentMessageAllowed(uint32_t message_id, uint16_t command);

class Gate {
public:
	Gate();

	Decision handle(uint32_t now_ms, Skill skill, uint32_t request_id,
	                uint32_t confirmation_code, bool arguments_valid,
	                const Snapshot& snapshot);
	Decision update(uint32_t now_ms, const Snapshot& snapshot);
	Decision rejectIllegalMessage();

	State state() const { return state_; }
	Fault fault() const { return fault_; }

private:
	bool heartbeatFresh(uint32_t now_ms) const;
	Decision latch(Fault fault, Result result);

	State state_;
	Fault fault_;
	bool agent_owns_arm_;
	bool heartbeat_seen_;
	uint32_t last_heartbeat_ms_;
	bool last_execution_valid_;
	uint32_t last_execution_request_;
	Skill last_execution_skill_;
};

}  // namespace agent_safety
