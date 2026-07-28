// Agent MAVLink入口：固定来源只能进入安全门，不能进入原有直控分支。

#if WIFI_ENABLED

#include <MAVLink.h>
#include "agent_safety.h"
#include "agent_status.h"
#include "flight_command_v1.h"
#include "util.h"

extern int mavlinkSysId;
agent_status::Snapshot agentStatusSnapshot();

static Rate agentStatusRate(10);
static const char kAgentStatusName[] = "AGT_STAT";

static void sendAgentStatus() {
	if (!agentStatusRate) return;
	int32_t value = 0;
	if (!agent_status::encode(agentStatusSnapshot(), value)) return;
	mavlink_message_t status;
	mavlink_msg_named_value_int_pack(
		mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &status, millis(),
		kAgentStatusName, value);
	sendMessage(&status);
}

agent_safety::Result handleAgentSafetyCommand(
	uint32_t now_ms, agent_safety::Skill skill, uint32_t request_id,
	uint32_t confirmation_code, bool arguments_valid,
	const flight_skills::Request& request);
agent_safety::Result rejectAgentSafetyMessage();

static bool exactUnsigned(float value, uint32_t maximum, uint32_t& output) {
	if (!isfinite(value) || value < 0 || value > maximum) return false;
	uint32_t converted = static_cast<uint32_t>(value);
	if (value != static_cast<float>(converted)) return false;
	output = converted;
	return true;
}

static uint8_t agentMavlinkResult(agent_safety::Result result) {
	switch (result) {
		case agent_safety::RESULT_ACCEPTED:
		case agent_safety::RESULT_DUPLICATE:
			return MAV_RESULT_ACCEPTED;
		case agent_safety::RESULT_NOT_IMPLEMENTED:
			return MAV_RESULT_UNSUPPORTED;
		case agent_safety::RESULT_ALREADY_ARMED:
		case agent_safety::RESULT_THROTTLE_NOT_LOW:
		case agent_safety::RESULT_INVERTED:
		case agent_safety::RESULT_BATTERY_UNSAFE:
		case agent_safety::RESULT_ATTITUDE_INVALID:
		case agent_safety::RESULT_HEARTBEAT_STALE:
		case agent_safety::RESULT_MANUAL_CONTROL_ACTIVE:
		case agent_safety::RESULT_NOT_LANDED:
		case agent_safety::RESULT_AGENT_NOT_ARMED:
			return MAV_RESULT_TEMPORARILY_REJECTED;
		default:
			return MAV_RESULT_DENIED;
	}
}

static void sendAgentAck(const mavlink_message_t& request, uint16_t command,
	                     agent_safety::Result result) {
	mavlink_message_t ack;
	mavlink_msg_command_ack_pack(
		mavlinkSysId, MAV_COMP_ID_AUTOPILOT1, &ack, command,
		agentMavlinkResult(result), UINT8_MAX, static_cast<int32_t>(result),
		request.sysid, request.compid);
	sendMessage(&ack);
}

bool routeAgentMavlink(const void* raw_message) {
	const mavlink_message_t& message =
		*static_cast<const mavlink_message_t*>(raw_message);
	if (message.sysid != agent_safety::kSourceSystem ||
	    message.compid != agent_safety::kSourceComponent) {
		return false;
	}

	if (message.msgid != MAVLINK_MSG_ID_COMMAND_LONG) {
		rejectAgentSafetyMessage();
		return true;
	}

	mavlink_command_long_t command;
	mavlink_msg_command_long_decode(&message, &command);
	if (command.target_system != mavlinkSysId ||
	    command.target_component != MAV_COMP_ID_AUTOPILOT1 ||
	    !agent_safety::isAgentMessageAllowed(message.msgid, command.command)) {
		agent_safety::Result result = rejectAgentSafetyMessage();
		sendAgentAck(message, command.command, result);
		return true;
	}

	uint32_t skill_value = 0;
	uint32_t request_id = 0;
	uint32_t confirmation_code = 0;
	bool valid = exactUnsigned(command.param1, UINT16_MAX, skill_value) &&
	             exactUnsigned(command.param2, 16777215UL, request_id) &&
	             exactUnsigned(command.param3, 16777215UL, confirmation_code);
	flight_skills::Request decoded_request = {};
	const bool arguments_valid = flight_command_v1::isHighLevelSkill(skill_value)
		? flight_command_v1::decode(skill_value, request_id, command.param4,
		                          command.param5, command.param6, command.param7,
		                          decoded_request)
		: command.param4 == 0.0f && command.param5 == 0.0f &&
		  command.param6 == 0.0f && command.param7 == 0.0f;
	if (!valid) request_id = 0;

	agent_safety::Result result = handleAgentSafetyCommand(
		millis(), static_cast<agent_safety::Skill>(skill_value), request_id,
		confirmation_code, arguments_valid, decoded_request);
	sendAgentAck(message, command.command, result);
	return true;
}

#endif
