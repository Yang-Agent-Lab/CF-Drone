#include "flight_command_v1.h"

#include <cmath>

namespace flight_command_v1 {

namespace {

bool finite(float value) {
	return std::isfinite(value);
}

bool zero(float value) {
	return value == 0.0f;
}

}  // namespace

bool isHighLevelSkill(uint16_t skill) {
	switch (skill) {
		case 100:
		case 101:
		case 102:
		case 103:
		case 104:
		case 105:
			return true;
	}
	return false;
}

bool decode(uint16_t skill, uint32_t request_id, float param4, float param5,
	        float param6, float param7, flight_skills::Request& output) {
	output = {};
	if (!finite(param4) || !finite(param5) || !finite(param6) ||
	    !finite(param7) || param7 != 1.0f) {
		return false;
	}

	flight_skills::Request decoded = {};
	decoded.request_id = request_id;
	const float max_hold_duration_s =
		static_cast<float>(flight_skills::kMaxHoldDurationMs) / 1000.0f;

	switch (skill) {
		case 100:
			if (!zero(param5) || !zero(param6) || param4 <= 0.0f ||
			    param4 > flight_skills::kMaxTakeoffHeightM) return false;
			decoded.skill = flight_skills::SKILL_TAKEOFF;
			decoded.height_m = param4;
			break;
		case 101:
			if (!zero(param4) || !zero(param5) || !zero(param6)) return false;
			decoded.skill = flight_skills::SKILL_LAND;
			break;
		case 102: {
			if (!zero(param5) || !zero(param6) || param4 <= 0.0f ||
			    param4 > max_hold_duration_s) return false;
			const uint32_t duration =
				static_cast<uint32_t>(std::ceil(param4 * 1000.0f));
			if (duration > flight_skills::kMaxHoldDurationMs) return false;
			decoded.skill = flight_skills::SKILL_HOLD;
			decoded.duration_ms = duration;
			break;
		}
		case 103: {
			if (param6 != flight_skills::kMaxHorizontalSpeedMps) return false;
			const float distance = std::sqrt(param4 * param4 + param5 * param5);
			if (!finite(distance) || distance <= 0.0f ||
			    distance > flight_skills::kMaxMoveDistanceM) return false;
			decoded.skill = flight_skills::SKILL_MOVE_BODY;
			decoded.body_x_m = param4;
			decoded.body_y_m = param5;
			decoded.speed_m_s = param6;
			break;
		}
		case 104:
			if (!zero(param6) || param4 == 0.0f ||
			    std::fabs(param4) > flight_skills::kMaxYawDeltaDeg ||
			    param5 != flight_skills::kMaxYawRateDegS) return false;
			decoded.skill = flight_skills::SKILL_YAW;
			decoded.yaw_delta_deg = param4;
			decoded.yaw_rate_deg_s = param5;
			break;
		case 105:
			if (!zero(param5) || !zero(param6) ||
			    param4 != flight_skills::kMaxHorizontalSpeedMps) return false;
			decoded.skill = flight_skills::SKILL_RETURN_HOME;
			decoded.speed_m_s = param4;
			break;
		default:
			return false;
	}

	output = decoded;
	return true;
}

}  // namespace flight_command_v1
