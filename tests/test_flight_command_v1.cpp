#include "agent_safety.h"
#include "flight_command_v1.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

using namespace flight_command_v1;

static void assertCleared(const flight_skills::Request& request) {
	assert(request.request_id == 0);
	assert(request.skill == flight_skills::SKILL_NONE);
	assert(request.height_m == 0.0f);
	assert(request.duration_ms == 0);
	assert(request.body_x_m == 0.0f);
	assert(request.body_y_m == 0.0f);
	assert(request.speed_m_s == 0.0f);
	assert(request.yaw_delta_deg == 0.0f);
	assert(request.yaw_rate_deg_s == 0.0f);
}

static void testGoldenVectors() {
	struct Vector {
		uint16_t skill;
		float param4;
		float param5;
		float param6;
		flight_skills::Skill decoded_skill;
	};
	const Vector vectors[] = {
		{agent_safety::SKILL_TAKEOFF, 0.75f, 0.0f, 0.0f,
		 flight_skills::SKILL_TAKEOFF},
		{agent_safety::SKILL_HOLD, 2.0f, 0.0f, 0.0f,
		 flight_skills::SKILL_HOLD},
		{agent_safety::SKILL_MOVE_BODY, 0.3f, -0.4f,
		 flight_skills::kMaxHorizontalSpeedMps, flight_skills::SKILL_MOVE_BODY},
		{agent_safety::SKILL_YAW, -90.0f, flight_skills::kMaxYawRateDegS, 0.0f,
		 flight_skills::SKILL_YAW},
		{agent_safety::SKILL_RETURN_HOME, flight_skills::kMaxHorizontalSpeedMps,
		 0.0f, 0.0f, flight_skills::SKILL_RETURN_HOME},
		{agent_safety::SKILL_LAND, 0.0f, 0.0f, 0.0f,
		 flight_skills::SKILL_LAND},
	};

	for (unsigned int i = 0; i < sizeof(vectors) / sizeof(vectors[0]); ++i) {
		assert(isHighLevelSkill(vectors[i].skill));
		flight_skills::Request request = {};
		assert(decode(vectors[i].skill, 2, vectors[i].param4, vectors[i].param5,
		              vectors[i].param6, 1.0f, request));
		assert(request.request_id == 2);
		assert(request.skill == vectors[i].decoded_skill);
	}

	flight_skills::Request takeoff = {};
	assert(decode(agent_safety::SKILL_TAKEOFF, 2, 0.75f, 0.0f, 0.0f, 1.0f,
	              takeoff));
	assert(takeoff.height_m == 0.75f);
	flight_skills::Request hold = {};
	assert(decode(agent_safety::SKILL_HOLD, 2, 2.0f, 0.0f, 0.0f, 1.0f, hold));
	assert(hold.duration_ms == 2000);
	struct HoldDuration {
		float seconds;
		uint32_t milliseconds;
	};
	const HoldDuration hold_durations[] = {
		{0.0001f, 1},
		{1.2345f, 1235},
		{5.0f, 5000},
	};
	for (unsigned int i = 0;
	     i < sizeof(hold_durations) / sizeof(hold_durations[0]); ++i) {
		flight_skills::Request rounded_hold = {};
		assert(decode(agent_safety::SKILL_HOLD, 2, hold_durations[i].seconds,
		              0.0f, 0.0f, 1.0f, rounded_hold));
		assert(rounded_hold.duration_ms == hold_durations[i].milliseconds);
	}
	flight_skills::Request move = {};
	assert(decode(agent_safety::SKILL_MOVE_BODY, 2, 0.3f, -0.4f,
	              flight_skills::kMaxHorizontalSpeedMps, 1.0f, move));
	assert(move.body_x_m == 0.3f && move.body_y_m == -0.4f);
	assert(move.speed_m_s == flight_skills::kMaxHorizontalSpeedMps);
	assert(!isHighLevelSkill(agent_safety::SKILL_HEARTBEAT));
	assert(!isHighLevelSkill(agent_safety::SKILL_ARM));
	assert(!isHighLevelSkill(agent_safety::SKILL_EMERGENCY_STOP));
}

static void testRejectsAndClearsOutput() {
	struct Rejected {
		uint16_t skill;
		float param4;
		float param5;
		float param6;
		float param7;
	};
	const Rejected rejected[] = {
		{999, 0.0f, 0.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_TAKEOFF, 0.0f, 0.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_TAKEOFF, flight_skills::kMaxTakeoffHeightM + 0.01f,
		 0.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_TAKEOFF, NAN, 0.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_HOLD, 0.0f, 0.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_HOLD,
		 static_cast<float>(flight_skills::kMaxHoldDurationMs) / 1000.0f + 0.01f,
		 0.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_MOVE_BODY, 0.0f, 0.0f,
		 flight_skills::kMaxHorizontalSpeedMps, 1.0f},
		{agent_safety::SKILL_MOVE_BODY, flight_skills::kMaxMoveDistanceM, 0.1f,
		 flight_skills::kMaxHorizontalSpeedMps, 1.0f},
		{agent_safety::SKILL_MOVE_BODY, 0.1f, 0.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_YAW, 0.0f, flight_skills::kMaxYawRateDegS, 0.0f,
		 1.0f},
		{agent_safety::SKILL_YAW, flight_skills::kMaxYawDeltaDeg + 0.01f,
		 flight_skills::kMaxYawRateDegS, 0.0f, 1.0f},
		{agent_safety::SKILL_YAW, 1.0f, 0.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_RETURN_HOME, 0.0f, 0.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_LAND, 0.0f, 0.0f, 0.0f, 2.0f},
		{agent_safety::SKILL_LAND, INFINITY, 0.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_TAKEOFF, 0.5f, 1.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_HOLD, 1.0f, 0.0f, 1.0f, 1.0f},
		{agent_safety::SKILL_MOVE_BODY, 0.1f, 0.0f,
		 flight_skills::kMaxHorizontalSpeedMps, 0.0f},
		{agent_safety::SKILL_YAW, 1.0f, flight_skills::kMaxYawRateDegS, 1.0f,
		 1.0f},
		{agent_safety::SKILL_RETURN_HOME, flight_skills::kMaxHorizontalSpeedMps,
		 1.0f, 0.0f, 1.0f},
		{agent_safety::SKILL_LAND, 0.0f, 1.0f, 0.0f, 1.0f},
	};

	for (unsigned int i = 0; i < sizeof(rejected) / sizeof(rejected[0]); ++i) {
		flight_skills::Request request = {};
		request.request_id = 99;
		request.skill = flight_skills::SKILL_YAW;
		request.height_m = 1.0f;
		assert(!decode(rejected[i].skill, 2, rejected[i].param4,
		               rejected[i].param5, rejected[i].param6,
		               rejected[i].param7, request));
		assertCleared(request);
	}
}

int main() {
	testGoldenVectors();
	testRejectsAndClearsOutput();
	puts("flight command v1 decoder tests: PASS");
}
