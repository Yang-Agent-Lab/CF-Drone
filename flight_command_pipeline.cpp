#include "flight_command_pipeline.h"

namespace flight_command_pipeline {

namespace {

bool flightSkill(agent_safety::Skill skill) {
	return skill >= agent_safety::SKILL_TAKEOFF &&
	       skill <= agent_safety::SKILL_RETURN_HOME;
}

}  // namespace

Pipeline::Pipeline() : gate_(), machine_(), last_target_() {}

flight_skills::SafetyStatus Pipeline::safetyStatus(
	const agent_safety::Snapshot& snapshot, uint64_t now_ms) const {
	flight_skills::SafetyStatus status = {};
	status.upstream_gate_ok = gate_.state() == agent_safety::STATE_AGENT_ARMED &&
	                          gate_.agentOwnsArm();
	status.manual_takeover = snapshot.manual_control_active;
	status.battery_ok = snapshot.battery_ok;
	status.inverted = snapshot.inverted;
	status.heartbeat_fresh = gate_.heartbeatFresh(static_cast<uint32_t>(now_ms));
	return status;
}

Outcome Pipeline::outcome(const agent_safety::Decision& safety,
	                      flight_skills::Result flight_result,
	                      bool flight_attempted) const {
	Outcome result = {safety, safety.result, flight_result, flight_attempted};
	return result;
}

Outcome Pipeline::handle(
	uint64_t now_ms, agent_safety::Skill skill, uint32_t request_id,
	uint32_t confirmation_code, bool arguments_valid,
	const flight_skills::Request& request,
	const agent_safety::Snapshot& snapshot,
	const flight_skills::VehicleState& vehicle,
	const sensor_telemetry::HealthySnapshot& sensors) {
	const uint32_t gate_now_ms = static_cast<uint32_t>(now_ms);
	agent_safety::Decision safety = gate_.handle(
		gate_now_ms, skill, request_id, confirmation_code, arguments_valid,
		snapshot);
	if (safety.result != agent_safety::RESULT_ACCEPTED) {
		return outcome(safety, flight_skills::RESULT_SAFETY_FAULT, false);
	}

	const flight_skills::SafetyStatus status = safetyStatus(snapshot, now_ms);
	if (skill == agent_safety::SKILL_ARM && safety.arm) {
		const flight_skills::Result flight =
			machine_.beginAtArm(now_ms, vehicle, status);
		return outcome(safety, flight, false);
	}
	if (!flightSkill(skill)) {
		return outcome(safety, flight_skills::RESULT_ACCEPTED, false);
	}

	const flight_skills::Result flight =
		machine_.start(now_ms, request, sensors, vehicle, status);
	if (flight == flight_skills::RESULT_ACCEPTED) {
		gate_.commitFlightSkill(request_id, skill);
		return outcome(safety, flight, true);
	}

	safety.result = agent_safety::RESULT_FLIGHT_REJECTED;
	return outcome(safety, flight, true);
}

Outcome Pipeline::update(
	uint64_t now_ms, const agent_safety::Snapshot& snapshot,
	const flight_skills::VehicleState& vehicle,
	const sensor_telemetry::HealthySnapshot& sensors) {
	agent_safety::Decision safety = gate_.update(
		static_cast<uint32_t>(now_ms), snapshot);
	last_target_ = machine_.update(now_ms, sensors, vehicle,
	                              safetyStatus(snapshot, now_ms));
	return outcome(safety, flight_skills::RESULT_ACCEPTED, false);
}

Outcome Pipeline::rejectIllegalMessage() {
	return outcome(gate_.rejectIllegalMessage(),
	               flight_skills::RESULT_SAFETY_FAULT, false);
}

}  // namespace flight_command_pipeline
