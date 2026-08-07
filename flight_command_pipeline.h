#pragma once

#include "agent_safety.h"
#include "flight_skills.h"

// The only production route from a decoded Agent skill to the software-only
// state machine. A command is acknowledged only after both layers accept it.
namespace flight_command_pipeline {

struct Outcome {
	agent_safety::Decision safety;
	agent_safety::Result result;
	flight_skills::Result flight_result;
	bool flight_attempted;
};

class Pipeline {
public:
	Pipeline();

	Outcome handle(uint64_t now_ms, agent_safety::Skill skill,
	               uint32_t request_id, uint32_t confirmation_code,
	               bool arguments_valid, const flight_skills::Request& request,
	               const agent_safety::Snapshot& snapshot,
	               const flight_skills::VehicleState& vehicle,
	               const sensor_telemetry::HealthySnapshot& sensors);
	Outcome update(uint64_t now_ms, const agent_safety::Snapshot& snapshot,
	               const flight_skills::VehicleState& vehicle,
	               const sensor_telemetry::HealthySnapshot& sensors);
	Outcome rejectIllegalMessage();

	const agent_safety::Gate& gate() const { return gate_; }
	const flight_skills::Machine& machine() const { return machine_; }
	flight_skills::Target lastTarget() const { return last_target_; }

private:
	flight_skills::SafetyStatus safetyStatus(
		const agent_safety::Snapshot& snapshot, uint64_t now_ms) const;
	Outcome outcome(const agent_safety::Decision& safety,
	                flight_skills::Result flight_result,
	                bool flight_attempted) const;

	agent_safety::Gate gate_;
	flight_skills::Machine machine_;
	flight_skills::Target last_target_;
};

}  // namespace flight_command_pipeline
