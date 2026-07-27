#pragma once

#include <stdint.h>

// Hardware-neutral sensor sample validation for future flight skills.
// This file intentionally contains no Arduino or sensor-driver dependency.
namespace sensor_telemetry {

enum Health : uint8_t {
	HEALTH_UNDETECTED,
	HEALTH_WARMING,
	HEALTH_HEALTHY,
	HEALTH_STALE,
	HEALTH_OUT_OF_RANGE,
	HEALTH_LOW_QUALITY,
	HEALTH_TIME_REGRESSION,
};

struct OpticalFlowSample {
	uint64_t timestamp_ms;
	bool detected;
	bool valid;
	float angular_rate_x_rad_s;
	float angular_rate_y_rad_s;
	uint8_t quality;
};

struct RangeSample {
	uint64_t timestamp_ms;
	bool detected;
	bool valid;
	float distance_m;
	uint8_t quality;
};

struct BarometerSample {
	uint64_t timestamp_ms;
	bool detected;
	bool valid;
	float pressure_pa;
	float temperature_c;
	float relative_altitude_m;
	uint8_t quality;
};

struct OpticalFlowReading {
	float angular_rate_x_rad_s;
	float angular_rate_y_rad_s;
};

struct RangeReading {
	float distance_m;
};

struct BarometerReading {
	float pressure_pa;
	float temperature_c;
	float relative_altitude_m;
};

class TelemetryAggregator;

// A consumer gets measurements only through these guarded accessors.  A false
// return leaves the caller's output untouched; invalid data is never replaced
// with a seemingly valid zero.
class HealthySnapshot {
public:
	Health opticalFlowHealth() const { return optical_flow_health_; }
	Health rangeHealth() const { return range_health_; }
	Health barometerHealth() const { return barometer_health_; }

	bool opticalFlow(OpticalFlowReading* output) const;
	bool range(RangeReading* output) const;
	bool barometer(BarometerReading* output) const;

private:
	friend class TelemetryAggregator;
	HealthySnapshot();

	Health optical_flow_health_;
	Health range_health_;
	Health barometer_health_;
	OpticalFlowReading optical_flow_;
	RangeReading range_;
	BarometerReading barometer_;
};

class TelemetryAggregator {
public:
	static const uint64_t kOpticalFlowMaxAgeMs = 100;
	static const uint64_t kRangeMaxAgeMs = 200;
	static const uint64_t kBarometerMaxAgeMs = 500;
	static const uint8_t kMinimumQuality = 30;

	TelemetryAggregator();

	void submitOpticalFlow(const OpticalFlowSample& sample);
	void submitRange(const RangeSample& sample);
	void submitBarometer(const BarometerSample& sample);
	HealthySnapshot snapshot(uint64_t now_ms);

private:
	bool acceptTimestamp(uint64_t timestamp_ms, bool& seen,
	                     uint64_t& last_timestamp_ms, bool& time_regressed);

	OpticalFlowSample optical_flow_;
	RangeSample range_;
	BarometerSample barometer_;
	bool optical_flow_seen_;
	bool range_seen_;
	bool barometer_seen_;
	bool optical_flow_time_regressed_;
	bool range_time_regressed_;
	bool barometer_time_regressed_;
	uint64_t optical_flow_last_timestamp_ms_;
	uint64_t range_last_timestamp_ms_;
	uint64_t barometer_last_timestamp_ms_;
	bool snapshot_time_seen_;
	uint64_t last_snapshot_time_ms_;
};

}  // namespace sensor_telemetry
