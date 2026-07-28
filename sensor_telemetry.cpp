#include "sensor_telemetry.h"

#include <cmath>

namespace sensor_telemetry {

namespace {

bool finite(float value) {
	return std::isfinite(value);
}

bool healthy(Health health) {
	return health == HEALTH_HEALTHY;
}

Health baseHealth(bool seen, bool detected, bool valid, bool time_regressed,
	              uint64_t timestamp_ms, uint64_t now_ms, uint64_t max_age_ms) {
	if (!seen || !detected) return HEALTH_UNDETECTED;
	if (!valid) return HEALTH_WARMING;
	if (time_regressed || now_ms < timestamp_ms) return HEALTH_TIME_REGRESSION;
	if (now_ms - timestamp_ms > max_age_ms) return HEALTH_STALE;
	return HEALTH_HEALTHY;
}

Health opticalFlowHealth(const OpticalFlowSample& sample, bool seen,
	                     bool time_regressed, uint64_t now_ms) {
	Health health = baseHealth(seen, sample.detected, sample.valid,
	                           time_regressed, sample.timestamp_ms, now_ms,
	                           TelemetryAggregator::kOpticalFlowMaxAgeMs);
	if (!healthy(health)) return health;
	if (!finite(sample.angular_rate_x_rad_s) ||
	    !finite(sample.angular_rate_y_rad_s) ||
	    std::fabs(sample.angular_rate_x_rad_s) > 20.0f ||
	    std::fabs(sample.angular_rate_y_rad_s) > 20.0f) {
		return HEALTH_OUT_OF_RANGE;
	}
	return sample.quality < TelemetryAggregator::kMinimumQuality
		? HEALTH_LOW_QUALITY : HEALTH_HEALTHY;
}

Health rangeHealth(const RangeSample& sample, bool seen, bool time_regressed,
	               uint64_t now_ms) {
	Health health = baseHealth(seen, sample.detected, sample.valid,
	                           time_regressed, sample.timestamp_ms, now_ms,
	                           TelemetryAggregator::kRangeMaxAgeMs);
	if (!healthy(health)) return health;
	if (!finite(sample.distance_m) || sample.distance_m < 0.04f ||
	    sample.distance_m > 4.0f) {
		return HEALTH_OUT_OF_RANGE;
	}
	return sample.quality < TelemetryAggregator::kMinimumQuality
		? HEALTH_LOW_QUALITY : HEALTH_HEALTHY;
}

Health barometerHealth(const BarometerSample& sample, bool seen,
	                   bool time_regressed, uint64_t now_ms) {
	Health health = baseHealth(seen, sample.detected, sample.valid,
	                           time_regressed, sample.timestamp_ms, now_ms,
	                           TelemetryAggregator::kBarometerMaxAgeMs);
	if (!healthy(health)) return health;
	if (!finite(sample.pressure_pa) || !finite(sample.temperature_c) ||
	    !finite(sample.relative_altitude_m) || sample.pressure_pa < 30000.0f ||
	    sample.pressure_pa > 110000.0f || sample.temperature_c < -40.0f ||
	    sample.temperature_c > 85.0f || sample.relative_altitude_m < -1000.0f ||
	    sample.relative_altitude_m > 10000.0f) {
		return HEALTH_OUT_OF_RANGE;
	}
	return sample.quality < TelemetryAggregator::kMinimumQuality
		? HEALTH_LOW_QUALITY : HEALTH_HEALTHY;
}

}  // namespace

HealthySnapshot::HealthySnapshot()
	: optical_flow_health_(HEALTH_UNDETECTED),
	  range_health_(HEALTH_UNDETECTED),
	  barometer_health_(HEALTH_UNDETECTED),
	  optical_flow_(),
	  range_(),
	  barometer_() {}

bool HealthySnapshot::opticalFlow(OpticalFlowReading* output) const {
	if (!output || !healthy(optical_flow_health_)) return false;
	*output = optical_flow_;
	return true;
}

bool HealthySnapshot::range(RangeReading* output) const {
	if (!output || !healthy(range_health_)) return false;
	*output = range_;
	return true;
}

bool HealthySnapshot::barometer(BarometerReading* output) const {
	if (!output || !healthy(barometer_health_)) return false;
	*output = barometer_;
	return true;
}

TelemetryAggregator::TelemetryAggregator()
	: optical_flow_(),
	  range_(),
	  barometer_(),
	  optical_flow_seen_(false),
	  range_seen_(false),
	  barometer_seen_(false),
	  optical_flow_time_regressed_(false),
	  range_time_regressed_(false),
	  barometer_time_regressed_(false),
	  optical_flow_last_timestamp_ms_(0),
	  range_last_timestamp_ms_(0),
	  barometer_last_timestamp_ms_(0),
	  snapshot_time_seen_(false),
	  last_snapshot_time_ms_(0) {}

bool TelemetryAggregator::acceptTimestamp(uint64_t timestamp_ms, bool& seen,
	                                        uint64_t& last_timestamp_ms,
	                                        bool& time_regressed) {
	if (seen && timestamp_ms < last_timestamp_ms) {
		time_regressed = true;
		return false;
	}
	seen = true;
	last_timestamp_ms = timestamp_ms;
	time_regressed = false;
	return true;
}

void TelemetryAggregator::submitOpticalFlow(const OpticalFlowSample& sample) {
	if (acceptTimestamp(sample.timestamp_ms, optical_flow_seen_,
	                    optical_flow_last_timestamp_ms_,
	                    optical_flow_time_regressed_)) {
		optical_flow_ = sample;
	}
}

void TelemetryAggregator::submitRange(const RangeSample& sample) {
	if (acceptTimestamp(sample.timestamp_ms, range_seen_, range_last_timestamp_ms_,
	                    range_time_regressed_)) {
		range_ = sample;
	}
}

void TelemetryAggregator::submitBarometer(const BarometerSample& sample) {
	if (acceptTimestamp(sample.timestamp_ms, barometer_seen_,
	                    barometer_last_timestamp_ms_,
	                    barometer_time_regressed_)) {
		barometer_ = sample;
	}
}

HealthySnapshot TelemetryAggregator::snapshot(uint64_t now_ms) {
	const bool clock_regressed = snapshot_time_seen_ && now_ms < last_snapshot_time_ms_;
	if (!clock_regressed) {
		snapshot_time_seen_ = true;
		last_snapshot_time_ms_ = now_ms;
	}

	HealthySnapshot result;
	result.optical_flow_health_ = opticalFlowHealth(
		optical_flow_, optical_flow_seen_,
		clock_regressed || optical_flow_time_regressed_, now_ms);
	result.range_health_ = rangeHealth(range_, range_seen_,
	                                  clock_regressed || range_time_regressed_, now_ms);
	result.barometer_health_ = barometerHealth(
		barometer_, barometer_seen_,
		clock_regressed || barometer_time_regressed_, now_ms);

	if (healthy(result.optical_flow_health_)) {
		result.optical_flow_.angular_rate_x_rad_s = optical_flow_.angular_rate_x_rad_s;
		result.optical_flow_.angular_rate_y_rad_s = optical_flow_.angular_rate_y_rad_s;
	}
	if (healthy(result.range_health_)) result.range_.distance_m = range_.distance_m;
	if (healthy(result.barometer_health_)) {
		result.barometer_.pressure_pa = barometer_.pressure_pa;
		result.barometer_.temperature_c = barometer_.temperature_c;
		result.barometer_.relative_altitude_m = barometer_.relative_altitude_m;
	}
	return result;
}

}  // namespace sensor_telemetry
