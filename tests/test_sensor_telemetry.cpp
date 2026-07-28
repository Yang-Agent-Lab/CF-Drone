#include "sensor_telemetry.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

using namespace sensor_telemetry;

static OpticalFlowSample healthyFlow(uint64_t timestamp_ms) {
	OpticalFlowSample sample = {timestamp_ms, true, true, 0.1f, -0.2f, 200};
	return sample;
}

static RangeSample healthyRange(uint64_t timestamp_ms) {
	RangeSample sample = {timestamp_ms, true, true, 1.5f, 200};
	return sample;
}

static BarometerSample healthyBarometer(uint64_t timestamp_ms) {
	BarometerSample sample = {timestamp_ms, true, true, 101325.0f, 20.0f, 2.0f, 200};
	return sample;
}

static void testHealthySnapshotOnlyReturnsHealthyValues() {
	TelemetryAggregator telemetry;
	telemetry.submitOpticalFlow(healthyFlow(100));
	telemetry.submitRange(healthyRange(100));
	telemetry.submitBarometer(healthyBarometer(100));
	HealthySnapshot snapshot = telemetry.snapshot(150);
	assert(snapshot.opticalFlowHealth() == HEALTH_HEALTHY);
	assert(snapshot.rangeHealth() == HEALTH_HEALTHY);
	assert(snapshot.barometerHealth() == HEALTH_HEALTHY);

	OpticalFlowReading flow = {};
	RangeReading range = {};
	BarometerReading barometer = {};
	assert(snapshot.opticalFlow(&flow));
	assert(snapshot.range(&range));
	assert(snapshot.barometer(&barometer));
	assert(flow.angular_rate_x_rad_s == 0.1f);
	assert(range.distance_m == 1.5f);
	assert(barometer.relative_altitude_m == 2.0f);
}

static void testUndetectedAndWarmingDoNotReturnZeroes() {
	TelemetryAggregator telemetry;
	HealthySnapshot undetected = telemetry.snapshot(0);
	RangeReading retained = {123.0f};
	assert(undetected.rangeHealth() == HEALTH_UNDETECTED);
	assert(!undetected.range(&retained));
	assert(retained.distance_m == 123.0f);

	RangeSample warming = {1, true, false, 0.0f, 0};
	telemetry.submitRange(warming);
	HealthySnapshot snapshot = telemetry.snapshot(1);
	assert(snapshot.rangeHealth() == HEALTH_WARMING);
	assert(!snapshot.range(&retained));
	assert(retained.distance_m == 123.0f);
}

static void testAgeFiniteRangeAndQualityFailures() {
	TelemetryAggregator stale;
	stale.submitOpticalFlow(healthyFlow(0));
	assert(stale.snapshot(TelemetryAggregator::kOpticalFlowMaxAgeMs + 1)
	           .opticalFlowHealth() == HEALTH_STALE);

	TelemetryAggregator nonfinite;
	RangeSample nan_range = healthyRange(10);
	nan_range.distance_m = NAN;
	nonfinite.submitRange(nan_range);
	assert(nonfinite.snapshot(10).rangeHealth() == HEALTH_OUT_OF_RANGE);

	TelemetryAggregator impossible_range;
	RangeSample too_far = healthyRange(10);
	too_far.distance_m = 4.1f;
	impossible_range.submitRange(too_far);
	assert(impossible_range.snapshot(10).rangeHealth() == HEALTH_OUT_OF_RANGE);

	TelemetryAggregator out_of_range;
	BarometerSample pressure = healthyBarometer(10);
	pressure.pressure_pa = 120000.0f;
	out_of_range.submitBarometer(pressure);
	assert(out_of_range.snapshot(10).barometerHealth() == HEALTH_OUT_OF_RANGE);

	TelemetryAggregator low_quality;
	OpticalFlowSample flow = healthyFlow(10);
	flow.quality = TelemetryAggregator::kMinimumQuality - 1;
	low_quality.submitOpticalFlow(flow);
	assert(low_quality.snapshot(10).opticalFlowHealth() == HEALTH_LOW_QUALITY);
}

static void testTimeRegressionFailsClosed() {
	TelemetryAggregator telemetry;
	telemetry.submitRange(healthyRange(100));
	assert(telemetry.snapshot(100).rangeHealth() == HEALTH_HEALTHY);

	telemetry.submitRange(healthyRange(99));
	assert(telemetry.snapshot(100).rangeHealth() == HEALTH_TIME_REGRESSION);
	telemetry.submitRange(healthyRange(101));
	assert(telemetry.snapshot(101).rangeHealth() == HEALTH_HEALTHY);

	assert(telemetry.snapshot(100).rangeHealth() == HEALTH_TIME_REGRESSION);
}

static void testPartialFailureKeepsOtherSensorsUsable() {
	TelemetryAggregator telemetry;
	OpticalFlowSample bad_flow = healthyFlow(50);
	bad_flow.quality = 0;
	telemetry.submitOpticalFlow(bad_flow);
	telemetry.submitRange(healthyRange(50));
	telemetry.submitBarometer(healthyBarometer(50));
	HealthySnapshot snapshot = telemetry.snapshot(50);
	assert(snapshot.opticalFlowHealth() == HEALTH_LOW_QUALITY);
	assert(snapshot.rangeHealth() == HEALTH_HEALTHY);
	assert(snapshot.barometerHealth() == HEALTH_HEALTHY);

	OpticalFlowReading flow = {};
	RangeReading range = {};
	BarometerReading barometer = {};
	assert(!snapshot.opticalFlow(&flow));
	assert(snapshot.range(&range));
	assert(snapshot.barometer(&barometer));
}

int main() {
	testHealthySnapshotOnlyReturnsHealthyValues();
	testUndetectedAndWarmingDoNotReturnZeroes();
	testAgeFiniteRangeAndQualityFailures();
	testTimeRegressionFailsClosed();
	testPartialFailureKeepsOtherSensorsUsable();
	puts("sensor telemetry tests: PASS");
}
