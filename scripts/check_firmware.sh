#!/bin/sh
set -eu

case "${1:-}" in
	safety|sensors|control|command|pipeline|status) ;;
	*)
		echo "usage: $0 {safety|sensors|control|command|pipeline|status}" >&2
	exit 2
		;;
esac

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/cf-drone-check.XXXXXX")
trap 'rm -rf "$tmp_dir"' EXIT HUP INT TERM

cd "$repo_dir"

if [ "$1" = "sensors" ]; then
	"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic -I. \
		sensor_telemetry.cpp tests/test_sensor_telemetry.cpp \
		-o "$tmp_dir/test_sensor_telemetry"
	"$tmp_dir/test_sensor_telemetry"
	exit 0
fi

if [ "$1" = "control" ]; then
	"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic -I. \
		sensor_telemetry.cpp flight_skills.cpp tests/test_flight_skills.cpp \
		-o "$tmp_dir/test_flight_skills"
	"$tmp_dir/test_flight_skills"

	if grep -En \
		'(motors\[|attitudeTarget|ratesTarget|torqueTarget|thrustTarget|MAVLINK|Arduino)' \
		flight_skills.h flight_skills.cpp >/dev/null; then
		echo "飞行技能状态机包含禁止的硬件或直控符号" >&2
		exit 1
	fi

	echo "flight skill software-only boundary checks: PASS"
	exit 0
fi

if [ "$1" = "command" ]; then
	"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic -I. \
		flight_command_v1.cpp tests/test_flight_command_v1.cpp \
		-o "$tmp_dir/test_flight_command_v1"
	"$tmp_dir/test_flight_command_v1"

	grep -q 'flight_command_v1::isHighLevelSkill' agent_mavlink.ino || {
		echo "Agent入口没有区分高层技能参数" >&2
		exit 1
	}
	grep -q 'flight_command_v1::decode' agent_mavlink.ino || {
		echo "Agent入口没有调用v1解码器" >&2
		exit 1
	}
	if grep -En 'flight_skills::Machine' agent_mavlink.ino >/dev/null; then
		echo "v1解码入口不得直接启动飞行技能状态机" >&2
		exit 1
	fi
	exit 0
fi

if [ "$1" = "pipeline" ]; then
	"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic -I. \
		agent_safety.cpp sensor_telemetry.cpp flight_skills.cpp \
		flight_command_v1.cpp flight_command_pipeline.cpp \
		tests/test_flight_command_pipeline.cpp -o "$tmp_dir/test_flight_command_pipeline"
	"$tmp_dir/test_flight_command_pipeline"
	exit 0
fi

if [ "$1" = "status" ]; then
	"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic -I. \
		agent_status.cpp tests/test_agent_status.cpp \
		-o "$tmp_dir/test_agent_status"
	"$tmp_dir/test_agent_status"

	grep -q 'mavlink_msg_named_value_int_pack' agent_mavlink.ino || {
		echo "Agent状态没有使用MAVLink common NAMED_VALUE_INT" >&2
		exit 1
	}
	grep -q '"AGT_STAT"' agent_mavlink.ino || {
		echo "Agent状态名称不是固定AGT_STAT" >&2
		exit 1
	}
	grep -q 'Rate agentStatusRate(10)' agent_mavlink.ino || {
		echo "Agent状态发布频率没有限制为10Hz" >&2
		exit 1
	}
	grep -q 'sendAgentStatus();' mavlink.ino || {
		echo "MAVLink发送入口没有发布Agent状态" >&2
		exit 1
	}
	if grep -En \
		'(motors|attitudeTarget|ratesTarget|torqueTarget|thrustTarget|PID)' \
		agent_status.h agent_status.cpp >/dev/null; then
		echo "Agent状态编解码器包含控制或硬件符号" >&2
		exit 1
	fi

	echo "agent status protocol checks: PASS"
	exit 0
fi

"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic -I. \
	agent_safety.cpp tests/test_agent_safety.cpp -o "$tmp_dir/test_agent_safety"
"$tmp_dir/test_agent_safety"

if grep -En \
	'(motors|attitudeTarget|ratesTarget|torqueTarget|setParameter|doCommand|controlRoll|controlPitch|controlYaw|controlThrottle)' \
	agent_mavlink.ino >/dev/null; then
	echo "Agent MAVLink入口包含禁止的直控符号" >&2
	exit 1
fi

grep -q 'agentFlightPipeline.handle' safety.ino || {
	echo "生产安全门没有进入受保护飞行命令管线" >&2
	exit 1
}
grep -q 'agentSensorTelemetry.snapshot' safety.ino || {
	echo "生产飞行命令没有使用失败关闭的传感器快照" >&2
	exit 1
}
if grep -En 'agentSensorTelemetry\.submit' safety.ino >/dev/null; then
	echo "生产安全门不得伪造健康传感器数据" >&2
	exit 1
fi

guard_line=$(grep -n 'if (routeAgentMavlink(_msg)) return;' mavlink.ino | cut -d: -f1)
manual_line=$(grep -n 'MAVLINK_MSG_ID_MANUAL_CONTROL' mavlink.ino | head -n 1 | cut -d: -f1)
if [ -z "$guard_line" ] || [ -z "$manual_line" ] || [ "$guard_line" -ge "$manual_line" ]; then
	echo "Agent安全门没有位于旧MAVLink控制分支之前" >&2
	exit 1
fi

for required in \
	MAVLINK_MSG_ID_MANUAL_CONTROL \
	MAVLINK_MSG_ID_PARAM_SET \
	MAVLINK_MSG_ID_SERIAL_CONTROL \
	MAVLINK_MSG_ID_SET_ATTITUDE_TARGET \
	MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET \
	MAV_CMD_COMPONENT_ARM_DISARM \
	MAV_CMD_DO_SET_MODE
do
	grep -q "$required" mavlink.ino || {
		echo "缺少源代码边界检查目标: $required" >&2
		exit 1
	}
done

echo "agent source boundary checks: PASS"
