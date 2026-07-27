#!/bin/sh
set -eu

if [ "${1:-}" != "safety" ]; then
	echo "usage: $0 safety" >&2
	exit 2
fi

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
tmp_dir=$(mktemp -d "${TMPDIR:-/tmp}/cf-drone-safety.XXXXXX")
trap 'rm -rf "$tmp_dir"' EXIT HUP INT TERM

cd "$repo_dir"
"${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic -I. \
	agent_safety.cpp tests/test_agent_safety.cpp \
	-o "$tmp_dir/test_agent_safety"
"$tmp_dir/test_agent_safety"

if grep -En \
	'(motors|attitudeTarget|ratesTarget|torqueTarget|setParameter|doCommand|controlRoll|controlPitch|controlYaw|controlThrottle)' \
	agent_mavlink.ino >/dev/null; then
	echo "Agent MAVLink入口包含禁止的直控符号" >&2
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
