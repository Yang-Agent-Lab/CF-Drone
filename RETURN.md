# E4T2-FIRMWARE-COMMAND-V1

状态：已完成，未提交、未推送、未合并。

## 完成内容

- 新增纯 C++ `flight_command_v1` 解码器：只将 v1 高层技能的
  `param4` 到 `param7` 转为 `flight_skills::Request`，失败时输出清零。
- 六种高层技能严格检查版本、保留位、有限数、零值及范围；范围上限复用
  `flight_skills.h` 常量。悬停时长接受任意正的有限秒数，向上取整为毫秒且不超过
  5000 毫秒（例如 0.0001 秒为 1 毫秒）。
- MAVLink 入口只对六种高层技能调用解码器；心跳、解锁、上锁和急停仍要求四个
  参数全为零。安全门仍返回“尚未实现”，未调用 `flight_skills::Machine`，未生成
  控制目标，也未改动 PID、混控或电机输出。
- 新增 `command` 检查，覆盖六组协议向量、悬停时长向上取整、拒绝路径、失败清零，
  以及入口不启动飞行技能状态机的源码边界。

## 验证结果

本地检查均通过：

```text
./scripts/check_firmware.sh safety
./scripts/check_firmware.sh sensors
./scripts/check_firmware.sh control
./scripts/check_firmware.sh command
git diff --check
```

最终修正后，总指挥在 OrbStack `uav-dev`（Ubuntu ARM64，Arduino CLI 1.5.1，
ESP32 core 3.3.10）中，以临时副本重新完成三目标板编译；未连接设备、访问
`/dev`、扫描网络、刷写或控制无人机：

| 目标板 | 程序空间 | 动态内存 | 结果 |
|---|---:|---:|---|
| ESP32 Dev Module | 84% | 34% | 通过 |
| ESP32-C3 Dev Module | 89% | 30% | 通过 |
| ESP32-S3 Dev Module | 83% | 33% | 通过 |

ESP32 通用目标的首次编译仅出现既有驱动、MAVLink 生成代码和网页遥控代码警告；
最终修正后的三目标板编译均成功。

## 验证飞行链路

- 新增软件测试：同一组 v1 命令按“解码后才交给飞行状态机”的顺序，依次覆盖起飞、
  悬停、机体系移动、偏航、返回局部原点和降落完成；同时覆盖非法移动速度在解码层
  被拒绝，状态机保持未启动。
- 新增 `./scripts/check_firmware.sh pipeline` 检查入口。本次未改动飞控生产代码、
  硬件入口或控制参数；未连接设备、访问 `/dev`、扫描网络或刷写。
