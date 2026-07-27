// 主程序
// 源代码使用Arduino IDE 2.3.x以上版本编译，不支持老版本的1.8.19！
// 必须安装ESP32开发板核心库（即esp32 by Espressif Systems）和依赖（MAVLINK等）
// 开发板可选择ESP32 Dev Module或WeMOS D1 MINI ESP32

#include "vector.h"
#include "quaternion.h"
#include "util.h"
#include "agent_safety.h"

#define WIFI_ENABLED 1
#define WEB_RC_ENABLED 1  // 启用Web遥控器

#if WIFI_ENABLED
#include <MAVLink.h>
#endif

float t = NAN; // current step time, s
float dt; // time delta from previous step, s
float controlRoll, controlPitch, controlYaw, controlThrottle; // pilot's inputs, range [-1, 1]
float controlMode = NAN;
Vector gyro; // gyroscope data
Vector acc; // accelerometer data, m/s/s
Vector rates; // filtered angular rates, rad/s
Quaternion attitude; // estimated attitude
bool landed; // are we landed and stationary

void setup() {
	Serial.begin(115200);
#ifdef CONFIG_IDF_TARGET_ESP32C3
	// ESP32-C3 使用内置 USB-CDC：Serial.begin() 立即返回，但 USB 枚举需约 1-2 秒。
	// 若不等待，setup() 前半段的所有打印在主机打开串口前已发出，用户看不到。
	// 最多等 3 秒：有串口监视器时立即继续；烧录后脱机运行时 3 秒后也会继续。
	{ unsigned long _t0 = millis(); while (!Serial && millis() - _t0 < 3000) delay(10); }
	delay(100); // 额外缓冲，确保主机接收缓冲区就绪
#endif
	print("程序初始化！\n");
	disableBrownOut();
	setupParameters();
	setupLED();
	setupMotors();
	setLED(true);
#if WIFI_ENABLED
	setupWiFi();
#endif
#if WEB_RC_ENABLED
	setupWebRC();  // 初始化Web遥控器
#endif
	setupIMU();
	setupRC();
	setLED(false);
	print("初始化完成！\n");
// Web RC 启动地址已由 setupWebRC() 内部打印（端口80），此处不再重复
}

void loop() {
	readIMU();
	step();
	readRC();
#if WEB_RC_ENABLED
	readWebRC();  // 读取Web遥控器输入
	processConsoleCommandQueue(); // 将网页命令放到主循环执行，避免阻塞HTTP回调
#endif
	estimate();
	updateBatteryVoltage();
	control();
	sendMotors();
	handleInput();
#if WIFI_ENABLED
	processMavlink();
#endif
	logData();
	syncParameters();
	updateLED();
}
