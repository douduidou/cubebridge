// cubebridge - ESP32 BLE gateway that bridges physical smart cubes
// into virtual GAN cubes for GAN-protocol apps.
// Copyright (C) 2026 The cubebridge authors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 3 of the License.

#pragma once
// gan_profile: 虚拟 GAN 设备的运行时身份档案 (icarry / i3)
//   同一套 Gen2 协议与加密, 只切换对外可见的身份面:
//   广播名 / DIS 字符串 / HARDWARE 事件型号码与版本 / gyro 能力位。
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *id;            // 串口命令用的短名
    const char *name;          // 广播名 + DIS Model Number
    const char *hardware_name; // Gen2 HARDWARE 事件里的 8 字节型号码
    uint8_t hw_major, hw_minor;
    uint8_t fw_major, fw_minor;
    bool gyro_supported;       // HARDWARE 事件 bit104
} gan_profile_t;

// 从 NVS 读取当前档案 (无记录时默认 icarry)
void gan_profile_load(void);

const gan_profile_t *gan_profile_active(void);
const char *gan_profile_id(void);

// 选择档案并持久化; 返回 false 表示 id 无效
bool gan_profile_select(const char *id);

#ifdef __cplusplus
}
#endif
