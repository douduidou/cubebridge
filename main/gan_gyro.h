// cubebridge - ESP32 BLE gateway that bridges physical smart cubes
// into virtual GAN cubes for GAN-protocol apps.
// Copyright (C) 2026 The cubebridge authors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 3 of the License.

#pragma once
// gan_gyro: 源魔方姿态 -> GAN Gen2 姿态的固定映射 (K)
//   源侧 0xCC 陀螺帧给出 4 个 int16 分量 (源字节序, 定标 ~997.4)。
//   GAN 侧需要单位四元数 (w,x,y,z)。两者之间是固定的坐标系变换,
//   由"同姿态对照"实测解出 (四个姿态: 白上/绿前, 红上/绿前, 白上/红前, 绿上/白前)。
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 从 NVS 读取当前映射 (无记录时默认 k1)
void gan_gyro_load(void);

const char *gan_gyro_id(void);
// 选择映射并持久化; 返回 false 表示 id 无效
bool gan_gyro_select(const char *id);

// 目视校准: 镜像开关 (0=无, 1..4 表示翻转源四元数对应分量) 与三轴微调 (单位 0.1°)
bool gan_gyro_mirror_set(int axis);      // 0..4
int gan_gyro_mirror_get(void);
bool gan_gyro_trim_add(int axis, int deci_deg);   // axis: 0=X,1=Y,2=Z
void gan_gyro_trim_reset(void);
void gan_gyro_trim_get(int out[3]);

// raw[4]: 源四元数 (源字节序原始 int16); out[4]: GAN 单位四元数 (w,x,y,z)
void gan_gyro_map(const int16_t raw[4], float out[4]);

#ifdef __cplusplus
}
#endif
