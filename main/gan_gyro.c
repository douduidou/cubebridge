// cubebridge - ESP32 BLE gateway that bridges physical smart cubes
// into virtual GAN cubes for GAN-protocol apps.
// Copyright (C) 2026 The cubebridge authors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 3 of the License.

#include "gan_gyro.h"

#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "gangyro";

// 候选映射: out = k ⊗ (源分量按 perm 重排, 可选共轭)
//   k1/k2 由四个同姿态对照点求解, 离线残差都在手工摆放误差 (3~5°) 内;
//   最终以魔方星球实机显示为准, 可串口切换。
typedef struct {
    const char *id;
    float k[4];       // 左乘四元数 (w,x,y,z)
    uint8_t perm[4];  // 源分量重排顺序
    bool conjugate;   // 是否取共轭 (源系手性相反时)
    float r[4];       // 右乘四元数 (本体坐标系修正; 全 0 表示单位)
} gyro_gain_t;

static const gyro_gain_t GAINS[] = {
    // g1: 实测标定的自洽解 (固定姿态 A 精确对齐, B/C/D 残差 4~7°)
    //     分量顺序 (1,2,0,3) 取共轭; 默认使用本项
    {
        .id = "g1",
        .k = {+0.661423f, +0.704649f, +0.180619f, +0.182663f},
        .perm = {1, 2, 0, 3},
        .conjugate = true,
    },
    {
        .id = "k1",
        .k = {+0.172802f, +0.647511f, +0.154370f, -0.725974f},
        .perm = {0, 3, 2, 1},
        .conjugate = false,
    },
    {
        .id = "k2",
        .k = {-0.167729f, +0.157328f, -0.689400f, +0.686907f},
        .perm = {2, 0, 3, 1},
        .conjugate = true,
    },
    // 实测 App 显示比真实姿态整体差一个 120° 旋转 E=(0.5,-0.5,0.5,-0.5)
    // (白上/绿前 被显示成 橙上/黄前), 以下为各方向的常量修正。
    {
        .id = "k3",   // 世界系修正 (期望正解)
        .k = {+0.202818f, +0.695959f, +0.677527f, +0.124355f},
        .perm = {0, 3, 2, 1},
        .conjugate = false,
    },
    {
        .id = "k4",   // 世界系反方向
        .k = {-0.030016f, -0.048448f, -0.523157f, -0.850329f},
        .perm = {0, 3, 2, 1},
        .conjugate = false,
    },
    {
        .id = "k5",   // 本体坐标系修正
        .k = {+0.202818f, +0.124355f, -0.695959f, -0.677527f},
        .perm = {0, 3, 2, 1},
        .conjugate = false,
    },
    {
        .id = "k6",   // 本体坐标系反方向
        .k = {-0.030016f, +0.523157f, +0.850329f, -0.048448f},
        .perm = {0, 3, 2, 1},
        .conjugate = false,
    },
    {
        .id = "k7",
        .k = {-0.850683f, -0.003954f, -0.525625f, -0.006447f},
        .perm = {2, 0, 3, 1},
        .conjugate = true,
    },
    {
        .id = "k8",
        .k = {+0.682953f, +0.161282f, -0.163775f, +0.693354f},
        .perm = {2, 0, 3, 1},
        .conjugate = true,
    },
    // 绕立方体 4 条对角线的 120°/240° 旋转 (8 种), 覆盖全部"对称歧义"可能
    { .id = "c1", .k = {+0.048448f, -0.030016f, +0.850329f, -0.523157f}, .perm = {0, 3, 2, 1}, .conjugate = false },
    { .id = "c2", .k = {-0.677527f, +0.124355f, +0.202818f, -0.695959f}, .perm = {0, 3, 2, 1}, .conjugate = false },
    { .id = "c3", .k = {+0.202818f, +0.695959f, +0.677527f, +0.124355f}, .perm = {0, 3, 2, 1}, .conjugate = false },
    { .id = "c4", .k = {-0.523157f, +0.850329f, +0.030016f, -0.048448f}, .perm = {0, 3, 2, 1}, .conjugate = false },
    { .id = "c5", .k = {+0.695959f, -0.202818f, +0.124355f, -0.677527f}, .perm = {0, 3, 2, 1}, .conjugate = false },
    { .id = "c6", .k = {-0.030016f, -0.048448f, -0.523157f, -0.850329f}, .perm = {0, 3, 2, 1}, .conjugate = false },
    { .id = "c7", .k = {+0.850329f, +0.523157f, -0.048448f, -0.030016f}, .perm = {0, 3, 2, 1}, .conjugate = false },
    { .id = "c8", .k = {+0.124355f, +0.677527f, -0.695959f, -0.202818f}, .perm = {0, 3, 2, 1}, .conjugate = false },
    // 本体坐标系 (右乘) 修正: q_out = K1 ⊗ (q_src ⊗ r)
    // 实测 App 的朝向误差是"本体绕对角线的 120°", 只能用右乘消掉 (整组左乘已实测排除)。
    { .id = "d1", .k = {+0.172802f, +0.647511f, +0.154370f, -0.725974f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {+0.5f, -0.5f, +0.5f, -0.5f} },
    { .id = "d2", .k = {+0.172802f, +0.647511f, +0.154370f, -0.725974f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {+0.5f, +0.5f, +0.5f, +0.5f} },
    { .id = "d3", .k = {+0.172802f, +0.647511f, +0.154370f, -0.725974f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {+0.5f, +0.5f, -0.5f, -0.5f} },
    { .id = "d4", .k = {+0.172802f, +0.647511f, +0.154370f, -0.725974f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {+0.5f, -0.5f, -0.5f, +0.5f} },
    { .id = "d5", .k = {+0.172802f, +0.647511f, +0.154370f, -0.725974f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {-0.5f, +0.5f, +0.5f, -0.5f} },
    { .id = "d6", .k = {+0.172802f, +0.647511f, +0.154370f, -0.725974f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {-0.5f, +0.5f, -0.5f, +0.5f} },
    { .id = "d7", .k = {+0.172802f, +0.647511f, +0.154370f, -0.725974f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {-0.5f, -0.5f, +0.5f, +0.5f} },
    { .id = "d8", .k = {+0.172802f, +0.647511f, +0.154370f, -0.725974f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {-0.5f, -0.5f, -0.5f, -0.5f} },
    // d1 基础上再补一个绕红轴的 90° (d1 实测残余正好是绕红轴 90°)
    { .id = "e1", .k = {+0.172802f, +0.647511f, +0.154370f, -0.725974f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {+0.707107f, +0.000000f, +0.000000f, -0.707107f} },
    { .id = "e2", .k = {+0.172802f, +0.647511f, +0.154370f, -0.725974f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {+0.000000f, -0.707107f, +0.707107f, +0.000000f} },
    { .id = "e3", .k = {-0.335670f, +0.580049f, +0.622497f, -0.404185f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {+0.5f, -0.5f, +0.5f, -0.5f} },
    { .id = "e4", .k = {+0.580049f, +0.335670f, -0.404185f, -0.622497f}, .perm = {0, 3, 2, 1}, .conjugate = false, .r = {+0.5f, -0.5f, +0.5f, -0.5f} },
};

static const gyro_gain_t *s_active = &GAINS[0];
static int s_mirror;              // 0=无, 1..4 = 翻转源四元数第 N 个分量 (镜像/奇偶修正)
static int16_t s_trim_d10[3];     // 三轴微调, 单位 0.1°

static void save_cal(void) {
    nvs_handle_t h;
    if (nvs_open("gangyro", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, "mirror", s_mirror);
    nvs_set_i16(h, "tx", s_trim_d10[0]);
    nvs_set_i16(h, "ty", s_trim_d10[1]);
    nvs_set_i16(h, "tz", s_trim_d10[2]);
    nvs_commit(h);
    nvs_close(h);
}

bool gan_gyro_mirror_set(int axis) {
    if (axis < 0 || axis > 4) return false;
    s_mirror = axis;
    save_cal();
    ESP_LOGI(TAG, "镜像=%d", s_mirror);
    return true;
}

int gan_gyro_mirror_get(void) { return s_mirror; }

bool gan_gyro_trim_add(int axis, int deci_deg) {
    if (axis < 0 || axis > 2) return false;
    int v = (int)s_trim_d10[axis] + deci_deg;
    if (v > 1800) v = 1800;
    if (v < -1800) v = -1800;
    s_trim_d10[axis] = (int16_t)v;
    save_cal();
    ESP_LOGI(TAG, "微调 X=%.1f° Y=%.1f° Z=%.1f°", s_trim_d10[0] / 10.0,
             s_trim_d10[1] / 10.0, s_trim_d10[2] / 10.0);
    return true;
}

void gan_gyro_trim_reset(void) {
    s_trim_d10[0] = s_trim_d10[1] = s_trim_d10[2] = 0;
    save_cal();
    ESP_LOGI(TAG, "微调已清零");
}

void gan_gyro_trim_get(int out[3]) {
    out[0] = s_trim_d10[0];
    out[1] = s_trim_d10[1];
    out[2] = s_trim_d10[2];
}

static const gyro_gain_t *find_gain(const char *id) {
    if (!id) return NULL;
    for (size_t i = 0; i < sizeof(GAINS) / sizeof(GAINS[0]); i++) {
        if (strcasecmp(GAINS[i].id, id) == 0) return &GAINS[i];
    }
    return NULL;
}

void gan_gyro_load(void) {
    nvs_handle_t h;
    char id[8] = {0};
    size_t len = sizeof(id);
    int32_t mirror = 0;
    if (nvs_open("gangyro", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_str(h, "active", id, &len) == ESP_OK) {
            const gyro_gain_t *g = find_gain(id);
            if (g) s_active = g;
        }
        if (nvs_get_i32(h, "mirror", &mirror) == ESP_OK && mirror >= 0 && mirror <= 4)
            s_mirror = (int)mirror;
        nvs_get_i16(h, "tx", &s_trim_d10[0]);
        nvs_get_i16(h, "ty", &s_trim_d10[1]);
        nvs_get_i16(h, "tz", &s_trim_d10[2]);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "映射=%s 镜像=%d 微调=%.1f/%.1f/%.1f°", s_active->id, s_mirror,
             s_trim_d10[0] / 10.0, s_trim_d10[1] / 10.0, s_trim_d10[2] / 10.0);
}

const char *gan_gyro_id(void) { return s_active->id; }

bool gan_gyro_select(const char *id) {
    const gyro_gain_t *g = find_gain(id);
    if (!g) return false;
    s_active = g;
    nvs_handle_t h;
    if (nvs_open("gangyro", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, "active", g->id);
        nvs_commit(h);
        nvs_close(h);
    }
    ESP_LOGI(TAG, "切换映射 -> %s", g->id);
    return true;
}

void gan_gyro_map(const int16_t raw[4], float out[4]) {
    const gyro_gain_t *g = s_active;
    float q[4];
    int16_t r2[4];
    for (int i = 0; i < 4; i++) r2[i] = raw[i];
    if (s_mirror >= 1 && s_mirror <= 4) r2[s_mirror - 1] = (int16_t)(-r2[s_mirror - 1]);
    for (int i = 0; i < 4; i++) q[i] = (float)r2[g->perm[i]];
    if (g->conjugate) {
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-3f) {
        out[0] = 1.0f; out[1] = 0.0f; out[2] = 0.0f; out[3] = 0.0f;
        return;
    }
    for (int i = 0; i < 4; i++) q[i] /= n;

    // 本体坐标系修正 (右乘): q <- q ⊗ r
    if (g->r[0] != 0.0f || g->r[1] != 0.0f || g->r[2] != 0.0f || g->r[3] != 0.0f) {
        const float r0 = g->r[0], r1 = g->r[1], r2 = g->r[2], r3 = g->r[3];
        const float t0 = q[0] * r0 - q[1] * r1 - q[2] * r2 - q[3] * r3;
        const float t1 = q[0] * r1 + q[1] * r0 + q[2] * r3 - q[3] * r2;
        const float t2 = q[0] * r2 - q[1] * r3 + q[2] * r0 + q[3] * r1;
        const float t3 = q[0] * r3 + q[1] * r2 - q[2] * r1 + q[3] * r0;
        q[0] = t0; q[1] = t1; q[2] = t2; q[3] = t3;
    }

    const float *a = g->k;
    out[0] = a[0] * q[0] - a[1] * q[1] - a[2] * q[2] - a[3] * q[3];
    out[1] = a[0] * q[1] + a[1] * q[0] + a[2] * q[3] - a[3] * q[2];
    out[2] = a[0] * q[2] - a[1] * q[3] + a[2] * q[0] + a[3] * q[1];
    out[3] = a[0] * q[3] + a[1] * q[2] - a[2] * q[1] + a[3] * q[0];

    float m = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
    if (m > 1e-6f) {
        for (int i = 0; i < 4; i++) out[i] /= m;
    }
    // 不做符号归一: q 与 -q 等价, 但强制翻转会在连续流中引入跳变,
    // 源四元数本身连续, 左乘常量后依然连续。

    // 三轴微调 (左乘): trim = Rz ⊗ Ry ⊗ Rx
    if (s_trim_d10[0] || s_trim_d10[1] || s_trim_d10[2]) {
        const float k = 0.05f * (float)M_PI / 180.0f;   // 0.1° -> 半角弧度
        const float ax[4] = {cosf(s_trim_d10[0] * k), sinf(s_trim_d10[0] * k), 0.0f, 0.0f};
        const float ay[4] = {cosf(s_trim_d10[1] * k), 0.0f, sinf(s_trim_d10[1] * k), 0.0f};
        const float az[4] = {cosf(s_trim_d10[2] * k), 0.0f, 0.0f, sinf(s_trim_d10[2] * k)};
        float t1[4], trim[4], o[4];
        t1[0] = ay[0] * ax[0] - ay[1] * ax[1] - ay[2] * ax[2] - ay[3] * ax[3];
        t1[1] = ay[0] * ax[1] + ay[1] * ax[0] + ay[2] * ax[3] - ay[3] * ax[2];
        t1[2] = ay[0] * ax[2] - ay[1] * ax[3] + ay[2] * ax[0] + ay[3] * ax[1];
        t1[3] = ay[0] * ax[3] + ay[1] * ax[2] - ay[2] * ax[1] + ay[3] * ax[0];
        trim[0] = az[0] * t1[0] - az[1] * t1[1] - az[2] * t1[2] - az[3] * t1[3];
        trim[1] = az[0] * t1[1] + az[1] * t1[0] + az[2] * t1[3] - az[3] * t1[2];
        trim[2] = az[0] * t1[2] - az[1] * t1[3] + az[2] * t1[0] + az[3] * t1[1];
        trim[3] = az[0] * t1[3] + az[1] * t1[2] - az[2] * t1[1] + az[3] * t1[0];
        o[0] = trim[0] * out[0] - trim[1] * out[1] - trim[2] * out[2] - trim[3] * out[3];
        o[1] = trim[0] * out[1] + trim[1] * out[0] + trim[2] * out[3] - trim[3] * out[2];
        o[2] = trim[0] * out[2] - trim[1] * out[3] + trim[2] * out[0] + trim[3] * out[1];
        o[3] = trim[0] * out[3] + trim[1] * out[2] - trim[2] * out[1] + trim[3] * out[0];
        for (int i = 0; i < 4; i++) out[i] = o[i];
    }
}
