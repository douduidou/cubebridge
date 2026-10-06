// cubebridge - ESP32 BLE gateway that bridges physical smart cubes
// into virtual GAN cubes for GAN-protocol apps.
// Copyright (C) 2026 The cubebridge authors
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, version 3 of the License.

#include "gan_profile.h"

#include <string.h>
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "ganprof";

// i3 档案的广播名取自真机抓包 (X-Man i3 广播名实测为 "GANi30rb")。
// 8 字节型号码目前按同名推测, 待魔方星球实机验证后可经串口/源码修正。
static const gan_profile_t PROFILES[] = {
    {
        .id = "icarry",
        .name = "GAN356iCarry",
        .hardware_name = "GAN356iC",
        .hw_major = 1, .hw_minor = 1,
        .fw_major = 1, .fw_minor = 0,
        .gyro_supported = false,
    },
    {
        .id = "i3",
        .name = "GANi30rb",
        .hardware_name = "GANi30rb",
        .hw_major = 1, .hw_minor = 0,
        .fw_major = 1, .fw_minor = 0,
        .gyro_supported = true,
    },
};

static const gan_profile_t *s_active = &PROFILES[0];

static const gan_profile_t *find_profile(const char *id) {
    if (!id) return NULL;
    for (size_t i = 0; i < sizeof(PROFILES) / sizeof(PROFILES[0]); i++) {
        if (strcasecmp(PROFILES[i].id, id) == 0) return &PROFILES[i];
    }
    return NULL;
}

static void save_active(const char *id) {
    nvs_handle_t h;
    if (nvs_open("ganprof", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_str(h, "active", id);
    nvs_commit(h);
    nvs_close(h);
}

void gan_profile_load(void) {
    nvs_handle_t h;
    char id[16] = {0};
    size_t len = sizeof(id);
    if (nvs_open("ganprof", NVS_READONLY, &h) == ESP_OK) {
        if (nvs_get_str(h, "active", id, &len) == ESP_OK) {
            const gan_profile_t *p = find_profile(id);
            if (p) s_active = p;
            else ESP_LOGW(TAG, "未知档案 '%s', 使用默认 %s", id, s_active->id);
        }
        nvs_close(h);
    }
    ESP_LOGI(TAG, "档案=%s name=%s model=%s gyro=%d", s_active->id, s_active->name,
             s_active->hardware_name, (int)s_active->gyro_supported);
}

const gan_profile_t *gan_profile_active(void) { return s_active; }
const char *gan_profile_id(void) { return s_active->id; }

bool gan_profile_select(const char *id) {
    const gan_profile_t *p = find_profile(id);
    if (!p) return false;
    s_active = p;
    save_active(p->id);
    ESP_LOGI(TAG, "切换档案 -> %s name=%s model=%s gyro=%d", p->id, p->name,
             p->hardware_name, (int)p->gyro_supported);
    return true;
}
