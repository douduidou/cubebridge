# 第三方声明

本项目中由作者原创的代码以 GPL-3.0 发布（见 [LICENSE](LICENSE)）。下列第三方项目仅作为协议与算法的**参考来源**，未直接复制其代码文件；各项目归其原作者所有。

## 参考的开源项目

| 项目 | 用途 | 许可证 |
| --- | --- | --- |
| [bazingamc/cubebridge](https://github.com/bazingamc/cubebridge) | 本仓库的上游工程：ESP32/ESP32-S3 蓝牙双角色网关、GAN Gen2 虚拟魔方、网页烧录说明 | GPL-3.0 |
| [csTimer](https://github.com/cs0x7f/cstimer)（`mathlib.js`、`gancube.js`、`moyucube.js`、`moyu32cube.js`、`qiyicube.js`） | CubieCube 模型基准；GAN / 魔域 MHC / 魔域 MY3 / 奇艺协议解析布局；时间换算依据 | GPL-3.0 |
| [gan-web-bluetooth](https://github.com/afedotov/gan-web-bluetooth) | GAN 协议事实：RESET 固定报文等 | MIT |
| [smartcube-web-bluetooth](https://github.com/poliva/smartcube-web-bluetooth) | 多品牌智能魔方 Web Bluetooth 库（协议调研参考） | MIT |

本项目整体以 GPL-3.0 发布，与上表 GPL-3.0 的 csTimer 兼容；MIT 许可的引用项目可再授权并入 GPL-3.0 项目，均无冲突。

## 其他来源

- 魔域 MHC / WCU_MY3、奇艺 QY-QYSC 驱动逻辑与 GAN AES 密钥派生、加解密逻辑基于作者私有 Flutter 项目移植（作者保留其版权，此处按 GPL-3.0 授权发布）。
- 各品牌协议常量（AES 根密钥 / IV、服务 UUID、opcode、帧位布局）为公开的逆向工程事实数据。

## 平台组件

固件基于以下开源平台组件构建，其许可证各自适用：

- [ESP-IDF](https://github.com/espressif/esp-idf)：Apache-2.0（含 NimBLE 主机协议栈、mbedtls、FreeRTOS 等）

## 陀螺仪（姿态）转发的实测依据

以下事实来自对真机 GAN356 i3 的 BLE 抓包（本项目 `gansniff` 工具）与魔方星球实机对照，
用于把源魔方的姿态数据转换成 GAN Gen2 帧：

- 真机 Gen2 服务（`6e400001-…`）的 `mode 1` 帧里带**两个近乎相同的四元数**（bits 4..67 与 bits 80..143，
  实测相差约 1°），帧尾低字节恒为 `0x8A`；只填第一个四元数会让 App 的 3D 姿态停在首帧。
- 真机每帧的四元数为 16 位符号-幅值 Q15；角速度字段为 3 个 4 位符号-幅值（静止时为 0）。
- 请求硬件信息（`0x05`）的应答需要以 `mode 1` 承载硬件位段（`gan-web-bluetooth` 的解析方式）：
  bits 8..15 硬件主版本、16..23 硬件次版本、24..31 固件主版本、32..39 固件次版本、
  40..103 8 字节型号码；用 `mode 5` 返回时魔方星球不会启用陀螺仪功能。
- 真机另外暴露一条 128 位姿态服务（`f95a48e6-a721-11e9-a2a3-022ae2dbcce4`，含 read/write/notify 三特征），
  本项目已复刻该服务以保持与真机一致的 GATT 结构。
- 源魔方（XMD Tornado V4 AI，奇艺 QY-QYSC 路径）的 `0xCC` 陀螺帧为
  `CC 10 | 计数 2B | 四元数 4×int16 大端 | CRC16-MODBUS`，四元数定标约 997.4（|q| 恒定）；
  其分量顺序与 GAN 不同，需要用固定映射（左乘 + 分量重排/共轭）转换，映射参数以实机对照标定。
