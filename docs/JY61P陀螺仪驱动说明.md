# JY61P 陀螺仪驱动说明

本文档记录当前工程中 JY61P 三维姿态测量传感器的接线、读取方式、软件接口和任务日志含义。

## 对应文件

- `BSP/inc/bsp_jy62.h`：为兼容上层任务代码，文件名和 `JY62_*` API 暂时保留，但底层已经改为 JY61P 软件 I2C。
- `main.syscfg`：`JY61P_IIC` 引脚配置。
- `main.c`：启动置零、任务开始置零、直线航向修正、弧线航向进度判断和 JY61P 日志打印。
- `app_config.h`：航向导航开关、置零延时、打印周期和各任务航向修正参数。
- `docs/硬件接线说明.md`：JY61P 实物接线表。

## 当前接线

| JY61P 信号 | MSPM0G3507 | SysConfig 名称 | 说明 |
|---|---|---|---|
| `SCL` | `PA10 / A10` | `JY61P_IIC_SCL_PIN` | I2C 时钟 |
| `SDA` | `PA11 / A11` | `JY61P_IIC_SDA_PIN` | I2C 数据 |
| `VCC` | 按模块规格接 `3V3` 或 `5V` | - | 以实物供电要求为准 |
| `GND` | `GND` | - | 必须与主控、电机驱动、灰度模块共地 |

官方例程默认 `SCL=PA1`、`SDA=PA0`，但本工程 `PA0/PA1` 已经用于 TB6612 PWM，所以改为 `PA10/PA11`。
当前 SysConfig 中 `PA10/PA11` 使用普通 GPIO 模拟 I2C；驱动在等待 ACK 和读取数据时会把 `SDA` 切到输入状态，避免与 JY61P 同时驱动数据线。

## 读取协议

| 项目 | 值 |
|---|---|
| 设备地址 | `0x50` |
| 角度寄存器起始地址 | `0x3D` |
| 读取长度 | 6 字节 |
| 数据顺序 | RollL, RollH, PitchL, PitchH, YawL, YawH |
| 换算 | `raw / 32768 * 180deg` |

初始化时会按官方例程执行寄存器解锁、Z 轴归零、角度归零和保存，然后软件层再把当前 yaw 作为相对零点。

## 软件接口

为了减少上层改动，当前仍使用原来的接口名：

| 函数 | 用途 |
|---|---|
| `JY62_Init()` | 初始化 JY61P 软件 I2C、按官方例程归零并读取一次角度 |
| `JY62_SetYawZeroToCurrent()` | 把当前 yaw 记为软件零点 |
| `JY62_GetNavigation(&nav)` | 读取一次 JY61P 并生成导航数据 |
| `JY62_PeekNavigation(&nav)` | 兼容旧接口，当前也会读取一次 JY61P |

常用字段：

| 字段 | 含义 |
|---|---|
| `valid` | 本次 I2C 读取成功且零点有效 |
| `yaw_cdeg` | 原始 yaw，单位 0.01 度 |
| `yaw_relative_cdeg` | 相对最近一次软件置零的 yaw，单位 0.01 度 |
| `roll_cdeg / pitch_cdeg` | 姿态角，单位 0.01 度 |
| `gyro_z_filtered_mdps` | 由相邻 yaw 差分估算的转向速度，仅作兼容旧控制字段 |
| `frame_count` | 成功读取次数 |
| `checksum_error` | 这里复用为 I2C 错误计数 |

## 日志格式

典型日志：

```text
JY61P mode=TASK1_AB t=1000 df=1 ok=1 flags=0x04 yaw_cdeg=25 rel_cdeg=2 gz_mdps=0 gyro_z_filtered_mdps=0 rx=84 frames=14 i2c_err=0
```

判断方法：

- `ok=1`：JY61P 读取正常。
- `frames` 持续增长：控制循环正在持续读取角度。
- `i2c_err=0`：I2C 通信质量正常。
- 车头转动时，`yaw_cdeg` 和 `rel_cdeg` 应连续变化。
- 如果 `ok=0` 或 `i2c_err` 增长，优先检查供电、共地、`SCL/SDA` 是否接反。

调车时建议保存包含 `JY61P mode=...`、直线段 `yaw/h_corr`、弧线段 `yaw/ystep/yprog` 的日志片段，方便判断是航向零点、灰度入线，还是编码器距离导致偏差。
