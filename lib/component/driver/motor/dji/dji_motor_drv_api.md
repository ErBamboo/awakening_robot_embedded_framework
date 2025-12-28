# DJI 电机驱动库使用手册

**作者：Gemini 3.0 Pro**

**审查修改：余浩**

**适用硬件**：RoboMaster C610, C620, GM6020等

**核心特性**：O(1) 调度、静态内存池、自动分组、严格冲突检测、零动态内存分配

------

## 1. 使用注意事项

在使用本库之前，请务必阅读以下关键点，以避免常见的配置错误或运行时异常。

### 1.1 资源配置 (`dji_motor_conf.h`)

- **内存池大小**：`DJI_MOTOR_MAX_TX_UNITS` 宏决定了系统能同时管理的**独立 CAN 控制帧**数量。
  - 计算公式参考：`(C6x0数量 / 4) + (GM6020数量 * N)`。建议预留余量（默认 16 通常足够双路 CAN 使用）。
  - 若注册时返回 `AWLF_ERROR_MEMORY`，请增大此值。
- **RX 映射表范围**：通过 `DJI_MOTOR_RX_ID_MIN` 和 `MAX` 裁剪接收 ID 范围，可节省 RAM。确保所有使用的电机 ID 都在此范围内。

### 1.2 硬件过滤器

- **Bank 独占性**：在调用 `dji_motor_bus_init` 时传入的 `filterBankID` 必须是该 CAN 设备未被其他模块占用的过滤器组编号。
- **多模块共存**：驱动内部利用 `msg.bank` 区分数据来源。即使多个模块（如 DJI 电机和 自定义协议）共用同一个 CAN 驱动，只要 Bank ID 不同，数据流就不会串扰。

### 1.3 GM6020 的特殊性与冲突

- **电压模式风险**：若使用  **GM6020 的电压模式** (`DJI_CTRL_MODE_VOLTAGE`)，请注意其 ID 1-4 会占用 `0x1FF` 控制帧。这意味着同一条总线上 **不能同时存在** ID 为 5-8 的 C610/C620 电机和电压模式下ID为 1- 4 的GM6020电机，否则注册时会报 `AWLF_ERR_CONFLICT`。

### 1.4 调用时序

- **Set Output**：`dji_motor_set_output` 仅操作内存，不涉及IO，极快且线程安全，可在任意时刻调用。
- **Sync**：`dji_motor_bus_sync` 负责实际 CAN 发送。**建议在控制任务的末尾统一调用一次**，以利用 CAN 的组包特性（一帧报文控制 4 个电机），最大化总线带宽利用率。

------

## 2. API 使用说明

所有 API 均定义在 `dji_motor_drv.h` 中。

### 2.1 初始化与注册

#### `dji_motor_bus_init`

初始化总线管理器并配置底层 CAN 过滤器。

```c
AwlfRet_e dji_motor_bus_init(DJIMotorBus_s* bus, Device_t canDev, uint8_t filterBankID);
```

- **bus**: 总线管理器结构体指针。
- **canDev**: 已打开的底层 CAN 设备句柄。
- **filterBankID**: 分配给该模块使用的硬件过滤器组编号。
- **返回**: `AWLF_OK` 成功，其他为错误码。

#### `dji_motor_register`

注册电机，自动建立路由与冲突检测。

```c
AwlfRet_e dji_motor_register(DJIMotorBus_s* bus, DJIMotorDrv_s* motor, 
                             DJIMotorType_e type, uint8_t id, DJIMotorCtrlMode_e mode);
```

- **motor**: 电机对象句柄。
- **type**: `DJI_MOTOR_TYPE_C610` / `DJI_MOTOR_TYPE_C620` / `DJI_MOTOR_TYPE_GM6020`等。
- **id**: 电调物理 ID (1-8)。
- **mode**: `DJI_CTRL_MODE_CURRENT` (电流) 或 `DJI_CTRL_MODE_VOLTAGE` (电压, 仅 GM6020)。
- **返回**:
  - `AWLF_OK`: 成功。
  - `AWLF_ERR_CONFLICT`: 发生 ID 冲突（RX 或 TX）。
  - `AWLF_ERROR_MEMORY`: 静态内存池已满。

------

### 2.2 核心控制

#### `dji_motor_set_output`

设置目标输出值（写入缓存，不发送）。·

```c
void dji_motor_set_output(DJIMotorDrv_s* motor, int16_t output);
```

- **output**: 控制量。
  - C610/C620 (电流): Range `[-16384, 16384]` (对应 -10A~10A / -20A~20A)。
  - GM6020 (电压): Range `[-25000, 25000]`。
  - GM6020 (电流): Range `[-16384, 16384]`。

#### `dji_motor_bus_sync`

同步发送总线数据。将所有有更新的缓存组包发送。

```c
void dji_motor_bus_sync(DJIMotorBus_s* bus);
```

------

### 2.3 数据获取

所有 Getter 函数内部已根据电机型号自动处理了物理量转换。

| **函数名**                  | **返回单位** | **说明**                                          |
| --------------------------- | ------------ | ------------------------------------------------- |
| `dji_motor_get_total_angle` | 度 (°)       | 多圈累计角度，软件自动处理过零。                  |
| `dji_motor_get_velocity`    | RPM          | 转速。                                            |
| `dji_motor_get_current`     | 安培 (A)     | **实际转矩电流**。已根据电机 的不同量程自动换算。 |
| `dji_motor_get_temp`        | 摄氏度 (℃)   | 电机温度。                                        |
| `dji_motor_get_error_code`  | Hex          | 原始错误码 (参见宏定义 `DJI_ERR_xxx`)。           |

------

### 2.4 错误处理

#### `dji_motor_config_error_callback`

注册错误状态回调函数。

```c
void dji_motor_config_error_callback(DJIMotorDrv_s* motor, DJIMotorErrorCb_t callback);
```

- **机制**：**边沿触发**。仅在错误码发生变化时调用，避免中断风暴。

#### `dji_motor_clear_error`

手动清除软件层面的错误标记。

```c
void dji_motor_clear_error(DJIMotorDrv_s* motor);
```

------

## 3. 混合使用示例

场景描述：

在 CAN1 总线上同时控制以下 3 个电机，展示不同类型电机的共存与控制。

1. **底盘电机 (C620)**: ID 1，电流控制。
2. **发射机构 (C610)**: ID 5，电流控制。
3. **云台电机 (GM6020)**: ID 1，电流控制 。

```c
#include "dji_motor_drv.h"

/* 1. 定义对象 */
static DJIMotorBus_s  can1_bus;
static DJIMotorDrv_s     chassis_motor; // C620 ID 1
static DJIMotorDrv_s     shooter_motor; // C610 ID 5
static DJIMotorDrv_s     gimbal_motor;  // GM6020 ID 1

/* 2. 硬件与驱动初始化 */
void motor_system_init(Device_t can_dev_handle)
{
    /* 初始化总线管理器，使用 Filter Bank 0 */
    dji_motor_bus_init(&can1_bus, can_dev_handle, 0);

    /* 注册 C620 ID:1 (监听 0x200 Slot 0) */
    dji_motor_register(&can1_bus, &chassis_motor, 
                       DJI_MOTOR_TYPE_C620, 1, DJI_CTRL_MODE_CURRENT);

    /* 注册 C610 ID:5 (监听 0x1FF Slot 0) */
    dji_motor_register(&can1_bus, &shooter_motor, 
                       DJI_MOTOR_TYPE_C610, 5, DJI_CTRL_MODE_CURRENT);

    /* 注册 GM6020 ID:1 (监听 0x1FE Slot 0) 
     * 注意：如果这里用电压模式，会尝试监听 0x1FF Slot 0，与 shooter_motor 冲突报错！
     */
    dji_motor_register(&can1_bus, &gimbal_motor, 
                       DJI_MOTOR_TYPE_GM6020, 1, DJI_CTRL_MODE_CURRENT);
}

/* 3. 控制任务 (1kHz / 1ms 周期) */
void control_task_1ms(void)
{
    
    /* --- 第一步：获取反馈 & 计算 PID --- */
    
    // C620 控制
    float speed_chassis = dji_motor_get_velocity(&chassis_motor);
    float out_chassis = pid_calc_chassis(target_speed, speed_chassis);
    dji_motor_set_output(&chassis_motor, (int16_t)out_chassis);

    // C610 控制
    float speed_shooter = dji_motor_get_velocity(&shooter_motor);
    float out_shooter = pid_calc_shooter(target_rpm, speed_shooter);
    dji_motor_set_output(&shooter_motor, (int16_t)out_shooter);

    // GM6020 控制 (读取多圈角度)
    float angle_gm = dji_motor_get_total_angle(&gimbal_motor);
    float out_gm = pid_calc_gimbal(target_angle, angle_gm);
    dji_motor_set_output(&gimbal_motor, (int16_t)out_gm);

    /* --- 第二步：同步发送 --- */
    
    /* * 此时，驱动底层会自动组包：
     * 1. 0x200 帧: 包含 chassis_motor 数据
     * 2. 0x1FF 帧: 包含 shooter_motor 数据
     * 3. 0x1FE 帧: 包含 gimbal_motor 数据
     * 调用 sync 一次性将这 3 帧推送到 CAN 硬件 FIFO
     */
    dji_motor_bus_sync(&can1_bus);
}
```