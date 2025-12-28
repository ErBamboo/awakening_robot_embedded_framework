/**
 * @file dji_motor_driver.h
 * @brief RoboMaster DJI 智能电机通用驱动
 * @author Yuhao
 * @date   2025/12/22
 */

#ifndef __DJI_MOTOR_DRIVER_H__
#define __DJI_MOTOR_DRIVER_H__

#include "device/core/device.h"
#include "dji_motor_conf.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* --- 自动推导配置参数 --- */
#define DJI_RX_ID_START       DJI_MOTOR_RX_ID_MIN
#define DJI_RX_ID_END         DJI_MOTOR_RX_ID_MAX
#define DJI_MOTOR_RX_MAP_SIZE (DJI_RX_ID_END - DJI_RX_ID_START + 1)

/* --- 协议常量与错误码 --- */
#define DJI_TX_ID_C6X0_1_4     0x200
#define DJI_TX_ID_MIX_1_FF     0x1FF
#define DJI_TX_ID_GM6020_V_5_7 0x2FF
#define DJI_TX_ID_GM6020_C_1_4 0x1FE
#define DJI_TX_ID_GM6020_C_5_7 0x2FE

/* 错误码定义 */
#define DJI_ERR_NONE      0x00
#define DJI_ERR_MEM       0x01
#define DJI_ERR_VOLT      0x02
#define DJI_ERR_PHASE     0x03
#define DJI_ERR_SENSOR    0x04
#define DJI_ERR_TEMP_HIGH 0x05
#define DJI_ERR_STALL     0x06
#define DJI_ERR_CALIB     0x07
#define DJI_ERR_OVERHEAT  0x08

/* --- 类型定义 --- */

typedef enum
{
    DJI_MOTOR_TYPE_C610 = 0,
    DJI_MOTOR_TYPE_C620,
    DJI_MOTOR_TYPE_GM6020,
    DJI_MOTOR_TYPE_UNKNOWN
} DJIMotorType_e;

typedef enum
{
    DJI_CTRL_MODE_CURRENT = 0,
    DJI_CTRL_MODE_VOLTAGE
} DJIMotorCtrlMode_e;

/* 前置声明 */
typedef struct DJIMotorTxUnit DJIMotorTxUnit_s;
typedef struct DJIMotorDrv       DJIMotorDrv_s;

/**
 * @brief 电机错误回调函数定义
 * @param motor 出错的电机指针
 * @param error_code 当前错误码 (0表示恢复正常)
 */
typedef void (*DJIMotorErrorCb_t)(DJIMotorDrv_s* motor, uint8_t error_code);

/** @brief 发送单元 (Linked List Node) */
struct DJIMotorTxUnit
{
    struct list_head list;
    uint32_t         canId;
    uint8_t          txBuffer[8];
    uint8_t          isDirty;
    uint8_t          usageMask;     // 简单位图思想，0x01 表示txbuf的第 0-1 个字节已被分配，0x02 表示第 2-3 个字节被分配，以此类推
};

/** @brief DJI 电机对象句柄 */
struct DJIMotorDrv
{
    /* 静态路由配置 (Init 时确定) */
    struct
    {
        uint16_t          rxId;         /* 接收 ID */
        uint8_t           txBufIdx;     /* 发送单元索引 */
        DJIMotorTxUnit_s* txUnit;       /* 静态路由对应的发送单元 */
        DJIMotorType_e    type;         /* 电机类型 */
    } link;

    /* 运行时反馈数据 (Private - 请使用 Access API 访问) */
    struct
    {
        int16_t angle;    /* 0-8191 */
        int16_t velocity; /* rpm */
        int16_t current;  /* raw current */
        uint8_t temp;     /* temp */

        /* 状态管理 */
        uint8_t errorCode;     /* 当前错误码 */
        uint8_t lastErrorCode; /* 上一次错误码(用于边沿检测) */

        /* 扩展数据 */
        int32_t  totalAngle; /* 多圈累计角度 */
        uint16_t lastAngle;  /* 上一次角度 */
        int32_t  roundCount; /* 圈数计数 */
    } measure;

    /* 控制设定 */
    int16_t            targetOutput;    /* 目标输出 */
    DJIMotorCtrlMode_e mode;            /* 运行模式 */
    float              scale;           /* 反馈电流编码值 和 真实的电流比例 */
    /* 回调函数 */
    DJIMotorErrorCb_t errorCallback;
};

/** @brief DJI 电机总线管理器 */
typedef struct
{
    Device_t         canDev;
    uint8_t          filterBank;
    DJIMotorDrv_s*      rxMap[DJI_MOTOR_RX_MAP_SIZE];
    struct list_head txList;
} DJIMotorBus_s;

/* --- Core API --- */

/**
 * @brief 初始化 DJI 电机总线管理器
 * 
 * @param bus 电机总线管理器指针
 * @param canDev CAN 设备句柄
 * @param filterBankID CAN 过滤器 ID
 * @return AwlfRet_e 初始化状态
 */
AwlfRet_e dji_motor_bus_init(DJIMotorBus_s* bus, Device_t canDev, uint8_t filterBankID);

/**
 * @brief 注册 DJI 电机到总线
 * 
 * @param bus 电机总线管理器指针
 * @param motor 电机对象指针
 * @param type 电机类型
 * @param id 电机 ID (0-255)
 * @param mode 控制模式
 * @return AwlfRet_e 注册状态
 */
AwlfRet_e dji_motor_register(DJIMotorBus_s* bus, DJIMotorDrv_s* motor, DJIMotorType_e type, uint8_t id, DJIMotorCtrlMode_e mode);

/**
 * @brief 设置电机输出值
 * 
 * @param motor 电机对象指针
 * @param output 目标输出值 (-32768 to 32767)
 */
void dji_motor_set_output(DJIMotorDrv_s* motor, int16_t output);

/**
 * @brief 同步发送所有待处理的电机指令
 * 
 * @param bus 电机总线管理器指针
 */
void dji_motor_bus_sync(DJIMotorBus_s* bus);

/* --- Error Handling API --- */

/**
 * @brief 注册错误状态回调函数
 * @note 当电机错误码发生变化时(报错或恢复)触发
 */
static inline void dji_motor_config_error_callback(DJIMotorDrv_s* motor, DJIMotorErrorCb_t callback)
{
    if (motor)
        motor->errorCallback = callback;
}

/**
 * @brief 清除软件层面的错误标记
 * @note 硬件错误码取决于电机反馈，若故障未排除，下一帧仍会置位
 */
static inline void dji_motor_clear_error(DJIMotorDrv_s* motor)
{
    if (motor)
    {
        // 清除软件层面的错误码记录
        motor->measure.errorCode = 0;
        motor->measure.lastErrorCode = 0;
        // 注意：如果电机硬件故障未排除，下一帧反馈数据仍会覆盖此值
    }
}

/* --- Data Access API --- */

/** @brief 获取多圈累计角度 (单位: 度) */
static inline float dji_motor_get_total_angle(DJIMotorDrv_s* motor)
{
    if (!motor)
        return 0.0f;
    // 8192 units = 360 degrees
    return (float)motor->measure.totalAngle * (360.0f / 8192.0f);
}

static inline float dji_motor_get_singgle_angle(DJIMotorDrv_s* motor)
{
    if (!motor)
        return 0.0f;
    // 8192 units = 360 degrees
    return (float)motor->measure.angle * (360.0f / 8192.0f);
}

/** @brief 获取当前转速 (单位: RPM) */
static inline float dji_motor_get_velocity(DJIMotorDrv_s* motor)
{
    if (!motor)
        return 0.0f;
    return (float)motor->measure.velocity; // RPM 直接返回
}

/** @brief 获取当前实际扭矩电流 (单位: A) - 已根据电机型号换算 */
static inline float dji_motor_get_current(DJIMotorDrv_s* motor)
{
    if (!motor || motor->link.type >= DJI_MOTOR_TYPE_UNKNOWN)
        return 0.0f;
    return (float)motor->measure.current * motor->scale;
}

/** @brief 获取电机温度 (单位: 摄氏度) */
static inline float dji_motor_get_temp(DJIMotorDrv_s* motor)
{
    if (!motor)
        return 0.0f;
    return (float)motor->measure.temp;
}

/** @brief 获取当前错误码 */
static inline uint8_t dji_motor_get_error_code(DJIMotorDrv_s* motor)
{
    if (!motor)
        return 0;
    return motor->measure.errorCode;
}

/** @brief 重置电机反馈状态(如累计角度归零) */
static inline void dji_motor_reset_feedback(DJIMotorDrv_s* motor)
{
    if (motor)
    {
        motor->measure.totalAngle = 0;
        motor->measure.roundCount = 0;
        motor->measure.angle = 0;
        // lastAngle 不重置，以避免下一次计算产生巨额跳变
    }
}

#ifdef __cplusplus
}
#endif

#endif /* __DJI_MOTOR_DRIVER_H__ */