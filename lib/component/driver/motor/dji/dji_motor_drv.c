/**
 * @file dji_motor_driver.c
 * @brief DJI 电机驱动实现
 */

#include "driver/motor/dji/dji_motor_drv.h"
#include "device/peripheral/pal_can_dev.h"
#include <string.h>
#include <math.h>

/* --- 静态内存池管理 --- */
static DJIMotorTxUnit_s s_tx_pool[DJI_MOTOR_MAX_TX_UNITS];
static uint8_t s_pool_usage_map[DJI_MOTOR_MAX_TX_UNITS] = {0};

static DJIMotorTxUnit_s* __alloc_tx_unit_static(void)
{
    for (int i = 0; i < DJI_MOTOR_MAX_TX_UNITS; i++)
    {
        if (s_pool_usage_map[i] == 0)
        {
            s_pool_usage_map[i] = 1;
            memset(&s_tx_pool[i], 0, sizeof(DJIMotorTxUnit_s));
            INIT_LIST_HEAD(&s_tx_pool[i].list);
            return &s_tx_pool[i];
        }
    }
    return NULL;
}

static DJIMotorTxUnit_s* __get_or_create_tx_unit(DJIMotorBus_s* bus, uint32_t canId)
{
    DJIMotorTxUnit_s* unit;
    struct list_head* pos;
    list_for_each(pos, &bus->txList)
    {
        unit = list_entry(pos, DJIMotorTxUnit_s, list);
        if (unit->canId == canId)
            return unit;
    }
    unit = __alloc_tx_unit_static();
    if (unit == NULL)
    {
        while(1){} // TODO: assert
        return NULL;
    }
    unit->canId = canId;
    list_add_tail(&unit->list, &bus->txList);
    return unit;
}

/* --- 内部核心回调 --- */

static void __dji_rx_callback(Device_t dev, void* param, size_t filter, size_t count)
{
    DJIMotorBus_s* bus = (DJIMotorBus_s*)param;
    CanUserMsg_s msg;
    uint8_t buf[8];
    msg.userBuf = buf;
    msg.bank = filter;

    for (size_t i = 0; i < count; i++)
    {
        if (device_read(dev, 0, &msg, 1) > 0)
        {
            uint32_t id = msg.dsc.id;

            if (id < DJI_RX_ID_START || id > DJI_RX_ID_END)
            {
                while(1){} // TODO: assert, 错误ID: 0x%08X
                continue;
            }

            DJIMotorDrv_s* motor = bus->rxMap[id - DJI_RX_ID_START];
            if (motor)
            {
                uint8_t* d = msg.userBuf;

                /* 1. 保存旧状态 */
                motor->measure.lastAngle = motor->measure.angle;
                motor->measure.lastErrorCode = motor->measure.errorCode;

                /* 2. 解析新数据 */
                motor->measure.angle = (int16_t)(d[0] << 8 | d[1]);
                motor->measure.velocity = (int16_t)(d[2] << 8 | d[3]);
                motor->measure.current = (int16_t)(d[4] << 8 | d[5]);
                motor->measure.temp = d[6];
                motor->measure.errorCode = d[7];

                /* 3. 多圈角度计算 */
                int16_t diff = motor->measure.angle - motor->measure.lastAngle;
                // 过零处理: 8191 -> 0 (diff < -4096) 或 0 -> 8191 (diff > 4096)
                if (diff < -4096)
                    motor->measure.roundCount++;
                else if (diff > 4096)
                    motor->measure.roundCount--;
                motor->measure.totalAngle = motor->measure.roundCount * 8192 + motor->measure.angle;

                /* 4. 错误回调触发 (边沿触发) */
                if (motor->errorCallback && (motor->measure.errorCode != motor->measure.lastErrorCode))
                    motor->errorCallback(motor, motor->measure.errorCode);
            }
        }
    }
}

/* --- Core API --- */

AwlfRet_e dji_motor_bus_init(DJIMotorBus_s* bus, Device_t canDev, uint8_t filterBankID)
{
    if (!bus || !canDev)
        return AWLF_ERROR_PARAM;
    memset(bus, 0, sizeof(DJIMotorBus_s));
    bus->canDev = canDev;
    INIT_LIST_HEAD(&bus->txList);

    CanFilterCfg_s filter;
    filter.bank = filterBankID;
    filter.workMode = CAN_FILTER_MODE_MASK;
    filter.idType = CAN_FILTER_ID_STD;
    filter.id = 0x200;
    filter.mask = 0x7F0;
    filter.rx_callback = __dji_rx_callback;
    filter.param = bus;

    return device_ctrl(canDev, CAN_CMD_SET_FILTER, &filter);
}

AwlfRet_e dji_motor_register(DJIMotorBus_s* bus, DJIMotorDrv_s* motor, DJIMotorType_e type, uint8_t id, DJIMotorCtrlMode_e mode)
{
    if (!bus || !motor || id == 0 || type > DJI_MOTOR_TYPE_UNKNOWN)
        return AWLF_ERROR_PARAM;

    memset(motor, 0, sizeof(DJIMotorDrv_s)); // 清空电机对象，防止野指针

    uint16_t rxId;
    if (type == DJI_MOTOR_TYPE_GM6020)
        rxId = 0x204 + id;
    else
        rxId = 0x200 + id;

    if (rxId < DJI_RX_ID_START || rxId > DJI_RX_ID_END)
        return AWLF_ERROR_PARAM;
    uint8_t mapIdx = rxId - DJI_RX_ID_START;
    if (bus->rxMap[mapIdx] != NULL)
        return AWLF_ERR_CONFLICT;

    uint32_t targetCanId = 0;
    uint8_t bufIdx = 0;

    if (type == DJI_MOTOR_TYPE_GM6020)
    {
        if (mode == DJI_CTRL_MODE_VOLTAGE)
        {
            targetCanId = (id <= 4) ? DJI_TX_ID_MIX_1_FF : DJI_TX_ID_GM6020_V_5_7;
            bufIdx = (id - (id <= 4 ? 1 : 5)) * 2;
        }
        else
        {
            targetCanId = (id <= 4) ? DJI_TX_ID_GM6020_C_1_4 : DJI_TX_ID_GM6020_C_5_7;
            bufIdx = (id - (id <= 4 ? 1 : 5)) * 2;
        }
    }
    else
    {
        targetCanId = (id <= 4) ? DJI_TX_ID_C6X0_1_4 : DJI_TX_ID_MIX_1_FF;
        bufIdx = (id - (id <= 4 ? 1 : 5)) * 2;
    }

    DJIMotorTxUnit_s* unit = __get_or_create_tx_unit(bus, targetCanId);
    if (unit == NULL)
        return AWLF_ERROR_MEMORY;

    uint8_t slotBit = (1 << (bufIdx / 2));
    if (unit->usageMask & slotBit)  // 插槽被占用，代表发生了物理ID冲突
    {
        while(1){}  // TODO: assert, 错误ID: 0x%08X, 冲突ID: 0x%08X
        return AWLF_ERR_CONFLICT;
    }

    // 根据电机型号进行换算
    // C610 (M2006): Range -10A ~ 10A (-16384 ~ 16384)
    // C620 (M3508): Range -20A ~ 20A (-16384 ~ 16384)
    // GM6020:       Range -3A ~ 3A   (-16384 ~ 16384)
    // 执行注册
    switch (motor->link.type)
    {
        case DJI_MOTOR_TYPE_C610: motor->scale = 10.0f / 16384.0f; break;
        case DJI_MOTOR_TYPE_C620: motor->scale = 20.0f / 16384.0f; break;
        case DJI_MOTOR_TYPE_GM6020: motor->scale = (mode == DJI_CTRL_MODE_CURRENT) ? 3.0f / 16384.0f : 24.0f / 25000.0f; break;
        default:
            while(1){} // TODO: assert, 未知电机类型
        break;;
    }
    unit->usageMask |= slotBit;
    motor->link.rxId = rxId;
    motor->link.txUnit = unit;
    motor->link.txBufIdx = bufIdx;
    motor->link.type = type;
    motor->mode = mode;

    // 初始化状态
    motor->measure.lastAngle = 0; // 首次读取时会产生一次跳变，通常应用层会丢弃第一帧或做特殊处理
    bus->rxMap[mapIdx] = motor;

    return AWLF_OK;
}

void dji_motor_set_output(DJIMotorDrv_s* motor, int16_t output)
{
    if (!motor || !motor->link.txUnit)
        return;
    motor->targetOutput = output;
    uint8_t* b = motor->link.txUnit->txBuffer;
    uint8_t i = motor->link.txBufIdx;
    b[i] = (uint8_t)(output >> 8);
    b[i + 1] = (uint8_t)(output & 0xFF);
    motor->link.txUnit->isDirty = 1;
}

void dji_motor_bus_sync(DJIMotorBus_s* bus)
{
    if (!bus)
        return;
    DJIMotorTxUnit_s* unit;
    struct list_head* pos;
    list_for_each(pos, &bus->txList)
    {
        unit = list_entry(pos, DJIMotorTxUnit_s, list);
        if (unit->usageMask && unit->isDirty)
        {
            CanUserMsg_s msg;
            msg.dsc = CAN_DATA_MSG_DSC_INIT(unit->canId, CAN_IDE_STD, 8);
            msg.userBuf = unit->txBuffer;
            device_write(bus->canDev, 0, &msg, 1);
            unit->isDirty = 0;
        }
    }
}

