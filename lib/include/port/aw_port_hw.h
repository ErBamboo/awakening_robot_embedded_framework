#ifndef __AWLF_PORT__HW_H__
#define __AWLF_PORT__HW_H__

/* hw port files 根据实际情况替换 */
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PORT_PRIMASK_ENABLED  (0) // 当PRIMASK寄存器为x时，开启中断
#define PORT_PRIMASK_DISABLED (!PORT_PRIMASK_ENABLED)

#if (PORT_PRIMASK_ENABLED == PORT_PRIMASK_DISABLED)
    #error "PORT_PRIMASK_ENABLED should not be equal to PORT_PRIMASK_DISABLED"
#endif

#define port_enable_int()                 __enable_irq()
#define port_disable_int()                __disable_irq()
#define port_get_primask()                __get_PRIMASK()
#define port_set_primask(x)               __set_PRIMASK(x)

#ifdef __cplusplus
}
#endif

#endif /* __AWLF_PORT__HW_H__ */
