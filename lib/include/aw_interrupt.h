#ifndef __AWLF_API_H__
#define __AWLF_API_H__

#include "port/aw_port_hw.h"       /* 硬件相关API */
#include "port/aw_port_compiler.h" /* 编译器相关API */

/* 硬件中断 */
#define aw_hw_enable_interrupt_force()  port_enable_int()
#define aw_hw_disable_interrupt_force() port_disable_int()

#define aw_hw_get_primask()             port_get_primask()
#define aw_hw_set_primask(x)            port_set_primask(x)

/* 硬件中断控制 */
#define aw_hw_disable_interrupt()                                                                                              \
    ({                                                                                                                           \
        uint32_t primask = aw_hw_get_primask();                                                                                \
        aw_hw_set_primask(PORT_PRIMASK_DISABLED);                                                                              \
        primask;                                                                                                                 \
    })
#define aw_hw_restore_interrupt(x)                                                                                             \
    ({                                                                                                                           \
        if (x == PORT_PRIMASK_ENABLED)                                                                                           \
            aw_hw_set_primask(PORT_PRIMASK_ENABLED);                                                                           \
    })

#endif /* __AWLF_API_H__ */