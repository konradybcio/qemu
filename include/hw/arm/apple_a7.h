#ifndef HW_ARM_APPLE_A7_H
#define HW_ARM_APPLE_A7_H

#include "qemu-common.h"
#include "cpu.h"
#include "exec/hwaddr.h"
#include "qemu/queue.h"
#include "hw/arm/xnu_dtb.h"
#include "hw/cpu/cluster.h"

#define A7_MAX_CPU 2

#define TYPE_APPLE_A7 "apple-a7-cpu"
OBJECT_DECLARE_TYPE(AppleA7State, AppleA7Class, APPLE_A7)

#define TYPE_APPLE_A7_CLUSTER "apple-a7-cluster"
OBJECT_DECLARE_SIMPLE_TYPE(AppleA7Cluster, APPLE_A7_CLUSTER)

#define A7_CPREG_VAR_NAME(name) cpreg_##name
#define A7_CPREG_VAR_DEF(name) uint64_t A7_CPREG_VAR_NAME(name)

#define kDeferredIPITimerDefault 64000

typedef struct AppleA7Class {
    /*< private >*/
    ARMCPUClass base_class;
    /*< public >*/

    DeviceRealize parent_realize;
    DeviceUnrealize parent_unrealize;
    DeviceReset   parent_reset;
} AppleA7Class;

typedef struct AppleA7State {
    ARMCPU parent_obj;
    MemoryRegion impl_reg;
    MemoryRegion coresight_reg;
    MemoryRegion memory;
    MemoryRegion sysmem;
    uint32_t cpu_id;
    uint32_t phys_id;
    uint32_t cluster_id;
    uint64_t mpidr;
    uint64_t ipi_sr;
    hwaddr cluster_reg[2];
    qemu_irq fast_ipi;
    A7_CPREG_VAR_DEF(ARM64_REG_HID0);
    A7_CPREG_VAR_DEF(ARM64_REG_HID1);
    A7_CPREG_VAR_DEF(ARM64_REG_HID3);
    A7_CPREG_VAR_DEF(ARM64_REG_HID4);
    A7_CPREG_VAR_DEF(ARM64_REG_HID5);
    A7_CPREG_VAR_DEF(ARM64_REG_HID7);
    A7_CPREG_VAR_DEF(ARM64_REG_HID8);
    A7_CPREG_VAR_DEF(ARM64_REG_HID9);
    A7_CPREG_VAR_DEF(ARM64_REG_HID11);
    A7_CPREG_VAR_DEF(ARM64_REG_HID13);
    A7_CPREG_VAR_DEF(ARM64_REG_HID14);
    A7_CPREG_VAR_DEF(ARM64_REG_HID16);
    A7_CPREG_VAR_DEF(ARM64_REG_LSU_ERR_STS);
    A7_CPREG_VAR_DEF(PMC0);
    A7_CPREG_VAR_DEF(PMC1);
    A7_CPREG_VAR_DEF(PMCR0);
    A7_CPREG_VAR_DEF(PMCR1);
    A7_CPREG_VAR_DEF(PMSR);
    A7_CPREG_VAR_DEF(S3_4_c15_c0_5);
    A7_CPREG_VAR_DEF(ARM64_REG_CYC_OVRD);
    A7_CPREG_VAR_DEF(ARM64_REG_ACC_CFG);
    A7_CPREG_VAR_DEF(S3_5_c15_c10_1);
} AppleA7State;

AppleA7State *apple_a7_cpu_create(DTBNode *node);
bool apple_a7_cpu_is_sleep(AppleA7State *tcpu);
bool apple_a7_cpu_is_powered_off(AppleA7State *tcpu);
void apple_a7_cpu_start(AppleA7State *tcpu);
void apple_a7_cpu_reset(AppleA7State *tcpu);
void apple_a7_cpu_off(AppleA7State *tcpu);
#endif /* HW_ARM_APPLE_A7_H */
