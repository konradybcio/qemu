#include "qemu/osdep.h"
#include "qemu/queue.h"
#include "qemu/timer.h"
#include "exec/address-spaces.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/irq.h"
#include "hw/or-irq.h"
#include "hw/arm/xnu_dtb.h"
#include "hw/arm/apple_a7.h"
#include "arm-powerctl.h"
#include "sysemu/reset.h"
#include "qapi/error.h"
#include "qemu/main-loop.h"

#define VMSTATE_A7_CPREG(name) \
        VMSTATE_UINT64(A7_CPREG_VAR_NAME(name), AppleA7State)

#define A7_CPREG_DEF(p_name, p_op0, p_op1, p_crn, p_crm, p_op2, p_access, p_reset) \
    {                                                                               \
        .cp = CP_REG_ARM64_SYSREG_CP,                                               \
        .name = #p_name, .opc0 = p_op0, .crn = p_crn, .crm = p_crm,                 \
        .opc1 = p_op1, .opc2 = p_op2, .access = p_access, .resetvalue = p_reset,    \
        .state = ARM_CP_STATE_AA64, .type = ARM_CP_OVERRIDE,                        \
        .fieldoffset = offsetof(AppleA7State, A7_CPREG_VAR_NAME(p_name))          \
                       - offsetof(ARMCPU, env)                                      \
    }

#define MPIDR_AFF0_SHIFT 0
#define MPIDR_AFF0_WIDTH 8
#define MPIDR_AFF0_MASK  (((1 << MPIDR_AFF0_WIDTH) - 1) << MPIDR_AFF0_SHIFT)
#define MPIDR_AFF1_SHIFT 8
#define MPIDR_AFF1_WIDTH 8
#define MPIDR_AFF1_MASK  (((1 << MPIDR_AFF1_WIDTH) - 1) << MPIDR_AFF1_SHIFT)
#define MPIDR_AFF2_SHIFT 16
#define MPIDR_AFF2_WIDTH 8
#define MPIDR_AFF2_MASK  (((1 << MPIDR_AFF2_WIDTH) - 1) << MPIDR_AFF2_SHIFT)

#define MPIDR_CPU_ID(mpidr_el1_val)             (((mpidr_el1_val) & MPIDR_AFF0_MASK) >> MPIDR_AFF0_SHIFT)

#define IPI_SR_SRC_CPU_SHIFT 8
#define IPI_SR_SRC_CPU_WIDTH 8
#define IPI_SR_SRC_CPU_MASK  (((1 << IPI_SR_SRC_CPU_WIDTH) - 1) << IPI_SR_SRC_CPU_SHIFT)
#define IPI_SR_SRC_CPU(ipi_sr_val)         (((ipi_sr_val) & IPI_SR_SRC_CPU_MASK) >> IPI_SR_SRC_CPU_SHIFT)

#define IPI_RR_TARGET_CLUSTER_SHIFT 16

#define IPI_RR_TYPE_IMMEDIATE (0 << 28)
#define IPI_RR_TYPE_RETRACT   (1 << 28)
#define IPI_RR_TYPE_DEFERRED  (2 << 28)
#define IPI_RR_TYPE_NOWAKE    (3 << 28)
#define IPI_RR_TYPE_MASK      (3 << 28)
#define NSEC_PER_USEC   1000ull         /* nanoseconds per microsecond */
#define USEC_PER_SEC    1000000ull      /* microseconds per second */
#define NSEC_PER_SEC    1000000000ull   /* nanoseconds per second */
#define NSEC_PER_MSEC   1000000ull      /* nanoseconds per millisecond */
#define RTCLOCK_SEC_DIVISOR     24000000ull

static void
absolutetime_to_nanoseconds(uint64_t abstime,
                            uint64_t *result)
{
	uint64_t t64;

	*result = (t64 = abstime / RTCLOCK_SEC_DIVISOR) * NSEC_PER_SEC;
	abstime -= (t64 * RTCLOCK_SEC_DIVISOR);
	*result += (abstime * NSEC_PER_SEC) / RTCLOCK_SEC_DIVISOR;
}

static void
nanoseconds_to_absolutetime(uint64_t nanosecs,
                            uint64_t *result)
{
	uint64_t t64;

	*result = (t64 = nanosecs / NSEC_PER_SEC) * RTCLOCK_SEC_DIVISOR;
	nanosecs -= (t64 * NSEC_PER_SEC);
	*result += (nanosecs * RTCLOCK_SEC_DIVISOR) / NSEC_PER_SEC;
}


static uint64_t ipi_cr = kDeferredIPITimerDefault;
static QEMUTimer *ipicr_timer = NULL;

inline bool apple_a7_cpu_is_sleep(AppleA7State *tcpu)
{
    return CPU(tcpu)->halted;
}

inline bool apple_a7_cpu_is_powered_off(AppleA7State *tcpu)
{
    return ARM_CPU(tcpu)->power_state == PSCI_OFF;
}

void apple_a7_cpu_start(AppleA7State *tcpu)
{
    int ret = QEMU_ARM_POWERCTL_RET_SUCCESS;

    if (ARM_CPU(tcpu)->power_state != PSCI_ON) {
        ret = arm_set_cpu_on_and_reset(tcpu->mpidr);
    }

    if (ret != QEMU_ARM_POWERCTL_RET_SUCCESS) {
        error_report("%s: failed to bring up CPU %d: err %d",
                __func__, tcpu->cpu_id, ret);
    }
}

void apple_a7_cpu_reset(AppleA7State *tcpu)
{
    int ret = QEMU_ARM_POWERCTL_RET_SUCCESS;

    if (ARM_CPU(tcpu)->power_state != PSCI_OFF) {
        ret = arm_reset_cpu(tcpu->mpidr);
    }

    if (ret != QEMU_ARM_POWERCTL_RET_SUCCESS) {
        error_report("%s: failed to reset CPU %d: err %d",
                __func__, tcpu->cpu_id, ret);
    }
}

void apple_a7_cpu_off(AppleA7State *tcpu)
{
    int ret = QEMU_ARM_POWERCTL_RET_SUCCESS;

    if (ARM_CPU(tcpu)->power_state != PSCI_OFF) {
        ret = arm_set_cpu_off(tcpu->mpidr);
    }

    if (ret != QEMU_ARM_POWERCTL_RET_SUCCESS) {
        error_report("%s: failed to turn off CPU %d: err %d",
                __func__, tcpu->cpu_id, ret);
    }
}

static const ARMCPRegInfo apple_a7_cp_reginfo_tcg[] = {
    A7_CPREG_DEF(ARM64_REG_HID0, 3, 0, 15, 0, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_HID1, 3, 0, 15, 1, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_HID3, 3, 0, 15, 3, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_HID4, 3, 0, 15, 4, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_HID5, 3, 0, 15, 5, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_HID7, 3, 0, 15, 7, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_HID8, 3, 0, 15, 8, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_HID9, 3, 0, 15, 9, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_HID11, 3, 0, 15, 11, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_HID13, 3, 0, 15, 14, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_HID14, 3, 0, 15, 15, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_HID16, 3, 0, 15, 15, 2, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_LSU_ERR_STS, 3, 3, 15, 0, 0, PL1_RW, 0),
    A7_CPREG_DEF(PMC0, 3, 2, 15, 0, 0, PL1_RW, 0),
    A7_CPREG_DEF(PMC1, 3, 2, 15, 1, 0, PL1_RW, 0),
    A7_CPREG_DEF(PMCR0, 3, 1, 15, 0, 0, PL1_RW, 0),
    A7_CPREG_DEF(PMCR1, 3, 1, 15, 1, 0, PL1_RW, 0),
    A7_CPREG_DEF(PMSR, 3, 1, 15, 13, 0, PL1_RW, 0),
    A7_CPREG_DEF(S3_4_c15_c0_5, 3, 4, 15, 0, 5, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_CYC_OVRD, 3, 5, 15, 5, 0, PL1_RW, 0),
    A7_CPREG_DEF(ARM64_REG_ACC_CFG, 3, 5, 15, 4, 0, PL1_RW, 0),
    A7_CPREG_DEF(S3_5_c15_c10_1, 3, 5, 15, 10, 1, PL0_RW, 0),

    // {
    //     .cp = CP_REG_ARM64_SYSREG_CP,
    //     .name = "ARM64_REG_IPI_RR_LOCAL",
    //     .opc0 = 3, .opc1 = 5, .crn = 15, .crm = 0, .opc2 = 0,
    //     .access = PL1_W, .type = ARM_CP_IO | ARM_CP_NO_RAW,
    //     .state = ARM_CP_STATE_AA64,
    //     .readfn = arm_cp_read_zero,
    //     .writefn = apple_a7_ipi_rr_local
    // },
    // {
    //     .cp = CP_REG_ARM64_SYSREG_CP,
    //     .name = "ARM64_REG_IPI_RR_GLOBAL",
    //     .opc0 = 3, .opc1 = 5, .crn = 15, .crm = 0, .opc2 = 1,
    //     .access = PL1_W, .type = ARM_CP_IO | ARM_CP_NO_RAW,
    //     .state = ARM_CP_STATE_AA64,
    //     .readfn = arm_cp_read_zero,
    //     .writefn = apple_a7_ipi_rr_global
    // },
    // {
    //     .cp = CP_REG_ARM64_SYSREG_CP,
    //     .name = "ARM64_REG_IPI_SR",
    //     .opc0 = 3, .opc1 = 5, .crn = 15, .crm = 1, .opc2 = 1,
    //     .access = PL1_RW, .type = ARM_CP_IO | ARM_CP_NO_RAW,
    //     .state = ARM_CP_STATE_AA64,
    //     .readfn = apple_a7_ipi_read_sr,
    //     .writefn = apple_a7_ipi_write_sr
    // },
    // {
    //     .cp = CP_REG_ARM64_SYSREG_CP,
    //     .name = "ARM64_REG_IPI_CR",
    //     .opc0 = 3, .opc1 = 5, .crn = 15, .crm = 3, .opc2 = 1,
    //     .access = PL1_RW, .type = ARM_CP_IO,
    //     .state = ARM_CP_STATE_AA64,
    //     .readfn = apple_a7_ipi_read_cr,
    //     .writefn = apple_a7_ipi_write_cr
    // },
    REGINFO_SENTINEL,
};

static void apple_a7_add_cpregs(AppleA7State *tcpu)
{
    ARMCPU *cpu = ARM_CPU(tcpu);
    define_arm_cp_regs(cpu, apple_a7_cp_reginfo_tcg);
}

static void apple_a7_realize(DeviceState *dev, Error **errp)
{
    AppleA7State *tcpu = APPLE_A7(dev);
    AppleA7Class *tclass = APPLE_A7_GET_CLASS(dev);
    DeviceState *fiq_or;
    Object *obj = OBJECT(dev);

    object_property_set_link(OBJECT(tcpu), "memory", OBJECT(&tcpu->memory),
                             errp);
    if (*errp) {
        return;
    }
    apple_a7_add_cpregs(tcpu);
    tclass->parent_realize(dev, errp);
    if (*errp) {
        return;
    }
    fiq_or = qdev_new(TYPE_OR_IRQ);
    object_property_add_child(obj, "fiq-or", OBJECT(fiq_or));
    qdev_prop_set_uint16(fiq_or, "num-lines", 16);
    qdev_realize_and_unref(fiq_or, NULL, errp);
    if (*errp) {
        return;
    }
    qdev_connect_gpio_out(fiq_or, 0, qdev_get_gpio_in(dev, ARM_CPU_FIQ));

    qdev_connect_gpio_out(dev, GTIMER_VIRT, qdev_get_gpio_in(fiq_or, 0));
    tcpu->fast_ipi = qdev_get_gpio_in(fiq_or, 1);
}

static void apple_a7_reset(DeviceState *dev)
{
    G_GNUC_UNUSED AppleA7State *tcpu = APPLE_A7(dev);
    G_GNUC_UNUSED AppleA7Class *tclass = APPLE_A7_GET_CLASS(dev);
    tclass->parent_reset(dev);
}

static void apple_a7_instance_init(Object *obj)
{
    ARMCPU *cpu = ARM_CPU(obj);
    object_property_set_uint(obj, "cntfrq", 24000000, &error_fatal);
}

AppleA7State *apple_a7_cpu_create(DTBNode *node)
{
    DeviceState  *dev;
    AppleA7State *tcpu;
    ARMCPU *cpu;
    Object *obj;
    DTBProp *prop;
    uint64_t mpidr;
    uint64_t freq;
    uint64_t *reg;

    obj = object_new(TYPE_APPLE_A7);
    dev = DEVICE(obj);
    tcpu = APPLE_A7(dev);
    cpu = ARM_CPU(tcpu);

    prop = find_dtb_prop(node, "name");
    dev->id = g_strdup((char *)prop->value);

    prop = find_dtb_prop(node, "reg");
    assert(prop->length == 4);
    tcpu->phys_id = *(unsigned int*)prop->value;

    // Bit 31 is RES
    mpidr = 0LL | tcpu->phys_id | (1LL << 31);

    cpu->midr = 0x611f0011; /* Apple Cyclone rev ?? */

    tcpu->mpidr = mpidr;

    /* remove debug regs from device tree */
    prop = find_dtb_prop(node, "reg-private");
    if (prop != NULL) {
        remove_dtb_prop(node, prop);
    }

    prop = find_dtb_prop(node, "cpu-uttdbg-reg");
    if (prop != NULL) {
        remove_dtb_prop(node, prop);
    }

    /* need to set the cpu freqs instead of iBoot */
    freq = 24000000;

    if (tcpu->cpu_id == 0) {
        prop = find_dtb_prop(node, "state");
        if (prop != NULL) {
            remove_dtb_prop(node, prop);
        }
        set_dtb_prop(node, "state", 8, (uint8_t *)"running");
    } else {
        object_property_set_bool(obj, "start-powered-off", true, NULL);
    }

    set_dtb_prop(node, "timebase-frequency", sizeof(uint64_t), (uint8_t *)&freq);
    set_dtb_prop(node, "fixed-frequency", sizeof(uint64_t), (uint8_t *)&freq);
    set_dtb_prop(node, "peripheral-frequency", sizeof(uint64_t), (uint8_t *)&freq);
    set_dtb_prop(node, "memory-frequency", sizeof(uint64_t), (uint8_t *)&freq);
    set_dtb_prop(node, "bus-frequency", sizeof(uint32_t), (uint8_t *)&freq);
    set_dtb_prop(node, "clock-frequency", sizeof(uint32_t), (uint8_t *)&freq);

    object_property_set_bool(obj, "has_el3", false, NULL);
    object_property_set_bool(obj, "has_el2", false, NULL);

    memory_region_init(&tcpu->memory, obj, "cpu-memory", UINT64_MAX);
    memory_region_init_alias(&tcpu->sysmem, obj, "sysmem", get_system_memory(),
                             0, UINT64_MAX);
    memory_region_add_subregion_overlap(&tcpu->memory, 0, &tcpu->sysmem, -2);

    prop = find_dtb_prop(node, "cpu-impl-reg");
    if (prop) {
        assert(prop->length == 16);

        reg = (uint64_t*)prop->value;

        memory_region_init_ram_device_ptr(&tcpu->impl_reg, obj,
                                          TYPE_APPLE_A7 ".impl-reg",
                                          reg[1], g_malloc0(reg[1]));
        memory_region_add_subregion(get_system_memory(),
                                    reg[0], &tcpu->impl_reg);
    }

    prop = find_dtb_prop(node, "coresight-reg");
    if (prop) {
        assert(prop->length == 16);

        reg = (uint64_t*)prop->value;

        memory_region_init_ram_device_ptr(&tcpu->coresight_reg, obj,
                                          TYPE_APPLE_A7 ".coresight-reg",
                                          reg[1], g_malloc0(reg[1]));
        memory_region_add_subregion(get_system_memory(),
                                    reg[0], &tcpu->coresight_reg);
    }

    prop = find_dtb_prop(node, "cpm-impl-reg");
    if (prop) {
        assert(prop->length == 16);
        // memcpy(tcpu->cluster_reg, prop->value, 16);
    }
    return tcpu;
}

static Property apple_a7_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_apple_a7 = {
    .name = "apple_a7",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_A7_CPREG(ARM64_REG_HID0),
        VMSTATE_A7_CPREG(ARM64_REG_HID1),
        VMSTATE_A7_CPREG(ARM64_REG_HID3),
        VMSTATE_A7_CPREG(ARM64_REG_HID4),
        VMSTATE_A7_CPREG(ARM64_REG_HID5),
        VMSTATE_A7_CPREG(ARM64_REG_HID7),
        VMSTATE_A7_CPREG(ARM64_REG_HID8),
        VMSTATE_A7_CPREG(ARM64_REG_HID9),
        VMSTATE_A7_CPREG(ARM64_REG_HID11),
        VMSTATE_A7_CPREG(ARM64_REG_HID13),
        VMSTATE_A7_CPREG(ARM64_REG_HID14),
        VMSTATE_A7_CPREG(ARM64_REG_HID16),
        VMSTATE_A7_CPREG(ARM64_REG_LSU_ERR_STS),
        VMSTATE_A7_CPREG(PMC0),
        VMSTATE_A7_CPREG(PMC1),
        VMSTATE_A7_CPREG(PMCR0),
        VMSTATE_A7_CPREG(PMCR1),
        VMSTATE_A7_CPREG(PMSR),
        VMSTATE_A7_CPREG(S3_4_c15_c0_5),
        VMSTATE_A7_CPREG(ARM64_REG_CYC_OVRD),
        VMSTATE_A7_CPREG(ARM64_REG_ACC_CFG),
        VMSTATE_A7_CPREG(S3_5_c15_c10_1),
        VMSTATE_UINT64(env.keys.m.lo, ARMCPU),
        VMSTATE_UINT64(env.keys.m.hi, ARMCPU),
        VMSTATE_END_OF_LIST()
    }
};

static void apple_a7_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AppleA7Class *tc = APPLE_A7_CLASS(klass);

    device_class_set_parent_realize(dc, apple_a7_realize, &tc->parent_realize);
    device_class_set_parent_reset(dc, apple_a7_reset, &tc->parent_reset);
    dc->desc = "Apple A7 CPU";
    dc->vmsd = &vmstate_apple_a7;
    set_bit(DEVICE_CATEGORY_CPU, dc->categories);
    device_class_set_props(dc, apple_a7_properties);
}

static const TypeInfo apple_a7_info = {
    .name = TYPE_APPLE_A7,
    .parent = ARM_CPU_TYPE_NAME("max"),
    .instance_size = sizeof(AppleA7State),
    .instance_init = apple_a7_instance_init,
    .class_size = sizeof(AppleA7Class),
    .class_init = apple_a7_class_init,
};

static void apple_a7_register_types(void)
{
    type_register_static(&apple_a7_info);
}

type_init(apple_a7_register_types);
