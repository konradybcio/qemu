/*
 * iPhone 11 - S5L8960X
 *
 * Copyright (c) 2019 Johnathan Afek <jonyafek@me.com>
 * Copyright (c) 2021 Nguyen Hoang Trung (TrungNguyen1909)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu-common.h"
#include "hw/arm/boot.h"
#include "exec/address-spaces.h"
#include "hw/misc/unimp.h"
#include "sysemu/block-backend.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "sysemu/runstate.h"
#include "qemu/error-report.h"
#include "hw/platform-bus.h"
#include "arm-powerctl.h"

#include "hw/arm/s5l8960x.h"
#include "hw/arm/s5l8960x-config.c.inc"
#include "hw/arm/apple_a7.h"

#include "hw/irq.h"
#include "hw/or-irq.h"
#include "hw/intc/apple_aic.h"
#include "hw/block/apple_ans.h"
#include "hw/arm/apple_sart.h"
#include "hw/gpio/apple_gpio.h"
#include "hw/i2c/apple_hw_i2c.h"
#include "hw/usb/apple_otg.h"
#include "hw/usb/apple_typec.h"
#include "hw/watchdog/apple_wdt.h"
#include "hw/misc/apple_aes.h"
#include "hw/nvram/apple_nvram.h"
#include "hw/spmi/apple_spmi.h"
#include "hw/spmi/apple_spmi_pmu.h"
#include "hw/misc/apple_smc.h"
#include "hw/arm/apple_dart.h"
#include "hw/dma/apple_sio.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/apple_spi.h"
#include "hw/char/apple_uart.h"

#include "hw/arm/xnu_pf.h"
#include "hw/display/m1_fb.h"

#define S5L8960X_DRAM_BASE         (0x800000000)
#define S5L8960X_DRAM_SIZE         (1 * GiB)

/* This is from /chosen/memory-map MOST LIKELY WRONG */
#define S5L8960X_KERNEL_REGION_BASE (0x803804000)
#define S5L8960X_KERNEL_REGION_SIZE (0x1a1a230)

/* NOTE: starting from SPI1 */
#define S5L8960X_SPI_BASE(_x)      (0x0a804000 + (_x) * APPLE_SPI_MMIO_SIZE)

#define S5L8960X_DWC2_IRQ          (241)

/* UART0-UART6 */
#define S5L8960X_NUM_UARTS         (7)
/* SPI1-SPI3 */
#define S5L8960X_NUM_SPIS          (3)

#define S5L8960X_ANS_TEXT_BASE     (0x83f800000)
#define S5L8960X_ANS_TEXT_SIZE     (0x4a000)
#define S5L8960X_ANS_DATA_BASE     (S5L8960X_ANS_TEXT_BASE + S5L8960X_ANS_TEXT_SIZE)
#define S5L8960X_ANS_DATA_SIZE     (0x7b6000)

#define S5L8960X_DISPLAY_BASE      (0x83ee00000)
#define S5L8960X_DISPLAY_SIZE      (0x87b000)

#define S5L8960X_PANIC_BASE        (0x83f67b000)
#define S5L8960X_PANIC_SIZE        (0x80000)

#define NOP_INST 0xd503201f
#define MOV_W0_01_INST 0x52800020
#define MOV_X13_0_INST 0xd280000d
#define RET_INST 0xd65f03c0
#define RETAB_INST 0xd65f0fff

static void s5l8960x_start_cpus(MachineState* machine, uint64_t cpu_mask)
{
    S5L8960XMachineState* tms = S5L8960X_MACHINE(machine);
    int i;

    for(i = 0; i < machine->smp.cpus; i++) {
        if (test_bit(i, (unsigned long*)&cpu_mask)
            && apple_a7_cpu_is_powered_off(tms->cpus[i])) {
            apple_a7_cpu_start(tms->cpus[i]);
        }
    }
}

static void s5l8960x_create_s3c_uart(const S5L8960XMachineState *tms, uint32_t port,
                                  Chardev *chr)
{
    DeviceState *dev;
    hwaddr base;
    //first fetch the uart mmio address
    int vector;
    DTBProp *prop;
    hwaddr *uart_offset;
    DTBNode *child = find_dtb_node(tms->device_tree, "arm-io/uart0");
    char name[32] = { 0 };

    assert(port < S5L8960X_NUM_UARTS);

    assert(child != NULL);
    snprintf(name, sizeof(name), "uart%d", port);

    prop = find_dtb_prop(child, "reg");
    assert(prop != NULL);

    uart_offset = (hwaddr *)prop->value;
    base = tms->soc_base_pa + uart_offset[0] + uart_offset[1] * port;

    prop = find_dtb_prop(child, "interrupts");
    assert(prop);

    vector = *(uint32_t*)prop->value + port;
    dev = apple_uart_create(base, 15, 0, chr,
                            qdev_get_gpio_in(DEVICE(tms->aic), vector));
    assert(dev);
    dev->id = g_strdup(name);
}

static void s5l8960x_patch_kernel(struct mach_header_64 *hdr)
{
    //disable_kprintf_output = 0
    // *(uint32_t *)vtop_static(0xFFFFFFF0077142C8) = 0;
    kpf();
}

static bool s5l8960x_check_panic(MachineState *machine)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    if (!tms->panic_size) {
        return false;
    }
    g_autofree struct xnu_embedded_panic_header *panic_info =
                                                   g_malloc0(tms->panic_size);
    g_autofree void *buffer = g_malloc0(tms->panic_size);

    address_space_rw(&address_space_memory, tms->panic_base,
                     MEMTXATTRS_UNSPECIFIED, panic_info,
                     tms->panic_size, 0);
    address_space_rw(&address_space_memory, tms->panic_base,
                     MEMTXATTRS_UNSPECIFIED, buffer,
                     tms->panic_size, 1);

    return panic_info->eph_magic == EMBEDDED_PANIC_MAGIC;
}

static size_t get_kaslr_random()
{
    size_t value = 0;
    qemu_guest_getrandom(&value, sizeof(value), NULL);
    return value;
}

#define L2_GRANULE          ((512) * (16384 / 8))
#define L2_GRANULE_MASK     (L2_GRANULE - 1)

static void get_kaslr_slides(S5L8960XMachineState *tms,
                             hwaddr *phys_slide_out, hwaddr *virt_slide_out)
{
    hwaddr slide_phys = 0, slide_virt = 0;
    const size_t slide_granular = (1 << 14);
    const size_t slide_granular_mask = slide_granular - 1;
    const size_t slide_virt_max = 0x100 * (2 * 1024 * 1024);
    size_t random_value = get_kaslr_random();

    if (tms->kaslr_off) {
        *phys_slide_out = 0;
        *virt_slide_out = 0;
        return;
    }

    slide_virt = (random_value & ~slide_granular_mask) % slide_virt_max;
    if (slide_virt == 0) {
        slide_virt = slide_virt_max;
    }
    slide_phys = slide_virt & L2_GRANULE_MASK;

    *phys_slide_out = slide_phys;
    *virt_slide_out = slide_virt;
}

static void s5l8960x_load_classic_kc(S5L8960XMachineState *tms, const char *cmdline)
{
    MachineState *machine = MACHINE(tms);
    struct mach_header_64 *hdr = tms->kernel;
    MemoryRegion *sysmem = tms->sysmem;
    AddressSpace *nsas = &address_space_memory;
    hwaddr virt_low;
    hwaddr virt_end;
    hwaddr dtb_va;
    hwaddr top_of_kernel_data_pa;
    hwaddr mem_size;
    hwaddr phys_ptr;
    hwaddr slide_phys = 0;
    hwaddr slide_virt = 0;
    macho_boot_info_t info = &tms->bootinfo;
    g_autofree xnu_pf_range_t *last_range = NULL;
    g_autofree xnu_pf_range_t *text_range = NULL;
    DTBNode *memory_map = get_dtb_node(tms->device_tree, "/chosen/memory-map");

    /*
     * Setup the memory layout:
     * The trustcache is right in front of the __TEXT section, aligned to 16k
     * Then we have all the kernel sections.
     * After that we have ramdisk
     * After that we have the kernel boot args
     * After that we have the device tree
     * After that we have the rest of the RAM
     */

    g_phys_base = (hwaddr)macho_get_buffer(hdr);
    macho_highest_lowest(hdr, &virt_low, &virt_end);
    last_range = xnu_pf_segment(hdr, "__LAST");
    text_range = xnu_pf_segment(hdr, "__TEXT");

    get_kaslr_slides(tms, &slide_phys, &slide_virt);

    g_phys_base = phys_ptr = align_up(S5L8960X_KERNEL_REGION_BASE, 16 * MiB);
    phys_ptr += slide_phys;
    g_virt_base += slide_virt - slide_phys;

    fprintf(stderr, "TC entry: 0x" TARGET_FMT_lx "\n", phys_ptr);
    /* TrustCache */
    info->trustcache_pa = vtop_static(text_range->va + slide_virt) - 
                          info->trustcache_size;

    macho_load_trustcache(tms->trustcache, info->trustcache_size,
                          nsas, sysmem, info->trustcache_pa);
    phys_ptr += align_4k_high(info->trustcache_size);
    fprintf(stderr, "TC end: 0x" TARGET_FMT_lx "\n", phys_ptr);

    info->entry = arm_load_macho(hdr, nsas, sysmem, memory_map,
                                 g_phys_base + slide_phys, slide_virt);
    fprintf(stderr, "g_virt_base: 0x" TARGET_FMT_lx "\n"
                    "g_phys_base: 0x" TARGET_FMT_lx "\n",
                    g_virt_base, g_phys_base);
    fprintf(stderr, "slide_virt: 0x" TARGET_FMT_lx "\n"
                    "slide_phys: 0x" TARGET_FMT_lx "\n",
                    slide_virt, slide_phys);
    fprintf(stderr, "entry: 0x" TARGET_FMT_lx "\n", info->entry);

    virt_end += slide_virt;
    phys_ptr = vtop_static(align_4k_high(virt_end));

    /* ramdisk */
    if (machine->initrd_filename) {
        info->ramdisk_pa = phys_ptr;
        fprintf(stderr, "RD base: 0x" TARGET_FMT_lx "\n", phys_ptr);
        macho_load_ramdisk(machine->initrd_filename, nsas, sysmem, info->ramdisk_pa, &info->ramdisk_size);
        info->ramdisk_size = align_4k_high(info->ramdisk_size);
        phys_ptr += info->ramdisk_size;
        fprintf(stderr, "RD end: 0x" TARGET_FMT_lx "\n", phys_ptr);
    }

    /* Kernel boot args */
    fprintf(stderr, "bootargs base: 0x" TARGET_FMT_lx "\n", phys_ptr);
    info->bootargs_pa = phys_ptr;
    phys_ptr += align_4k_high(0x4000);

    /* device tree */
    info->dtb_pa = phys_ptr;
    dtb_va = ptov_static(info->dtb_pa);
    phys_ptr += align_4k_high(info->dtb_size);

    mem_size = S5L8960X_KERNEL_REGION_SIZE -
               (g_phys_base - S5L8960X_KERNEL_REGION_BASE);

    macho_load_dtb(tms->device_tree, nsas, sysmem, "DeviceTree", info);

    top_of_kernel_data_pa = (align_4k_high(phys_ptr) + 0x3000ull) & ~0x3fffull;

    fprintf(stderr, "cmdline: [%s]\n", cmdline);
    macho_setup_bootargs("BootArgs", nsas, sysmem, info->bootargs_pa,
                         g_virt_base, g_phys_base, mem_size,
                         top_of_kernel_data_pa, dtb_va, info->dtb_size,
                         tms->video, cmdline);
    g_virt_base = virt_low;
}

static void s5l8960x_load_fileset_kc(S5L8960XMachineState *tms, const char *cmdline)
{
    MachineState *machine = MACHINE(tms);
    struct mach_header_64 *hdr = tms->kernel;
    MemoryRegion *sysmem = tms->sysmem;
    AddressSpace *nsas = &address_space_memory;
    hwaddr virt_low;
    hwaddr virt_end;
    hwaddr dtb_va;
    hwaddr top_of_kernel_data_pa;
    hwaddr mem_size;
    hwaddr phys_ptr;
    hwaddr slide_phys = 0;
    hwaddr slide_virt = 0;
    uint64_t l2_remaining = 0;
    uint64_t extradata_size = 0;
    macho_boot_info_t info = &tms->bootinfo;
    g_autofree xnu_pf_range_t *last_range = NULL;
    DTBNode *memory_map = get_dtb_node(tms->device_tree, "/chosen/memory-map");

    /*
     * Setup the memory layout:
     * First we have the device tree
     * The trustcache is right after the device tree
     * Then we have all the kernel sections.
     * After that we have ramdisk
     * After that we have the kernel boot args
     * After that we have the rest of the RAM
     */

    g_phys_base = (hwaddr)macho_get_buffer(hdr);
    macho_highest_lowest(hdr, &virt_low, &virt_end);
    g_virt_base = virt_low;
    last_range = xnu_pf_segment(hdr, "__PRELINK_INFO");

    extradata_size = align_4k_high(info->dtb_size + info->trustcache_size);
    assert(extradata_size < L2_GRANULE);

    get_kaslr_slides(tms, &slide_phys, &slide_virt);

    l2_remaining = (virt_low + slide_virt) & L2_GRANULE_MASK;

    if (extradata_size >= l2_remaining) {
        uint64_t grown_slide = align_4k_high(extradata_size - l2_remaining);
        slide_phys += grown_slide;
        slide_virt += grown_slide;
    }

    phys_ptr = align_up(S5L8960X_KERNEL_REGION_BASE, 32 * MiB) | (virt_low & L2_GRANULE_MASK);
    g_phys_base = phys_ptr & ~L2_GRANULE_MASK;
    phys_ptr += slide_phys;
    phys_ptr -= extradata_size;

    /* device tree */
    info->dtb_pa = phys_ptr;
    phys_ptr += info->dtb_size;

    /* TrustCache */
    info->trustcache_pa = phys_ptr;
    macho_load_trustcache(tms->trustcache, info->trustcache_size,
                          nsas, sysmem, info->trustcache_pa);
    phys_ptr += align_4k_high(info->trustcache_size);

    g_virt_base += slide_virt;
    g_virt_base -= phys_ptr - g_phys_base;
    info->entry = arm_load_macho(hdr, nsas, sysmem, memory_map,
                                 phys_ptr, slide_virt);
    fprintf(stderr, "g_virt_base: 0x" TARGET_FMT_lx "\n"
                    "g_phys_base: 0x" TARGET_FMT_lx "\n",
                    g_virt_base, g_phys_base);
    fprintf(stderr, "slide_virt: 0x" TARGET_FMT_lx "\n"
                    "slide_phys: 0x" TARGET_FMT_lx "\n",
                    slide_virt, slide_phys);
    fprintf(stderr, "entry: 0x" TARGET_FMT_lx "\n", info->entry);

    virt_end += slide_virt;
    phys_ptr = vtop_static(align_4k_high(virt_end));
    dtb_va = ptov_static(info->dtb_pa);

    /* ramdisk */
    if (machine->initrd_filename) {
        info->ramdisk_pa = phys_ptr;
        macho_load_ramdisk(machine->initrd_filename, nsas, sysmem,
                           info->ramdisk_pa, &info->ramdisk_size);
        info->ramdisk_size = align_4k_high(info->ramdisk_size);
        phys_ptr += info->ramdisk_size;
    }

    /* Kernel boot args */
    info->bootargs_pa = phys_ptr;
    phys_ptr += align_4k_high(0x4000);

    mem_size = S5L8960X_KERNEL_REGION_SIZE -
               (g_phys_base - S5L8960X_KERNEL_REGION_BASE);

    macho_load_dtb(tms->device_tree, nsas, sysmem, "DeviceTree", info);

    top_of_kernel_data_pa = (align_4k_high(phys_ptr) + 0x3000ull) & ~0x3fffull;

    fprintf(stderr, "cmdline: [%s]\n", cmdline);
    macho_setup_bootargs("BootArgs", nsas, sysmem, info->bootargs_pa,
                         g_virt_base, g_phys_base, mem_size,
                         top_of_kernel_data_pa, dtb_va, info->dtb_size,
                         tms->video, cmdline);
    g_virt_base = virt_low;
}

static void s5l8960x_memory_setup(MachineState *machine)
{
    struct mach_header_64 *hdr;
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    AppleNvramState *nvram = NULL;
    macho_boot_info_t info = &tms->bootinfo;
    DTBNode *memory_map = get_dtb_node(tms->device_tree, "/chosen/memory-map");
    g_autofree char *cmdline = NULL;


    #if 0
    The end of DRAM:
    0x8fa298000, 0x2300000: VRAM
    0x8fc598000, 0x3900000: ANS
    0x8ffeb0000, 0x100000: PRAM
    0x8fffb4000, 0x4000: GFX handoff
    0x8fffb8000, 0x40000: GFX shared region
    0x8ffff8000, 0x4000: GPU region
    #endif

    if (s5l8960x_check_panic(machine)) {
        qemu_system_guest_panicked(NULL);
        return;
    }
    info->dram_base = S5L8960X_DRAM_BASE;
    info->dram_size = S5L8960X_DRAM_SIZE;

    nvram = APPLE_NVRAM(qdev_find_recursive(sysbus_get_default(), "nvram"));
    if (!nvram) {
        error_setg(&error_abort, "%s: Failed to find nvram device", __func__);
        return;
    };
    apple_nvram_load(nvram);

    fprintf(stderr, "boot_mode: %u\n", tms->boot_mode);
    switch (tms->boot_mode) {
    case kBootModeEnterRecovery:
        env_set(nvram, "auto-boot", "false", 0);
        tms->boot_mode = kBootModeAuto;
        break;
    case kBootModeExitRecovery:
        env_set(nvram, "auto-boot", "true", 0);
        tms->boot_mode = kBootModeAuto;
        break;
    default:
        break;
    }

    fprintf(stderr, "auto-boot=%s\n", env_get_bool(nvram, "auto-boot", false) ? "true" : "false");
    switch (tms->boot_mode) {
    case kBootModeAuto:
        if (!env_get_bool(nvram, "auto-boot", false)) {
            asprintf(&cmdline, "-restore rd=md0 nand-enable-reformat=1 -progress %s", machine->kernel_cmdline);
            break;
        }
        QEMU_FALLTHROUGH;
    default:
        asprintf(&cmdline, "%s", machine->kernel_cmdline);
    }

    apple_nvram_save(nvram);

    info->nvram_size = nvram->len;

    if (info->nvram_size > XNU_MAX_NVRAM_SIZE) {
        info->nvram_size = XNU_MAX_NVRAM_SIZE;
    }
    if (apple_nvram_serialize(nvram, info->nvram_data, sizeof(info->nvram_data)) < 0) {
        error_report("%s: Failed to read NVRAM", __func__);
    }

    if (tms->ticket_filename) {
        if (!g_file_get_contents(tms->ticket_filename, &info->ticket_data, (gsize *)&info->ticket_length, NULL)) {
            error_report("%s: Failed to read ticket from file %s", __func__, tms->ticket_filename);
        }
    }

    if (xnu_contains_boot_arg(cmdline, "-restore", false)) {
        /* HACK: Use DEV Hardware model to restore without FDR errors */
        set_dtb_prop(tms->device_tree, "compatible", 28, "N53DEV\0iPhone6,2\0AppleARM\0$");
    } else {
        set_dtb_prop(tms->device_tree, "compatible", 27, "N53AP\0iPhone6,2\0AppleARM\0$");
    }

    if (!xnu_contains_boot_arg(cmdline, "rd=", true)) {
        DTBNode *chosen = find_dtb_node(tms->device_tree, "chosen");
        DTBProp *prop = find_dtb_prop(chosen, "root-matching");

        if (prop) {
            snprintf((char *)prop->value, prop->length, "<dict><key>IOProviderClass</key><string>IOMedia</string><key>IOPropertyMatch</key><dict><key>Partition ID</key><integer>1</integer></dict></dict>");
        }
    }

    DTBNode *pram = find_dtb_node(tms->device_tree, "pram");
    if (pram) {
        uint64_t panic_reg[2] = { 0 };
        uint64_t panic_base = S5L8960X_PANIC_BASE;
        uint64_t panic_size = S5L8960X_PANIC_SIZE;

        panic_reg[0] = panic_base;
        panic_reg[1] = panic_size;

        set_dtb_prop(pram, "reg", 16, &panic_reg);
        DTBNode *chosen = find_dtb_node(tms->device_tree, "chosen");
        set_dtb_prop(chosen, "embedded-panic-log-size", 8,
                     &panic_size);
        tms->panic_base = panic_base;
        tms->panic_size = panic_size;
    }

    DTBNode *vram = find_dtb_node(tms->device_tree, "vram");
    if (vram) {
        uint64_t vram_reg[2] = { 0 };
        uint64_t vram_base = S5L8960X_DISPLAY_BASE;
        uint64_t vram_size = S5L8960X_DISPLAY_SIZE;
        vram_reg[0] = vram_base;
        vram_reg[1] = vram_size;
        set_dtb_prop(vram, "reg", 16, &vram_reg);
    }

    hdr = tms->kernel;
    assert(hdr);

    macho_allocate_segment_records(memory_map, hdr);

    macho_populate_dtb(tms->device_tree, info);

    switch (hdr->filetype) {
    case MH_EXECUTE:
        s5l8960x_load_classic_kc(tms, cmdline);
        break;
    case MH_FILESET:
        s5l8960x_load_fileset_kc(tms, cmdline);
        break;
    default:
        error_setg(&error_abort, "%s: Unsupported kernelcache type: 0x%x\n",
                   __func__, hdr->filetype);                
        break;
    }
}

static void pmgr_unk_reg_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    // hwaddr base = (hwaddr) opaque;
    // fprintf(stderr, "PMGR reg WRITE unk @ 0x" TARGET_FMT_lx " base: 0x" TARGET_FMT_lx " value: 0x" TARGET_FMT_lx "\n", base + addr, base, data);
}

static uint64_t pmgr_unk_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    hwaddr base = (hwaddr) opaque;

    // fprintf(stderr, "PMGR reg READ unk @ 0x" TARGET_FMT_lx " base: 0x" TARGET_FMT_lx "\n", base + addr, base);
    if (((uint64_t)(base + addr) & 0x10e70000) == 0x10e70000) {
        return (108<<4) | 0x200000;
    }

    return 0;
}

static const MemoryRegionOps pmgr_unk_reg_ops = {
    .write = pmgr_unk_reg_write,
    .read = pmgr_unk_reg_read,
};

static void pmgr_reg_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    MachineState *machine = MACHINE(opaque);
    S5L8960XMachineState *tms = S5L8960X_MACHINE(opaque);
    uint32_t value = data;

    if (addr >= 0x80000 && addr <= 0x8c000) {
        value = (value & 0xf) << 4 | (value & 0xf);
    }
    // fprintf(stderr, "PMGR reg WRITE @ 0x" TARGET_FMT_lx " value: 0x" TARGET_FMT_lx "\n", addr, data);
    switch (addr) {
    case 0xd4004:
        s5l8960x_start_cpus(machine, data);
        return;
    }
    memcpy(tms->pmgr_reg + addr, &value, size);
}

static uint64_t pmgr_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(opaque);
    // fprintf(stderr, "PMGR reg READ @ 0x" TARGET_FMT_lx "\n", addr);
    uint64_t result = 0;
    switch(addr) {
    case 0xf0010: /* AppleS5L8960XPMGR::commonSramCheck */
        return 0x5000;
    default:
        break;
    }
    memcpy(&result, tms->pmgr_reg + addr, size);
    return result;
}

static const MemoryRegionOps pmgr_reg_ops = {
    .write = pmgr_reg_write,
    .read = pmgr_reg_read,
};

static void s5l8960x_cpu_setup(MachineState *machine)
{
    unsigned int i;
    DTBNode *root;
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    GList *iter;
    GList *next = NULL;

    root = find_dtb_node(tms->device_tree, "cpus");
    assert(root);

    for (iter = root->child_nodes, i = 0; iter != NULL; iter = next,i++) {
        DTBNode *node;

        next = iter->next;
        node = (DTBNode *)iter->data;
        if (i >= machine->smp.cpus) {
            remove_dtb_node(root, node);
            continue;
        }

        tms->cpus[i] = apple_a7_cpu_create(node);

        qdev_realize(DEVICE(tms->cpus[i]), NULL, &error_fatal);
    }
}

static void s5l8960x_create_aic(MachineState *machine)
{
    unsigned int i;
    hwaddr *reg;
    DTBProp *prop;
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    DTBNode *soc = find_dtb_node(tms->device_tree, "arm-io");
    DTBNode *child;
    DTBNode *timebase;

    assert(soc != NULL);
    child = find_dtb_node(soc, "aic");
    assert(child != NULL);
    timebase = find_dtb_node(soc, "aic-timebase");
    assert(timebase);

    tms->aic = apple_aic_create(machine->smp.cpus, child, timebase);
    object_property_add_child(OBJECT(machine), "aic", OBJECT(tms->aic));
    assert(tms->aic);
    sysbus_realize(tms->aic, &error_fatal);

    prop = find_dtb_prop(child, "reg");
    assert(prop != NULL);

    reg = (hwaddr*)prop->value;

    for(i = 0; i < machine->smp.cpus; i++) {
        memory_region_add_subregion_overlap(&tms->cpus[i]->memory,
                                            tms->soc_base_pa + reg[0],
                                            sysbus_mmio_get_region(tms->aic, i),
                                                                   0);
        sysbus_connect_irq(tms->aic, i,
                           qdev_get_gpio_in(DEVICE(tms->cpus[i]),
                                            ARM_CPU_IRQ));
    }

}

static void s5l8960x_pmgr_setup(MachineState* machine)
{
    uint64_t *reg;
    int i;
    char name[32];
    DTBProp *prop;
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    DTBNode *child = find_dtb_node(tms->device_tree, "arm-io");

    assert(child != NULL);
    child = find_dtb_node(child, "pmgr");
    assert(child != NULL);

    prop = find_dtb_prop(child, "reg");
    assert(prop);

    reg = (uint64_t*)prop->value;

    for(i = 0; i < prop->length / 8; i+=2) {
        MemoryRegion* mem = g_new(MemoryRegion, 1);
        if (i > 0) {
            snprintf(name, 32, "pmgr-unk-reg-%d", i);
            memory_region_init_io(mem, OBJECT(machine), &pmgr_unk_reg_ops, (void*)reg[i], name, reg[i+1]);
        } else {
            memory_region_init_io(mem, OBJECT(machine), &pmgr_reg_ops, tms, "pmgr-reg", reg[i+1]);
        }
        memory_region_add_subregion(tms->sysmem, reg[i] + reg[i+1] < tms->soc_size ? tms->soc_base_pa + reg[i] : reg[i], mem);
    }

    {
        MemoryRegion *mem = g_new(MemoryRegion, 1);

        snprintf(name, 32, "pmp-reg");
        memory_region_init_io(mem, OBJECT(machine), &pmgr_unk_reg_ops, (void*)0x3BC00000, name, 0x60000);
        memory_region_add_subregion(tms->sysmem, tms->soc_base_pa + 0x3BC00000, mem);
    }
    set_dtb_prop(child, "voltage-states0", sizeof(a7_voltage_states0), a7_voltage_states0);
    set_dtb_prop(child, "voltage-states1", sizeof(a7_voltage_states1), a7_voltage_states1);
    set_dtb_prop(child, "bridge-settings", sizeof(a7_bridge_settings), a7_bridge_settings);
}

static void s5l8960x_create_dart(MachineState *machine, const char *name)
{
    AppleDARTState *dart = NULL;
    DTBProp *prop;
    uint64_t *reg;
    uint32_t* ints;
    int i;
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    DTBNode *child = find_dtb_node(tms->device_tree, "arm-io");

    assert(child);
    child = find_dtb_node(child, name);
    if (!child) return;

    dart = apple_dart_create(child);
    assert(dart);
    object_property_add_child(OBJECT(machine), name, OBJECT(dart));

    prop = find_dtb_prop(child, "reg");
    assert(prop);

    reg = (uint64_t *)prop->value;

    for (int i = 0; i < prop->length / 16; i++) {
        sysbus_mmio_map(SYS_BUS_DEVICE(dart), i, tms->soc_base_pa + reg[i*2]);
    }

    prop = find_dtb_prop(child, "interrupts");
    assert(prop);
    ints = (uint32_t*)prop->value;

    for(i = 0; i < prop->length / sizeof(uint32_t); i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dart), i,
                           qdev_get_gpio_in(DEVICE(tms->aic), ints[i]));
    }

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dart), &error_fatal);
}

static void s5l8960x_create_ans(MachineState* machine)
{
    int i;
    uint32_t *ints;
    DTBProp *prop;
    uint64_t *reg;
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    SysBusDevice *ans;
    DTBNode *child = find_dtb_node(tms->device_tree, "arm-io");
    DTBNode *iop_nub;
    struct xnu_iop_segment_range segranges[2] = { 0 };

    assert(child != NULL);
    child = find_dtb_node(child, "ans");
    assert(child != NULL);
    iop_nub = find_dtb_node(child, "iop-ans-nub");
    assert(iop_nub != NULL);

    prop = find_dtb_prop(iop_nub, "region-base");
    *(uint64_t *)prop->value = S5L8960X_ANS_DATA_BASE;

    prop = find_dtb_prop(iop_nub, "region-size");
    *(uint64_t *)prop->value = S5L8960X_ANS_DATA_SIZE;

    set_dtb_prop(iop_nub, "segment-names", 14, "__TEXT;__DATA");

    segranges[0].phys = S5L8960X_ANS_TEXT_BASE;
    segranges[0].virt = 0x0;
    segranges[0].remap = S5L8960X_ANS_TEXT_BASE;
    segranges[0].size = S5L8960X_ANS_TEXT_SIZE;
    segranges[0].flag = 0x1;

    segranges[1].phys = S5L8960X_ANS_DATA_BASE;
    segranges[1].virt = S5L8960X_ANS_TEXT_SIZE;
    segranges[1].remap = S5L8960X_ANS_DATA_BASE;
    segranges[1].size = S5L8960X_ANS_DATA_SIZE;
    segranges[1].flag = 0x0;

    set_dtb_prop(iop_nub, "segment-ranges", 64, segranges);

    ans = apple_ans_create(child, tms->rtbuddyv2_protocol_version);
    assert(ans);

    object_property_add_child(OBJECT(machine), "ans", OBJECT(ans));
    prop = find_dtb_prop(child, "reg");
    assert(prop);
    reg = (uint64_t*)prop->value;

    /*
    0: AppleA7IOP akfRegMap
    1: AppleASCWrapV2 coreRegisterMap
    2: AppleA7IOP autoBootRegMap
    3: NVMe BAR
    */

    for (i = 0; i < 4; i++) {
        sysbus_mmio_map(ans, i, tms->soc_base_pa + reg[i << 1]);
    }

    prop = find_dtb_prop(child, "interrupts");
    assert(prop);
    assert(prop->length == 4 * 4);
    ints = (uint32_t*)prop->value;

    for(i = 0; i < prop->length / sizeof(uint32_t); i++) {
        sysbus_connect_irq(ans, i, qdev_get_gpio_in(DEVICE(tms->aic), ints[i]));
    }

    sysbus_realize_and_unref(ans, &error_fatal);
}

static void s5l8960x_create_gpio(MachineState *machine, const char *name)
{
    DeviceState *gpio = NULL;
    DTBProp *prop;
    uint64_t *reg;
    uint32_t *ints;
    int i;
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    DTBNode *child = find_dtb_node(tms->device_tree, "arm-io");

    child = find_dtb_node(child, name);
    assert(child);
    gpio = apple_gpio_create(child);
    assert(gpio);
    object_property_add_child(OBJECT(machine), name, OBJECT(gpio));

    prop = find_dtb_prop(child, "reg");
    assert(prop);
    reg = (uint64_t*)prop->value;
    sysbus_mmio_map(SYS_BUS_DEVICE(gpio), 0, tms->soc_base_pa + reg[0]);
    prop = find_dtb_prop(child, "interrupts");
    assert(prop);

    ints = (uint32_t*)prop->value;

    for(i = 0; i < prop->length / sizeof(uint32_t); i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(gpio), i, qdev_get_gpio_in(DEVICE(tms->aic), ints[i]));
    }

    sysbus_realize_and_unref(SYS_BUS_DEVICE(gpio), &error_fatal);
}

static void s5l8960x_create_i2c(MachineState *machine, const char *name)
{
    SysBusDevice *i2c = NULL;
    DTBProp *prop;
    uint64_t *reg;
    uint32_t *ints;
    int i;
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    DTBNode *child = find_dtb_node(tms->device_tree, "arm-io");

    child = find_dtb_node(child, name);
    if (!child) return;
    i2c = apple_hw_i2c_create(name);
    assert(i2c);
    object_property_add_child(OBJECT(machine), name, OBJECT(i2c));

    prop = find_dtb_prop(child, "reg");
    assert(prop);
    reg = (uint64_t *)prop->value;
    sysbus_mmio_map(i2c, 0, tms->soc_base_pa + reg[0]);
    prop = find_dtb_prop(child, "interrupts");
    assert(prop);

    ints = (uint32_t*)prop->value;

    for(i = 0; i < prop->length / sizeof(uint32_t); i++) {
        sysbus_connect_irq(i2c, i, qdev_get_gpio_in(DEVICE(tms->aic), ints[i]));
    }

    sysbus_realize_and_unref(i2c, &error_fatal);
}

static void s5l8960x_create_spi(MachineState *machine, uint32_t port)
{
    SysBusDevice *spi = NULL;
    DeviceState *gpio = NULL;
    DTBProp *prop;
    uint64_t *reg;
    uint32_t *ints;
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    DTBNode *child = find_dtb_node(tms->device_tree, "arm-io");
    Object *sio;
    char name[32] = { 0 };
    hwaddr base = tms->soc_base_pa + S5L8960X_SPI_BASE(port);
    uint32_t irq = a7_spi_irqs[port];
    uint32_t cs_pin = a7_spi_cs_pins[port];

    assert(port < S5L8960X_NUM_SPIS);
    snprintf(name, sizeof(name), "spi%d", port);
    child = find_dtb_node(child, name);

    if (child) {
        spi = apple_spi_create(child);
    } else {
        spi = SYS_BUS_DEVICE(qdev_new(TYPE_APPLE_SPI));
        DEVICE(spi)->id = g_strdup(name);
    }
    assert(spi);
    object_property_add_child(OBJECT(machine), name, OBJECT(spi));

    // sio = object_property_get_link(OBJECT(machine), "sio", &error_fatal);
    // assert(object_property_add_const_link(OBJECT(spi), "sio", sio));
    sysbus_realize_and_unref(SYS_BUS_DEVICE(spi), &error_fatal);

    if (child) {
        prop = find_dtb_prop(child, "reg");
        assert(prop);
        reg = (uint64_t*)prop->value;
        base = tms->soc_base_pa + reg[0];

        prop = find_dtb_prop(child, "interrupts");
        assert(prop);
        ints = (uint32_t *)prop->value;
        irq = ints[0];
    }
    sysbus_mmio_map(spi, 0, base);

    /* The second sysbus IRQ is the cs line */
    sysbus_connect_irq(SYS_BUS_DEVICE(spi), 0,
                       qdev_get_gpio_in(DEVICE(tms->aic), irq));

    if (child) {
        prop = find_dtb_prop(child, "function-spi_cs0");
        if (prop) {
            ints = (uint32_t *)prop->value;
            cs_pin = ints[2];
        }
    }
    if (cs_pin != -1) {
        gpio = DEVICE(object_property_get_link(OBJECT(machine), "gpio", &error_fatal));
        assert(gpio);
        qdev_connect_gpio_out(gpio, cs_pin,
                              qdev_get_gpio_in_named(DEVICE(spi),
                                                     SSI_GPIO_CS, 0));
    }
}

static void s5l8960x_create_usb(MachineState *machine)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    DTBNode *child = find_dtb_node(tms->device_tree, "arm-io");
    DTBNode *usbc = find_dtb_node(child, "usb-complex");
    DTBNode *usbd = find_dtb_node(usbc, "usb-device");
    DTBProp *prop;
    DeviceState *atc;
    AppleDARTState *dart;
    IOMMUMemoryRegion *iommu = NULL;
    uint32_t *ints;

    set_dtb_prop(usbd, "device-mac-address", 6, "\xbc\xde\x48\x33\x44\x55");
    set_dtb_prop(usbd, "host-mac-address", 6, "\xbc\xde\x48\x00\x11\x22");

    prop = find_dtb_prop(usbd, "interrupts");
    assert(prop);
    ints = (uint32_t *)prop->value;

#if 0
    for(int i = 0; i < 4; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(atc), i,
                           qdev_get_gpio_in(DEVICE(tms->aic), ints[i]));
    }
    sysbus_connect_irq(SYS_BUS_DEVICE(atc), 4,
                       qdev_get_gpio_in(DEVICE(tms->aic), S5L8960X_DWC2_IRQ));
#endif
}

static void s5l8960x_create_wdt(MachineState *machine)
{
    int i;
    uint32_t *ints;
    DTBProp *prop;
    uint64_t *reg;
    uint32_t value;
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    SysBusDevice *wdt;
    DTBNode *child = find_dtb_node(tms->device_tree, "arm-io");

    assert(child != NULL);
    child = find_dtb_node(child, "wdt");
    assert(child != NULL);

    wdt = apple_wdt_create(child);
    assert(wdt);

    object_property_add_child(OBJECT(machine), "wdt", OBJECT(wdt));
    prop = find_dtb_prop(child, "reg");
    assert(prop);
    reg = (uint64_t*)prop->value;

    /*
    0: reg
    1: scratch reg
    */
    sysbus_mmio_map(wdt, 0, tms->soc_base_pa + reg[0]);
    sysbus_mmio_map(wdt, 1, tms->soc_base_pa + reg[2]);

    prop = find_dtb_prop(child, "interrupts");
    assert(prop);
    ints = (uint32_t*)prop->value;

    for(i = 0; i < prop->length / sizeof(uint32_t); i++) {
        sysbus_connect_irq(wdt, i, qdev_get_gpio_in(DEVICE(tms->aic), ints[i]));
    }

    /* TODO: MCC */
    prop = find_dtb_prop(child, "function-panic_flush_helper");
    if (prop) {
        remove_dtb_prop(child, prop);
    }

    prop = find_dtb_prop(child, "function-panic_halt_helper");
    if (prop) {
        remove_dtb_prop(child, prop);
    }

    value = 1;
    set_dtb_prop(child, "no-pmu", 4, (uint8_t*)&value);

    sysbus_realize_and_unref(wdt, &error_fatal);
}

#if 0
static void s5l8960x_create_pmu(MachineState *machine, const char *parent,
                             const char *name)
{
    DeviceState *pmu = NULL;
    AppleSPMIState *spmi = NULL;
    DTBProp *prop;
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    DTBNode *child = find_dtb_node(tms->device_tree, "arm-io");
    uint32_t *ints;

    assert(child);
    child = find_dtb_node(child, parent);
    if (!child) return;

    spmi = APPLE_SPMI(object_property_get_link(OBJECT(machine), parent,
                      &error_fatal));
    assert(spmi);

    child = find_dtb_node(child, name);
    if (!child) return;

    pmu = apple_spmi_pmu_create(child);
    assert(pmu);
    object_property_add_child(OBJECT(machine), name, OBJECT(pmu));

    prop = find_dtb_prop(child, "interrupts");
    assert(prop);
    ints = (uint32_t *)prop->value;

    qdev_connect_gpio_out(pmu, 0, qdev_get_gpio_in(DEVICE(spmi), ints[0]));
    spmi_slave_realize_and_unref(SPMI_SLAVE(pmu), spmi->bus, &error_fatal);
}
#endif

static void s5l8960x_create_boot_display(MachineState *machine)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    SysBusDevice *fb = NULL;
    MemoryRegion *vram = NULL;
    tms->video.v_baseAddr = S5L8960X_DISPLAY_BASE;
    tms->video.v_rowBytes = 640 * 4;
    tms->video.v_width = 640;
    tms->video.v_height = 1136;
    tms->video.v_depth = 32 | ((2 - 1) << 16);
    tms->video.v_display = 1;

    if (xnu_contains_boot_arg(machine->kernel_cmdline, "-s", false)
        || xnu_contains_boot_arg(machine->kernel_cmdline, "-v", false)) {
        tms->video.v_display = 0;
    }

    fb = SYS_BUS_DEVICE(qdev_new(TYPE_M1_FB));
    object_property_set_uint(OBJECT(fb), "width", 640, &error_fatal);
    object_property_set_uint(OBJECT(fb), "height", 1136, &error_fatal);

    vram = g_new(MemoryRegion, 1);
    memory_region_init_ram(vram, OBJECT(fb), "vram", S5L8960X_DISPLAY_SIZE, &error_fatal);
    memory_region_add_subregion_overlap(tms->sysmem, tms->video.v_baseAddr, vram, 1);

    object_property_add_const_link(OBJECT(fb), "vram", OBJECT(vram));
    object_property_add_child(OBJECT(machine), "fb", OBJECT(fb));

    sysbus_realize_and_unref(fb, &error_fatal);
}

static void s5l8960x_cpu_reset(void *opaque)
{
    MachineState *machine = MACHINE(opaque);
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    CPUState *cpu;

    CPU_FOREACH(cpu) {
        AppleA7State *tcpu = (AppleA7State *)object_dynamic_cast(OBJECT(cpu),
                                                               TYPE_APPLE_A7);
        if (tcpu) {
            object_property_set_int(OBJECT(cpu), "rvbar",
                                    tms->bootinfo.entry & ~0xfff,
                                    &error_abort);
            cpu_reset(cpu);
        }
    }
}

static void s5l8960x_machine_reset(MachineState* machine)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);

    qemu_devices_reset();
    memset(&tms->pmgr_reg, 0, sizeof(tms->pmgr_reg));
    if (!runstate_check(RUN_STATE_RESTORE_VM)
        && !runstate_check(RUN_STATE_PRELAUNCH)) {
        if (!runstate_check(RUN_STATE_PAUSED)
            || qemu_reset_requested_get() != SHUTDOWN_CAUSE_NONE) {
            s5l8960x_memory_setup(MACHINE(tms));
        }
    }
    s5l8960x_cpu_reset(tms);
}

static void s5l8960x_machine_init_done(Notifier *notifier, void *data)
{
    S5L8960XMachineState *tms = container_of(notifier, S5L8960XMachineState,
                                          init_done_notifier);
    s5l8960x_memory_setup(MACHINE(tms));
    s5l8960x_cpu_reset(tms);
}

static void s5l8960x_machine_init(MachineState *machine)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(machine);
    struct mach_header_64 *hdr;
    uint64_t kernel_low = 0, kernel_high = 0;
    uint32_t build_version;
    uint32_t data;
    uint8_t buffer[0x40] = { 0 };
    uint32_t display_rotation = 0;
    uint32_t display_scale = 2;
    DTBNode *child;
    DTBProp *prop;
    hwaddr *ranges;

    tms->sysmem = get_system_memory();
    allocate_ram(tms->sysmem, "DRAM", S5L8960X_DRAM_BASE, S5L8960X_DRAM_SIZE, 0);

    hdr = macho_load_file(machine->kernel_filename);
    assert(hdr);
    tms->kernel = hdr;
    xnu_header = hdr;
    build_version = macho_build_version(hdr);
    fprintf(stderr, "Loading %s %u.%u...\n", macho_platform_string(hdr),
                                             BUILD_VERSION_MAJOR(build_version),
                                             BUILD_VERSION_MINOR(build_version));
    tms->build_version = build_version;

    if (tms->rtbuddyv2_protocol_version == 0) {
        switch (BUILD_VERSION_MAJOR(build_version)) {
            case 13:
                tms->rtbuddyv2_protocol_version = 10;
                break;
            case 14:
                tms->rtbuddyv2_protocol_version = 11;
                break;
            case 15:
            case 16:
                tms->rtbuddyv2_protocol_version = 12;
                break;
            default:
                break;
        }
    }

    macho_highest_lowest(hdr, &kernel_low, &kernel_high);
    fprintf(stderr, "kernel_low: 0x" TARGET_FMT_lx "\n"
                    "kernel_high: 0x" TARGET_FMT_lx "\n",
                    kernel_low, kernel_high);

    g_virt_base = kernel_low;
    g_phys_base = (hwaddr)macho_get_buffer(hdr);

    s5l8960x_patch_kernel(hdr);
puts("kernel patched");
    tms->device_tree = load_dtb_from_file(machine->dtb);

    tms->trustcache = load_trustcache_from_file(tms->trustcache_filename,
                                                &tms->bootinfo.trustcache_size);

    data = 24000000;
    set_dtb_prop(tms->device_tree, "clock-frequency", 4, &data);

    child = find_dtb_node(tms->device_tree, "arm-io");

    assert(child != NULL);

    data = 0x11; /* B1 */
    set_dtb_prop(child, "chip-revision", 4, &data);

    set_dtb_prop(child, "clock-frequencies", sizeof(a7_clock_freq), a7_clock_freq);

    prop = find_dtb_prop(child, "ranges");
    assert(prop != NULL);

    ranges = (hwaddr *)prop->value;
    tms->soc_base_pa = ranges[1];
    tms->soc_size = ranges[2];

    memset(buffer, 0, sizeof(buffer));
    memcpy(buffer, "ME433", 5);
    set_dtb_prop(tms->device_tree, "model-number", 32, buffer);
    memset(buffer, 0, sizeof(buffer));
    memcpy(buffer, "DN/A", 4);
    set_dtb_prop(tms->device_tree, "region-info", 32, buffer);
    memset(buffer, 0, sizeof(buffer));
    set_dtb_prop(tms->device_tree, "config-number", 0x40, buffer);
    memset(buffer, 0, sizeof(buffer));
    memcpy(buffer, "C39ZRMDEN72J", 12);
    set_dtb_prop(tms->device_tree, "serial-number", 32, buffer);
    memset(buffer, 0, sizeof(buffer));
    memcpy(buffer, "C39948108J9N72J1F", 17);
    set_dtb_prop(tms->device_tree, "mlb-serial-number", 32, buffer);
    memset(buffer, 0, sizeof(buffer));
    memcpy(buffer, "A1457", 5);
    set_dtb_prop(tms->device_tree, "regulatory-model-number", 32, buffer);

    child = get_dtb_node(tms->device_tree, "chosen");
    data = 0x8960;
    set_dtb_prop(child, "chip-id", 4, &data);
    data = 0x2;
    set_dtb_prop(child, "board-id", 4, &data);

    uint64_t ecid = 0x1122334455667788;
    set_dtb_prop(child, "unique-chip-id", 8, &ecid);

    /* update the display parameters */
    set_dtb_prop(child, "display-rotation", sizeof(display_rotation),
                    &display_rotation);

    set_dtb_prop(child, "display-scale", sizeof(display_scale),
                    &display_scale);

    child = get_dtb_node(tms->device_tree, "product");
    /* TODO: SEP, iOS 15 data encryption */
    // "iPhone 5s"
    set_dtb_prop(child, "product-name", 8, "FastSim");

    s5l8960x_cpu_setup(machine);

    s5l8960x_create_aic(machine);

    for (int i = 0; i < S5L8960X_NUM_UARTS; i++) {
        s5l8960x_create_s3c_uart(tms, i, serial_hd(i));
    }

    s5l8960x_pmgr_setup(machine);

    s5l8960x_create_ans(machine);

    s5l8960x_create_gpio(machine, "gpio");

    s5l8960x_create_i2c(machine, "i2c0");
    s5l8960x_create_i2c(machine, "i2c1");
    s5l8960x_create_i2c(machine, "i2c2");
    s5l8960x_create_i2c(machine, "i2c3");

    // s5l8960x_create_dart(machine, "dart-scaler");
    // s5l8960x_create_dart(machine, "dart-isp");
    // s5l8960x_create_dart(machine, "dart-jpeg");
    s5l8960x_create_usb(machine);

    s5l8960x_create_wdt(machine);

    for (int i = 0; i < S5L8960X_NUM_SPIS; i++) {
        s5l8960x_create_spi(machine, i);
    }

    s5l8960x_create_boot_display(machine);

    tms->init_done_notifier.notify = s5l8960x_machine_init_done;
    qemu_add_machine_init_done_notifier(&tms->init_done_notifier);
}

static void s5l8960x_set_trustcache_filename(Object *obj, const char *value, Error **errp)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(obj);

    g_free(tms->trustcache_filename);
    tms->trustcache_filename = g_strdup(value);
}

static char *s5l8960x_get_trustcache_filename(Object *obj, Error **errp)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(obj);

    return g_strdup(tms->trustcache_filename);
}

static void s5l8960x_set_ticket_filename(Object *obj, const char *value, Error **errp)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(obj);

    g_free(tms->ticket_filename);
    tms->ticket_filename = g_strdup(value);
}

static char *s5l8960x_get_ticket_filename(Object *obj, Error **errp)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(obj);

    return g_strdup(tms->ticket_filename);
}

static void s5l8960x_set_boot_mode(Object *obj, const char *value, Error **errp)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(obj);

    if (g_str_equal(value, "auto")) {
        tms->boot_mode = kBootModeAuto;
    } else if (g_str_equal(value, "manual")) {
        tms->boot_mode = kBootModeManual;
    } else if (g_str_equal(value, "enter_recovery")) {
        tms->boot_mode = kBootModeEnterRecovery;
    } else if (g_str_equal(value, "exit_recovery")) {
        tms->boot_mode = kBootModeExitRecovery;
    } else {
        tms->boot_mode = kBootModeAuto;
        error_setg(errp, "Invalid boot mode: %s", value);
    }
}

static char *s5l8960x_get_boot_mode(Object *obj, Error **errp)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(obj);

    switch (tms->boot_mode) {
    case kBootModeManual:
        return g_strdup("manual");
    case kBootModeEnterRecovery:
        return g_strdup("enter_recovery");
    case kBootModeExitRecovery:
        return g_strdup("exit_recovery");
    default:
    case kBootModeAuto:
        return g_strdup("auto");
    }
}

static void s5l8960x_get_rtbuddyv2_protocol_version(Object *obj, Visitor *v,
                                                 const char *name, void *opaque,
                                                 Error **errp)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(obj);
    int64_t value = tms->rtbuddyv2_protocol_version;

    visit_type_int(v, name, &value, errp);
}

static void s5l8960x_set_rtbuddyv2_protocol_version(Object *obj, Visitor *v,
                                                 const char *name, void *opaque,
                                                 Error **errp)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(obj);
    int64_t value;

    if (!visit_type_int(v, name, &value, errp)) {
        return;
    }

    tms->rtbuddyv2_protocol_version = value;
}

static void s5l8960x_set_kaslr_off(Object *obj, bool value, Error **errp)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(obj);

    tms->kaslr_off = value;
}

static bool s5l8960x_get_kaslr_off(Object *obj, Error **errp)
{
    S5L8960XMachineState *tms = S5L8960X_MACHINE(obj);

    return tms->kaslr_off;
}

static ram_addr_t s5l8960x_machine_fixup_ram_size(ram_addr_t size)
{
    if (size != S5L8960X_DRAM_SIZE) {
        warn_report("The S5L8960X machine only supports 1 GiB RAM. Overriding");
    }
    return S5L8960X_DRAM_SIZE;
}

static void s5l8960x_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "S5L8960X";
    mc->init = s5l8960x_machine_init;
    mc->reset = s5l8960x_machine_reset;
    mc->max_cpus = A7_MAX_CPU;
    // this disables the error message "Failed to query for block devices!"
    // when starting qemu - must keep at least one device
    mc->no_sdcard = 1;
    mc->no_floppy = 1;
    mc->no_cdrom = 1;
    mc->no_parallel = 1;
    mc->default_cpu_type = TYPE_APPLE_A7;
    mc->minimum_page_bits = 12; /* 4K */
    mc->default_ram_size = S5L8960X_DRAM_SIZE;
    mc->fixup_ram_size = s5l8960x_machine_fixup_ram_size;

    object_class_property_add_str(oc, "trustcache-filename",
                                  s5l8960x_get_trustcache_filename,
                                  s5l8960x_set_trustcache_filename);
    object_class_property_set_description(oc, "trustcache-filename",
                                   "Set the trustcache filename to be loaded");
    object_class_property_add_str(oc, "ticket-filename",
                                  s5l8960x_get_ticket_filename,
                                  s5l8960x_set_ticket_filename);
    object_class_property_set_description(oc, "ticket-filename",
                                    "Set the APTicket filename to be loaded");
    object_class_property_add_str(oc, "boot-mode",
                                  s5l8960x_get_boot_mode,
                                  s5l8960x_set_boot_mode);
    object_class_property_set_description(oc, "boot-mode",
                                    "Set boot mode of the machine");
    object_class_property_add(oc, "rtbuddyv2-protocol-version", "int",
        s5l8960x_get_rtbuddyv2_protocol_version,
        s5l8960x_set_rtbuddyv2_protocol_version,
        NULL, NULL);
    object_class_property_set_description(oc, "rtbuddyv2-protocol-version",
        "Override RTBuddyV2 protocol version");
    object_class_property_add_bool(oc, "kaslr-off",
                                  s5l8960x_get_kaslr_off,
                                  s5l8960x_set_kaslr_off);
    object_class_property_set_description(oc, "kaslr-off",
                                          "Disable KASLR");
}

static const TypeInfo s5l8960x_machine_info = {
    .name = TYPE_S5L8960X_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(S5L8960XMachineState),
    .class_size = sizeof(S5L8960XMachineClass),
    .class_init = s5l8960x_machine_class_init,
};

static void s5l8960x_machine_types(void)
{
    type_register_static(&s5l8960x_machine_info);
}

type_init(s5l8960x_machine_types)
