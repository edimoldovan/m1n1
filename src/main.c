/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "../build/build_cfg.h"
#include "../build/build_tag.h"

#include "../config.h"

#include "adt.h"
#include "aic.h"
#include "cpufreq.h"
#include "display.h"
#include "exception.h"
#include "fb.h"
#include "firmware.h"
#include "gxf.h"
#include "heapblock.h"
#include "mcc.h"
#include "memory.h"
#include "nvme.h"
#include "payload.h"
#include "pcie.h"
#include "pmgr.h"
#include "sep.h"
#include "smp.h"
#include "string.h"
#include "tps6598x.h"
#include "uart.h"
#include "uartproxy.h"
#include "usb.h"
#include "utils.h"
#include "wdt.h"
#include "xnuboot.h"

#define CSR_UNAUTHENTICATED_ROOT BIT(11)

struct vector_args next_stage;

const char version_tag[] = "##m1n1_ver##" BUILD_TAG;
const char *const m1n1_version = version_tag + 12;

u32 board_id = ~0, chip_id = ~0;

void get_device_info(void)
{
    const char *model = (const char *)adt_getprop(adt, 0, "model", NULL);
    const char *target = (const char *)adt_getprop(adt, 0, "target-type", NULL);

    printf("Device info:\n");

    if (model)
        printf("  Model: %s\n", model);

    if (target)
        printf("  Target: %s\n", target);

    is_mac = !!strstr(model, "Mac");

    int chosen = adt_path_offset(adt, "/chosen");
    if (chosen > 0) {
        if (ADT_GETPROP(adt, chosen, "board-id", &board_id) < 0)
            printf("Failed to find board-id\n");
        if (ADT_GETPROP(adt, chosen, "chip-id", &chip_id) < 0)
            printf("Failed to find chip-id\n");

        printf("  Board-ID: 0x%x\n", board_id);
        printf("  Chip-ID: 0x%x\n", chip_id);
    } else {
        printf("No chosen node!\n");
    }

    printf("\n");
}

void run_actions(__maybe_unused u64 lp_sip0)
{
    bool usb_up = false;

#ifndef BRINGUP
#ifdef EARLY_PROXY_TIMEOUT
    if (!cur_boot_args.video.display && lp_sip0 == 127) {
        printf("Bringing up USB for early debug...\n");

        usb_init();
        usb_iodev_init();

        usb_up = true;

        printf("Waiting for proxy connection... ");
        for (int i = 0; i < EARLY_PROXY_TIMEOUT * 100; i++) {
            for (int j = 0; j < USB_IODEV_COUNT; j++) {
                iodev_id_t iodev = IODEV_USB0 + j;

                if (!(iodev_get_usage(iodev) & USAGE_UARTPROXY))
                    continue;

                usb_iodev_vuart_setup(iodev);
                iodev_handle_events(iodev);
                if (iodev_can_write(iodev) || iodev_can_write(IODEV_USB_VUART)) {
                    printf(" Connected!\n");
                    uartproxy_run(NULL);
                    return;
                }
            }

            mdelay(10);
            if (i % 100 == 99)
                printf(".");
        }
        printf(" Timed out\n");
    }
#endif
#endif

    printf("Checking for payloads...\n");

    if (payload_run() == 0) {
        printf("Valid payload found\n");
        return;
    }
    fb_set_active(true);

    printf("No valid payload found\n");

#ifndef BRINGUP
    if (!usb_up) {
        usb_init();
        usb_iodev_init();
    }
#endif

    printf("Running proxy...\n");

    uartproxy_run(NULL);
}

/*
 * fbcrumb: J714c boot crumbs, white bands in the iBoot framebuffer, same
 * geometry as linux-aurora's asm/fbcrumb.h. Band 20 before mmu_shutdown,
 * band 21 right before the jump to the next stage. T6050 only. Each crumb
 * also arms the WDT: the reset-to-proxy time tells the host the last
 * milestone reached (none 20 s, band 20 at 40 s, band 21 at 60 s, kernel 80 s and up).
 */
#define FBCRUMB_BASE   0x11fd31e6880ULL
#define FBCRUMB_STRIDE 0x2f40
#define FBCRUMB_BYTES  (16 * FBCRUMB_STRIDE)
#define CRUMB_RAM      0x10800000000ULL /* last milestone, read back over the proxy after the reset */
#define CRUMB_MAGIC    0xC0FFEE0000000000ULL
#define CRUMB_MARK     0x4D314E3100000000ULL /* m1n1 reached the handoff */

static void fbcrumb(int band, u32 wdt_seconds)
{
    if (chip_id != T6050)
        return;

    if (band >= 0) {
        volatile u64 *p = (volatile u64 *)(FBCRUMB_BASE + (u64)band * 60 * FBCRUMB_STRIDE);

        for (size_t i = 0; i < FBCRUMB_BYTES / 8; i++)
            p[i] = ~0ULL;
        sysop("dsb sy");
    }
    write64(CRUMB_RAM, CRUMB_MAGIC | (band & 0xff));
    write64(CRUMB_RAM + 8, CRUMB_MARK);
    sysop("dsb sy");
    if (wdt_seconds)
        wdt_arm(wdt_seconds);
}

void m1n1_main(void)
{
    u64 lp_sip0 = 0;

    printf("\n\nm1n1 %s\n", m1n1_version);
    printf("Copyright The Asahi Linux Contributors\n");
    printf("Licensed under the GNU General Public License v2 or later\n\n");

    printf("Running in EL%lu\n\n", mrs(CurrentEL) >> 2);

    firmware_init();

    heapblock_init();

#ifndef BRINGUP
    int node = adt_path_offset(adt, "/chosen/asmb");
    if (node >= 0) {
        ADT_GETPROP(adt, node, "lp-sip0", &lp_sip0);
        printf("Boot policy: sip0 = %ld\n", lp_sip0);
    }

    // csrutil authenticated-root disable
    if ((lp_sip0 & CSR_UNAUTHENTICATED_ROOT) || USE_DEBUG_USB)
        usb_enable_debugusb();

    if (supports_gxf())
        gxf_init();
    mcc_init();
    mmu_init();
    aic_init();
    smp_init();
#endif
    wdt_disable();
#ifndef BRINGUP
    pmgr_init();
#ifdef USE_FB
    display_init();
    // Kick DCP to sleep, so dodgy monitors which cause reconnect cycles don't cause us to lose the
    // framebuffer.
    display_shutdown(DCP_SLEEP_IF_EXTERNAL);
    // On idevice we need to always clear, because otherwise it looks scuffed on white devices
    fb_init(!is_mac);
    fb_display_logo();
#ifdef FB_SILENT_MODE
    fb_set_active(!cur_boot_args.video.display);
#else
    fb_set_active(true);
#endif
#endif

    cpufreq_fixup();
    sep_init();
#endif

    printf("Initialization complete.\n");

    run_actions(lp_sip0);

    if (!next_stage.entry) {
        panic("Nothing to do!\n");
    }

    printf("Preparing to run next stage at %p...\n", next_stage.entry);
    fbcrumb(-1, 0);

    nvme_shutdown();
    exception_shutdown();
#ifndef BRINGUP
    usb_iodev_shutdown();
    display_shutdown(DCP_SLEEP_IF_EXTERNAL);
#ifdef USE_FB
    fb_shutdown(next_stage.restore_logo);
#endif
    fbcrumb(20, 0);
    mmu_shutdown();
    fbcrumb(21, 0);
#endif

    printf("Vectoring to next stage...\n");

    next_stage.entry(next_stage.args[0], next_stage.args[1], next_stage.args[2], next_stage.args[3],
                     next_stage.args[4]);

    panic("Next stage returned!\n");
}
