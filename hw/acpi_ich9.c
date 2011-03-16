/*
 * ACPI implementation
 *
 * Copyright (c) 2006 Fabrice Bellard
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License version 2 as published by the Free Software Foundation.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>
 */
/*
 *  Copyright (c) 2009 Isaku Yamahata <yamahata at valinux co jp>
 *                     VA Linux Systems Japan K.K.
 *
 *  This is based on acpi.c.
 */
#include "hw.h"
#include "pc.h"
#include "pci.h"
#include "qemu-timer.h"
#include "sysemu.h"
#include "acpi.h"

#include "q35.h"

//#define DEBUG

#ifdef DEBUG
#define ICH9_DEBUG(fmt, ...)    do { printf("%s "fmt, __func__, ## __VA_ARGS__); } while (0)
#else
#define ICH9_DEBUG(fmt, ...)    do { } while (0)
#endif

static void pm_ioport_write_fallback(void *opaque, uint32_t addr, int len,
                                     uint32_t val);
static uint32_t pm_ioport_read_fallback(void *opaque, uint32_t addr, int len);

static void pm_update_sci(ICH9_LPCPmRegs *pm)
{
    int sci_level, pm1a_sts;

    pm1a_sts = acpi_pm1_evt_get_sts(&pm->pm1a, pm->tmr.overflow_time);

    sci_level = (((pm1a_sts & pm->pm1a.en) &
                  (ACPI_BITMASK_RT_CLOCK_ENABLE |
                   ACPI_BITMASK_POWER_BUTTON_ENABLE |
                   ACPI_BITMASK_GLOBAL_LOCK_ENABLE |
                   ACPI_BITMASK_TIMER_ENABLE)) != 0);
    qemu_set_irq(pm->irq, sci_level);

    /* schedule a timer interruption if needed */
    acpi_pm_tmr_update(&pm->tmr,
                       (pm->pm1a.en & ACPI_BITMASK_TIMER_ENABLE) &&
                       !(pm1a_sts & ACPI_BITMASK_TIMER_STATUS));
}

static void ich9_pm_update_sci_fn(ACPIPMTimer *tmr)
{
    ICH9_LPCPmRegs *pm = container_of(tmr, ICH9_LPCPmRegs, tmr);
    pm_update_sci(pm);
}

static void pm_ioport_writeb(void *opaque, uint32_t addr, uint32_t val)
{
    ICH9_LPCPmRegs *pm = opaque;

    switch (addr & ICH9_PMIO_MASK) {
    case ICH9_PMIO_GPE0_STS ... (ICH9_PMIO_GPE0_STS + ICH9_PMIO_GPE0_LEN - 1):
        acpi_gpe_ioport_writeb(&pm->gpe0, addr, val);
        break;
    default:
        break;
    }

    ICH9_DEBUG("port=0x%04x val=0x%04x\n", addr, val);
}

static uint32_t pm_ioport_readb(void *opaque, uint32_t addr)
{
    ICH9_LPCPmRegs *pm = opaque;
    uint32_t val = 0;

    switch(addr & ICH9_PMIO_MASK) {
    case ICH9_PMIO_GPE0_STS ... (ICH9_PMIO_GPE0_STS + ICH9_PMIO_GPE0_LEN - 1):
        val = acpi_gpe_ioport_readb(&pm->gpe0, addr);
        break;
    default:
        val = 0;
        break;
    }
    ICH9_DEBUG("port=0x%04x val=0x%04x\n", addr, val);
    return val;
}

static void pm_ioport_writew(void *opaque, uint32_t addr, uint32_t val)
{
    ICH9_LPCPmRegs *pm = opaque;

    switch(addr & ICH9_PMIO_MASK) {
    case ICH9_PMIO_PM1_STS:
        acpi_pm1_evt_write_sts(&pm->pm1a, &pm->tmr, val);
        pm_update_sci(pm);
        break;
    case ICH9_PMIO_PM1_EN:
        pm->pm1a.en = val;
        pm_update_sci(pm);
        break;
    case ICH9_PMIO_PM1_CNT:
        acpi_pm1_cnt_write(&pm->pm1a, &pm->pm1_cnt, val);
        break;
    default:
        pm_ioport_write_fallback(opaque, addr, 2, val);
        break;
    }
    ICH9_DEBUG("port=0x%04x val=0x%04x\n", addr, val);
}

static uint32_t pm_ioport_readw(void *opaque, uint32_t addr)
{
    ICH9_LPCPmRegs *pm = opaque;
    uint32_t val;

    switch(addr & ICH9_PMIO_MASK) {
    case ICH9_PMIO_PM1_STS:
        val = acpi_pm1_evt_get_sts(&pm->pm1a, pm->tmr.overflow_time);
        break;
    case ICH9_PMIO_PM1_EN:
        val = pm->pm1a.en;
        break;
    case ICH9_PMIO_PM1_CNT:
        val = pm->pm1_cnt.cnt;
        break;
    default:
        val = pm_ioport_read_fallback(opaque, addr, 2);
        break;
    }
    ICH9_DEBUG("port=0x%04x val=0x%04x\n", addr, val);
    return val;
}

static void pm_ioport_writel(void *opaque, uint32_t addr, uint32_t val)
{
    ICH9_LPCPmRegs *pm = opaque;

    switch(addr & ICH9_PMIO_MASK) {
    case ICH9_PMIO_SMI_EN:
        pm->smi_en = val;
        break;
    default:
        pm_ioport_write_fallback(opaque, addr, 4, val);
        break;
    }
    ICH9_DEBUG("port=0x%04x val=0x%08x\n", addr, val);
}

static uint32_t pm_ioport_readl(void *opaque, uint32_t addr)
{
    ICH9_LPCPmRegs *pm = opaque;
    uint32_t val;

    switch(addr & ICH9_PMIO_MASK) {
    case ICH9_PMIO_PM1_TMR:
        val = acpi_pm_tmr_get(&pm->tmr);
        break;
    case ICH9_PMIO_SMI_EN:
        val = pm->smi_en;
        break;

    default:
        val = pm_ioport_read_fallback(opaque, addr, 4);
        break;
    }
    ICH9_DEBUG("port=0x%04x val=0x%08x\n", addr, val);
    return val;
}

static void pm_ioport_write_fallback(void *opaque, uint32_t addr, int len,
                                     uint32_t val)
{
    int subsize = (len == 4)? 2: 1;
    IOPortWriteFunc *ioport_write =
        (subsize == 2)? pm_ioport_writew: pm_ioport_writeb;

    int i;

    for (i = 0; i < len; i += subsize) {
        ioport_write(opaque, addr, val);
        val >>= 8 * subsize;
    }
}

static uint32_t pm_ioport_read_fallback(void *opaque, uint32_t addr, int len)
{
    int subsize = (len == 4)? 2: 1;
    IOPortReadFunc *ioport_read =
        (subsize == 2)? pm_ioport_readw: pm_ioport_readb;

    uint32_t val;
    int i;

    val = 0;
    for (i = 0; i < len; i += subsize) {
        val <<= 8 * subsize;
        val |= ioport_read(opaque, addr);
    }

    return val;
}

void ich9_pm_iospace_update(ICH9_LPCPmRegs *pm, uint32_t pm_io_base)
{
    ICH9_DEBUG("to 0x%x\n", pm_io_base);

    assert((pm_io_base & ICH9_PMIO_MASK) == 0);

    if (pm->pm_io_base != 0) {
        isa_unassign_ioport(pm->pm_io_base, ICH9_PMIO_SIZE);
    }

    /* don't map at 0 */
    if (pm_io_base == 0) {
        return;
    }

    register_ioport_write(pm_io_base, ICH9_PMIO_SIZE, 1, pm_ioport_writeb, pm);
    register_ioport_read(pm_io_base, ICH9_PMIO_SIZE, 1, pm_ioport_readb, pm);
    register_ioport_write(pm_io_base, ICH9_PMIO_SIZE, 2, pm_ioport_writew, pm);
    register_ioport_read(pm_io_base, ICH9_PMIO_SIZE, 2, pm_ioport_readw, pm);
    register_ioport_write(pm_io_base, ICH9_PMIO_SIZE, 4, pm_ioport_writel, pm);
    register_ioport_read(pm_io_base, ICH9_PMIO_SIZE, 4, pm_ioport_readl, pm);

    pm->pm_io_base = pm_io_base;
    acpi_gpe_blk(&pm->gpe0, pm_io_base + ICH9_PMIO_GPE0_STS);
}

static int ich9_pm_post_load(void *opaque, int version_id)
{
    ICH9_LPCPmRegs *pm = opaque;
    uint32_t pm_io_base = pm->pm_io_base;
    pm->pm_io_base = 0;
    ich9_pm_iospace_update(pm, pm_io_base);
    return 0;
}

#define VMSTATE_GPE_ARRAY(_field, _state)                            \
 {                                                                   \
     .name       = (stringify(_field)),                              \
     .version_id = 0,                                                \
     .num        = ICH9_PMIO_GPE0_LEN,                               \
     .info       = &vmstate_info_uint8,                              \
     .size       = sizeof(uint8_t),                                  \
     .flags      = VMS_ARRAY | VMS_POINTER,                          \
     .offset     = vmstate_offset_pointer(_state, _field, uint8_t),  \
 }

const VMStateDescription vmstate_ich9_pm = {
    .name = "ich9_pm",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = ich9_pm_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16(pm1a.sts, ICH9_LPCPmRegs),
        VMSTATE_UINT16(pm1a.en, ICH9_LPCPmRegs),
        VMSTATE_UINT16(pm1_cnt.cnt, ICH9_LPCPmRegs),
        VMSTATE_TIMER(tmr.timer, ICH9_LPCPmRegs),
        VMSTATE_INT64(tmr.overflow_time, ICH9_LPCPmRegs),
        VMSTATE_GPE_ARRAY(gpe0.sts, ICH9_LPCPmRegs),
        VMSTATE_GPE_ARRAY(gpe0.en, ICH9_LPCPmRegs),
        VMSTATE_UINT32(smi_en, ICH9_LPCPmRegs),
        VMSTATE_UINT32(smi_sts, ICH9_LPCPmRegs),
        VMSTATE_END_OF_LIST()
    }
};

static void pm_reset(void *opaque)
{
    ICH9_LPCPmRegs *pm = opaque;
    ich9_pm_iospace_update(pm, 0);

    acpi_pm1_evt_reset(&pm->pm1a);
    acpi_pm1_cnt_reset(&pm->pm1_cnt);
    acpi_pm_tmr_reset(&pm->tmr);
    acpi_gpe_reset(&pm->gpe0);

    pm_update_sci(pm);
}

static void pm_powerdown(void *opaque, int irq, int power_failing)
{
    ICH9_LPCPmRegs *pm = opaque;
    ACPIPM1EVT *pm1a = pm? &pm->pm1a: NULL;
    ACPIPMTimer *tmr = pm? &pm->tmr: NULL;

    acpi_pm1_evt_power_down(pm1a, tmr);
}

void ich9_pm_init(ICH9_LPCPmRegs *pm, qemu_irq sci_irq, qemu_irq cmos_s3)
{
    acpi_pm_tmr_init(&pm->tmr, ich9_pm_update_sci_fn);
    acpi_pm1_cnt_init(&pm->pm1_cnt, cmos_s3);
    acpi_gpe_init(&pm->gpe0, ICH9_PMIO_GPE0_LEN);

    pm->irq = sci_irq;
    qemu_register_reset(pm_reset, pm);
    qemu_system_powerdown = *qemu_allocate_irqs(pm_powerdown, pm, 1);
}
