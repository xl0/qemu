/*
 * Copyright (c) 2006 Fabrice Bellard
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
/*
 * QEMU GMCH/ICH9 PCI Bridge Emulation
 *
 *  Copyright (c) 2009, 2010, 2011
 *                Isaku Yamahata <yamahata at valinux co jp>
 *                VA Linux Systems Japan K.K.
 *
 *  This is based on piix_pci.c, but heavily modified.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>
 */

#include "hw.h"
#include "range.h"
#include "isa.h"
#include "sysbus.h"
#include "pc.h"
#include "apm.h"
#include "apic.h"
#include "pci.h"
#include "pcie_host.h"
#include "pci_bridge.h"
#include "pci_p2pbr.h"
#include "q35.h"
#include "acpi.h"
#include "acpi_ich9.h"
#include "pam.h"


struct ICH9_LPCState;

typedef struct ICH9_LPCIrqState {
    struct ICH9_LPCState *lpc;
    qemu_irq *pic;
    qemu_irq *ioapic;
} ICH9_LPCIrqState;

typedef struct GMCH_PCIHost {
    PCIExpressHost      host;

    PCIDevice    *dev;
    ICH9_LPCIrqState irq_state;
} GMCH_PCIHost;

typedef struct GMCH_PCIState {
    PCIDevice   d;
    /*
     * GMCH_PCIHost   *gmch_host;
     * In order to get GMCH_PCIHost
     *  PCIDevice -> qdev -> parent_bus -> qdev -upcast-> GMCH_PCIHost
     */

    PAM pam;
} GMCH_PCIState;

typedef struct ICH9_LPCState {
    /* ICH9 LPC PCI to ISA bridge */
    PCIDevice d;

    /* (pci device, intx) -> pirq
     * In real chipset case, the unused slots are never used
     * as ICH9 supports only D25-D32 irq routing.
     * On the other hand in qemu case, any slot/function can be populated
     * via command line option.
     * So fallback interrupt routing for any devices in any slots is necessary.
     */
    uint8_t irr[PCI_SLOT_MAX][PCI_NUM_PINS];

    APMState apm;
    ICH9_LPCPmRegs pm;
    uint32_t sci_level; /* track sci level */

    /* 10.1 Chipset Configuration registers(Memory Space)
       which is pointed by RCBA */
    uint8_t chip_config[ICH9_CC_SIZE];
    int rbca_index;
} ICH9_LPCState;


/****************************************************************************
 * GMCH PCI host
 */
/* ich9 irq */
static int ich9_lpc_map_irq(void *opaque, PCIDevice *pci_dev, int intx);
static void ich9_lpc_set_irq(void *opaque, int irq_num, int level);
static int ich9_lpc_sci_irq(ICH9_LPCState *lpc);

static GMCH_PCIHost *gmch_pcihost_from_qdev(DeviceState *gmch_host_qdev)
{
    SysBusDevice *sysdev = sysbus_from_qdev(gmch_host_qdev);
    PCIHostState *pci = FROM_SYSBUS(PCIHostState, sysdev);
    PCIExpressHost *pcie = DO_UPCAST(PCIExpressHost, pci, pci);
    return DO_UPCAST(GMCH_PCIHost, host, pcie);
}

static int gmch_pcihost_initfn(SysBusDevice *dev)
{
    GMCH_PCIHost *s = gmch_pcihost_from_qdev(&dev->qdev);

    pci_host_conf_register_ioport(GMCH_HOST_BRIDGE_CONFIG_ADDR, &s->host.pci);
    pci_host_data_register_ioport(GMCH_HOST_BRIDGE_CONFIG_DATA, &s->host.pci);

    if (pcie_host_init(&s->host) < 0) {
        abort();
    }

    return 0;
}

static SysBusDeviceInfo gmch_pcihost_info = {
    .init         = gmch_pcihost_initfn,
    .qdev.name    = "gmch-pcihost",
    .qdev.size    = sizeof(GMCH_PCIHost),
    .qdev.no_user = 1,
    .qdev.props = (Property[]) {
        {
            .name = "MCFG",
            .info = &qdev_prop_uint64,
            .offset = offsetof(GMCH_PCIHost, host.base_addr),
            .defval = (uint64_t[]){ GMCH_HOST_BRIDGE_PCIEXBAR_DEFAULT },
        },
        DEFINE_PROP_END_OF_LIST(),
    },
};

/* host bridge */
PCIBus *gmch_host_init(DeviceState **gmch_hostp,
                       qemu_irq *pic, qemu_irq *ioapic)
{
    DeviceState *dev;
    GMCH_PCIHost *s;
    PCIBus *b;

    dev = qdev_create(NULL, "gmch-pcihost");
    s = gmch_pcihost_from_qdev(dev);
    s->irq_state.pic = pic;
    s->irq_state.ioapic = ioapic;

    b = pci_bus_new(dev, "pcie.0", 0);
    pci_bus_irqs(b, ich9_lpc_set_irq, ich9_lpc_map_irq, &s->irq_state,
                 ICH9_LPC_NB_PIRQS);
    s->host.pci.bus = b;
    qdev_init_nofail(dev);

    *gmch_hostp = dev;
    return b;
}


/****************************************************************************
 * GMCH
 */
static GMCH_PCIState *gmch_from_pci(PCIDevice *gmch_pci)
{
    return DO_UPCAST(GMCH_PCIState, d, gmch_pci);
}

/* PCIE MMCFG */
static void gmch_update_pciexbar(GMCH_PCIState *gs)
{
    PCIDevice* pci_dev = &gs->d;
    BusState *bus = qdev_get_parent_bus(&pci_dev->qdev);
    DeviceState *qdev = bus->parent;
    GMCH_PCIHost *s = gmch_pcihost_from_qdev(qdev);

    uint64_t pciexbar;
    int enable;
    uint64_t addr;
    uint64_t addr_mask;
    uint32_t length;

    pciexbar = pci_get_quad(pci_dev->config + GMCH_HOST_BRIDGE_PCIEXBAR);
    enable = pciexbar & GMCH_HOST_BRIDGE_PCIEXBAREN;

    addr_mask = GMCH_HOST_BRIDGE_PCIEXBAR_ADMSK;
    switch (pciexbar & GMCH_HOST_BRIDGE_PCIEXBAR_LENGTH_MASK) {
    case GMCH_HOST_BRIDGE_PCIEXBAR_LENGTH_256M:
        length = 256 * 1024 * 1024;
        break;
    case GMCH_HOST_BRIDGE_PCIEXBAR_LENGTH_128M:
        length = 128 * 1024 * 1024;
        addr_mask |= GMCH_HOST_BRIDGE_PCIEXBAR_128ADMSK |
            GMCH_HOST_BRIDGE_PCIEXBAR_64ADMSK;
        break;
    case GMCH_HOST_BRIDGE_PCIEXBAR_LENGTH_64M:
        length = 64 * 1024 * 1024;
        addr_mask |= GMCH_HOST_BRIDGE_PCIEXBAR_64ADMSK;
        break;
    case GMCH_HOST_BRIDGE_PCIEXBAR_LENGTH_RVD:
    default:
        enable = 0;
        length = 0;
        abort();
        break;
    }
    addr = pciexbar & addr_mask;

    pcie_host_mmcfg_update(&s->host, enable, addr, length);
}

/* PAM */
static void gmch_update_pam(GMCH_PCIState *gs)
{
    int i;
    for (i = 0; i <= PAM_IDX_MAX; i++) {
        pam_update(&gs->pam, i, gs->d.config[GMCH_HOST_BRIDGE_PAM0 + i]);
    }
}

/* SMRAM */
static void gmch_update_smram(GMCH_PCIState *gs)
{
    smram_update(&gs->pam, gs->d.config[GMCH_HOST_BRDIGE_SMRAM]);
}

static void gmch_set_smm(int smm, void *arg)
{
    GMCH_PCIState *gs = arg;
    smram_set_smm(&gs->pam, smm, gs->d.config[GMCH_HOST_BRDIGE_SMRAM]);
}

static void gmch_write_config(PCIDevice *d,
                              uint32_t address, uint32_t val, int len)
{
    GMCH_PCIState *gs = gmch_from_pci(d);

    /* XXX: implement SMRAM.D_LOCK */
    pci_default_write_config(d, address, val, len);

    if (ranges_overlap(address, len, GMCH_HOST_BRIDGE_PAM0,
                       GMCH_HOST_BRIDGE_PAM_SIZE)) {
        gmch_update_pam(gs);
    }

    if (ranges_overlap(address, len, GMCH_HOST_BRIDGE_PCIEXBAR,
                       GMCH_HOST_BRIDGE_PCIEXBAR_SIZE)) {
        gmch_update_pciexbar(gs);
    }

    if (ranges_overlap(address, len, GMCH_HOST_BRDIGE_SMRAM,
                       GMCH_HOST_BRDIGE_SMRAM_SIZE)) {
        gmch_update_smram(gs);
    }
}

static void gmch_update(GMCH_PCIState *gs)
{
    gmch_update_pciexbar(gs);
    gmch_update_pam(gs);
    gmch_update_smram(gs);
}

static int gmch_post_load(void *opaque, int version_id)
{
    GMCH_PCIState *gs = opaque;
    gmch_update(gs);
    return 0;
}

static const VMStateDescription vmstate_gmch = {
    .name = "gmch",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = gmch_post_load,
    .fields = (VMStateField []) {
        VMSTATE_PCI_DEVICE(d, GMCH_PCIState),
        VMSTATE_UINT8(pam.smm_enabled, GMCH_PCIState),
        VMSTATE_END_OF_LIST()
    }
};

static void gmch_reset(DeviceState *qdev)
{
    PCIDevice *d = DO_UPCAST(PCIDevice, qdev, qdev);
    GMCH_PCIState *gs = gmch_from_pci(d);

    pci_set_quad(d->config + GMCH_HOST_BRIDGE_PCIEXBAR,
                 GMCH_HOST_BRIDGE_PCIEXBAR_DEFAULT);

    d->config[GMCH_HOST_BRDIGE_SMRAM] = GMCH_HOST_BRIDGE_SMRAM_DEFAULT;

    gmch_update(gs);
}

static int gmch_initfn(PCIDevice *d)
{
    GMCH_PCIState *gs = gmch_from_pci(d);

    pci_config_set_vendor_id(d->config, PCI_VENDOR_ID_INTEL);
    pci_config_set_device_id(d->config, PCI_DEVICE_ID_INTEL_Q35_MCH);
    pci_config_set_revision(d->config, GMCH_HOST_BRIDGE_REVISION_DEFUALT);
    pci_config_set_class(d->config, PCI_CLASS_BRIDGE_HOST);

    cpu_smm_register(&gmch_set_smm, gs);
    pam_init_memory_mappings(&gs->pam);

    return 0;
}

static PCIDeviceInfo gmch_info = {
    .qdev.name    = "gmch",
    .qdev.desc    = "Host bridge",
    .qdev.size    = sizeof(GMCH_PCIState),
    .qdev.vmsd    = &vmstate_gmch,
    .qdev.no_user = 1,
    .init         = gmch_initfn,
    .config_write = gmch_write_config,
    .qdev.reset   = gmch_reset,
};

/* host bridge */
PCIDevice *gmch_init(DeviceState *gmch_host, PCIBus *b)
{
    GMCH_PCIHost *s = gmch_pcihost_from_qdev(gmch_host);
    PCIDevice *d;

    d = pci_create_simple_multifunction(b, 0, false, "gmch");
    s->dev = d;

    return d;
}

/*****************************************************************************/
/* ICH9 DMI-to-PCI bridge */
#define I82801ba_SSVID_OFFSET   0x50
#define I82801ba_SSVID_SVID     0
#define I82801ba_SSVID_SSID     0

static PCIBridge *i82801ba11_init(PCIBus *bus, int devfn, const char *bus_name,
                                  bool multifunction)
{
    const PCIP2PBridgeInit init = {
        .bus = bus,
        .devfn = devfn,
        .multifunction = multifunction,

        .bus_name = bus_name,
        .map_irq = pci_swizzle_map_irq_fn,
    };
    const PCIP2PBridgeProp prop = {
        .vendor_id = PCI_VENDOR_ID_INTEL,
        .device_id = PCI_DEVICE_ID_INTEL_82801BA_11,
        .revision_id = ICH9_D2P_A2_REVISION,
        .prog_interface = PCI_CLASS_BRDIGE_PCI_INF_SUB,

        .ssvid_cap = I82801ba_SSVID_OFFSET,
        .svid = I82801ba_SSVID_SVID,
        .ssid = I82801ba_SSVID_SSID,
    };
    return pci_p2pbr_create_simple(&init, &prop);
}

PCIBus *ich9_d2pbr_init(PCIBus *bus, int devfn, int sec_bus)
{
    PCIBridge *br;
    char buf[16];

    snprintf(buf, sizeof(buf), "pci.%d", sec_bus);
    br = i82801ba11_init(bus, devfn, buf, true);
    if (br == NULL) {
        return NULL;
    }
    return pci_bridge_get_sec_bus(br);
}


/*****************************************************************************/
/* ICH9 LPC PCI to ISA bridge */

static void ich9_lpc_reset(DeviceState *qdev);

static ICH9_LPCState *ich9_lpc_from_pci(PCIDevice *lpc_pci)
{
    return DO_UPCAST(ICH9_LPCState, d, lpc_pci);
}

/* chipset configuration register
 * to access chipset configuration registers, pci_[sg]et_{byte, word, long}
 * are used.
 * Although it's not pci configuration space, it's little endian as Intel.
 */

static void ich9_cc_update_ir(uint8_t irr[PCI_NUM_PINS], uint32_t ir)
{
    int intx;
    for (intx = 0; intx < PCI_NUM_PINS; intx++) {
        irr[intx] = (ir >> (intx * ICH9_CC_DIR_SHIFT)) & ICH9_CC_DIR_MASK;
    }
}

static void ich9_cc_update(ICH9_LPCState *lpc)
{
    int slot;
    int reg_offset;
    int intx;

    /* D{25 - 31}IR, but D30IR is read only to 0. */
    for (slot = 25, reg_offset = 0; slot < 32; slot++, reg_offset++) {
        if (slot != 30) {
            ich9_cc_update_ir(lpc->irr[slot],
                              lpc->chip_config[ICH9_CC_D31IR + reg_offset]);
        }
    }

    /*
     * D30: DMI2PCI bridge
     * It is arbitrarily decided how INTx lines of PCI devicesbehind the bridge
     * are connected to pirq lines. Our choice is PIRQ[E-H].
     * INT[A-D] are connected to PIRQ[E-H]
     */
    for (intx = 0; intx < PCI_NUM_PINS; intx++) {
        lpc->irr[30][intx] = intx + 4;
    }
}

static void ich9_cc_init(ICH9_LPCState *lpc)
{
    int slot;
    int intx;

    /* the default irq routing is arbitrary as long as it matches with
     * acpi irq routing table.
     * The one that is incompatible with piix_pci(= bochs) one is
     * intentionally chosen to let the users know that the different
     * board is used.
     *
     * int[A-D] -> pirq[E-F]
     * avoid pirq A-D because they are used for pci express port
     */
    for (slot = 0; slot < PCI_SLOT_MAX; slot++) {
        for (intx = 0; intx < PCI_NUM_PINS; intx++) {
            lpc->irr[slot][intx] = (slot + intx) % 4 + 4;
        }
    }
    ich9_cc_update(lpc);
}

static void ich9_cc_reset(ICH9_LPCState *lpc)
{
    uint8_t *c = lpc->chip_config;

    memset(lpc->chip_config, 0, sizeof(lpc->chip_config));

    pci_set_long(c + ICH9_CC_D31IR, ICH9_CC_DIR_DEFAULT);
    pci_set_long(c + ICH9_CC_D30IR, ICH9_CC_D30IR_DEFAULT);
    pci_set_long(c + ICH9_CC_D29IR, ICH9_CC_DIR_DEFAULT);
    pci_set_long(c + ICH9_CC_D28IR, ICH9_CC_DIR_DEFAULT);
    pci_set_long(c + ICH9_CC_D27IR, ICH9_CC_DIR_DEFAULT);
    pci_set_long(c + ICH9_CC_D26IR, ICH9_CC_DIR_DEFAULT);
    pci_set_long(c + ICH9_CC_D25IR, ICH9_CC_DIR_DEFAULT);

    ich9_cc_update(lpc);
}

static void ich9_cc_addr_len(uint32_t *addr, int *len)
{
    *addr &= ICH9_CC_ADDR_MASK;
    if (*addr + *len >= ICH9_CC_SIZE) {
        *len = ICH9_CC_SIZE - *addr;
    }
}

/* val: little endian */
static void ich9_cc_write(ICH9_LPCState *lpc, uint32_t addr,
                          uint32_t val, int len)
{
    ich9_cc_addr_len(&addr, &len);
    memcpy(lpc->chip_config + addr, &val, len);
}

/* return value: little endian */
static uint32_t ich9_cc_read(ICH9_LPCState *lpc, uint32_t addr, int len)
{
    uint32_t val = 0;
    ich9_cc_addr_len(&addr, &len);
    memcpy(&val, lpc->chip_config + addr, len);
    return val;
}

#define ICH9_CC_MMIO_WRITE(type, len)                           \
    static void ich9_cc_mmio_write ## type                      \
    (void *opaque, target_phys_addr_t addr, uint32_t val)       \
    {                                                           \
        ich9_cc_write(opaque, addr, val, len);                  \
    }

#define ICH9_CC_MMIO_READ(type, len)            \
    static uint32_t ich9_cc_mmio_read ## type   \
    (void *opaque, target_phys_addr_t addr)     \
    {                                           \
        return ich9_cc_read(opaque, addr, len); \
    }

ICH9_CC_MMIO_WRITE(b, 1)
ICH9_CC_MMIO_WRITE(w, 2)
ICH9_CC_MMIO_WRITE(l, 4)

ICH9_CC_MMIO_READ(b, 1)
ICH9_CC_MMIO_READ(w, 2)
ICH9_CC_MMIO_READ(l, 4)

static CPUWriteMemoryFunc * const ich9_cc_mmio_write[] = {
    ich9_cc_mmio_writeb,
    ich9_cc_mmio_writew,
    ich9_cc_mmio_writel,
};

static CPUReadMemoryFunc * const ich9_cc_mmio_read[] = {
    ich9_cc_mmio_readb,
    ich9_cc_mmio_readw,
    ich9_cc_mmio_readl,
};

/* IRQ routing */
/* */
static void ich9_lpc_rout(uint8_t pirq_rout, int *pic_irq, int *pic_dis)
{
    *pic_irq = pirq_rout & ICH9_LPC_PIRQ_ROUT_MASK;
    *pic_dis = pirq_rout & ICH9_LPC_PIRQ_ROUT_IRQEN;
}

static void ich9_lpc_pic_irq(ICH9_LPCState *lpc, int irq_num,
                             int *pic_irq, int *pic_dis)
{
    switch (irq_num) {
    case 0 ... 3: /* A-D */
        ich9_lpc_rout(lpc->d.config[ICH9_LPC_PIRQA_ROUT + irq_num],
                      pic_irq, pic_dis);
        return;
    case 4 ... 7: /* E-H */
        ich9_lpc_rout(lpc->d.config[ICH9_LPC_PIRQE_ROUT + (irq_num - 4)],
                      pic_irq, pic_dis);
        return;
    default:
        break;
    }
    abort();
}

/* pic_irq: i8254 irq 0-15 */
static void ich9_lpc_update_pic(ICH9_LPCIrqState *irq_state, int pic_irq)
{
    GMCH_PCIHost *s = container_of(irq_state, GMCH_PCIHost, irq_state);
    ICH9_LPCState *lpc = irq_state->lpc;
    int i, pic_level;

    /* The pic level is the logical OR of all the PCI irqs mapped to it */
    pic_level = 0;
    for (i = 0; i < ICH9_LPC_NB_PIRQS; i++) {
        int tmp_irq;
        int tmp_dis;
        ich9_lpc_pic_irq(lpc, i, &tmp_irq, &tmp_dis);
        if (!tmp_dis && pic_irq == tmp_irq) {
            pic_level |= pci_bus_get_irq_level(s->host.pci.bus, i);
        }
    }
    if (pic_irq == ich9_lpc_sci_irq(lpc)) {
        pic_level |= lpc->sci_level;
    }

    qemu_set_irq(irq_state->pic[pic_irq], pic_level);
}

/* pirq: pirq[A-H] 0-7*/
static void ich9_lpc_update_by_pirq(ICH9_LPCIrqState *irq_state, int pirq)
{
    ICH9_LPCState *lpc = irq_state->lpc;
    int pic_irq;
    int pic_dis;

    ich9_lpc_pic_irq(lpc, pirq, &pic_irq, &pic_dis);
    assert(pic_irq < ICH9_LPC_PIC_NUM_PINS);
    if (pic_dis) {
        return;
    }

    ich9_lpc_update_pic(irq_state, pic_irq);
}

/* APIC mode: GSIx: PIRQ[A-H] -> GSI 16, ... no pirq shares same APIC pins. */
static int ich9_pirq_to_gsi(int pirq)
{
    return pirq + ICH9_LPC_PIC_NUM_PINS;
}

static int ich9_gsi_to_pirq(int gsi)
{
    return gsi - ICH9_LPC_PIC_NUM_PINS;
}

static void ich9_lpc_update_apic(ICH9_LPCIrqState *irq_state, int gsi)
{
    GMCH_PCIHost *s = container_of(irq_state, GMCH_PCIHost, irq_state);
    ICH9_LPCState *lpc = irq_state->lpc;
    int level;

    level = pci_bus_get_irq_level(s->host.pci.bus, ich9_gsi_to_pirq(gsi));
    if (gsi == ich9_lpc_sci_irq(lpc)) {
        level |= lpc->sci_level;
    }

    qemu_set_irq(irq_state->ioapic[gsi], level);
}

/* return the pirq number (PIRQ[A-H]:0-7) corresponding to
   a given device irq pin. */
static int ich9_lpc_map_irq(void *opaque, PCIDevice *pci_dev, int intx)
{
    ICH9_LPCIrqState *irq_state = opaque;
    return irq_state->lpc->irr[PCI_SLOT(pci_dev->devfn)][intx];
}

static void ich9_lpc_set_irq(void *opaque, int pirq, int level)
{
    ICH9_LPCIrqState *irq_state = opaque;

    assert(0 <= pirq);
    assert(pirq < ICH9_LPC_NB_PIRQS);

    ich9_lpc_update_apic(irq_state, ich9_pirq_to_gsi(pirq));
    ich9_lpc_update_by_pirq(irq_state, pirq);
}

static int ich9_lpc_sci_irq(ICH9_LPCState *lpc)
{
    switch (lpc->d.config[ICH9_LPC_ACPI_CTRL] &
            ICH9_LPC_ACPI_CTRL_SCI_IRQ_SEL_MASK) {
    case ICH9_LPC_ACPI_CTRL_9:
        return 9;
    case ICH9_LPC_ACPI_CTRL_10:
        return 10;
    case ICH9_LPC_ACPI_CTRL_11:
        return 11;
    case ICH9_LPC_ACPI_CTRL_20:
        return 20;
    case ICH9_LPC_ACPI_CTRL_21:
        return 21;
    default:
        /* reserved */
        break;
    }
    return -1;
}

static void ich9_set_sci(void *opaque, int irq_num, int level)
{
    ICH9_LPCIrqState *irq_state = opaque;
    ICH9_LPCState *lpc = irq_state->lpc;
    int irq;

    assert(irq_num == 0);
    level = !!level;
    if (level == lpc->sci_level) {
        return;
    }
    lpc->sci_level = level;

    irq = ich9_lpc_sci_irq(lpc);
    if (irq < 0) {
        return;
    }

    ich9_lpc_update_apic(irq_state, irq);
    if (irq < ICH9_LPC_PIC_NUM_PINS) {
        ich9_lpc_update_pic(irq_state, irq);
    }
}

void ich9_lpc_pm_init(DeviceState *gmch_host, PCIDevice *lpc_pci,
                      qemu_irq cmos_s3)
{
    GMCH_PCIHost *s = gmch_pcihost_from_qdev(gmch_host);
    ICH9_LPCState *lpc = ich9_lpc_from_pci(lpc_pci);
    qemu_irq *sci_irq;

    sci_irq = qemu_allocate_irqs(ich9_set_sci, &s->irq_state, 1);
    ich9_pm_init(&lpc->pm, sci_irq[0], cmos_s3);

    ich9_lpc_reset(&lpc->d.qdev);
}

/* APM */
static void ich9_apm_ctrl_changed(uint32_t val, void *arg)
{
    ICH9_LPCState *lpc = arg;

    /* ACPI specs 3.0, 4.7.2.5 */
    acpi_pm1_cnt_update(&lpc->pm.pm1_cnt,
                        val == ICH9_APM_ACPI_ENABLE,
                        val == ICH9_APM_ACPI_DISABLE);

    /* SMI_EN = PMBASE + 30. SMI control and enable register */
    if (lpc->pm.smi_en & ICH9_PMIO_SMI_EN_APMC_EN) {
        cpu_interrupt(first_cpu, CPU_INTERRUPT_SMI);
    }
}

/* config:PMBASE */
static void
ich9_lpc_pmbase_update(ICH9_LPCState *lpc)
{
    uint32_t pm_io_base = pci_get_long(lpc->d.config + ICH9_LPC_PMBASE);
    pm_io_base &= ICH9_LPC_PMBASE_BASE_ADDRESS_MASK;

    ich9_pm_iospace_update(&lpc->pm, pm_io_base);
}

/* config:RBCA */
static void ich9_lpc_rcba_update(ICH9_LPCState *lpc, uint32_t rbca_old)
{
    uint32_t rbca = pci_get_long(lpc->d.config + ICH9_LPC_RCBA);

    if (rbca_old & ICH9_LPC_RCBA_EN) {
        cpu_register_physical_memory(rbca_old & ICH9_LPC_RCBA_BA_MASK,
                                     ICH9_CC_SIZE, IO_MEM_UNASSIGNED);
    }
    if (rbca & ICH9_LPC_RCBA_EN) {
        cpu_register_physical_memory(rbca & ICH9_LPC_RCBA_BA_MASK,
                                     ICH9_CC_SIZE, lpc->rbca_index);
    }
}

static int ich9_lpc_post_load(void *opaque, int version_id)
{
    ICH9_LPCState *lpc = opaque;

    ich9_lpc_pmbase_update(lpc);
    ich9_lpc_rcba_update(lpc, 0 /* disabled ICH9_LPC_RBCA_EN */);
    return 0;
}

static void ich9_lpc_config_write(PCIDevice *d,
                                  uint32_t addr, uint32_t val, int len)
{
    ICH9_LPCState *lpc = ich9_lpc_from_pci(d);
    uint32_t rbca_old = pci_get_long(d->config + ICH9_LPC_RCBA);

    pci_default_write_config(d, addr, val, len);
    if (ranges_overlap(addr, len, ICH9_LPC_PMBASE, 4)) {
        ich9_lpc_pmbase_update(lpc);
    }
    if (ranges_overlap(addr, len, ICH9_LPC_RCBA, 4)) {
        ich9_lpc_rcba_update(lpc, rbca_old);
    }
}

static void ich9_lpc_reset(DeviceState *qdev)
{
    PCIDevice *d = DO_UPCAST(PCIDevice, qdev, qdev);
    ICH9_LPCState *lpc = ich9_lpc_from_pci(d);
    uint32_t rbca_old = pci_get_long(d->config + ICH9_LPC_RCBA);
    int i;

    for (i = 0; i < 4; i++) {
        pci_set_byte(d->config + ICH9_LPC_PIRQA_ROUT + i,
                     ICH9_LPC_PIRQ_ROUT_DEFAULT);
    }
    for (i = 0; i < 4; i++) {
        pci_set_byte(d->config + ICH9_LPC_PIRQE_ROUT + i,
                     ICH9_LPC_PIRQ_ROUT_DEFAULT);
    }
    pci_set_byte(d->config + ICH9_LPC_ACPI_CTRL, ICH9_LPC_ACPI_CTRL_DEFAULT);

    pci_set_long(d->config + ICH9_LPC_PMBASE, ICH9_LPC_PMBASE_DEFAULT);
    pci_set_long(d->config + ICH9_LPC_RCBA, ICH9_LPC_RCBA_DEFAULT);

    ich9_cc_reset(lpc);

    ich9_lpc_pmbase_update(lpc);
    ich9_lpc_rcba_update(lpc, rbca_old);

    lpc->sci_level = 0;
}

static int ich9_lpc_initfn(PCIDevice *d)
{
    ICH9_LPCState *lpc = ich9_lpc_from_pci(d);

    isa_bus_new(&d->qdev);
    pci_config_set_vendor_id(d->config, PCI_VENDOR_ID_INTEL);
    pci_config_set_device_id(d->config, PCI_DEVICE_ID_INTEL_ICH9_8); /* ICH9 LPC */
    pci_config_set_revision(d->config, ICH9_A2_LPC_REVISION);
    pci_config_set_class(d->config, PCI_CLASS_BRIDGE_ISA);

    pci_set_long(d->wmask + ICH9_LPC_PMBASE,
                 ICH9_LPC_PMBASE_BASE_ADDRESS_MASK);

    lpc->rbca_index = cpu_register_io_memory(ich9_cc_mmio_read,
                                             ich9_cc_mmio_write,
                                             lpc, DEVICE_LITTLE_ENDIAN);

    ich9_cc_init(lpc);
    apm_init(&lpc->apm, ich9_apm_ctrl_changed, lpc);
    return 0;
}

static const VMStateDescription vmstate_ich9_lpc = {
    .name = "ICH9LPC",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = ich9_lpc_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(d, ICH9_LPCState),
        VMSTATE_STRUCT(apm, ICH9_LPCState, 0, vmstate_apm, APMState),
        VMSTATE_STRUCT(pm, ICH9_LPCState, 0, vmstate_ich9_pm, ICH9_LPCPmRegs),
        VMSTATE_UINT8_ARRAY(chip_config, ICH9_LPCState, ICH9_CC_SIZE),
        VMSTATE_UINT32(sci_level, ICH9_LPCState),
        VMSTATE_END_OF_LIST()
    }
};

PCIDevice *gmch_lpc_init(DeviceState *gmch_host, PCIBus *bus)
{
    GMCH_PCIHost *s = gmch_pcihost_from_qdev(gmch_host);
    PCIDevice *d;
    ICH9_LPCState *lpc;

    d = pci_create_simple_multifunction(bus, PCI_DEVFN(ICH9_LPC_DEV,
                                                       ICH9_LPC_FUNC),
                                        true, "ICH9 LPC");
    lpc = ich9_lpc_from_pci(d);
    s->irq_state.lpc = lpc;
    return &lpc->d;
}

static PCIDeviceInfo ich9_lpc_info = {
    .qdev.name    = "ICH9 LPC",
    .qdev.desc    = "ICH9 LPC bridge",
    .qdev.size    = sizeof(ICH9_LPCState),
    .qdev.vmsd    = &vmstate_ich9_lpc,
    .qdev.no_user = 1,
    .init         = ich9_lpc_initfn,
    .config_write = ich9_lpc_config_write,
    .qdev.reset   = ich9_lpc_reset,
};

static void q35_register(void)
{
    sysbus_register_withprop(&gmch_pcihost_info);
    pci_qdev_register(&gmch_info);
    pci_qdev_register(&ich9_lpc_info);
}
device_init(q35_register);
