/*
 * QEMU PC System Emulator
 *
 * Copyright (c) 2003-2004 Fabrice Bellard
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
 *  Q35 chipset based pc system emulator
 *
 *  Copyright (c) 2009, 2010
 *                     Isaku Yamahata <yamahata at valinux co jp>
 *                     VA Linux Systems Japan K.K.
 *
 *  This is based on pc.c, but heavily modified.
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
#include "arch_init.h"
#include "pc.h"
#include "fdc.h"
#include "pci.h"
#include "pci_bridge.h"
#include "pci_p2pbr.h"
#include "ioh3420.h"
#include "xio3130_upstream.h"
#include "xio3130_downstream.h"
#include "block.h"
#include "blockdev.h"
#include "sysemu.h"
#include "audio/audio.h"
#include "net.h"
#include "smbus.h"
#include "boards.h"
#include "monitor.h"
#include "fw_cfg.h"
#include "hpet_emul.h"
#include "watchdog.h"
#include "smbios.h"
#include "ide.h"
#include "usb-uhci.h"

#include "q35.h"

/* ICH9 AHCI has 6 ports */
#define MAX_SATA_PORTS     6

#define I21154_REV            0x05
#define I21154_PI             0x00

static PCIBridge *i21154_init(PCIBus *bus, int devfn, const char *bus_name,
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
        .vendor_id = PCI_VENDOR_ID_DEC,
        .device_id = PCI_DEVICE_ID_DEC_21154,
        .revision_id = I21154_REV,
        .prog_interface = I21154_PI,
    };
    return pci_p2pbr_create_simple(&init, &prop);
}

static void pc_q35_bridge_init(PCIBus *host_bus, PCIBus *pci_bus)
{
    uint8_t dev;
    uint8_t sec_bus;
    uint8_t port = 0;
    uint8_t chassis = 0;
    uint16_t slot = 0;
    uint8_t upstream_port;
    PCIESlot *s;
    uint8_t fn;
    PCIESlot *root_port;
    PCIBus *root_port_bus;
    char buf[16];

    /* PCI to PCI bridge b6:d[29 - 31]:f0, 6:[1c - 1f].0 with subordinate bus
       of 7 - 9 on b0:d30:f0, 0.1e.0 = bus */
#define Q35_P2P_BRDIGE_DEV_BASE         28
#define Q35_P2P_BRDIGE_DEV_MAX          32
#define Q35_P2P_BRDIGE_SUBBUS_BASE      (ICH9_D2P_SECONDARY_DEFAULT + 1)
    for (dev = Q35_P2P_BRDIGE_DEV_BASE; dev < Q35_P2P_BRDIGE_DEV_MAX; dev++) {
        PCIBridge *br;
        sec_bus = Q35_P2P_BRDIGE_SUBBUS_BASE + dev - Q35_P2P_BRDIGE_DEV_BASE;

        snprintf(buf, sizeof(buf), "pci.%d", sec_bus);
        br = i21154_init(pci_bus, PCI_DEVFN(dev, 0), buf, true);
    }

    /* PCIe root port b0:d1:f0 in GMCH.
     * Actually it's vid/did = 0x8086:0x29c1, but we substitute ioh for it.
     */
    sec_bus = 32;
    snprintf(buf, sizeof(buf), "pcie.%d", sec_bus);
    s = ioh3420_init(host_bus, PCI_DEVFN(GMCH_PCIE_DEV, GMCH_PCIE_FUNC), true,
                     buf, pci_swizzle_map_irq_fn, port, chassis, slot);


    /* more slots. ICH9 doesn't have those, but many slots are wanted. */
//#define Q35_MANY_SLOTS
#undef Q35_MANY_SLOTS

#ifdef Q35_MANY_SLOTS
#define Q35_NR_ROOTPORT         6
#define Q35_NR_UPSTREAM         8
#define Q35_NR_DOWNSTREAM       16
#else
#define Q35_NR_ROOTPORT         1
#define Q35_NR_UPSTREAM         1
#define Q35_NR_DOWNSTREAM       1
#endif

    /* PCIe root port b0:d23:f[0-5], 0.17.[0-5] */
    for (fn = 0; fn < Q35_NR_ROOTPORT; fn++) {
        sec_bus++;
        port++;
        slot++;

        snprintf(buf, sizeof(buf), "pcie.%d", sec_bus);
        s = ioh3420_init(host_bus, PCI_DEVFN(23, fn), true,
                         buf, pci_swizzle_map_irq_fn, port, chassis, slot);
    }

    /* PCIe root port b0:d24:f0 */
    sec_bus++;
    port++;
    slot++;
    snprintf(buf, sizeof(buf), "pcie.%d", sec_bus);
    root_port = ioh3420_init(host_bus, PCI_DEVFN(24, 0), true,
                             buf, pci_swizzle_map_irq_fn, port, chassis, slot);
    root_port_bus = pci_bridge_get_sec_bus(&root_port->port.br);

    /* 8 * 16 = 128 slots */
    upstream_port = 0;
    for (fn = 0; fn < Q35_NR_UPSTREAM; fn++) {
        PCIEPort *upstream;
        PCIBus *upstream_bus;
        uint16_t downstream_port;

        uint8_t ds_dev_max;
        uint8_t ds_dev;
        uint8_t ds_fn_max;
        uint8_t ds_fn;

        /* PCIe upstream port d0:f[0-7] */
        sec_bus++;
        snprintf(buf, sizeof(buf), "pcie.%d", sec_bus);
        upstream = xio3130_upstream_init(root_port_bus, PCI_DEVFN(0, fn),
                                         true, buf, pci_swizzle_map_irq_fn,
                                         upstream_port);

        upstream_bus = pci_bridge_get_sec_bus(&upstream->br);
        upstream_port++;

        /* PCIe downstream port */
        downstream_port = 0;
        ds_fn_max = MIN(Q35_NR_DOWNSTREAM / PCI_SLOT_MAX, PCI_FUNC_MAX);
        ds_dev_max = MIN(Q35_NR_DOWNSTREAM / (ds_fn_max + 1), PCI_SLOT_MAX);

        for (ds_dev = 0; ds_dev <= ds_dev_max &&
                 downstream_port < Q35_NR_DOWNSTREAM; ds_dev++) {
            for (ds_fn = 0; ds_fn <= ds_fn_max &&
                     downstream_port < Q35_NR_DOWNSTREAM; ds_fn++) {
                sec_bus++;
                slot++;
                snprintf(buf, sizeof(buf), "pcie.%d", sec_bus);

                xio3130_downstream_init(upstream_bus, PCI_DEVFN(ds_dev, ds_fn),
                                        true, buf, pci_swizzle_map_irq_fn,
                                        downstream_port, chassis, slot);
                downstream_port++;
            }
        }
    }

    /* PCIe root port b0:d28:f[0-6] in ICH9.
     * Actually it's vid/did = 0x8086:0x294[02468A], but we substitute ioh
     * for them.
     */
    for (fn = 0; fn < ICH9_PCIE_FUNC_MAX; fn++) {
        sec_bus++;
        port++;
        slot++;

        snprintf(buf, sizeof(buf), "pcie.%d", sec_bus);
        s = ioh3420_init(host_bus, PCI_DEVFN(ICH9_PCIE_DEV, fn), true,
                         buf, pci_swizzle_map_irq_fn,
                         port, chassis, slot);
    }
}

static void pc_q35_init_early(qemu_irq *isa_irq, IsaIrqState *isa_irq_state,
                              DeviceState **gmch_host_p,
                              PCIBus **host_bus_p, PCIBus **pci_bus_p,
                              PCIDevice **lpc_p)
{
    DeviceState *gmch_host;
    PCIBus *host_bus;
    PCIBus *pci_bus;

    PCIDevice *gmch_state;
    PCIDevice *lpc;

    /* create pci host bus */
    host_bus = gmch_host_init(&gmch_host, isa_irq, isa_irq_state->ioapic);
    gmch_state = gmch_init(gmch_host, host_bus);

    /* create conventional pci bus: pcie2pci bridge */
    pci_bus = ich9_d2pbr_init(host_bus, PCI_DEVFN(ICH9_D2P_BRIDGE_DEV,
                                                  ICH9_D2P_BRIDGE_FUNC),
                              ICH9_D2P_SECONDARY_DEFAULT);

    /* create child pci/pcie buses */
    pc_q35_bridge_init(host_bus, pci_bus);

    /* create ISA bus */
    lpc = gmch_lpc_init(gmch_host, host_bus);

    *gmch_host_p = gmch_host;
    *host_bus_p = host_bus;
    *pci_bus_p = pci_bus;
    *lpc_p = lpc;
}

static void pc_q35_init_late(BusState **idebus, ISADevice *rtc_state,
                             DeviceState *gmch_host,
                             PCIBus *host_bus, PCIBus *pci_bus,
                             PCIDevice *lpc)
{
    qemu_irq *cmos_s3;
    PCIDevice *ahci;
    DriveInfo *hd[MAX_SATA_PORTS * MAX_IDE_DEVS];

    /* connect pm stuff to lpc */
    cmos_s3 = qemu_allocate_irqs(pc_cmos_set_s3_resume, rtc_state, 1);
    ich9_lpc_pm_init(gmch_host, lpc, *cmos_s3);

    /* ahci and SATA device */
    ide_drive_get(hd, MAX_SATA_PORTS);
    ahci = pci_create_simple_multifunction(host_bus,
                                           PCI_DEVFN(ICH9_SATA1_DEV,
                                                     ICH9_SATA1_FUNC),
                                           true, "ich9-ahci");
    pci_ahci_ide_create_devs(ahci, hd);
    idebus[0] = qdev_get_child_bus(&ahci->qdev, "ide.0");
    idebus[1] = qdev_get_child_bus(&ahci->qdev, "ide.1");

    if (usb_enabled) {
        /* Should we create 6 UHCI according to ich9 spec? */
        pci_create_simple_multifunction(
            host_bus, PCI_DEVFN(ICH9_USB_UHCI1_DEV, ICH9_USB_UHCI1_FUNC),
            true, "ich9-usb-uhci1");
        /* XXX: EHCI */
    }

    /* TODO: Populate SPD eeprom data.  */
    smbus_eeprom_init(ich9_smb_init(host_bus,
                                    PCI_DEVFN(ICH9_SMB_DEV, ICH9_SMB_FUNC),
                                    0xb100),
                      8, NULL, 0);
}

/* PC hardware initialisation */
static void pc_q35_init(ram_addr_t ram_size,
                        const char *boot_device,
                        const char *kernel_filename,
                        const char *kernel_cmdline,
                        const char *initrd_filename,
                        const char *cpu_model)
{
    ram_addr_t below_4g_mem_size, above_4g_mem_size;
    DeviceState *gmch_host;
    PCIBus *host_bus;
    PCIBus *pci_bus;
    PCIDevice *lpc;
    qemu_irq *isa_irq;
    IsaIrqState *isa_irq_state;
    BusState *idebus[MAX_SATA_PORTS];
    ISADevice *rtc_state;

    pc_cpus_init(cpu_model);

    /* allocate ram and load rom/bios */
    pc_memory_init(ram_size, kernel_filename, kernel_cmdline, initrd_filename,
                   &below_4g_mem_size, &above_4g_mem_size);

    /* irq lines */
    isa_irq = pc_isa_irq(&isa_irq_state);
    ioapic_init(isa_irq_state);

    pc_q35_init_early(isa_irq, isa_irq_state,
                      &gmch_host, &host_bus, &pci_bus, &lpc);
    isa_bus_irqs(isa_irq);
    pc_register_ferr_irq(isa_get_irq(13));

    /* init basic PC hardware */
    pc_basic_device_init(isa_irq, &rtc_state);

    pc_q35_init_late(idebus, rtc_state, gmch_host, host_bus, pci_bus, lpc);

    pc_cmos_init(below_4g_mem_size, above_4g_mem_size, boot_device,
                 idebus[0], idebus[1], rtc_state);

    /* the rest devices to which pci devfn is automatically assigned */
    pc_vga_init(host_bus);
    audio_init(isa_irq, pci_bus);
    pc_nic_init(pci_bus);
    pc_pci_device_init(pci_bus);
}

static QEMUMachine pc_q35_machine = {
    .name = "pc_q35",
    .desc = "Q35 chipset PC",
    .init = pc_q35_init,
    .max_cpus = 255,
};

static void pc_q35_machine_init(void)
{
    qemu_register_machine(&pc_q35_machine);
}

machine_init(pc_q35_machine_init);
