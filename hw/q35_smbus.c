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
 *  This is based on acpi.c, but heavily rewritten.
 */
#include "hw.h"
#include "pc.h"
#include "pm_smbus.h"
#include "pci.h"
#include "sysemu.h"
#include "i2c.h"
#include "smbus.h"

#include "q35.h"

typedef struct ICH9_SMBState {
    PCIDevice dev;

    PMSMBus smb;
} ICH9_SMBState;

static ICH9_SMBState *ich9_pci_to_smb(PCIDevice* pci_dev)
{
    return DO_UPCAST(ICH9_SMBState, dev, pci_dev);
}

static const VMStateDescription vmstate_ich9_smbus = {
    .name = "ich9_smb",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(dev, struct ICH9_SMBState),
        VMSTATE_END_OF_LIST()
    }
};

static void ich9_smb_ioport_writeb(void *opaque, uint32_t addr, uint32_t val)
{
    ICH9_SMBState *s = opaque;
    uint8_t hostc = s->dev.config[ICH9_SMB_HOSTC];

    if ((hostc & ICH9_SMB_HOSTC_HST_EN) && !(hostc & ICH9_SMB_HOSTC_I2C_EN)) {
        uint64_t offset = addr - s->dev.io_regions[ICH9_SMB_SMB_BASE_BAR].addr;
        smb_ioport_writeb(&s->smb, offset, val);
    }
}

static uint32_t ich9_smb_ioport_readb(void *opaque, uint32_t addr)
{
    ICH9_SMBState *s = opaque;
    uint8_t hostc = s->dev.config[ICH9_SMB_HOSTC];

    if ((hostc & ICH9_SMB_HOSTC_HST_EN) && !(hostc & ICH9_SMB_HOSTC_I2C_EN)) {
        uint64_t offset = addr - s->dev.io_regions[ICH9_SMB_SMB_BASE_BAR].addr;
        return smb_ioport_readb(&s->smb, offset);
    }

    return 0xff;
}

static void ich9_smb_map_ioport(PCIDevice *dev, int region_num,
                                uint64_t addr, uint64_t size, int type)
{
    ICH9_SMBState *s = ich9_pci_to_smb(dev);

    assert(size == ICH9_SMB_SMB_BASE_SIZE);
    assert(type == PCI_BASE_ADDRESS_SPACE_IO);

    register_ioport_write(addr, 64, 1, ich9_smb_ioport_writeb, s);
    register_ioport_read(addr, 64, 1, ich9_smb_ioport_readb, s);
}

static int ich9_smb_initfn(PCIDevice *d)
{
    ICH9_SMBState *s = ich9_pci_to_smb(d);

    pci_config_set_vendor_id(d->config, PCI_VENDOR_ID_INTEL);
    pci_config_set_device_id(d->config, PCI_DEVICE_ID_INTEL_ICH9_6);

    pci_set_word(d->wmask + PCI_STATUS,
                 PCI_STATUS_SIG_SYSTEM_ERROR | PCI_STATUS_DETECTED_PARITY);

    pci_config_set_revision(d->config, ICH9_A2_SMB_REVISION);
    pci_config_set_prog_interface(d->config, ICH9_SMB_PI);
    pci_config_set_class(d->config, PCI_CLASS_SERIAL_SMBUS);

    /* TODO? D31IP.SMIP in chipset configuration space */
    pci_config_set_interrupt_pin(d->config, 0x01); /* interrupt pin 1 */

    pci_set_byte(d->config + ICH9_SMB_HOSTC, 0);

    /*
     * update parameters based on
     * paralell_hds[0]
     * serial_hds[0]
     * serial_hds[0]
     * fdc
     *
     * Is there any OS that depends on them?
     */

    /* TODO smb_io_base */
    pci_set_byte(d->config + ICH9_SMB_HOSTC, 0);
    /* TODO bar0, bar1: 64bit BAR support*/
    pci_register_bar(d, ICH9_SMB_SMB_BASE_BAR,
                     ICH9_SMB_SMB_BASE_SIZE, PCI_BASE_ADDRESS_SPACE_IO,
                     &ich9_smb_map_ioport);

    pm_smbus_init(&d->qdev, &s->smb);
    return 0;
}

i2c_bus *ich9_smb_init(PCIBus *bus, int devfn, uint32_t smb_io_base)
{
    PCIDevice *d =
        pci_create_simple_multifunction(bus, devfn, true, "ICH9 SMB");
    ICH9_SMBState *s = ich9_pci_to_smb(d);
    return s->smb.smbus;
}

static PCIDeviceInfo ich9_smb_info = {
    .qdev.name = "ICH9 SMB",
    .qdev.desc = "ICH9 SMBUS Bridge",
    .qdev.size = sizeof(ICH9_SMBState),
    .qdev.vmsd = &vmstate_ich9_smbus,
    .qdev.no_user = 1,
    .init = ich9_smb_initfn,
};

static void ich9_smb_register(void)
{
    pci_qdev_register(&ich9_smb_info);
}

device_init(ich9_smb_register);
