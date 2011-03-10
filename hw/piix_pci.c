/*
 * QEMU i440FX/PIIX3 PCI Bridge Emulation
 *
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

#include "hw.h"
#include "pc.h"
#include "pci.h"
#include "pci_host.h"
#include "isa.h"
#include "sysbus.h"
#include "range.h"
#include "pam.h"

/*
 * I440FX chipset data sheet.
 * http://download.intel.com/design/chipsets/datashts/29054901.pdf
 */

typedef PCIHostState I440FXState;

typedef struct PIIX3State {
    PCIDevice dev;
    int32_t dummy_for_save_load_compat[4];
    qemu_irq *pic;
} PIIX3State;

struct PCII440FXState {
    PCIDevice dev;
    PAM pam;
    PIIX3State *piix3;
};


#define I440FX_PAM      0x59
#define I440FX_PAM_SIZE 7
#define I440FX_SMRAM    0x72

static void piix3_set_irq(void *opaque, int irq_num, int level);

/* return the global irq number corresponding to a given device irq
   pin. We could also use the bus number to have a more precise
   mapping. */
static int pci_slot_get_pirq(void *opaque, PCIDevice *pci_dev, int irq_num)
{
    int slot_addend;
    slot_addend = (pci_dev->devfn >> 3) - 1;
    return (irq_num + slot_addend) & 3;
}

static void i440fx_update_memory_mappings(PCII440FXState *d)
{
    int i;

    for (i = 0; i <= PAM_IDX_MAX; i++) {
        pam_update(&d->pam, i, d->dev.config[I440FX_PAM + i]);
    }

    smram_update(&d->pam, d->dev.config[I440FX_SMRAM]);
}

static void i440fx_set_smm(int val, void *arg)
{
    PCII440FXState *d = arg;
    smram_set_smm(&d->pam, val, d->dev.config[I440FX_SMRAM]);
}

static void i440fx_write_config(PCIDevice *dev,
                                uint32_t address, uint32_t val, int len)
{
    PCII440FXState *d = DO_UPCAST(PCII440FXState, dev, dev);

    /* XXX: implement SMRAM.D_LOCK */
    pci_default_write_config(dev, address, val, len);
    if (ranges_overlap(address, len, I440FX_PAM, I440FX_PAM_SIZE) ||
        range_covers_byte(address, len, I440FX_SMRAM)) {
        i440fx_update_memory_mappings(d);
    }
}

static int i440fx_load_old(QEMUFile* f, void *opaque, int version_id)
{
    PCII440FXState *d = opaque;
    int ret, i;

    ret = pci_device_load(&d->dev, f);
    if (ret < 0)
        return ret;
    i440fx_update_memory_mappings(d);
    qemu_get_8s(f, &d->pam.smm_enabled);

    if (version_id == 2) {
        for (i = 0; i < 4; i++) {
            qemu_get_be32(f); /* dummy load for compatibility */
        }
    }

    return 0;
}

static int i440fx_post_load(void *opaque, int version_id)
{
    PCII440FXState *d = opaque;

    i440fx_update_memory_mappings(d);
    return 0;
}

static const VMStateDescription vmstate_i440fx = {
    .name = "I440FX",
    .version_id = 3,
    .minimum_version_id = 3,
    .minimum_version_id_old = 1,
    .load_state_old = i440fx_load_old,
    .post_load = i440fx_post_load,
    .fields      = (VMStateField []) {
        VMSTATE_PCI_DEVICE(dev, PCII440FXState),
        VMSTATE_UINT8(pam.smm_enabled, PCII440FXState),
        VMSTATE_END_OF_LIST()
    }
};

static int i440fx_pcihost_initfn(SysBusDevice *dev)
{
    I440FXState *s = FROM_SYSBUS(I440FXState, dev);

    pci_host_conf_register_ioport(0xcf8, s);

    pci_host_data_register_ioport(0xcfc, s);
    return 0;
}

static int i440fx_initfn(PCIDevice *dev)
{
    PCII440FXState *d = DO_UPCAST(PCII440FXState, dev, dev);

    pci_config_set_vendor_id(d->dev.config, PCI_VENDOR_ID_INTEL);
    pci_config_set_device_id(d->dev.config, PCI_DEVICE_ID_INTEL_82441);
    d->dev.config[0x08] = 0x02; // revision
    pci_config_set_class(d->dev.config, PCI_CLASS_BRIDGE_HOST);

    d->dev.config[I440FX_SMRAM] = 0x02;

    cpu_smm_register(&i440fx_set_smm, d);
    pam_init_memory_mappings(&d->pam);
    return 0;
}

PCIBus *i440fx_init(int *piix3_devfn, qemu_irq *pic, ram_addr_t ram_size)
{
    DeviceState *dev;
    PCIBus *b;
    PCIDevice *d;
    I440FXState *s;
    PCII440FXState *i440fx_state;
    PIIX3State *piix3;

    dev = qdev_create(NULL, "i440FX-pcihost");
    s = FROM_SYSBUS(I440FXState, sysbus_from_qdev(dev));
    b = pci_bus_new(&s->busdev.qdev, NULL, 0);
    s->bus = b;
    qdev_init_nofail(dev);

    d = pci_create_simple(b, 0, "i440FX");
    i440fx_state = DO_UPCAST(PCII440FXState, dev, d);

    piix3 = DO_UPCAST(PIIX3State, dev,
                      pci_create_simple_multifunction(b, -1, true, "PIIX3"));
    piix3->pic = pic;
    pci_bus_irqs(b, piix3_set_irq, pci_slot_get_pirq, piix3, 4);
    i440fx_state->piix3 = piix3;

    *piix3_devfn = piix3->dev.devfn;

    ram_size = ram_size / 8 / 1024 / 1024;
    if (ram_size > 255)
        ram_size = 255;
    i440fx_state->dev.config[0x57]=ram_size;

    return b;
}

/* PIIX3 PCI to ISA bridge */

static void piix3_set_irq(void *opaque, int irq_num, int level)
{
    int i, pic_irq, pic_level;
    PIIX3State *piix3 = opaque;

    /* now we change the pic irq level according to the piix irq mappings */
    /* XXX: optimize */
    pic_irq = piix3->dev.config[0x60 + irq_num];
    if (pic_irq < 16) {
        /* The pic level is the logical OR of all the PCI irqs mapped
           to it */
        pic_level = 0;
        for (i = 0; i < 4; i++) {
            if (pic_irq == piix3->dev.config[0x60 + i]) {
                pic_level |= pci_bus_get_irq_level(piix3->dev.bus, i);
            }
        }
        qemu_set_irq(piix3->pic[pic_irq], pic_level);
    }
}

static void piix3_reset(void *opaque)
{
    PIIX3State *d = opaque;
    uint8_t *pci_conf = d->dev.config;

    pci_conf[0x04] = 0x07; // master, memory and I/O
    pci_conf[0x05] = 0x00;
    pci_conf[0x06] = 0x00;
    pci_conf[0x07] = 0x02; // PCI_status_devsel_medium
    pci_conf[0x4c] = 0x4d;
    pci_conf[0x4e] = 0x03;
    pci_conf[0x4f] = 0x00;
    pci_conf[0x60] = 0x80;
    pci_conf[0x61] = 0x80;
    pci_conf[0x62] = 0x80;
    pci_conf[0x63] = 0x80;
    pci_conf[0x69] = 0x02;
    pci_conf[0x70] = 0x80;
    pci_conf[0x76] = 0x0c;
    pci_conf[0x77] = 0x0c;
    pci_conf[0x78] = 0x02;
    pci_conf[0x79] = 0x00;
    pci_conf[0x80] = 0x00;
    pci_conf[0x82] = 0x00;
    pci_conf[0xa0] = 0x08;
    pci_conf[0xa2] = 0x00;
    pci_conf[0xa3] = 0x00;
    pci_conf[0xa4] = 0x00;
    pci_conf[0xa5] = 0x00;
    pci_conf[0xa6] = 0x00;
    pci_conf[0xa7] = 0x00;
    pci_conf[0xa8] = 0x0f;
    pci_conf[0xaa] = 0x00;
    pci_conf[0xab] = 0x00;
    pci_conf[0xac] = 0x00;
    pci_conf[0xae] = 0x00;
}

static void piix3_pre_save(void *opaque)
{
    int i;
    PIIX3State *piix3 = opaque;

    for (i = 0; i < ARRAY_SIZE(piix3->dummy_for_save_load_compat); i++) {
        piix3->dummy_for_save_load_compat[i] =
            pci_bus_get_irq_level(piix3->dev.bus, i);
    }
}

static const VMStateDescription vmstate_piix3 = {
    .name = "PIIX3",
    .version_id = 3,
    .minimum_version_id = 2,
    .minimum_version_id_old = 2,
    .pre_save = piix3_pre_save,
    .fields      = (VMStateField []) {
        VMSTATE_PCI_DEVICE(dev, PIIX3State),
        VMSTATE_INT32_ARRAY_V(dummy_for_save_load_compat, PIIX3State, 4, 3),
        VMSTATE_END_OF_LIST()
    }
};

static int piix3_initfn(PCIDevice *dev)
{
    PIIX3State *d = DO_UPCAST(PIIX3State, dev, dev);
    uint8_t *pci_conf;

    isa_bus_new(&d->dev.qdev);

    pci_conf = d->dev.config;
    pci_config_set_vendor_id(pci_conf, PCI_VENDOR_ID_INTEL);
    pci_config_set_device_id(pci_conf, PCI_DEVICE_ID_INTEL_82371SB_0); // 82371SB PIIX3 PCI-to-ISA bridge (Step A1)
    pci_config_set_class(pci_conf, PCI_CLASS_BRIDGE_ISA);

    qemu_register_reset(piix3_reset, d);
    return 0;
}

static PCIDeviceInfo i440fx_info[] = {
    {
        .qdev.name    = "i440FX",
        .qdev.desc    = "Host bridge",
        .qdev.size    = sizeof(PCII440FXState),
        .qdev.vmsd    = &vmstate_i440fx,
        .qdev.no_user = 1,
        .no_hotplug   = 1,
        .init         = i440fx_initfn,
        .config_write = i440fx_write_config,
    },{
        .qdev.name    = "PIIX3",
        .qdev.desc    = "ISA bridge",
        .qdev.size    = sizeof(PIIX3State),
        .qdev.vmsd    = &vmstate_piix3,
        .qdev.no_user = 1,
        .no_hotplug   = 1,
        .init         = piix3_initfn,
    },{
        /* end of list */
    }
};

static SysBusDeviceInfo i440fx_pcihost_info = {
    .init         = i440fx_pcihost_initfn,
    .qdev.name    = "i440FX-pcihost",
    .qdev.fw_name = "pci",
    .qdev.size    = sizeof(I440FXState),
    .qdev.no_user = 1,
};

static void i440fx_register(void)
{
    sysbus_register_withprop(&i440fx_pcihost_info);
    pci_qdev_register_many(i440fx_info);
}
device_init(i440fx_register);
