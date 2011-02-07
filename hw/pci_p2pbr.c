/*
 * QEMU PCI P2P generic bridge.
 * In order to avoid create many P2P bridge device which only differs
 * in vendor id/device id and so on.
 *
 * Copyright (c) 2011 Isaku Yamahata <yamahata at valinux co jp>
 *                    VA Linux Systems Japan K.K.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "pci_bridge.h"
#include "pci_internals.h"
#include "pci_p2pbr.h"

typedef struct PCIP2PBridge {
    struct PCIBridge br;

    /* device specific initialization */
    pci_p2pbr_init_fn initfn;

    /* properties */
    uint16_t vendor_id;
    uint16_t device_id;
    uint8_t revision_id;
    uint8_t prog_interface;

    uint8_t ssvid_cap;
    uint16_t svid;
    uint16_t ssid;
} PCIP2PBridge;

static int pci_p2pbr_initfn(PCIDevice *d)
{
    PCIBridge *br = DO_UPCAST(PCIBridge, dev, d);
    PCIP2PBridge *p2pbr = DO_UPCAST(PCIP2PBridge, br, br);
    uint8_t *config = d->config;
    int rc;

    rc = pci_bridge_initfn(d);
    if (rc < 0) {
        return rc;
    }

    pci_config_set_vendor_id(config, p2pbr->vendor_id);
    pci_config_set_device_id(config, p2pbr->device_id);
    pci_config_set_revision(config, p2pbr->revision_id);
    pci_config_set_prog_interface(config, p2pbr->prog_interface);

    if (p2pbr->ssvid_cap > 0) {
        rc = pci_bridge_ssvid_init(d, p2pbr->ssvid_cap,
                                   p2pbr->svid, p2pbr->ssid);
        if (rc < 0) {
            return rc;
        }
    }

    if (p2pbr->initfn) {
        return p2pbr->initfn(d);
    }

    return 0;
}

#define PCI_P2P_BRIDGE  "PCI P2P bridge"

static PCIDeviceInfo pci_p2pbr_info = {
    .qdev.name = PCI_P2P_BRIDGE,
    .qdev.desc = "PCI PCI-to-PCI bridge",
    .qdev.size = sizeof(PCIP2PBridge),
    .qdev.reset = pci_bridge_reset,
    .qdev.vmsd = &vmstate_pci_device,

    .is_bridge = 1,
    .init = pci_p2pbr_initfn,
    .exit = pci_bridge_exitfn,
    .config_write = pci_bridge_write_config,

    .qdev.props = (Property[]) {
        DEFINE_PROP_UINT16("vendor_id", PCIP2PBridge, vendor_id, 0),
        DEFINE_PROP_UINT16("device_id", PCIP2PBridge, device_id, 0),
        DEFINE_PROP_UINT8("revision_id", PCIP2PBridge, revision_id, 0),
        DEFINE_PROP_UINT8("prog_interface", PCIP2PBridge, prog_interface, 0),

        DEFINE_PROP_UINT8("ssvid_cap", PCIP2PBridge, ssvid_cap, 0),
        DEFINE_PROP_UINT16("svid", PCIP2PBridge, svid, 0),
        DEFINE_PROP_UINT16("ssid", PCIP2PBridge, ssid, 0),

        DEFINE_PROP_END_OF_LIST(),
    }
};

static void pci_p2pbr_register(void)
{
    pci_qdev_register(&pci_p2pbr_info);
}

device_init(pci_p2pbr_register);

PCIBridge *pci_p2pbr_create(const PCIP2PBridgeInit *init)
{
    PCIDevice *d;
    PCIBridge *br;
    PCIP2PBridge *p2pbr;

    d = pci_create_multifunction(init->bus, init->devfn, init->multifunction,
                                 PCI_P2P_BRIDGE);

    br = DO_UPCAST(PCIBridge, dev, d);
    pci_bridge_map_irq(br, init->bus_name, init->map_irq);

    p2pbr = DO_UPCAST(PCIP2PBridge, br, br);
    p2pbr->initfn = init->initfn;

    return br;
}

void pci_p2pbr_prop_set(PCIBridge *br, const PCIP2PBridgeProp *prop)
{
    DeviceState *qdev = &br->dev.qdev;

    qdev_prop_set_uint16(qdev, "vendor_id", prop->vendor_id);
    qdev_prop_set_uint16(qdev, "device_id", prop->device_id);
    qdev_prop_set_uint8(qdev, "revision_id", prop->revision_id);
    qdev_prop_set_uint8(qdev, "prog_interface", prop->prog_interface);

    qdev_prop_set_uint8(qdev, "ssvid_cap", prop->ssvid_cap);
    qdev_prop_set_uint16(qdev, "svid", prop->svid);
    qdev_prop_set_uint16(qdev, "ssid", prop->ssid);
}

/* convenience function to create pci p2p bridge */
PCIBridge *pci_p2pbr_create_simple(const PCIP2PBridgeInit *init,
                                   const PCIP2PBridgeProp *prop)
{
    PCIBridge *br = pci_p2pbr_create(init);
    pci_p2pbr_prop_set(br, prop);
    qdev_init_nofail(&br->dev.qdev);
    return br;
}
