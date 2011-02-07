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

#ifndef QEMU_PCI_P2PBR_H
#define QEMU_PCI_P2PBR_H

#include "qdev.h"

typedef int (*pci_p2pbr_init_fn)(PCIDevice *d);

typedef struct PCIP2PBridgeInit
{
    PCIBus *bus;
    uint8_t devfn;
    bool multifunction;

    const char* bus_name;
    pci_map_irq_fn map_irq;

    pci_p2pbr_init_fn initfn;
} PCIP2PBridgeInit;

typedef struct PCIP2PBridgeProp
{
    uint16_t vendor_id;
    uint16_t device_id;
    uint8_t revision_id;
    uint8_t prog_interface;

    uint8_t ssvid_cap;
    uint8_t svid;
    uint8_t ssid;
} PCIP2PBridgeProp;

/* When setting PCIP2PBridgeProb, zero clear it for future compatibility */
PCIBridge *pci_p2pbr_create(const PCIP2PBridgeInit *init);
void pci_p2pbr_prop_set(PCIBridge *br, const PCIP2PBridgeProp *prop);

PCIBridge *pci_p2pbr_create_simple(const PCIP2PBridgeInit *init,
                                   const PCIP2PBridgeProp *prop);

#endif /* QEMU_PCI_P2PBR_H */
