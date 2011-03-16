/*
 * q35.h
 *
 * Copyright (c) 2009 Isaku Yamahata <yamahata at valinux co jp>
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
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>
 */

#ifndef HW_Q35_H
#define HW_Q35_H

#include "sysbus.h"
#include "acpi_ich9.h"

PCIBus *gmch_host_init(DeviceState **gmch_hostp,
                       qemu_irq *pic, qemu_irq *ioapic);

PCIDevice *gmch_init(DeviceState *gmch_host, PCIBus *b);
PCIBus *ich9_d2pbr_init(PCIBus *bus, int devfn, int sec_bus);
PCIDevice *gmch_lpc_init(DeviceState *gmch_host, PCIBus *bus);
void ich9_lpc_pm_init(DeviceState *gmch_host, PCIDevice *pci_lpc,
                      qemu_irq cmos_s3);

i2c_bus *ich9_smb_init(PCIBus *bus, int devfn, uint32_t smb_io_base);

#define Q35_MASK(bit, ms_bit, ls_bit)           ((uint##bit##_t)(((1ULL << ((ms_bit) + 1)) - 1) & ~((1ULL << ls_bit) - 1)))

/*
 * gmch part
 */

/* PCI configuration */
#define GMCH_HOST_BRIDGE                        "GMCH"

#define GMCH_HOST_BRIDGE_CONFIG_ADDR            0xcf8
#define GMCH_HOST_BRIDGE_CONFIG_DATA            0xcfc

/* D0:F0 configuration space */
#define  GMCH_HOST_BRIDGE_REVISION_DEFUALT      0x0

#define GMCH_HOST_BRIDGE_PCIEXBAR               0x60    /* 64bit register */
#define  GMCH_HOST_BRIDGE_PCIEXBAR_SIZE         8       /* 64bit register */
#define  GMCH_HOST_BRIDGE_PCIEXBAR_DEFAULT      0xe0000000
#define  GMCH_HOST_BRIDGE_PCIEXBAR_ADMSK        Q35_MASK(64, 35, 25)    /* bit 35:28 */
#define  GMCH_HOST_BRIDGE_PCIEXBAR_128ADMSK     ((uint64_t)(1 << 26))
#define  GMCH_HOST_BRIDGE_PCIEXBAR_64ADMSK      ((uint64_t)(1 << 25))
#define  GMCH_HOST_BRIDGE_PCIEXBAR_LENGTH_MASK  ((uint64_t)(0x3 << 1))
#define  GMCH_HOST_BRIDGE_PCIEXBAR_LENGTH_256M  ((uint64_t)(0x0 << 1))
#define  GMCH_HOST_BRIDGE_PCIEXBAR_LENGTH_128M  ((uint64_t)(0x1 << 1))
#define  GMCH_HOST_BRIDGE_PCIEXBAR_LENGTH_64M   ((uint64_t)(0x2 << 1))
#define  GMCH_HOST_BRIDGE_PCIEXBAR_LENGTH_RVD   ((uint64_t)(0x3 << 1))
#define  GMCH_HOST_BRIDGE_PCIEXBAREN            ((uint64_t)1)

#define GMCH_HOST_BRIDGE_PAM_NB                 7
#define GMCH_HOST_BRIDGE_PAM_SIZE               7
#define GMCH_HOST_BRIDGE_PAM0                   0x90
#define  GMCH_HOST_BRIDGE_PAM_BIOS_AREA         0xf0000
#define  GMCH_HOST_BRIDGE_PAM_AREA_SIZE         0x10000 /* 16KB */
#define GMCH_HOST_BRIDGE_PAM1                   0x91
#define  GMCH_HOST_BRIDGE_PAM_EXPAN_AREA        0xc0000
#define  GMCH_HOST_BRIDGE_PAM_EXPAN_SIZE        0x04000
#define GMCH_HOST_BRIDGE_PAM2                   0x92
#define GMCH_HOST_BRIDGE_PAM3                   0x93
#define GMCH_HOST_BRIDGE_PAM4                   0x94
#define  GMCH_HOST_BRIDGE_PAM_EXBIOS_AREA       0xe0000
#define  GMCH_HOST_BRIDGE_PAM_EXBIOS_SIZE       0x04000
#define GMCH_HOST_BRIDGE_PAM5                   0x95
#define GMCH_HOST_BRIDGE_PAM6                   0x96
#define  GMCH_HOST_BRIDGE_PAM_WE_HI             ((uint8_t)(0x2 << 4))
#define  GMCH_HOST_BRIDGE_PAM_RE_HI             ((uint8_t)(0x1 << 4))
#define  GMCH_HOST_BRIDGE_PAM_HI_MASK           ((uint8_t)(0x3 << 4))
#define  GMCH_HOST_BRIDGE_PAM_WE_LO             ((uint8_t)0x2)
#define  GMCH_HOST_BRIDGE_PAM_RE_LO             ((uint8_t)0x1)
#define  GMCH_HOST_BRIDGE_PAM_LO_MASK           ((uint8_t)0x3)
#define  GMCH_HOST_BRIDGE_PAM_WE                ((uint8_t)0x2)
#define  GMCH_HOST_BRIDGE_PAM_RE                ((uint8_t)0x1)
#define  GMCH_HOST_BRIDGE_PAM_MASK              ((uint8_t)0x3)

#define GMCH_HOST_BRDIGE_SMRAM                  0x9d
#define GMCH_HOST_BRDIGE_SMRAM_SIZE             1
#define  GMCH_HOST_BRIDGE_SMRAM_DEFAULT         ((uint8_t)0x2)
#define  GMCH_HOST_BRIDGE_SMRAM_D_OPEN          ((uint8_t)(1 << 6))
#define  GMCH_HOST_BRIDGE_SMRAM_D_CLS           ((uint8_t)(1 << 5))
#define  GMCH_HOST_BRIDGE_SMRAM_D_LCK           ((uint8_t)(1 << 4))
#define  GMCH_HOST_BRIDGE_SMRAM_G_SMRAME        ((uint8_t)(1 << 3))
#define  GMCH_HOST_BRIDGE_SMRAM_C_BASE_SEG_MASK ((uint8_t)0x7)
#define  GMCH_HOST_BRIDGE_SMRAM_C_BASE_SEG      ((uint8_t)0x2)  /* hardwired to b010 */
#define   GMCH_HOST_BRIDGE_SMRAM_C_BASE         0xa0000
#define   GMCH_HOST_BRIDGE_SMRAM_C_END          0xc0000
#define   GMCH_HOST_BRIDGE_SMRAM_C_SIZE         0x20000
#define GMCH_HOST_BRIDGE_UPPER_SYSTEM_BIOS_END  0x100000

#define GMCH_HOST_BRIDGE_ESMRAMC                0x9e
#define  GMCH_HOST_BRDIGE_ESMRAMC_H_SMRAME      ((uint8_t)(1 << 6))
#define  GMCH_HOST_BRDIGE_ESMRAMC_E_SMERR       ((uint8_t)(1 << 5))
#define  GMCH_HOST_BRDIGE_ESMRAMC_SM_CACHE      ((uint8_t)(1 << 4))
#define  GMCH_HOST_BRDIGE_ESMRAMC_SM_L1         ((uint8_t)(1 << 3))
#define  GMCH_HOST_BRDIGE_ESMRAMC_SM_L2         ((uint8_t)(1 << 2))
#define  GMCH_HOST_BRDIGE_ESMRAMC_TSEG_SZ_MASK  ((uint8_t)(0x3 << 1))
#define   GMCH_HOST_BRDIGE_ESMRAMC_TSEG_SZ_1MB  ((uint8_t)(0x0 << 1))
#define   GMCH_HOST_BRDIGE_ESMRAMC_TSEG_SZ_2MB  ((uint8_t)(0x1 << 1))
#define   GMCH_HOST_BRDIGE_ESMRAMC_TSEG_SZ_8MB  ((uint8_t)(0x2 << 1))
#define  GMCH_HOST_BRDIGE_ESMRAMC_T_EN          ((uint8_t)1)

/* D1:F0 PCIE* port*/
#define GMCH_PCIE_DEV                           1
#define GMCH_PCIE_FUNC                          0

/*
 * ich9 part
 */

/* ICH9: Chipset Configuration Registers */
#define ICH9_CC_SIZE                            (16 * 1024)     /* 16KB */
#define ICH9_CC_ADDR_MASK                       (ICH9_CC_SIZE - 1)

#define ICH9_CC
#define ICH9_CC_D28IP                           0x310C
#define  ICH9_CC_D28IP_SHIFT                    4
#define  ICH9_CC_D28IP_MASK                     0xf
#define  ICH9_CC_D28IP_DEFAULT                  0x00214321
#define ICH9_CC_D31IR                           0x3140
#define ICH9_CC_D30IR                           0x3142
#define ICH9_CC_D29IR                           0x3144
#define ICH9_CC_D28IR                           0x3146
#define ICH9_CC_D27IR                           0x3148
#define ICH9_CC_D26IR                           0x314C
#define ICH9_CC_D25IR                           0x3150
#define  ICH9_CC_DIR_DEFAULT                    0x3210
#define  ICH9_CC_D30IR_DEFAULT                  0x0
#define  ICH9_CC_DIR_SHIFT                      4
#define  ICH9_CC_DIR_MASK                       0x7
#define ICH9_CC_OIC                             0x31FF
#define  ICH9_CC_OIC_AEN                        0x1

/* D28:F[0-5] */
#define ICH9_PCIE_DEV                           28
#define ICH9_PCIE_FUNC_MAX                      6


/* D29:F0 USB UHCI Controller #1 */
#define ICH9_USB_UHCI1_DEV                      29
#define ICH9_USB_UHCI1_FUNC                     0

/* D30:F0 DMI-to-PCI brdige */
#define ICH9_D2P_BRIDGE                         "ICH9 D2P BRIDGE"
#define ICH9_D2P_BRIDGE_SAVEVM_VERSION          0

#define ICH9_D2P_BRIDGE_DEV                     30
#define ICH9_D2P_BRIDGE_FUNC                    0

#define ICH9_D2P_SECONDARY_DEFAULT              (256 - 8)

#define ICH9_D2P_A2_REVISION                    0x92


/* D31:F1 LPC controller */
#define ICH9_A2_LPC                             "ICH9 A2 LPC"
#define ICH9_A2_LPC_SAVEVM_VERSION              0

#define ICH9_LPC_DEV                            31
#define ICH9_LPC_FUNC                           0

#define ICH9_A2_LPC_REVISION                    0x2
#define ICH9_LPC_NB_PIRQS                       8       /* PCI A-H */

#define ICH9_LPC_PMBASE                         0x40
#define  ICH9_LPC_PMBASE_BASE_ADDRESS_MASK      Q35_MASK(32, 15, 7)
#define  ICH9_LPC_PMBASE_RTE                    0x1
#define  ICH9_LPC_PMBASE_DEFAULT                0x1
#define ICH9_LPC_ACPI_CTRL                      0x44
#define  ICH9_LPC_ACPI_CTRL_ACPI_EN             0x80
#define  ICH9_LPC_ACPI_CTRL_SCI_IRQ_SEL_MASK    Q35_MASK(8, 2, 0)
#define  ICH9_LPC_ACPI_CTRL_9                   0x0
#define  ICH9_LPC_ACPI_CTRL_10                  0x1
#define  ICH9_LPC_ACPI_CTRL_11                  0x2
#define  ICH9_LPC_ACPI_CTRL_20                  0x4
#define  ICH9_LPC_ACPI_CTRL_21                  0x5
#define  ICH9_LPC_ACPI_CTRL_DEFAULT             0x0

#define ICH9_LPC_PIRQA_ROUT                     0x60
#define ICH9_LPC_PIRQB_ROUT                     0x61
#define ICH9_LPC_PIRQC_ROUT                     0x62
#define ICH9_LPC_PIRQD_ROUT                     0x63

#define ICH9_LPC_PIRQE_ROUT                     0x68
#define ICH9_LPC_PIRQF_ROUT                     0x69
#define ICH9_LPC_PIRQG_ROUT                     0x6a
#define ICH9_LPC_PIRQH_ROUT                     0x6b

#define  ICH9_LPC_PIRQ_ROUT_IRQEN               0x80
#define  ICH9_LPC_PIRQ_ROUT_MASK                Q35_MASK(8, 3, 0)
#define  ICH9_LPC_PIRQ_ROUT_DEFAULT             0x80

#define ICH9_LPC_RCBA                           0xf0
#define  ICH9_LPC_RCBA_BA_MASK                  Q35_MASK(32, 31, 14)
#define  ICH9_LPC_RCBA_EN                       0x1
#define  ICH9_LPC_RCBA_DEFAULT                  0x0

#define ICH9_LPC_PIC_NUM_PINS                   16
#define ICH9_LPC_IOAPIC_NUM_PINS                24

/* D31:F2 SATA Controller #1 */
#define ICH9_SATA1_DEV                          31
#define ICH9_SATA1_FUNC                         2

/* D30:F1 power management I/O registers
   offset from the address ICH9_LPC_PMBASE */

/* ICH9 LPC PM I/O registers are 128 ports and 128-aligned */
#define ICH9_PMIO_SIZE                          128
#define ICH9_PMIO_MASK                          (ICH9_PMIO_SIZE - 1)

#define ICH9_PMIO_PM1_STS                       0x00
#define ICH9_PMIO_PM1_EN                        0x02
#define ICH9_PMIO_PM1_CNT                       0x04
#define ICH9_PMIO_PM1_TMR                       0x08
#define ICH9_PMIO_GPE0_STS                      0x20
#define ICH9_PMIO_GPE0_EN                       0x28
#define  ICH9_PMIO_GPE0_LEN                     16
#define ICH9_PMIO_SMI_EN                        0x30
#define  ICH9_PMIO_SMI_EN_APMC_EN               (1 << 5)
#define ICH9_PMIO_SMI_STS                       0x34

/* FADT ACPI_ENABLE/ACPI_DISABLE */
#define ICH9_APM_ACPI_ENABLE                    0x2
#define ICH9_APM_ACPI_DISABLE                   0x3


/* D31:F3 SMBus controller */
#define ICH9_A2_SMB_REVISION                    0x02
#define ICH9_SMB_PI                             0x00

#define ICH9_SMB_SMBMBAR0                       0x10
#define ICH9_SMB_SMBMBAR1                       0x14
#define  ICH9_SMB_SMBM_BAR                      0
#define  ICH9_SMB_SMBM_SIZE                     (1 << 8)
#define ICH9_SMB_SMB_BASE                       0x20
#define  ICH9_SMB_SMB_BASE_BAR                  4
#define  ICH9_SMB_SMB_BASE_SIZE                 (1 << 5)
#define ICH9_SMB_HOSTC                          0x40
#define  ICH9_SMB_HOSTC_SSRESET                 ((uint8_t)(1 << 3))
#define  ICH9_SMB_HOSTC_I2C_EN                  ((uint8_t)(1 << 2))
#define  ICH9_SMB_HOSTC_SMB_SMI_EN              ((uint8_t)(1 << 1))
#define  ICH9_SMB_HOSTC_HST_EN                  ((uint8_t)(1 << 0))

/* D31:F3 SMBus I/O and memory mapped I/O registers */
#define ICH9_SMB_DEV                            31
#define ICH9_SMB_FUNC                           3

#define ICH9_SMB_HST_STS                        0x00
#define ICH9_SMB_HST_CNT                        0x02
#define ICH9_SMB_HST_CMD                        0x03
#define ICH9_SMB_XMIT_SLVA                      0x04
#define ICH9_SMB_HST_D0                         0x05
#define ICH9_SMB_HST_D1                         0x06
#define ICH9_SMB_HOST_BLOCK_DB                  0x07

#endif /* HW_Q35_H */
