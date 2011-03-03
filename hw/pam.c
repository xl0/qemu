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
 *
 * Split out from piix_pci.c
 * Copyright (c) 2011 Isaku Yamahata <yamahata at valinux co jp>
 *                    VA Linux Systems Japan K.K.
 */

#include "sysemu.h"
#include "pam.h"

/* XXX: suppress when better memory API. We make the assumption that
   no device (in particular the VGA) changes the memory mappings in
   the 0xa0000-0x100000 range */
void pam_init_memory_mappings(PAM *pam)
{
    int i;
    for(i = 0; i < ARRAY_SIZE(pam->isa_page_descs); i++) {
        pam->isa_page_descs[i] =
            cpu_get_physical_page_desc(SMRAM_C_BASE + (i << TARGET_PAGE_BITS));
    }
}

static target_phys_addr_t isa_page_descs_get(PAM *pam, uint32_t addr)
{
    return pam->isa_page_descs[(addr - SMRAM_C_BASE) >> TARGET_PAGE_BITS];
}

void smram_update(PAM *pam, uint8_t smram)
{
    if ((pam->smm_enabled && (smram & SMRAM_G_SMRAME)) ||
        (smram & SMRAM_D_OPEN)) {
        cpu_register_physical_memory(SMRAM_C_BASE, SMRAM_C_SIZE, SMRAM_C_BASE);
    } else {
        uint32_t addr;
        for(addr = SMRAM_C_BASE;
            addr < SMRAM_C_END; addr += TARGET_PAGE_SIZE) {
            cpu_register_physical_memory(addr, TARGET_PAGE_SIZE,
                                         isa_page_descs_get(pam, addr));
        }
    }
}

void smram_set_smm(PAM *pam, int smm, uint8_t smram)
{
    uint8_t smm_enabled = (smm != 0);
    if (pam->smm_enabled != smm_enabled) {
        pam->smm_enabled = smm_enabled;
        smram_update(pam, smram);
    }
}

static void pam_update_seg(struct PAM *pam,
                           uint32_t start, uint32_t size, uint8_t attr)
{
    uint32_t addr;

#if 0
    printf("ISA mapping %08"PRIx32"-0x%08"PRIx32": %"PRId32"\n",
           start, start + size, attr);
#endif
    switch(attr) {
    case PAM_ATTR_WE | PAM_ATTR_RE:
        /* RAM */
        cpu_register_physical_memory(start, size, start);
        break;

    case PAM_ATTR_RE:
        /* ROM (XXX: not quite correct) */
        cpu_register_physical_memory(start, size, start | IO_MEM_ROM);
        break;

    case PAM_ATTR_WE:
    case 0: /* XXX: should distinguish read/write cases */
        for(addr = start; addr < start + size; addr += TARGET_PAGE_SIZE) {
            cpu_register_physical_memory(addr, TARGET_PAGE_SIZE,
                                         isa_page_descs_get(pam, addr));
        }
        break;

    default:
        abort();
        break;
    }
}

static uint8_t pam_attr(uint8_t val, int hi)
{
    return (val >> ((!!hi) * 4)) & PAM_ATTR_MASK;
}

void pam_update(PAM *pam, int idx, uint8_t val)
{
    uint32_t phys_addr;

    assert(0 <= idx && idx <= PAM_IDX_MAX);

    if (idx == 0) {
        pam_update_seg(pam, PAM_BIOS_BASE, PAM_BIOS_SIZE, pam_attr(val, 1));
        return;
    }

    phys_addr = PAM_EXPAN_BASE + PAM_EXPAN_SIZE * (idx - 1) * 2;
    pam_update_seg(pam, phys_addr, PAM_EXPAN_SIZE, pam_attr(val, 0));

    phys_addr += PAM_EXPAN_SIZE;
    pam_update_seg(pam, phys_addr, PAM_EXPAN_SIZE, pam_attr(val, 1));
}
