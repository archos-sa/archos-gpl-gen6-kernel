/*
 * drivers/media/video/omap/isp/ispmmu.c
 *
 * Driver Library for ISP MMU module in TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2007 Texas Instruments.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>

#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/scatterlist.h>
#include <asm/irq.h>
#include <asm/semaphore.h>


#include "isp.h"
#include "ispreg.h"
#include "ispmmu.h"


#define ISPMMU_L1D_TYPE_SHIFT		0
#define ISPMMU_L1D_TYPE_MASK		0x3
#define ISPMMU_L1D_TYPE_FAULT		0
#define ISPMMU_L1D_TYPE_FAULT1	3
#define ISPMMU_L1D_TYPE_PAGE		1
#define ISPMMU_L1D_TYPE_SECTION	2
#define ISPMMU_L1D_PAGE_ADDR_SHIFT	10

#define ISPMMU_L2D_TYPE_SHIFT		0
#define ISPMMU_L2D_TYPE_MASK		0x3
#define ISPMMU_L2D_TYPE_FAULT		0
#define ISPMMU_L2D_TYPE_LARGE_PAGE	1
#define ISPMMU_L2D_TYPE_SMALL_PAGE	2
#define ISPMMU_L2D_SMALL_ADDR_SHIFT	12
#define ISPMMU_L2D_SMALL_ADDR_MASK	0xFFFFF000
#define ISPMMU_L2D_M_ACCESSBASED	(1<<11)
#define ISPMMU_L2D_E_BIGENDIAN	(1<<9)
#define ISPMMU_L2D_ES_SHIFT		4
#define ISPMMU_L2D_ES_MASK		~(3<<4)
#define ISPMMU_L2D_ES_8BIT		0
#define ISPMMU_L2D_ES_16BIT		1
#define ISPMMU_L2D_ES_32BIT		2
#define ISPMMU_L2D_ES_NOENCONV	3

#define ISPMMU_TTB_ENTRIES_NR		4096

/* Number 1MB entries in TTB in one 32MB region */
#define ISPMMU_REGION_ENTRIES_NR	32

/* 128 region entries */
#define ISPMMU_REGION_NR		(ISPMMU_TTB_ENTRIES_NR 		\
					/ISPMMU_REGION_ENTRIES_NR)

/* Each region is 32MB */
#define ISPMMU_REGION_SIZE		(ISPMMU_REGION_ENTRIES_NR*(1<<20))

/* Number of entries per L2 Page table */
#define ISPMMU_L2D_ENTRIES_NR		256

/*
 * Statically allocate 16KB for L2 page tables. 16KB can be used for
 * up to 16 L2 page tables which cover up to 16MB space. We use an array of 16
 * to keep track of these 16 L2 page table's status. 
 */
#define L2P_TABLE_SIZE		1024
#define L2P_TABLE_NR 		41 // Currently supports 4*5MP shots
#define L2P_TABLES_SIZE 	(L2P_TABLE_SIZE*L2P_TABLE_NR)

/* Extra memory allocated to get ttb aligned on 16KB */
#define ISPMMU_TTB_MISALIGN_SIZE	0x3000

/* Page structure for statically allocated l1 and l2 page tables */
static struct page *ttb_page;
static struct page *l2p_page;

/*
* Allocate the same number as of TTB entries for easy tracking 
* even though L2P tables are limited to 16 or so 
*/
static u32 l2p_table_addr[4096];

/* An array of flags to keep the L2P table allotted */
static int l2p_table_allotted[L2P_TABLE_NR];

/* TTB virtual and physical address */
static u32 *ttb, ttb_p;

/* Worst case allocation for TTB for 16KB alignment */
static u32 ttb_aligned_size;

/* L2 page table base virtural and physical address */
static u32 l2_page_cache, l2_page_cache_p;

/* Structure for Mapping Attributes in the L1, L2 descriptor*/
struct ispmmu_mapattr{
	enum ISPMMU_MAP_ENDIAN endianism;
	enum ISPMMU_MAP_ELEMENTSIZE element_size;
	enum ISPMMU_MAP_MIXEDREGION mixed_size;
	enum ISPMMU_MAP_SIZE map_size;
};

static struct ispmmu_mapattr l1_mapattr_obj, l2_mapattr_obj;

/* Structure for saving/restoring mmu module registers*/
static struct isp_reg ispmmu_reg_list[]={ 
	{ISPMMU_SYSCONFIG, 0x0000},
	{ISPMMU_IRQENABLE, 0x0000},
	{ISPMMU_CNTL, 0x0000},
	{ISPMMU_TTB, 0x0000},
	{ISPMMU_LOCK, 0x0000},
	{ISPMMU_LD_TLB, 0x0000},
	{ISPMMU_CAM, 0x0000}, 
	{ISPMMU_RAM, 0x0000},
	{ISPMMU_GFLUSH, 0x0000},
	{ISPMMU_FLUSH_ENTRY, 0x0000},
	{ISP_TOK_TERM, 0x0000}
};

/*
 * Sets the L1,L2 descriptor with section/supersection/Largepage/Smallpage
 * base address or with L2 Page table address depending on the size parameter.
 * Returns the written L1/L2 descriptor.
 * pte_addr	: Pointer to the Indexed address in the L1 Page table ie TTB.
 * phy_addr	: Section/Supersection/L2page table physical address.
 * mapattr	: Mapping attributes applicable for Section/Supersections.
 */
static u32 
ispmmu_set_pte(u32* pte_addr, u32 phy_addr, struct ispmmu_mapattr mapattr)
{
	u32 pte =0;

	switch(mapattr.map_size){
		case PAGE :
			pte = ISPMMU_L1D_TYPE_PAGE << ISPMMU_L1D_TYPE_SHIFT;
			pte |= (phy_addr >> ISPMMU_L1D_PAGE_ADDR_SHIFT) 
				<< ISPMMU_L1D_PAGE_ADDR_SHIFT;
			break;
		case SMALLPAGE:
			pte = ISPMMU_L2D_TYPE_SMALL_PAGE <<
				ISPMMU_L2D_TYPE_SHIFT;
			pte &= ~ISPMMU_L2D_M_ACCESSBASED;
			if(mapattr.endianism)
				pte |= ISPMMU_L2D_E_BIGENDIAN ;
			else
				pte &= ~ISPMMU_L2D_E_BIGENDIAN ;
			pte &= ISPMMU_L2D_ES_MASK;
			pte |= mapattr.element_size << ISPMMU_L2D_ES_SHIFT;
			pte |= (phy_addr >> ISPMMU_L2D_SMALL_ADDR_SHIFT)
					<< ISPMMU_L2D_SMALL_ADDR_SHIFT;
			break;
		case L1DFAULT:
			pte = ISPMMU_L1D_TYPE_FAULT << ISPMMU_L1D_TYPE_SHIFT;
			break;
		case L2DFAULT:
			pte = ISPMMU_L2D_TYPE_FAULT << ISPMMU_L2D_TYPE_SHIFT;
			break;
		default:
			break;
	};

	*pte_addr = pte;
	return pte;
}

/*
 * Returns the index in the ttb for a free 32MB region
 * Returns 0 as an error code, if run out of regions.
 */
static u32
find_free_region_index(void)
{
	int idx =0;
	/* Find the first free 32M region in ttb. skip region 0 to avoid NULL pointer */
	for (idx = ISPMMU_REGION_ENTRIES_NR; idx < ISPMMU_TTB_ENTRIES_NR;
			idx += ISPMMU_REGION_ENTRIES_NR){
		if (((*(ttb + idx)) & ISPMMU_L1D_TYPE_MASK) ==
			(ISPMMU_L1D_TYPE_FAULT << ISPMMU_L1D_TYPE_SHIFT))
			break;
	}
	if (idx == ISPMMU_TTB_ENTRIES_NR) {
		DPRINTK_ISPMMU("run out of virtual space\n");
		return 0;
	}
	return idx;
}

/*
 * Returns the Page aligned address
 * addr		:Address to be page aligned
 */
static inline u32
page_aligned_addr(u32 addr)
{
	u32 paddress;
	paddress = addr & ~(PAGE_SIZE-1) ;
	return paddress;
}


/*
 * Returns the physical address of the allocated L2 page Table.
 * l2_table	: Virtual address of the allocated l2 table.
 */
static inline u32
l2_page_paddr(u32 l2_table)
{
	return (l2_page_cache_p + (l2_table - l2_page_cache));
}

/*
 * Allocates contigous memory for L2 page tables.
 */
static int
init_l2_page_cache(void)
{
	int i;
	u32 *l2p;

	l2p_page = alloc_pages(GFP_KERNEL,get_order(L2P_TABLES_SIZE));
	if (!l2p_page){
		DPRINTK_ISPMMU("ISP_ERR : No Memory for L2 page tables\n");
		return -ENOMEM;
	}
	l2p = page_address(l2p_page);
	l2_page_cache = (u32)l2p;
	l2_page_cache_p = __pa(l2p);
	l2_page_cache = (u32)ioremap_nocache(l2_page_cache_p,L2P_TABLES_SIZE);
	
	for (i = 0; i < L2P_TABLE_NR; i++)
		l2p_table_allotted[i] = 0;

	DPRINTK_ISPMMU("Mem for L2 page tables at l2_paddr = %x, \
			l2_vaddr = 0x%x, of bytes = 0x%x\n",
		l2_page_cache_p, l2_page_cache,L2P_TABLES_SIZE);
	/*HW Errata 1.40. Camera ISP: MMU endianess polarity inverted */
	if(is_sil_rev_less_than(OMAP3430_REV_ES2_0))
		l2_mapattr_obj.endianism = B_ENDIAN;
	else
		l2_mapattr_obj.endianism = L_ENDIAN;
	l2_mapattr_obj.element_size = ES_8BIT;
	l2_mapattr_obj.mixed_size = ACCESS_BASED;
	l2_mapattr_obj.map_size = L2DFAULT;
	return 0;
}

/*
 * Frees the memory of L2 page tables.
 */
static void
cleanup_l2_page_cache(void)
{
	if(l2p_page){
		ioremap_cached(l2_page_cache_p,L2P_TABLES_SIZE);
		__free_pages(l2p_page,get_order(L2P_TABLES_SIZE));
	}
}

/*
 * Finds the free L2 Page table slot.
 * Fills the allotted L2 Page table with default entries.
 * Returns the virtual address of the allotted L2 Pagetable,
 */
static u32
request_l2_page_table(void)
{
	int i,j;
	u32 l2_table;

	for (i = 0; i < L2P_TABLE_NR; i++) {
		if (!l2p_table_allotted[i])
			break;
	}
	if (i < L2P_TABLE_NR) {
		l2p_table_allotted[i] = 1;
		l2_table = l2_page_cache + (i * L2P_TABLE_SIZE);
		l2_mapattr_obj.map_size = L2DFAULT;
		/*Fill up all the entries with fault */
		for(j=0;j<ISPMMU_L2D_ENTRIES_NR;j++)
			ispmmu_set_pte((u32*)l2_table+j,0,l2_mapattr_obj);
		DPRINTK_ISPMMU("Allotted l2 page table at 0x%x\n",
					(u32)l2_table);
		return l2_table;
	}
	else{
		DPRINTK_ISPMMU("ISP_ERR : Cannot allocate more than 16 L2\
				Page Tables");
		return 0;
	}
}

/*
 * Frees the allotted L2 Page table slot.
 */
static int
free_l2_page_table(u32 l2_table)
{
	int i;

	DPRINTK_ISPMMU("Free l2 page table at 0x%x\n", l2_table);
	for (i = 0; i < L2P_TABLE_NR; i++)
		if (l2_table == (l2_page_cache + (i * L2P_TABLE_SIZE))) {
			if (!l2p_table_allotted[i]){
				DPRINTK_ISPMMU("L2 page not in use\n");
			}
			l2p_table_allotted[i] = 0;
			return 0;
		}
	DPRINTK_ISPMMU("L2 table not found\n");
	return -EINVAL;
}

/*
 * Map a physically contiguous buffer to ISP space. This call is used to
 * map a frame buffer
 * p_addr	: Physical address of the contigous mem to be mapped.
 * size		: Size of the contigous mem to be mapped.
 */
dma_addr_t
ispmmu_map(u32 p_addr, int size)
{
	int i, j, idx, num;
	u32 sz,first_padding;
	u32 p_addr_align, p_addr_align_end;
	u32 pd;
	u32 *l2_table;

	DPRINTK_ISPMMU("map: p_addr = 0x%x, size = 0x%x\n",p_addr,size);

	p_addr_align = page_aligned_addr(p_addr);

	first_padding = p_addr - p_addr_align;
	sz = size -first_padding;

	num = (sz/PAGE_SIZE) + ((sz%PAGE_SIZE)?1:0) + (first_padding ?1:0);
	p_addr_align_end = p_addr_align + num*PAGE_SIZE;

	DPRINTK_ISPMMU("buffer at 0x%x of size 0x%x spans to %d pages\n",
			p_addr, size, num);

	idx = find_free_region_index();
	if(!idx){
		DPRINTK_ISPMMU("Runs out of virtual space");
		return -EINVAL;
	}
	DPRINTK_ISPMMU("allocating region %d\n", idx/ISPMMU_REGION_ENTRIES_NR);

	/* how many second-level page tables we need */
	num = num/ISPMMU_L2D_ENTRIES_NR +
			((num%ISPMMU_L2D_ENTRIES_NR)?1:0);
	DPRINTK_ISPMMU("need %d second-level page tables (1KB each)\n", num);

	/* create second-level page tables */
	for (i = 0; i < num; i++) {
		l2_table = (u32 *)request_l2_page_table();
		if (!l2_table) {
			DPRINTK_ISPMMU("no memory\n");
			i--;
			goto release_mem;
		}

		/* Make the first level page descriptor */
		l1_mapattr_obj.map_size = PAGE;
		pd = ispmmu_set_pte(ttb+idx+i, l2_page_paddr((u32)l2_table),
			l1_mapattr_obj);
		DPRINTK_ISPMMU("L1 pte[%d] = 0x%x\n", idx+i, pd);

		/* Make the second Level page descriptors */
		l2_mapattr_obj.map_size = SMALLPAGE;
		for (j = 0; j < ISPMMU_L2D_ENTRIES_NR; j++) {
			pd = ispmmu_set_pte(l2_table + j,p_addr_align,
				l2_mapattr_obj);
			//DPRINTK_ISPMMU("L2 pte[%d] = 0x%x\n", j, pd);
			/*Contigous memory, just increment with Page size */
			p_addr_align += PAGE_SIZE;
			if (p_addr_align == p_addr_align_end)
				break;
		}
		/* save it so we can free this l2 table later */
		l2p_table_addr[idx + i] = (u32)l2_table;
	}

	DPRINTK_ISPMMU("mapped to ISP virtual address 0x%x\n",
		(u32)((idx << 20) + (p_addr & (PAGE_SIZE - 1))));

	omap_writel(1, ISPMMU_GFLUSH);
	return (dma_addr_t)((idx<<20) + (p_addr & (PAGE_SIZE - 1)));

release_mem:
	for (; i >= 0; i--) {
		free_l2_page_table(l2p_table_addr[idx + i]);
		l2p_table_addr[idx + i] = 0;
	}
	return 0;
}

/*
 * Map a physically discontiguous buffer to ISP space. This call is used to
 * map a user buffer or a vmalloc buffer. The sg list is a set of pages.
 * sg_list		: Address of the Scatter gather linked list.
 * sglen		: Number of elements in the sg list.
 */
dma_addr_t
ispmmu_map_sg(const struct scatterlist *sglist, int sglen)
{
	int i, j, idx, num, sg_num = 0;
	u32 pd,sg_element_addr;
	u32 *l2_table;

	DPRINTK_ISPMMU("Map_sg: sglen (num of pages) = %d\n", sglen);

	idx = find_free_region_index();
	if(!idx){
		DPRINTK_ISPMMU("Runs out of virtual space");
		return -EINVAL;
	}

	DPRINTK_ISPMMU("allocating region %d\n", idx/ISPMMU_REGION_ENTRIES_NR);

	/* How many second-level page tables we need */
	/*
	* Size of each sglist element does not exceed a page size
	* so consider the number of elements in the list for calcuating
	* number of L2P tables
	*/
	num = sglen/ISPMMU_L2D_ENTRIES_NR +
			((sglen%ISPMMU_L2D_ENTRIES_NR)?1:0);
	DPRINTK_ISPMMU("Need %d second-level page tables (1KB each)\n", num);

	/* create second-level page tables */
	for (i = 0; i < num; i++) {
		l2_table = (u32 *)request_l2_page_table();
		if (!l2_table) {
			DPRINTK_ISPMMU("No memory\n");
			i--;
			goto release_mem;
		}
		/* Make the first level page descriptor */
		l1_mapattr_obj.map_size = PAGE;
		pd = ispmmu_set_pte(ttb+idx+i, l2_page_paddr((u32)l2_table),
			l1_mapattr_obj);
		DPRINTK_ISPMMU("L1 pte[%d] = 0x%x\n", idx+i, pd);

		/* Make the second Level page descriptors */
		l2_mapattr_obj.map_size = SMALLPAGE;
		for (j = 0; j < ISPMMU_L2D_ENTRIES_NR; j++) {
			/*Assuming that sglist elements are always page aligned */
			sg_element_addr = sg_dma_address(sglist + sg_num);
			if((sg_num>0) &&
				page_aligned_addr(sg_element_addr)
				!= sg_element_addr)
				DPRINTK_ISPMMU("ISP_ERR : Intermediate SG elements  \
					are not page aligned = 0x%x\n"
					,sg_element_addr);
			pd = ispmmu_set_pte(l2_table+j,sg_element_addr,l2_mapattr_obj);

			//DPRINTK_ISPMMU("L2 pte[%d] = 0x%x\n", j, pd);

			sg_num++;
			if (sg_num == sglen)
				break;
		}
		/* save it so we can free this l2 table later */
		l2p_table_addr[idx + i] = (u32)l2_table;
	}

	DPRINTK_ISPMMU("mapped sg list to ISP virtual address 0x%x, idx=%d\n",
		(u32)((idx << 20) + (sg_dma_address(sglist+0) & (PAGE_SIZE - 1))), idx);

	omap_writel(1, ISPMMU_GFLUSH);
	return (dma_addr_t)((idx<<20) + (sg_dma_address(sglist+0) & (PAGE_SIZE - 1)));

release_mem:
	for (; i >= 0; i--) {
		free_l2_page_table(l2p_table_addr[idx + i]);
		l2p_table_addr[idx + i] = 0;
	}
	return 0;
}

/*
 * Unmap a ISP space that is mapped before via ispmmu_map and
 * ispmmu_map_sg.
 * v_addr	: Virtural address to be unmapped
 */
int
ispmmu_unmap(dma_addr_t v_addr)
{
	u32 v_addr_align;
	int idx;

	DPRINTK_ISPMMU("+ispmmu_unmap: 0x%x\n", v_addr);

	v_addr_align = page_aligned_addr(v_addr);
	idx = v_addr_align >> 20;
	if ((idx < ISPMMU_REGION_ENTRIES_NR) ||
		(idx >(ISPMMU_REGION_ENTRIES_NR*(ISPMMU_REGION_NR -1)))
		|| ((idx << 20) != v_addr_align)
		|| (idx%ISPMMU_REGION_ENTRIES_NR)) {
		DPRINTK_ISPMMU("Cannot unmap a non region-aligned space \
				0x%x\n", v_addr);
		return -EINVAL;
	}

	if (((*(ttb + idx)) & (ISPMMU_L1D_TYPE_MASK << ISPMMU_L1D_TYPE_SHIFT)) !=
			(ISPMMU_L1D_TYPE_PAGE << ISPMMU_L1D_TYPE_SHIFT)) {
		DPRINTK_ISPMMU("unmap a wrong region\n");
		return -EINVAL;
	}

	/* free the associated level-2 page tables */
	while (((*(ttb + idx)) & (ISPMMU_L1D_TYPE_MASK << ISPMMU_L1D_TYPE_SHIFT)) ==
			(ISPMMU_L1D_TYPE_PAGE << ISPMMU_L1D_TYPE_SHIFT)) {
		*(ttb + idx) = (ISPMMU_L1D_TYPE_FAULT << ISPMMU_L1D_TYPE_SHIFT);
		free_l2_page_table(l2p_table_addr[idx]);
		l2p_table_addr[idx++] = 0;
		if (!(idx%ISPMMU_REGION_ENTRIES_NR)) {
			DPRINTK_ISPMMU("Do not exceed this 32M region\n");
			break;
		}
	}
	omap_writel(1, ISPMMU_GFLUSH);

	DPRINTK_ISPMMU("-ispmmu_unmap()\n");
	return 0;
}

/*
 * Callback from ISP driver for MMU interrupt
 * status 	: IRQ status of ISPMMU
 * arg1		: Not used as of now.
 * arg2		: Not used as of now.
 */
static void
ispmmu_isr(unsigned long status, void *arg1, void *arg2)
{
	u32 irqstatus=0;

	irqstatus = omap_readl(ISPMMU_IRQSTATUS);
	DPRINTK_ISPMMU("mmu error 0x%lx, 0x%x\n", status, irqstatus);
	if(irqstatus & IRQENABLE_TLBMISS)
		DPRINTK_ISPMMU("ISP_ERR: TLB Miss\n");
	if(irqstatus & IRQENABLE_TRANSLNFAULT)
		DPRINTK_ISPMMU("ISP_ERR: Invalid descriptor in the translation table - Translation Fault\n");
	if(irqstatus & IRQENABLE_EMUMISS)
		DPRINTK_ISPMMU("ISP_ERR: TLB Miss during debug - Emulation mode\n");
	if(irqstatus & IRQENABLE_TWFAULT)
		DPRINTK_ISPMMU("ISP_ERR: Table Walk Fault\n");
	if(irqstatus & IRQENABLE_MULTIHITFAULT)
		DPRINTK_ISPMMU("ISP_ERR: Multiple Matches in the TLB\n");
	DPRINTK_ISPMMU("Fault address for the ISPMMU is 0x%x",
			omap_readl(ISPMMU_FAULT_AD));
	/*
	* TODO: Indicate the camera driver about the fault and it should
	* stop using the ISP
	*/
	omap_writel(irqstatus, ISPMMU_IRQSTATUS);
}

/*
 * Reserves memory for L1 and L2 Page tables.
 * Initializes the ISPMMU with TTB address, fault entries as default in the TTB table.
 * Enables MMU and TWL.
 * Sets the callback for the MMU error events.
 */
int ispmmu_init(void)
{
	int i, val = 0;
	struct isp_sysc isp_sysconfig;

	isp_get();

	/* reset */
	omap_writel(2, ISPMMU_SYSCONFIG);
	while (((omap_readl(ISPMMU_SYSSTATUS) & 0x1) != 0x1) && val--) {
		udelay(10);
	};
	if ((omap_readl(ISPMMU_SYSSTATUS) & 0x1) != 0x1) {
		DPRINTK_ISPMMU("can't take ISP MMU out of reset\n");
		isp_put();
		return -ENODEV;
	}
	isp_sysconfig.reset = 0;
	isp_sysconfig.idle_mode = 1;
	isp_power_settings(isp_sysconfig);

	ttb_page = alloc_pages(GFP_KERNEL,
				get_order(ISPMMU_TTB_ENTRIES_NR * 4));
	if (!ttb_page){
		DPRINTK_ISPMMU("No Memory for TTB\n");
		isp_put();
		return -ENOMEM;
	}

	ttb = page_address(ttb_page);
	ttb_p = __pa(ttb);
	ttb_aligned_size = ISPMMU_TTB_ENTRIES_NR * 4;
	ttb = ioremap_nocache(ttb_p,ttb_aligned_size);
	if ((ttb_p & 0xFFFFC000) != ttb_p) {
		DPRINTK_ISPMMU("ISP_ERR : TTB address not aligned at 16KB\n");
		__free_pages(ttb_page,get_order(ISPMMU_TTB_ENTRIES_NR * 4));
		ttb_aligned_size = (ISPMMU_TTB_ENTRIES_NR * 4)
				+ (ISPMMU_TTB_MISALIGN_SIZE);
		ttb_page = alloc_pages(GFP_KERNEL,
					get_order(ttb_aligned_size));
		if (!ttb_page){
			DPRINTK_ISPMMU("No Memory for TTB\n");
			isp_put();
			return -ENOMEM;
		}
		ttb = page_address(ttb_page);
		ttb_p = __pa(ttb);
		ttb = ioremap_nocache(ttb_p,ttb_aligned_size);
		if ((ttb_p & 0xFFFFC000) != ttb_p){
			/* Move the unaligned address to the next 16KB alignment */
			ttb = (u32*)(((u32)ttb & 0xFFFFC000) + 0x4000);
			ttb_p = __pa(ttb);
		}
	}



	
	DPRINTK_ISPMMU("TTB allocated at p = 0x%x, v = 0x%x, size = 0x%x\n",
		ttb_p, (u32)ttb, ttb_aligned_size);
	/*HW Errata 1.40. Camera ISP: MMU endianess polarity inverted */
	if(is_sil_rev_less_than(OMAP3430_REV_ES2_0))
		l1_mapattr_obj.endianism = B_ENDIAN;
	else
		l1_mapattr_obj.endianism = L_ENDIAN;
	l1_mapattr_obj.element_size = ES_8BIT;
	l1_mapattr_obj.mixed_size = ACCESS_BASED;
	l1_mapattr_obj.map_size = L1DFAULT;

	if ((val = init_l2_page_cache())) {
		DPRINTK_ISPMMU("ISP_ERR : init l2 page cache\n");
		ttb = page_address(ttb_page);
		ttb_p = __pa(ttb);
		ioremap_cached(ttb_p,ttb_aligned_size);
		__free_pages(ttb_page,get_order(ttb_aligned_size));

		isp_put();
		return val;
	}

	/* Setting all the entries to generate fault by default */
	for (i = 0; i < ISPMMU_TTB_ENTRIES_NR; i++) {
		ispmmu_set_pte(ttb + i, 0, l1_mapattr_obj);
	}

	/*
	* TTB 31:7 is the address, since TTB is on 16KB boundary the last
	* 14 bits are 0
	*/
	omap_writel(ttb_p, ISPMMU_TTB);

	/* Enable MMU with table walking logic */
	omap_writel((ISPMMU_MMUCNTL_MMU_EN|ISPMMU_MMUCNTL_TWL_EN),
			ISPMMU_CNTL);
	omap_writel(omap_readl(ISPMMU_IRQSTATUS), ISPMMU_IRQSTATUS);
	omap_writel(0xf, ISPMMU_IRQENABLE);

	isp_set_callback(CBK_MMU_ERR, ispmmu_isr, (void*)NULL, (void*)NULL);

	val = omap_readl(ISPMMU_REVISION);
	DPRINTK_ISPMMU("ISP MMU Rev %c.%c initialized\n",
		(val>>ISPMMU_REVISION_REV_MAJOR_SHIFT)+'0',
		(val & ISPMMU_REVISION_REV_MINOR_MASK)+'0');
	/* Release the clocks now */
	isp_put();
	return 0;

}

/*
 * Frees the L1 and L2 Page tables.
 * Unsets the callback for MMU
 */
void ispmmu_cleanup(void)
{

	/* free ttb */
	ttb = page_address(ttb_page);
	ttb_p = __pa(ttb);
	ioremap_cached(ttb_p,ttb_aligned_size);
	__free_pages(ttb_page,get_order(ttb_aligned_size));

	isp_unset_callback(CBK_MMU_ERR);

	cleanup_l2_page_cache();

	return;
}

/*
 * Saves the values of the mmu module registers.
 */
void 
ispmmu_save_context(void)
{
	DPRINTK_ISPMMU (" Saving context\n");
	isp_save_context(ispmmu_reg_list);	
}

/*
 * Restores the values of the mmu module registers.
 */
void 
ispmmu_restore_context(void)
{
	DPRINTK_ISPMMU (" Restoring context\n");
	isp_restore_context(ispmmu_reg_list);		
}

/*
 * Prints the values of the ISPMMU registers
 * Also prints other debug information stored
 */
void
ispmmu_print_status(void)
{
#ifdef OMAP_ISPMMU_DEBUG
	DPRINTK_ISPMMU("TTB v_addr = 0x%x, p_addr = 0x%x\n",(u32)ttb,ttb_p);
	DPRINTK_ISPMMU("L2P base v_addr = 0x%x, p_addr = 0x%x\n"
				,l2_page_cache,l2_page_cache_p);
	DPRINTK_ISPMMU("ISPMMU_REVISION = 0x%x\n", omap_readl(ISPMMU_REVISION));
	DPRINTK_ISPMMU("ISPMMU_SYSCONFIG = 0x%x\n", omap_readl(ISPMMU_SYSCONFIG));
	DPRINTK_ISPMMU("ISPMMU_SYSSTATUS = 0x%x\n", omap_readl(ISPMMU_SYSSTATUS));
	DPRINTK_ISPMMU("ISPMMU_IRQSTATUS = 0x%x\n", omap_readl(ISPMMU_IRQSTATUS));
	DPRINTK_ISPMMU("ISPMMU_IRQENABLE = 0x%x\n", omap_readl(ISPMMU_IRQENABLE));
	DPRINTK_ISPMMU("ISPMMU_WALKING_ST = 0x%x\n", omap_readl(ISPMMU_WALKING_ST));
	DPRINTK_ISPMMU("ISPMMU_CNTL = 0x%x\n", omap_readl(ISPMMU_CNTL));
	DPRINTK_ISPMMU("ISPMMU_FAULT_AD = 0x%x\n", omap_readl(ISPMMU_FAULT_AD));
	DPRINTK_ISPMMU("ISPMMU_TTB = 0x%x\n", omap_readl(ISPMMU_TTB));
	DPRINTK_ISPMMU("ISPMMU_LOCK = 0x%x\n", omap_readl(ISPMMU_LOCK));
	DPRINTK_ISPMMU("ISPMMU_LD_TLB= 0x%x\n", omap_readl(ISPMMU_LD_TLB));
	DPRINTK_ISPMMU("ISPMMU_CAM = 0x%x\n", omap_readl(ISPMMU_CAM));
	DPRINTK_ISPMMU("ISPMMU_RAM = 0x%x\n", omap_readl(ISPMMU_RAM));
	DPRINTK_ISPMMU("ISPMMU_GFLUSH = 0x%x\n", omap_readl(ISPMMU_GFLUSH));
	DPRINTK_ISPMMU("ISPMMU_FLUSH_ENTRY = 0x%x\n", omap_readl(ISPMMU_FLUSH_ENTRY));
	DPRINTK_ISPMMU("ISPMMU_READ_CAM = 0x%x\n", omap_readl(ISPMMU_READ_CAM));
	DPRINTK_ISPMMU("ISPMMU_READ_RAM = 0x%x\n", omap_readl(ISPMMU_READ_RAM));
#endif
}

EXPORT_SYMBOL_GPL(ispmmu_init);
EXPORT_SYMBOL_GPL(ispmmu_cleanup);
EXPORT_SYMBOL_GPL(ispmmu_map);
EXPORT_SYMBOL_GPL(ispmmu_map_sg);
EXPORT_SYMBOL_GPL(ispmmu_unmap);
EXPORT_SYMBOL_GPL(ispmmu_save_context);
EXPORT_SYMBOL_GPL(ispmmu_restore_context);
EXPORT_SYMBOL_GPL(ispmmu_print_status);

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("OMAP3430 ISP MMU Driver");
MODULE_LICENSE("GPL");



