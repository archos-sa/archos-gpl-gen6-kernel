#ifndef _LINUX_SCATTERLIST_H
#define _LINUX_SCATTERLIST_H

#include <asm/scatterlist.h>
#include <linux/mm.h>
#include <linux/string.h>


/**
 * sg_set_buf - Set sg entry to point at given data
 * @sg:		 SG entry
 * @buf:	 Data
 * @buflen:	 Data length
 *
 **/
static inline void sg_set_buf(struct scatterlist *sg, const void *buf,
			      unsigned int buflen)
{
	sg->page = virt_to_page(buf);
	sg->offset = offset_in_page(buf);
	sg->length = buflen;
}

/**
 * sg_init_one - Initialize a single entry sg list
 * @sg:		 SG entry
 * @buf:	 Virtual address for IO
 * @buflen:	 IO length
 *
 * Notes:
 *   This should not be used on a single entry that is part of a larger
 *   table. Use sg_init_table() for that.
 *
 **/
static inline void sg_init_one(struct scatterlist *sg, const void *buf,
			       unsigned int buflen)
{
	memset(sg, 0, sizeof(*sg));
	sg_set_buf(sg, buf, buflen);
}

/**
 * sg_virt - Return virtual address of an sg entry
 * @sg:      SG entry
 *
 * Description:
 *   This calls page_address() on the page in this sg entry, and adds the
 *   sg offset. The caller must know that the sg page has a valid virtual
 *   mapping.
 *
 **/
static inline void *sg_virt(struct scatterlist *sg)
{
	return page_address(sg->page) + sg->offset;
}

#endif /* _LINUX_SCATTERLIST_H */
