#ifndef _LIBATA_CEATA_H
#define _LIBATA_CEATA_H

#include <linux/libata.h>
#include <linux/mmc/card.h>
#include <linux/mmc/host.h>

struct rw_queue_entry {
	struct list_head list;
	struct ata_queued_cmd *qc;
};

struct ceata_port {
	const char *mmc_id;
	struct mmc_host	*mmc;
	struct ata_port *ap;
	void *private_data;
	void *internal_data;		
	
	void (*wake_up_hook)(struct ceata_port *);
	void (*reset)(struct ceata_port *);
	int io_count;
	int hd_access;
	int fio;
	int nbr;
	unsigned int err_flags;
	
	wait_queue_head_t wait_queue;	
	struct list_head q;
	struct rw_queue_entry qe[ATA_MAX_QUEUE];		
	spinlock_t rw_lock;
	int queue_count;

	struct task_struct	*rw_thread;
	u8 			*tf_buf;
};

#define CEATA_ERR_HANG	1

struct ceata_host {
	struct list_head list;
	struct device *dev;	
	struct ata_host *ata;
	struct ceata_port *ports;
	int n_ports;
	void *private_data;
	void *internal_data;
};

extern int ceata_add_port(struct ceata_host *chost, struct mmc_host *mmc, int init);
extern void ceata_del_port(struct ceata_host *chost, struct mmc_host *mmc, int release);
extern int ceata_register(struct ceata_host *host);
extern void ceata_unregister(struct ceata_host *host);

#endif /* _LIBATA_CEATA_H */
