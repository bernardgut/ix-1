/*
 * Copyright 2013-16 Board of Trustees of Stanford University
 * Copyright 2013-16 Ecole Polytechnique Federale Lausanne (EPFL)
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <drivers/common.h>
#include <drivers/i40e.h>

#include <rte_tcp.h>
#include <i40e_ethdev.h>
#include <i40e_rxtx.h>

#include <ix/byteorder.h>
#include <ix/ethdev.h>
#include <ix/dpdk.h>

#define I40E_RING_BASE_ALIGN 	128
//#define I40E_DMA_MEM_ALIGN 	4096

#define I40E_RDT_THRESH		32
#define L3_HEADER_LENGTH	9

/* device specific callbacks */
static int i40e_rx_poll(struct eth_rx_queue *rx);
static int i40e_rx_active(struct eth_rx_queue *rx);
static int i40e_tx_reclaim(struct eth_tx_queue *tx);
static int i40e_tx_xmit(struct eth_tx_queue *tx, int nr, struct mbuf **mbufs);

/* device specific operations defined for i40e device driver */
static void allmulticast_enable(struct ix_rte_eth_dev *dev);
static int dev_start(struct ix_rte_eth_dev *dev);
static int link_update(struct ix_rte_eth_dev *dev, int wait_to_complete);
static void promiscuous_disable(struct ix_rte_eth_dev *dev);
static int reta_update(struct ix_rte_eth_dev *dev, struct rte_eth_rss_reta *reta_conf);
static int rx_queue_setup(struct ix_rte_eth_dev *dev, int queue_idx,
			  int numa_node, uint16_t nb_desc);
static int tx_queue_setup(struct ix_rte_eth_dev *dev, int queue_idx,
			  int numa_node, uint16_t nb_desc);
static int fdir_add_perfect_filter(struct ix_rte_eth_dev *dev, struct rte_fdir_filter *fdir_ftr, uint16_t soft_id, uint8_t rx_queue, uint8_t drop);
static int fdir_remove_perfect_filter(struct ix_rte_eth_dev *dev, struct rte_fdir_filter *fdir_ftr, uint16_t soft_id);
static int rss_hash_conf_get(struct ix_rte_eth_dev *dev, struct ix_rte_eth_rss_conf *ix_reta_conf);
static void mac_addr_add(struct ix_rte_eth_dev *dev, struct eth_addr *mac_addr, uint32_t index, uint32_t vmdq);

struct ix_eth_dev_ops i40e_dev_ops = {
	.allmulticast_enable = allmulticast_enable,
	.dev_start = dev_start,
	.link_update = link_update,
	.promiscuous_disable = promiscuous_disable,
	.reta_update = reta_update,
	.rx_queue_setup = rx_queue_setup,
	.tx_queue_setup = tx_queue_setup,
	.fdir_add_perfect_filter = fdir_add_perfect_filter,
	.fdir_remove_perfect_filter = fdir_remove_perfect_filter,
	.rss_hash_conf_get = rss_hash_conf_get,
	.mac_addr_add = mac_addr_add,
};

static int dev_start_vf(struct ix_rte_eth_dev *dev);

struct ix_eth_dev_ops i40evf_dev_ops = {
	.allmulticast_enable = allmulticast_enable,
	.dev_start = dev_start_vf,
	.link_update = link_update,
	.promiscuous_disable = promiscuous_disable,
	.reta_update = reta_update,
	.rx_queue_setup = rx_queue_setup,
	.tx_queue_setup = tx_queue_setup,
	.fdir_add_perfect_filter = fdir_add_perfect_filter,
	.fdir_remove_perfect_filter = fdir_remove_perfect_filter,
	.rss_hash_conf_get = rss_hash_conf_get,
	.mac_addr_add = mac_addr_add,
};

static DEFINE_SPINLOCK(i40e_dev_lock);

static void allmulticast_enable(struct ix_rte_eth_dev *dev)
{
	rte_eth_allmulticast_enable(dev->port);
}

static int i40e_alloc_rx_mbufs(struct rx_queue *rxq)
{
	int i;

	for (i = 0; i < rxq->len; i++) {
		machaddr_t maddr;
		struct mbuf *b = mbuf_alloc_local();
		if (!b)
			goto fail;

		maddr = mbuf_get_data_machaddr(b);
		rxq->ring_entries[i].mbuf = b;
		((volatile union i40e_rx_desc *)rxq->ring)[i].read.hdr_addr = rte_cpu_to_le_64(maddr);
		((volatile union i40e_rx_desc *)rxq->ring)[i].read.pkt_addr = rte_cpu_to_le_64(maddr);
	}

	return 0;

fail:
	for (i--; i >= 0; i--)
		mbuf_free(rxq->ring_entries[i].mbuf);
	return -ENOMEM;
}

static int dev_start(struct ix_rte_eth_dev *dev){
	int ret = 0;
	int i;
	struct eth_rx_queue *erxq;
	struct eth_tx_queue *etxq;
	struct i40e_rx_queue *drxq;
	struct i40e_tx_queue *dtxq;
	struct i40e_hmc_obj_rxq rx_ctx;
	struct i40e_hmc_obj_txq tx_ctx;
	struct i40e_hw *hw;
	uint16_t pf_q;

	log_err("DEV_START\n");
	ret = rte_eth_dev_start(dev->port);
	if (ret < 0)
		return ret;

	/* Init the RX/TX queues in hardware to use our mbufs instead of dpdk's*/
	for (i = 0; i < dev->data->nb_rx_queues; i++){
		erxq = (struct eth_rx_queue *)dev->data->rx_queues[i];
		struct rx_queue *rxq = (struct rx_queue *)erxq->drv_rx_queue;
		drxq = rte_eth_devices[dev->port].data->rx_queues[i];
		/* because these values are not initialized until dev_start
		 * we save them at this point :*/
		rxq->reg_idx = drxq->reg_idx;
		rxq->rdt_reg_addr = (uint32_t*) drxq->qrx_tail;

		pf_q = drxq->reg_idx;
		hw = I40E_VSI_TO_HW(drxq->vsi);

		ret = i40e_get_lan_rx_queue_context(hw, pf_q, &rx_ctx);
		if (ret < 0){
			log_err("Failed to get LAN RX queue context.\n");
			return ret;
		}
		ret = i40e_alloc_rx_mbufs(rxq);
		if (ret){
			log_err("failed to allocate RX mbuffs.\n");
			return ret;
		}
		/* write to our own buffs*/
		rx_ctx.base = rxq->ring_physaddr / I40E_QUEUE_BASE_ADDR_UNIT;
		rx_ctx.qlen = rxq->len;

		ret = i40e_set_lan_rx_queue_context(hw, pf_q, &rx_ctx);
		if (ret != I40E_SUCCESS) {
			log_err("Failed to set LAN RX queue context.\n");
			return ret;
		}
		/*init the tail register*/
		I40E_PCI_REG_WRITE(rxq->rdt_reg_addr, rxq->len - 1);
	}

	for (i = 0; i < dev->data->nb_tx_queues; i++){
		etxq = (struct eth_tx_queue *)dev->data->tx_queues[i];
		struct tx_queue *txq = (struct tx_queue *)etxq->drv_tx_queue;
		dtxq = rte_eth_devices[dev->port].data->tx_queues[i];

		txq->reg_idx = dtxq->reg_idx;
		txq->tdt_reg_addr = (uint32_t *)dtxq->qtx_tail;

		pf_q = dtxq->reg_idx;
		hw = I40E_VSI_TO_HW(dtxq->vsi);

		ret = i40e_get_lan_tx_queue_context(hw, pf_q, &tx_ctx);
		if(ret < 0){
			log_err("Failed to get LAN TX queue context.\n");
			return ret;
		}

		tx_ctx.base = txq->ring_physaddr / I40E_QUEUE_BASE_ADDR_UNIT;
		tx_ctx.qlen = txq->len;

		ret = i40e_set_lan_tx_queue_context(hw, pf_q, &tx_ctx);
		if (ret != I40E_SUCCESS) {
			log_err("Failed to set LAN TX queue context.\n");
			return ret;
		}
	}
	return 0;
}

static int dev_start_vf(struct ix_rte_eth_dev *dev){
	panic("TODO: %s",__func__);
	return 0;
}

static int link_update(struct ix_rte_eth_dev *dev, int wait_to_complete)
{
	struct rte_eth_link dev_link;

	if (wait_to_complete)
		rte_eth_link_get(dev->port, &dev_link);
	else
		rte_eth_link_get_nowait(dev->port, &dev_link);

	dev->data->dev_link.link_speed = dev_link.link_speed;
	dev->data->dev_link.link_duplex = dev_link.link_duplex;
	dev->data->dev_link.link_status = dev_link.link_status;

	return 0;
}

static void promiscuous_disable(struct ix_rte_eth_dev *dev)
{
	rte_eth_promiscuous_disable(dev->port);
}

/**
 * FIXME the reta allocation in ethfg.c is very ixgbe specific. this need to
 * be rewritten to allow dynamic FG size. In the meantime, the only way
 * to use this feature on both network cards is to manually change
 * ETH_RSS_RETA_NUM_ENTRIES and ETH_MAX_NUM_FG to 512 (for i40e)
 * ETH_RSS_RETA_NUM_ENTRIES and ETH_MAX_NUM_FG to 128 (for ixgbe)
 * */
static int reta_update(struct ix_rte_eth_dev *dev, struct rte_eth_rss_reta *reta_conf)
{
	int ret, i;
	struct rte_eth_rss_reta_entry64 r_reta_conf[2];
	struct rte_eth_dev r_dev = rte_eth_devices[dev->port];
	/* first convert reta_conf to dpdk format*/
	r_reta_conf[0].mask = reta_conf->mask_lo;
	r_reta_conf[1].mask = reta_conf->mask_hi;

	for (i = 0; i < RTE_RETA_GROUP_SIZE; ++i){
		r_reta_conf[0].reta[i] = reta_conf->reta[i];
		r_reta_conf[1].reta[i] = reta_conf->reta[RTE_RETA_GROUP_SIZE + i];
	}
	spin_lock(&i40e_dev_lock);
	ret = r_dev.dev_ops->reta_update(&r_dev, r_reta_conf, ETH_RSS_RETA_NUM_ENTRIES);
	spin_unlock(&i40e_dev_lock);
	if(ret){
		log_err("i40e: unnable to update receive side scaling\
			       	rerouting table (RETA): %i\n", ret);
		return ret;
	}
	return 0;
}

static int rx_queue_setup(struct ix_rte_eth_dev *dev, int queue_idx,
			  int numa_node, uint16_t nb_desc)
{
	void *page;
	machaddr_t page_phys;
	int ret;
	struct rx_queue *rxq;
	//struct i40e_rx_queue *drxq;

	/*
	 * The number of receive descriptors must not exceed hardware
	 * maximum and must be a multiple of IXGBE_ALIGN.
	 */
	BUILD_ASSERT(align_up(sizeof(struct rx_queue), I40E_RING_BASE_ALIGN) +
		     align_up(sizeof(union i40e_rx_desc) * I40E_MAX_RING_DESC, I40E_RING_BASE_ALIGN) +
		     sizeof(struct rx_entry) * I40E_MAX_RING_DESC < PGSIZE_2MB);

	/*
	 * Additionally, for purely software performance optimization reasons,
	 * we require the number of descriptors to be a power of 2.
	 */
	if (nb_desc & (nb_desc - 1))
		return -EINVAL;

	/* NOTE: This is a hack, but it's the only way to support late/lazy
	 * queue setup in DPDK; a feature that IX depends on. */
	rte_eth_devices[dev->port].data->nb_rx_queues = queue_idx + 1;

	ret = rte_eth_rx_queue_setup(dev->port, queue_idx, nb_desc, numa_node, &dev->dev_info->default_rxconf, dpdk_pool);
	if (ret < 0)
		return ret;
	if (numa_node == -1) {
		page = mem_alloc_page_local(PGSIZE_2MB);
		if (page == MAP_FAILED)
			return -ENOMEM;
	} else {
		page = mem_alloc_page(PGSIZE_2MB, numa_node, MPOL_BIND);
		if (page == MAP_FAILED)
			return -ENOMEM;
	}
	memset(page, 0, PGSIZE_2MB);

	/* hijack dpdk usual memory with our own from bigpages:*/
	rxq = (struct rx_queue *) page;

        rxq->ring  = (void *)((uintptr_t) page +
			align_up(sizeof(struct rx_queue), I40E_RING_BASE_ALIGN));
	rxq->ring_entries = (struct rx_entry *)((uintptr_t) rxq->ring +
			align_up(sizeof(union i40e_rx_desc) * nb_desc, I40E_RING_BASE_ALIGN));

	rxq->len = nb_desc;
	rxq->head = 0;
	rxq->tail = rxq->len - 1;

	ret = mem_lookup_page_machine_addr(page, PGSIZE_2MB, &page_phys);
	if (ret)
		goto err;

	rxq->ring_physaddr = page_phys +
			     align_up(sizeof(struct rx_queue), I40E_RING_BASE_ALIGN);
	log_err("queue_setup : phys %p\n", rxq->ring_physaddr);
	/* this is now done in dev_start: */
	//rxq->reg_idx = drxq->reg_idx;
	//rxq->rdt_reg_addr = (uint32_t*) drxq->qrx_tail;

	rxq->erxq.poll = i40e_rx_poll;
	rxq->erxq.active = i40e_rx_active;
	dev->data->rx_queues[queue_idx] = &rxq->erxq;
	rxq->erxq.drv_rx_queue = rxq;
	/* release the dpdk memory location and all its buffers*/
	//i40e_dev_rx_queue_release(drxq);
	return 0;

err:
	mem_free_page(page, PGSIZE_2MB);
	return ret;
}

static void i40_reset_tx_queue(struct tx_queue *txq)
{
	int i;

	for (i = 0; i < txq->len; i++) {
		txq->ring_entries[i].mbuf = NULL;
	}

	txq->head = 0;
	txq->tail = 0;
}

static int tx_queue_setup(struct ix_rte_eth_dev *dev, int queue_idx,
			  int numa_node, uint16_t nb_desc)
{
	void *page;
	machaddr_t page_phys;
	int ret;
	struct tx_queue *txq;
	//struct i40e_tx_queue *dtxq;

	/*
	 * The number of receive descriptors must not exceed hardware
	 * maximum and must be a multiple of IXGBE_ALIGN.
	 */
	BUILD_ASSERT(align_up(sizeof(struct tx_queue), I40E_RING_BASE_ALIGN) +
		        align_up(sizeof(struct i40e_tx_desc) * nb_desc, I40E_RING_BASE_ALIGN) +
			sizeof(struct tx_entry) * I40E_MAX_RING_DESC < PGSIZE_2MB);

	/*
	 * Additionally, for purely software performance optimization reasons,
	 * we require the number of descriptors to be a power of 2.
	 */
	if (nb_desc & (nb_desc - 1))
		return -EINVAL;

	/* NOTE: This is a hack, but it's the only way to support late/lazy
	 * queue setup in DPDK; a feature that IX depends on. */
	rte_eth_devices[dev->port].data->nb_tx_queues = queue_idx + 1;

	ret = rte_eth_tx_queue_setup(dev->port, queue_idx, nb_desc, numa_node, &dev->dev_info->default_txconf);
	if (ret < 0)
		return ret;

	if (numa_node == -1) {
		page = mem_alloc_page_local(PGSIZE_2MB);
		if (page == MAP_FAILED)
			return -ENOMEM;
	} else {
		page = mem_alloc_page(PGSIZE_2MB, numa_node, MPOL_BIND);
		if (page == MAP_FAILED)
			return -ENOMEM;
	}
	memset(page, 0, PGSIZE_2MB);

	/* hijack dpdk usual memory with our own from bigpages:*/
	txq = (struct tx_queue *) page;
	txq->ring = (void *)((uintptr_t) page +
			align_up(sizeof(struct tx_queue), I40E_RING_BASE_ALIGN));
	txq->ring_entries = (struct tx_entry *)((uintptr_t) txq->ring +
			align_up(sizeof(struct i40e_tx_desc) * nb_desc, I40E_RING_BASE_ALIGN));
	txq->len = nb_desc;

	ret = mem_lookup_page_machine_addr(page, PGSIZE_2MB, &page_phys);
	if (ret)
		goto err;
	txq->ring_physaddr = page_phys +
			     align_up(sizeof(struct tx_queue), I40E_RING_BASE_ALIGN);

	/* this is now donw in dev_start :*/
	//txq->reg_idx = dtxq->reg_idx;
	//txq->tdt_reg_addr = (uint32_t *)dtxq->qtx_tail;

	txq->etxq.reclaim = i40e_tx_reclaim;
	txq->etxq.xmit = i40e_tx_xmit;
	i40_reset_tx_queue(txq);
	dev->data->tx_queues[queue_idx] = &txq->etxq;
	txq->etxq.drv_tx_queue = txq;
	/* release the dpdk memory location and all its buffers*/
	//i40e_dev_tx_queue_release(dtxq);
	return 0;

err:
	mem_free_page(page, PGSIZE_2MB);
	return ret;

}

static int fdir_add_perfect_filter(struct ix_rte_eth_dev *dev, struct rte_fdir_filter *fdir_ftr, uint16_t soft_id, uint8_t rx_queue, uint8_t drop)
{
	panic("TODO: %s\n",__func__);
	return 0;
}

static int fdir_remove_perfect_filter(struct ix_rte_eth_dev *dev, struct rte_fdir_filter *fdir_ftr, uint16_t soft_id)
{
	panic("TODO: %s\n",__func__);
	return 0;
}

static int rss_hash_conf_get(struct ix_rte_eth_dev *dev, struct ix_rte_eth_rss_conf *ix_reta_conf)
{
	panic("TODO: %s\n",__func__);
	return 0;
}

static void mac_addr_add(struct ix_rte_eth_dev *dev, struct eth_addr *mac_addr, uint32_t index, uint32_t vmdq)
{
	panic("TODO: %s\n",__func__);
}

/* Device specific callbacks */

static int i40e_rx_poll(struct eth_rx_queue *rx)
{
	struct rx_queue *rxq = (struct rx_queue *)rx->drv_rx_queue;
	volatile union i40e_rx_desc *rxdp;
	union i40e_rx_desc rxd;
	uint64_t qword1;
	uint64_t error_bits;
	uint32_t rx_status;
	struct mbuf *b, *new_b;
	struct rx_entry *rxqe;
	machaddr_t maddr;
	int nb_descs = 0;
	bool valid_checksum;
	int local_fg_id;
	long timestamp;

	timestamp = rdtsc();
	while (1) {
		rxdp = &((volatile union i40e_rx_desc *)rxq->ring)[rxq->head & (rxq->len - 1)];
		qword1 = rte_le_to_cpu_64(rxdp->wb.qword1.status_error_len);
		rx_status = (qword1 & I40E_RXD_QW1_STATUS_MASK) >>
			I40E_RXD_QW1_STATUS_SHIFT;

		valid_checksum = true;
		/* this check that there is at least one packet to receive :*/
		if (!(rx_status & (1 << I40E_RX_DESC_STATUS_DD_SHIFT))){
			break;
		}
		rxd = *rxdp;
		rxqe = &rxq->ring_entries[rxq->head & (rxq->len - 1)];

		error_bits = (qword1 >> I40E_RXD_QW1_ERROR_SHIFT);
		/* Check IP checksum calculated by hardware (if applicable) */
		if (unlikely(error_bits & (1 << I40E_RX_DESC_ERROR_IPE_SHIFT))){
			log_err("i40e: IP RX checksum error, dropping pkt\n");
			valid_checksum = false;
		}

		/* Check TCP checksum calculated by hardware (if applicable) */
		if (unlikely(error_bits & (1 << I40E_RX_DESC_ERROR_L4E_SHIFT))){
			log_err("i40e: TCP RX checksum error, dropping pkt\n");
			valid_checksum = false;
		}

		/* translate descriptor info into mbuf parameters */
		b = rxqe->mbuf;
		b->len = ((qword1 & I40E_RXD_QW1_LENGTH_PBUF_MASK) >>
				I40E_RXD_QW1_LENGTH_PBUF_SHIFT);

		if (qword1 & (1 << I40E_RX_DESC_STATUS_FLM_SHIFT)) {
			b->fg_id = MBUF_INVALID_FG_ID;
		} else {
			local_fg_id = (le32_to_cpu(rxd.wb.qword0.hi_dword.rss) &
				       (ETH_RSS_RETA_NUM_ENTRIES - 1));
			b->fg_id = rx->dev->data->rx_fgs[local_fg_id].fg_id;
		}
		b->timestamp = timestamp;

		new_b = mbuf_alloc_local();
		if (unlikely(!new_b)) {
			log_err("i40e: unable to allocate RX mbuf\n");
			goto out;
		}

		maddr = mbuf_get_data_machaddr(new_b);
		rxqe->mbuf = new_b;
		rxdp->read.hdr_addr = rte_cpu_to_le_64(maddr);
		rxdp->read.pkt_addr = rte_cpu_to_le_64(maddr);

		if (unlikely(!valid_checksum || eth_recv(rx, b))) {
			log_info("i40e: dropping packet\n");
			mbuf_free(b);
		}

		rxq->head++;
		nb_descs++;
	}

out:

	/*
	 * We threshold updates to the RX tail register because when it
	 * is updated too frequently (e.g. when written to on multiple
	 * cores even through separate queues) PCI performance
	 * bottlnecks have been observed.
	 */
	if ((uint16_t)(rxq->len - (rxq->tail + 1 - rxq->head)) >=
	    I40E_RDT_THRESH) {
		rxq->tail = rxq->head + rxq->len - 1;

		/* inform HW that more descriptors have become available */
		I40E_PCI_REG_WRITE(rxq->rdt_reg_addr,
				    (rxq->tail & (rxq->len - 1)));
	}

	return nb_descs;
}

/* i40e_rx_active - checks if the queue is active
 * A queue is active if there is a packet to receive.
 * returns true if active. false otherwise */
static int i40e_rx_active(struct eth_rx_queue *rx)
{
	struct rx_queue *rxq = (struct rx_queue*)rx->drv_rx_queue;
	uint64_t qword1 = rte_le_to_cpu_64(((volatile union i40e_rx_desc *)rxq->ring)[rxq->head & (rxq->len - 1)].wb.qword1.status_error_len);
	uint64_t status = (qword1 & I40E_RXD_QW1_STATUS_MASK) >>
		I40E_RXD_QW1_STATUS_SHIFT;
	return status & (1 << I40E_RX_DESC_STATUS_DD_SHIFT);
}

static int i40e_tx_reclaim(struct eth_tx_queue *tx)
{
	struct tx_queue *txq = (struct tx_queue *)tx->drv_tx_queue;
	struct tx_entry *txe;
	volatile struct i40e_tx_desc *txdp;
	int idx = 0, nb_desc = 0;

	while ((uint16_t)(txq->head + idx) != txq->tail) {
		txe = &txq->ring_entries[(txq->head + idx) & (txq->len - 1)];

		if (!txe->mbuf) {
			idx++;
			continue;
		}

		txdp = &((volatile struct i40e_tx_desc *)txq->ring)[(txq->head + idx) & (txq->len - 1)];
		if ((txdp->cmd_type_offset_bsz &
				rte_cpu_to_le_64(I40E_TXD_QW1_DTYPE_MASK)) !=
				rte_cpu_to_le_64(I40E_TX_DESC_DTYPE_DESC_DONE))
			break;

		mbuf_xmit_done(txe->mbuf);
		txe->mbuf = NULL;
		idx++;
		nb_desc = idx;
	}

	txq->head += nb_desc;
	return (uint16_t)(txq->len + txq->head - txq->tail);
}

/* Construct the tx flags */
static inline uint64_t
i40e_build_ctob(uint32_t td_cmd,
		uint32_t td_offset,
		unsigned int size,
		uint32_t td_tag)
{
	return rte_cpu_to_le_64(I40E_TX_DESC_DTYPE_DATA |
			((uint64_t)td_cmd  << I40E_TXD_QW1_CMD_SHIFT) |
			((uint64_t)td_offset << I40E_TXD_QW1_OFFSET_SHIFT) |
			((uint64_t)size  << I40E_TXD_QW1_TX_BUF_SZ_SHIFT) |
			((uint64_t)td_tag  << I40E_TXD_QW1_L2TAG1_SHIFT));
}

static int i40e_tx_xmit_one(struct tx_queue *txq, struct mbuf *mbuf)
{
	volatile struct i40e_tx_desc *txdp;
	machaddr_t maddr;
	int i, nr_iov = mbuf->nr_iov;
	uint32_t pay_len = mbuf->len;
	uint32_t td_cmd = 0;
	uint32_t td_offset = 0;

	/*
	 * Make sure enough space is available in the descriptor ring
	 * NOTE: This should work correctly even with overflow...
	 */
	if (unlikely((uint16_t)(txq->tail + nr_iov + 1 - txq->head) >= txq->len)) {
		i40e_tx_reclaim(&txq->etxq);
		if ((uint16_t)(txq->tail + nr_iov + 1 - txq->head) >= txq->len)
			return -EAGAIN;
	}

	/*
	 * Check mbuf's offload flags
	 * If flags match context 0 on NIC (IP and TCP chksum), use context
	 * Otherwise, no context
	 */
	if ((mbuf->ol_flags & PKT_TX_IP_CKSUM) &&
	    (mbuf->ol_flags & PKT_TX_TCP_CKSUM)) {
		td_cmd |= I40E_TX_DESC_CMD_IIPT_IPV4_CSUM;
		td_offset |= (L3_HEADER_LENGTH >> 2) <<
			I40E_TX_DESC_LENGTH_IPLEN_SHIFT;
		td_cmd |= I40E_TX_DESC_CMD_L4T_EOFT_TCP;
		td_offset |= (sizeof(struct tcp_hdr) >> 2) <<
		        I40E_TX_DESC_LENGTH_L4_FC_LEN_SHIFT;
	}

	for (i = 0; i < nr_iov; i++) {
		struct mbuf_iov iov = mbuf->iovs[i];
		txdp = &((volatile struct i40e_tx_desc *)txq->ring)[(txq->tail + i + 1) & (txq->len - 1)];

		if (i == nr_iov - 1) {
			td_cmd |= (I40E_TX_DESC_CMD_EOP |
				     I40E_TX_DESC_CMD_RS);
		}
		txdp->buffer_addr = cpu_to_le64((uintptr_t) iov.maddr);
		txdp->cmd_type_offset_bsz = i40e_build_ctob(td_cmd,
				td_offset, pay_len, 0);
		pay_len += iov.len;
	}

	txq->ring_entries[(txq->tail + nr_iov) & (txq->len - 1)].mbuf = mbuf;
	txdp = &((volatile struct i40e_tx_desc *)txq->ring)[txq->tail & (txq->len - 1)];

	maddr = mbuf_get_data_machaddr(mbuf);

	txdp->buffer_addr = cpu_to_le64(maddr);
	txdp->cmd_type_offset_bsz = i40e_build_ctob(td_cmd,
				td_offset, pay_len, 0);
	if (i == nr_iov - 1) {
		td_cmd |= (I40E_TX_DESC_CMD_EOP |
			     I40E_TX_DESC_CMD_RS);
	}
	txq->tail += nr_iov + 1;

	return 0;
}

static int i40e_tx_xmit(struct eth_tx_queue *tx, int nr, struct mbuf **mbufs)
{
	struct tx_queue *txq = (struct tx_queue *)tx->drv_tx_queue;
	int nb_pkts = 0;

	while (nb_pkts < nr) {
		if (i40e_tx_xmit_one(txq, mbufs[nb_pkts]))
			break;

		nb_pkts++;
	}

	if (nb_pkts)
		I40E_PCI_REG_WRITE(txq->tdt_reg_addr,
				(txq->tail & (txq->len - 1)));

	return nb_pkts;
}

