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

/*
 * ethdev.c - ethernet device support
 */

#include <string.h>

/* For struct sockaddr */
#include <sys/socket.h>

/* General DPDK includes */
#include <rte_config.h>
#include <rte_ethdev.h>
/* Overrides from DPDK in ix */
#undef likely
#undef unlikely
#undef EDEADLK
#undef EAGAIN
#undef ENAMETOOLONG
#undef ENOTEMPTY
#undef ENOLCK
#undef ENOSYS
#undef EBADMSG
#undef EMULTIHOP
#undef ENOLINK
#undef EPROTO
#undef LIST_HEAD
#undef PKT_TX_IP_CKSUM
#undef ETH_LINK_SPEED_AUTONEG
#undef ETH_LINK_HALF_DUPLEX
#undef ETH_LINK_FULL_DUPLEX
#undef ETH_RSS
#undef VMDQ_DCB
#undef ETH_DCB_RX
#undef ETH_DCB_NONE
#undef ETH_VMDQ_DCB_TX
#undef ETH_DCB_TX
#undef ETH_RSS_IPV4
#undef ETH_RSS_IPV6
#undef ETH_RSS_IPV6_EX
#undef ETH_RSS_IPV6_TCP_EX
#undef ETH_RSS_IPV6_UDP_EX
#undef PKT_TX_TCP_CKSUM

#include <ix/stddef.h>
#include <ix/errno.h>
#include <ix/ethdev.h>
#include <ix/log.h>
#include <ix/cpu.h>
#include <ix/cfg.h>
#include <drivers/ixgbe.h>
#include <drivers/i40e.h>

#include <net/ethernet.h>

int eth_dev_count;
struct ix_rte_eth_dev *eth_dev[NETHDEV];
static DEFINE_SPINLOCK(eth_dev_lock);

static const struct ix_rte_eth_conf default_conf = {
	.rxmode = {
		.split_hdr_size = 0,
		.header_split   = 0, /**< Header Split disabled */
		.hw_ip_checksum = 1, /**< IP checksum offload disabled */
		.hw_vlan_filter = 0, /**< VLAN filtering disabled */
		.jumbo_frame    = 0, /**< Jumbo Frame Support disabled */
		.hw_strip_crc   = 1, /**< CRC stripped by hardware */
		.mq_mode        = IX_ETH_MQ_RX_RSS,
	},
	.rx_adv_conf = {
		.rss_conf = {
			.rss_hf = ETH_RSS_IPV4_TCP | ETH_RSS_IPV4_UDP,
		},
	},
	.txmode = {
		.mq_mode = IX_ETH_MQ_TX_NONE,
	},
};

/**
 * eth_dev_get_hw_mac - retreives the default MAC address
 * @dev: the ethernet device
 * @mac_addr: pointer to store the mac
 */
void eth_dev_get_hw_mac(struct ix_rte_eth_dev *dev, struct eth_addr *mac_addr)
{
	memcpy(&mac_addr->addr[0], &dev->data->mac_addrs[0], ETH_ADDR_LEN);
}

/**
 * eth_dev_set_hw_mac - sets the default MAC address
 * @dev: the ethernet device
 * @mac_addr: pointer of mac
 */
void eth_dev_set_hw_mac(struct ix_rte_eth_dev *dev, struct eth_addr *mac_addr)
{
	dev->dev_ops->mac_addr_add(dev, mac_addr, 0, 0);
}

/**
 * eth_dev_add - registers an ethernet device
 * @dev: the ethernet device
 *
 * Returns 0 if successful, otherwise failure.
 */
int eth_dev_add(struct ix_rte_eth_dev *dev)
{
	int ret, i;

	dev->data->nb_rx_queues = 0;
	dev->data->nb_tx_queues = 0;

	dev->data->max_rx_queues =
		min(dev->dev_info->max_rx_queues, ETH_RSS_RETA_MAX_QUEUE);
	dev->data->max_tx_queues = dev->dev_info->max_tx_queues;

	dev->data->rx_queues = malloc(sizeof(struct eth_rx_queue *) *
				      dev->data->max_rx_queues);
	if (!dev->data->rx_queues)
		return -ENOMEM;

	dev->data->tx_queues = malloc(sizeof(struct eth_tx_queue *) *
				      dev->data->max_tx_queues);
	if (!dev->data->tx_queues) {
		ret = -ENOMEM;
		goto err_tx_queues;
	}

	dev->data->nb_rx_fgs = ETH_MAX_NUM_FG;
	dev->data->rx_fgs = malloc(sizeof(struct eth_fg) *
				   dev->data->nb_rx_fgs);
	if (!dev->data->rx_fgs) {
		ret = -ENOMEM;
		goto err_fgs;
	}

	for (i = 0; i < dev->data->nb_rx_fgs; i++) {
		struct eth_fg *fg = &dev->data->rx_fgs[i];
		fg->eth = dev;
		eth_fg_init(fg, i);
	}

	spin_lock(&eth_dev_lock);
	i = eth_dev_count++;
	spin_unlock(&eth_dev_lock);

	eth_dev[i] = dev;
	return 0;

err_fgs:
	free(dev->data->tx_queues);
err_tx_queues:
	free(dev->data->rx_queues);
	return ret;

}

static void eth_dev_setup_mac(struct ix_rte_eth_dev *dev)
{
	static int first = true;

	/* FIXME: this is awfully bonding-specific */
	if (first) {
		eth_dev_get_hw_mac(dev, &CFG.mac);
		first = false;
	} else {
		eth_dev_set_hw_mac(dev, &CFG.mac);
	}
}

/**
 * eth_dev_start - starts an ethernet device
 * @dev: the ethernet device
 *
 * Returns 0 if successful, otherwise failure.
 */
int eth_dev_start(struct ix_rte_eth_dev *dev)
{
	int ret;
	struct eth_addr macaddr;
	struct ix_rte_eth_link link;

	ret = dev->dev_ops->dev_start(dev);
	if (ret)
		return ret;

	dev->dev_ops->promiscuous_disable(dev);
	dev->dev_ops->allmulticast_enable(dev);

	eth_dev_get_hw_mac(dev, &macaddr);
	log_info("eth: started an ethernet device\n");
	log_info("eth:\tMAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
		 macaddr.addr[0], macaddr.addr[1],
		 macaddr.addr[2], macaddr.addr[3],
		 macaddr.addr[4], macaddr.addr[5]);

	dev->dev_ops->link_update(dev, 1);
	link = dev->data->dev_link;

	if (!link.link_status) {
		log_warn("eth:\tlink appears to be down, check connection.\n");
	} else {
		log_info("eth:\tlink up - speed %u Mbps, %s\n",
			 (uint32_t) link.link_speed,
			 (link.link_duplex == ETH_LINK_FULL_DUPLEX) ?
			 ("full-duplex") : ("half-duplex\n"));
	}

	eth_dev_setup_mac(dev);

	return 0;
}

/**
 * eth_dev_stop - stops an ethernet device
 * @dev: the ethernet device
 */
void eth_dev_stop(struct ix_rte_eth_dev *dev)
{
	int i;

	dev->dev_ops->dev_stop(dev);

	for (i = 0; i < dev->data->nb_tx_queues; i++)
		dev->dev_ops->tx_queue_release(dev->data->tx_queues[i]);

	for (i = 0; i < dev->data->nb_rx_queues; i++)
		dev->dev_ops->rx_queue_release(dev->data->rx_queues[i]);

	dev->data->nb_rx_queues = 0;
	dev->data->nb_tx_queues = 0;
	free(dev->data->tx_queues);
	free(dev->data->rx_queues);
}

/**
 * eth_dev_get_rx_queue - get the next available rx queue
 * @dev: the ethernet device
 * @rx_queue: pointer to store a pointer to struct eth_rx_queue
 *
 * Returns 0 if successful, otherwise failure.
 */
int eth_dev_get_rx_queue(struct ix_rte_eth_dev *dev,
			 struct eth_rx_queue **rx_queue)
{
	int rx_idx, ret;

	spin_lock(&eth_dev_lock);
	rx_idx = dev->data->nb_rx_queues;

	if (rx_idx >= dev->data->max_rx_queues) {
		spin_unlock(&eth_dev_lock);
		return -EMFILE;
	}


	ret = dev->dev_ops->rx_queue_setup(dev, rx_idx, -1,
					   ETH_DEV_RX_QUEUE_SZ);
	if (ret) {
		spin_unlock(&eth_dev_lock);
		return ret;
	}

//	ret = queue_init_one(dev->data->rx_queues[rx_idx]);
	if (ret)
		goto err;

	dev->data->nb_rx_queues++;
	spin_unlock(&eth_dev_lock);

	*rx_queue = dev->data->rx_queues[rx_idx];
	(*rx_queue)->queue_idx = rx_idx;
	(*rx_queue)->dev = dev;
	bitmap_init((*rx_queue)->assigned_fgs, dev->data->nb_rx_fgs, false);

	return 0;

err:
	spin_unlock(&eth_dev_lock);
	dev->dev_ops->rx_queue_release(dev->data->rx_queues[rx_idx]);
	return ret;
}

/**
 * eth_dev_get_tx_queue - get the next available tx queue
 * @dev: the ethernet device
 * @tx_queue: pointer to store a pointer to struct eth_tx_queue
 *
 * Returns 0 if successful, otherwise failure.
 */
int eth_dev_get_tx_queue(struct ix_rte_eth_dev *dev,
			 struct eth_tx_queue **tx_queue)
{
	int tx_idx, ret;

	spin_lock(&eth_dev_lock);
	tx_idx = dev->data->nb_tx_queues;

	if (tx_idx > dev->data->max_tx_queues) {
		spin_unlock(&eth_dev_lock);
		return -EMFILE;
	}

	ret = dev->dev_ops->tx_queue_setup(dev, tx_idx, -1,
					   ETH_DEV_TX_QUEUE_SZ);
	if (ret) {
		spin_unlock(&eth_dev_lock);
		return ret;
	}

	dev->data->nb_tx_queues++;
	spin_unlock(&eth_dev_lock);

	*tx_queue = dev->data->tx_queues[tx_idx];

	return 0;
}

/**
 * eth_dev_alloc - allocates an ethernet device
 * @private_len: the size of the private area
 *
 * Returns an ethernet device, or NULL if failure.
 */
struct ix_rte_eth_dev *eth_dev_alloc(size_t private_len)
{
	struct ix_rte_eth_dev *dev;

	dev = malloc(sizeof(struct ix_rte_eth_dev));
	if (!dev)
		return NULL;

	dev->pci_dev = NULL;
	dev->dev_ops = NULL;

	dev->data = malloc(sizeof(struct ix_rte_eth_dev_data));
	if (!dev->data) {
		free(dev);
		return NULL;
	}

	memset(dev->data, 0, sizeof(struct ix_rte_eth_dev_data));
	dev->data->dev_conf = default_conf;

	dev->data->dev_private = malloc(private_len);
	if (!dev->data->dev_private) {
		free(dev->data);
		free(dev);
		return NULL;
	}

	memset(dev->data->dev_private, 0, private_len);

	dev->dev_info = malloc(sizeof(struct rte_eth_dev_info));
	if (!dev->dev_info) {
		free(dev->data->dev_private);
		free(dev->data);
		free(dev);
		return NULL;
	}
	return dev;
}

/**
 * eth_dev_destroy - frees an ethernet device
 * @dev: the ethernet device
 */
void eth_dev_destroy(struct ix_rte_eth_dev *dev)
{
	if (dev->dev_ops && dev->dev_ops->dev_close)
		dev->dev_ops->dev_close(dev);

	free(dev->data->dev_private);
	free(dev->data);
	free(dev);
}

enum rte_eth_rx_mq_mode translate_conf_rxmode_mq_mode(enum ix_rte_eth_rx_mq_mode in)
{
	switch (in) {
	case IX_ETH_MQ_RX_RSS:
				return ETH_MQ_RX_RSS;
		default:
			assert(false);
		}
}

enum rte_eth_tx_mq_mode translate_conf_txmode_mq_mode(enum ix_rte_eth_tx_mq_mode in)
{
	switch (in) {
	case IX_ETH_MQ_TX_NONE:
				return ETH_MQ_TX_NONE;
		default:
			assert(false);
		}
}

uint16_t translate_conf_rss_hf(uint16_t in)
{
	uint16_t out = 0;

#define COPY_AND_RESET(dst, src, bit_out, bit_in) \
	if ((src) & (bit_in)) { \
		(dst) |= (bit_out); \
		(src) &= ~(bit_in); \
	}

	COPY_AND_RESET(out, in, ETH_RSS_NONFRAG_IPV4_TCP, ETH_RSS_IPV4_TCP);
	COPY_AND_RESET(out, in, ETH_RSS_NONFRAG_IPV4_UDP, ETH_RSS_IPV4_UDP);

#undef COPY_AND_RESET

	assert(in == 0);

	return out;
}

void translate_conf(struct rte_eth_conf *out, struct ix_rte_eth_conf *in)
{
	char *p;

	memset(out, 0, sizeof(struct rte_eth_conf));

#define COPY_AND_RESET(field) \
	do { \
		out->field = in->field; \
		in->field = 0; \
	} while (0)

#define COPY_AND_RESET2(field, f) \
	do { \
		out->field = f(in->field); \
		in->field = 0; \
	} while (0)

	COPY_AND_RESET2(rx_adv_conf.rss_conf.rss_hf, translate_conf_rss_hf);
	COPY_AND_RESET(rxmode.header_split);
	COPY_AND_RESET(rxmode.hw_ip_checksum);
	COPY_AND_RESET(rxmode.hw_strip_crc);
	COPY_AND_RESET(rxmode.hw_vlan_filter);
	COPY_AND_RESET(rxmode.jumbo_frame);
	COPY_AND_RESET2(rxmode.mq_mode, translate_conf_rxmode_mq_mode);
	COPY_AND_RESET(rxmode.split_hdr_size);
	COPY_AND_RESET2(txmode.mq_mode, translate_conf_txmode_mq_mode);

#undef COPY_AND_RESET
#undef COPY_AND_RESET2

	for (p = (char *) in; p < (char *) in + sizeof(*in); p++)
		assert(!*p);
}


int ix_eth_dev_init(const struct pci_addr *addr, struct ix_rte_eth_dev **ethp)
{
	int i;
	int ret;
	struct ix_rte_eth_dev *dev;
	struct rte_eth_conf conf;
	uint8_t portid;
	uint8_t nb_ports;
	bool found=false;

	dev = eth_dev_alloc(1);

	/* get the correct dev portid */
	nb_ports = rte_eth_dev_count();
	if(nb_ports == 0)
		panic("No Ethernet ports - bye.\n");
	log_err("nb ports : %u\n", nb_ports);
	portid=0;
	while(!found && portid < nb_ports){
		rte_eth_dev_info_get(portid, dev->dev_info);
		if(dev->dev_info->pci_dev->addr.domain == addr->domain
				&& dev->dev_info->pci_dev->addr.bus == addr->bus
				&& dev->dev_info->pci_dev->addr.devid == addr->slot
				&& dev->dev_info->pci_dev->addr.function == addr->func){
			found=true;
			break;
		}
		portid++;
	}
	if(!found)
		panic("requested device %04x:%02x:%02x.%d not found.\n",
				addr->domain, addr->bus,
				addr->slot, addr->func );

	/* set the custom driver dev_start functions*/
	if (strcmp(dev->dev_info->driver_name, "rte_ixgbe_pmd") == 0){
		dev->dev_ops = &ixgbe_dev_ops;
	} else if (strcmp(dev->dev_info->driver_name, "rte_ixgbevf_pmd") == 0){
		dev->dev_ops = &ixgbevf_dev_ops;
	} else if (strcmp(dev->dev_info->driver_name, "rte_i40e_pmd") == 0){
		dev->dev_ops = &i40e_dev_ops;
	} else if (strcmp(dev->dev_info->driver_name, "rte_i40evf_pmd") == 0){
		dev->dev_ops = &i40evf_dev_ops;
	} else {
		panic("No valid IX driver available. abort.");
	}

	/* configure the port */
	translate_conf(&conf, &dev->data->dev_conf);
	conf.fdir_conf.mode = RTE_FDIR_MODE_PERFECT;
	conf.fdir_conf.pballoc = RTE_FDIR_PBALLOC_256K;
	conf.fdir_conf.status = RTE_FDIR_REPORT_STATUS;
	conf.fdir_conf.mask.vlan_tci_mask = 0x0;
	conf.fdir_conf.mask.ipv4_mask.src_ip = 0xFFFFFFFF;
	conf.fdir_conf.mask.ipv4_mask.dst_ip = 0xFFFFFFFF;
	conf.fdir_conf.mask.src_port_mask = 0xFFFF;
	conf.fdir_conf.mask.dst_port_mask = 0xFFFF;
	conf.fdir_conf.mask.mac_addr_byte_mask = 0;
	conf.fdir_conf.mask.tunnel_type_mask = 0;
	conf.fdir_conf.mask.tunnel_id_mask = 0;
	conf.fdir_conf.drop_queue = 127;
	conf.fdir_conf.flex_conf.nb_payloads = 0;
	conf.fdir_conf.flex_conf.nb_flexmasks = 0;

	ret = rte_eth_dev_configure(portid, dev->dev_info->max_rx_queues, dev->dev_info->max_tx_queues, &conf);
	if (ret < 0){
		log_err("rte_eth_dev_configure : error %i\n", ret);
		return ret;
	}
	/* Init eth_dev */
	dev->port = portid;
	spin_lock_init(&dev->lock);

	dev->data->mac_addrs = calloc(1, ETH_ADDR_LEN);
	for (i = 0; i < ETHER_ADDR_LEN; i++)
		dev->data->mac_addrs[0].addr[i] = rte_eth_devices[portid].data->mac_addrs[0].addr_bytes[i];
	*ethp = dev;

	return 0;
}
