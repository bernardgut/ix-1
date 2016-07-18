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

/* For memmove and size_t */
#include <string.h>
  /* For struct sockaddr */
#include <sys/socket.h>
/* For pipe */
#include <unistd.h>

/* General DPDK includes */
#include <rte_config.h>
#include <rte_ethdev.h>

/* DPDK ixgbe includes */
#include <base/ixgbe_type.h>
#include <ixgbe_rxtx.h>

/* Defines from ixgbe that override i40e*/
#undef DEBUGFUNC
#undef DEBUGOUT
#undef DEBUGOUT1
#undef DEBUGOUT2
#undef DEBUGOUT3
#undef DEBUGOUT6
#undef DEBUGOUT7
#undef UNREFERENCED_1PARAMETER
#undef UNREFERENCED_2PARAMETER
#undef UNREFERENCED_3PARAMETER
#undef UNREFERENCED_4PARAMETER


/* DPDK i40e includes */
#ifndef X722_SUPPORT
#define X722_SUPPORT
#endif
#include <base/i40e_type.h>
#include <i40e_rxtx.h>

/* Defines from IX override DPDK */
#undef EAGAIN
 #undef EBADMSG
 #undef EDEADLK
 #undef EMULTIHOP
 #undef ENAMETOOLONG
 #undef ENOLCK
 #undef ENOLINK
 #undef ENOSYS
 #undef ENOTEMPTY
 #undef EPROTO
 #undef ETH_DCB_NONE
 #undef ETH_DCB_RX
 #undef ETH_DCB_TX
 #undef ETH_LINK_FULL_DUPLEX
 #undef ETH_LINK_HALF_DUPLEX
 #undef ETH_LINK_SPEED_AUTONEG
 #undef ETH_RSS
 #undef ETH_RSS_IPV4
 #undef ETH_RSS_IPV6
 #undef ETH_RSS_IPV6_EX
 #undef ETH_RSS_IPV6_TCP_EX
 #undef ETH_RSS_IPV6_UDP_EX
 #undef ETH_VMDQ_DCB_TX
 #undef IXGBE_MIN_RING_DESC
 #undef PKT_TX_IP_CKSUM
 #undef PKT_TX_TCP_CKSUM
 #undef VMDQ_DCB
 #undef likely
 #undef mb
 #undef min
 #undef prefetch
 #undef rmb
 #undef unlikely
 #undef wmb

#undef ARRAY_SIZE
#undef max
#undef container_of

/*IX includes*/
#include <ix/ethqueue.h>

struct rx_entry {
	struct mbuf *mbuf;
};

struct rx_queue{
	struct eth_rx_queue	erxq;
	void			*ring;
	machaddr_t		ring_physaddr;
	struct rx_entry		*ring_entries;

	volatile uint32_t	*rdt_reg_addr;
	uint16_t		reg_idx;

	uint16_t		head;
	uint16_t		tail;
	uint16_t		len;
};

struct tx_entry {
	struct mbuf *mbuf;
};

struct tx_queue {
	struct eth_tx_queue	etxq;
	void			*ring;
	machaddr_t		ring_physaddr;
	struct tx_entry		*ring_entries;

	volatile uint32_t	*tdt_reg_addr;
	uint16_t		reg_idx;
	uint16_t		queue_id;

	uint16_t		head;
	uint16_t		tail;
	uint16_t		len;

	uint16_t		ctx_curr;
	struct ixgbe_advctx_info ctx_cache[IXGBE_CTX_NUM];
};

