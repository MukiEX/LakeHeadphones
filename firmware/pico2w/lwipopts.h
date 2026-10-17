/* lwIP options for the web configuration page (USB network, NO_SYS).
 * Everything lwIP runs in the core 0 main loop; the USB side only hands
 * frames over through webcfg.c's buffers. */
#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS                      1
#define SYS_LIGHTWEIGHT_PROT        0
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define LWIP_RAW                    0
#define LWIP_TIMERS                 1

#define MEM_ALIGNMENT               4
#define MEM_SIZE                    4000
#define MEMP_NUM_PBUF               16
#define MEMP_NUM_TCP_PCB            4
#define MEMP_NUM_TCP_PCB_LISTEN     2
#define MEMP_NUM_TCP_SEG            24
#define MEMP_NUM_UDP_PCB            2
#define PBUF_POOL_SIZE              4

#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0
#define LWIP_ICMP                   1
#define LWIP_UDP                    1
#define LWIP_TCP                    1
#define LWIP_DHCP                   0
#define LWIP_DNS                    0
#define LWIP_IGMP                   0
#define LWIP_SINGLE_NETIF           1
#define LWIP_NETIF_HOSTNAME         0
#define LWIP_IP_ACCEPT_UDP_PORT(p)  ((p) == PP_NTOHS(67))   /* DHCP server sees broadcasts */
#define ETH_PAD_SIZE                0

#define TCP_MSS                     1460
#define TCP_SND_BUF                 (4 * TCP_MSS)
#define TCP_WND                     (4 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * TCP_SND_BUF + TCP_MSS - 1) / TCP_MSS)
#define LWIP_TCP_KEEPALIVE          0

#define LWIP_STATS                  0
#define LWIP_STATS_DISPLAY          0
#define LWIP_DEBUG                  0
#define LWIP_NOASSERT               1

#endif
