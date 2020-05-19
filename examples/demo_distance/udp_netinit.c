/****************************************************************************
 * examples/udp/udp_netinit.c
 *
 *   Copyright (C) 2007, 2011, 2015, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <net/if.h>
#include <netinet/ether.h>

#include "netutils/netlib.h"

#include "udp_netinit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#  define DEVNAME "eth0"

/****************************************************************************
 * udp_netinit
 ****************************************************************************/

int udp_netinit(void)
{
  uint8_t mac[IFHWADDRLEN];
  struct in_addr addr;

  mac[0] = 0x00;
  mac[1] = 0xe0;
  mac[2] = 0xde;
  mac[3] = 0xad;
  mac[4] = 0xbe;
  mac[5] = 0x01;

  netlib_setmacaddr(DEVNAME, mac);
  /* Set up our host address */
  
  addr.s_addr = HTONL(0xC0A80A0A);
  netlib_set_ipv4addr(DEVNAME, &addr);
  
  /* Set up the default router address */
  
  addr.s_addr = HTONL(0xC0A80A01);
  netlib_set_dripv4addr(DEVNAME, &addr);
  
  /* Setup the subnet mask */
  
  addr.s_addr = HTONL(0xffffff00);
  netlib_set_ipv4netmask(DEVNAME, &addr);
  
  
  /* New versions of netlib_set_ipvXaddr will not bring the network up,
  * So ensure the network is really up at this point.
  */
  
  netlib_ifup(DEVNAME);

  return 0;
}
