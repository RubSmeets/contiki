/*
 * sec-arp-server.c
 *
 *  Created on: Sep 24, 2013
 *      Author: crea
 */

#include "net/sec-arp-server.h"
#include "net/uip-ds6.h"
#include "dev/slip.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while (0)
#endif
