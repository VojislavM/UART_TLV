/*
 * inet.c
 *
 *  Created on: 29. jun. 2016
 *      Author: vojis
 */


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "inet.h"


#ifdef LITTLE_ENDIAN

uint16_t htons(uint16_t v) {
  return (v >> 8) | (v << 8);
}
uint32_t htonl(uint32_t v) {
  return htons(v >> 16) | (htons((uint16_t) v) << 16);
}

#else

uint16_t htons(uint16_t v) {
  return v;
}
uint32_t htonl(uint32_t v) {
  return v;
}

#endif

uint16_t ntohs(uint16_t v) {
  return htons(v);
}
uint32_t ntohl(uint32_t v) {
  return htonl(v);
}
