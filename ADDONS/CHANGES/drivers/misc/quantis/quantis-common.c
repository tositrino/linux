/*
 * Copyright (c) 2004, 2005, id Quantique SA, Switzerland
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of id Quantique nor the names of its contributors may be
 * used to endorse or promote products derived from this software without
 * specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**# Quantis PCI card **/

/**## Overview
 **
 ** This file contains the common routines to access the Quantis PCI
 ** card. The OS dependent parts are the softstate structure, for
 ** which you have to define the `QUANTIS_SOFT_T' type, and the
 ** register access, for which you have to define the `QUANTIS_REG'
 ** and `QUANTIS_REG_SET' macros. You also have to define the
 ** `QUANTIS_DEBUG2' and `QUANTIS_DEBUG0' macros used to print the
 ** status message, which is a string taking 2 parameters and 0
 ** parameters..
 **/

/**## Common defines
 **
 ** We define the register numbers, register names, mask names, and
 ** IDs.
 **
 ** We can now define the functions to control the PCI card. The
 ** vendor ID used by the PCI card is the HEVS vendor ID.
 **/

/**## PCI Card Registers
 **
 ** The Quantis PCI card has a memory register page 16 words long,
 ** consisting of 10 registers. These registers are split into the
 ** following functional **groups: the Core Control group, with the
 ** Enable Register `CC_ER', **the Disable Register `CC_DR' and the
 ** Status Register `CC_SR'. We **then have the Core Version group,
 ** with the Status Register `CV_SR', which is used to get the board
 ** version. Then there is the Fifo **Status group, with the Fifo
 ** Flush Register `FS_CA', the Fifo Status **Register `FS_RR' and the
 ** Fifo Data Read Register `FD_RR'. Finally, **the Module Group is
 ** used to enable and disable card modules, using **the Enable
 ** Register `MX_ER', the Disable Register `MX_DR' and the **Status
 ** Register `MX_SR'.
 **/

#define CC_ER 0
#define CC_DR 4
#define CC_SR 12
#define CV_SR 28
#define FS_CA 32
#define FS_RR 40
#define FD_RR 44
#define MX_ER 48
#define MX_DR 52
#define MX_SR 60

/**## PCI Card FIFO **/

/** The FIFO of the random number generator is 4096 bytes big. It can
 ** be either empty, one fourth full, half-full, three fourth full or
 ** completely full. The FIFO is flushed by writing a `0' to the Flush
 ** Register.
 **/

#define QUANTIS_FIFO_EMPTY 1
#define QUANTIS_FIFO_FL1   (1 << 1)
#define QUANTIS_FIFO_FL2   (1 << 2)
#define QUANTIS_FIFO_FL3   (1 << 3)
#define QUANTIS_FIFO_FULL  (1 << 6)
#define QUANTIS_FIFO_ERROR (1 << 7)

#define QUANTIS_FIFO_STATUS(scp) QUANTIS_REG(scp, FS_RR)
#define QUANTIS_FLUSH_FIFO(scp) QUANTIS_SET_REG(scp, FS_CA, 0)

/** Finally, we use a macro to print the state of the card. **/
#define QUANTIS_DEBUG_STATUS(string, scp) \
       QUANTIS_DEBUG2(string " (CC_SR = %04x, MX_SR = %04x)", \
                     QUANTIS_REG(scp, CC_SR), QUANTIS_REG(scp, MX_SR))

/**## PCI Card modules **/

/** The module registers are separated in 4 bytes with the flags of
 ** each device. However, the interface functions masks for each
 ** function, so we need conversion functions.
 **/

#define MX_SD  0
#define MX_SEN 1
#define MX_TM  2
#define MX_HEN 6
#define MX_HST 7

static INLINE u_int32_t mask2reg(u_int32_t mask, int type) {
  u_int32_t reg;
  reg = (mask & (1 << 0) ? 1 << 0  : 0)
    |   (mask & (1 << 1) ? 1 << 8  : 0)
    |   (mask & (1 << 2) ? 1 << 16 : 0)
    |   (mask & (1 << 3) ? 1 << 24 : 0);
  return reg << type;
}

static INLINE u_int32_t reg2mask(u_int32_t reg, int type) {
  reg >>= type;
  return (reg & (1 << 0) ? (1 << 0) : 0)
    |    (reg & (1 << 8)  ? (1 << 1) : 0)
    |    (reg & (1 << 16) ? (1 << 2) : 0)
    |    (reg & (1 << 24) ? (1 << 3) : 0);
}

/**## PCI Card routines **/

static INLINE u_int32_t quantis_rng_version(QUANTIS_SOFT_T *scp) {
  return QUANTIS_REG(scp, CV_SR);
}

static INLINE u_int32_t quantis_rng_modules_status(QUANTIS_SOFT_T *scp) {
  return
    reg2mask(QUANTIS_REG(scp, MX_SR), MX_SEN) &
    reg2mask(QUANTIS_REG(scp, MX_SR), MX_HST);
}

static INLINE int quantis_rng_error(QUANTIS_SOFT_T *scp) {
  u_int32_t reg    = QUANTIS_REG(scp, MX_SR);
  u_int32_t test   = reg2mask(reg, MX_TM);
  u_int32_t status = reg2mask(reg, MX_HST);
  u_int32_t enable = reg2mask(reg, MX_SEN);

  if (test) {
    return 0;
  } else {
    return (enable &~ status);
  }
}

/**
 ** This routines returns the number of bytes of random data in the
 ** FIFO.
 **/
static INLINE int quantis_rng_fifo_bytes(QUANTIS_SOFT_T *scp) {
  u_int32_t status = QUANTIS_FIFO_STATUS(scp);

#ifdef __linux__
  static int once = 1;

  if (once) {
      schedule();
      once = 0;
  }
#endif

  if (status & QUANTIS_FIFO_FULL) {
    return QUANTIS_FIFO_SIZE * 4;
  } else if (status & QUANTIS_FIFO_FL3) {
    return (3 * QUANTIS_FIFO_SIZE);
  } else if (status & QUANTIS_FIFO_FL2) {
    return (2 * QUANTIS_FIFO_SIZE);
  } else if (status & QUANTIS_FIFO_FL1) {
    return (1 * QUANTIS_FIFO_SIZE);
  } else {
    return 0;
  }
}

/** This routine resets the entire PCI board.
 **/
static INLINE void quantis_rng_reset(QUANTIS_SOFT_T *scp) {
  QUANTIS_DEBUG_STATUS("quantis_rng_reset", scp);

  QUANTIS_SET_REG(scp, CC_ER, 1);
  QUANTIS_DEBUG_STATUS("quantis_rng_reset (after ER reset)", scp);
  QUANTIS_SET_REG(scp, CC_DR, 1);
  QUANTIS_DEBUG_STATUS("quantis_rng_reset (after DR reset)", scp);
}

/** This routine returns the modules present as a mask of 4 bits. **/
static INLINE u_int32_t quantis_rng_modules_mask(QUANTIS_SOFT_T *scp) {
  QUANTIS_DEBUG_STATUS("quantis_rng_modules_mask", scp);

  return reg2mask(QUANTIS_REG(scp, MX_SR), MX_HEN);
}

/** This routine enables the modules specified by `mask'. **/
static INLINE void quantis_rng_enable_modules(QUANTIS_SOFT_T *scp, u_int32_t mask) {
  QUANTIS_DEBUG_STATUS("quantis_rng_enable_modules", scp);

  /* enable modules */
  QUANTIS_SET_REG(scp, MX_ER, mask2reg(mask, MX_SEN));
  QUANTIS_FLUSH_FIFO(scp);

  QUANTIS_DEBUG_STATUS("quantis_rng_enable_modules (after enabling)", scp);
}

/** This routine disables the modules specified by `mask'. **/
static INLINE void quantis_rng_disable_modules(QUANTIS_SOFT_T *scp, u_int32_t mask) {
  QUANTIS_DEBUG_STATUS("quantis_rng_disable_modules", scp);

  /* enable modules */
  QUANTIS_SET_REG(scp, MX_DR, mask2reg(mask, MX_SEN));
  QUANTIS_FLUSH_FIFO(scp);

  QUANTIS_DEBUG_STATUS("quantis_rng_disable_modules (after disabling)", scp);
}

/**
 ** Wait till the FIFO is not empty anymore, or a timeout happens
 ** (poll the FIFO 1000 times). A timeout is supposed to be a hard
 ** error.
 **/
static INLINE int quantis_rng_wait_fifo(QUANTIS_SOFT_T *scp) {
  unsigned int timeout = 10000;
  do {
    timeout--;
  } while ((timeout > 0) && (quantis_rng_fifo_bytes(scp) == 0));

  if (timeout <= 0) {
    return -1;
  } else {
    return 0;
  }
}

/**
 ** This routine fills `buffer' with `length' random bytes, waiting
 ** for the FIFO to be filled. However, if a timeout happens, the
 ** routine returns `-1' to signal an error.
 **/
static INLINE int quantis_rng_read(QUANTIS_SOFT_T *scp,
				   unsigned char *buffer, int length) {
  int bytes = 0;
  QUANTIS_DEBUG_STATUS("quantis_rng_read", scp);

  /* XXX verify at least one module is enabled. */
  
  if (quantis_rng_error(scp)) {
    QUANTIS_DEBUG0("Module status error");
    return -1;
  }
  
  while (bytes < length) {
    int count = 0;

  again:    
    if (quantis_rng_wait_fifo(scp) < 0) {
      QUANTIS_DEBUG0("Timeout");
      return -1;
    }

    count = quantis_rng_fifo_bytes(scp) * 4;
    if (count > (length - bytes))
      count = (length - bytes);

    while (count > 0) {
      /* this can potentially cause endianness problems */
      u_int32_t tmp = QUANTIS_REG(scp, FD_RR);
      int len;

      if (QUANTIS_FIFO_STATUS(scp) & QUANTIS_FIFO_ERROR) {
	/* if the FIFO has overflown, reset it */
	QUANTIS_FLUSH_FIFO(scp);
	goto again;
      }

      len = count > 4 ? 4 : count;

      if (bytes+len > 4 * QUANTIS_FIFO_SIZE) {
	return bytes;
      }

      
      memcpy(buffer + bytes, &tmp, len);
      count -= len;
      bytes += len;
    }
  } 

  return bytes;
}

/** We are the only functions that get to access the registers directly! **/
#undef CC_ER
#undef CC_DR
#undef CC_SR
#undef CV_SR
#undef FS_CA
#undef FS_RR
#undef FD_RR
#undef MX_ER
#undef MX_DR
#undef MX_SR
