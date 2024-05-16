/*
 * SPDIF encoder
 *
 * Copyright (C) 2015, 2023 Stephan "Kiffie" <kiffie.vanhaash@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __SPDIF_ENCODER_H__
#define __SPDIF_ENCODER_H__

#include <linux/types.h>

#define SPDIF_FRAMESIZE 16  /* size of encoded SPDIF frame in bytes */
#define SPDIF_BLOCKSIZE 192 /* number of frames per SPDIF block */
#define SPDIF_CHSTATSIZE (SPDIF_BLOCKSIZE/8) /* size of channel status block */

typedef struct spdif_encoder {
	uint16_t first_byte[256];
	uint16_t byte[256];

	bool last; /* value of last encoded bit */

	uint8_t frame_ctr;
	uint8_t channel_status[SPDIF_CHSTATSIZE];
} spdif_encoder_t;

typedef struct spdif_frame {
    uint32_t data[SPDIF_FRAMESIZE/sizeof(uint32_t)];
} spdif_frame_t;

#define SPDIF_PREAMBLE_X	0x00   /* channel A (left) */
#define SPDIF_PREAMBLE_Y	0x01   /* channel B (right) */
#define SPDIF_PREAMBLE_Z	0x02   /* channel A, start of block */
#define SPDIF_PREAMBLE_MASK	0x0f
#define SPDIF_P_MASK            0x80000000  /* parity bit */
#define SPDIF_C_MASK            0x40000000  /* channel status bit */
#define SPDIF_U_MASK            0x20000000  /* user data bit */
#define SPDIF_V_MASK            0x10000000  /* validity bit */


/* channel status bits (consumer mode) */

#define SPDIF_CS0_PROFESSIONAL      0x01
#define SPDIF_CS0_NONAUDIO          0x02
#define SPDIF_CS0_NOT_COPYRIGHT     0x04
#define SPDIF_CS0_EMPHASIS_MASK     0x38
#define SPDIF_CS0_EMPHASIS_NONE     0x00
#define SPDIF_CS0_EMPHASIS_5015     0x08

#define SPDIF_CS1_CATEGORY          0x7f
#define SPDIF_CS1_GENERAL           0x00
#define SPDIF_CS1_DDCONV            0x02
#define SPDIF_CS1_ORIGINAL          0x80

#define SPDIF_CS3_FS_MASK           0x0f
#define SPDIF_CS3_44100             0x00
#define SPDIF_CS3_UNSPEC            0x01
#define SPDIF_CS3_48000             0x02
#define SPDIF_CS3_32000             0x03
#define SPDIF_CS3_22050             0x04
#define SPDIF_CS3_24000             0x06
#define SPDIF_CS3_88200             0x08
#define SPDIF_CS3_96000             0x0a
#define SPDIF_CS3_176400            0x0c
#define SPDIF_CS3_192000            0x0e

#define SPDIF_CS3_CLOCK_MASK        0x30
#define SPDIF_CS3_CLOCK_1000PPM     0x00
#define SPDIF_CS3_CON_CLOCK_50PPM   0x10
#define SPDIF_CS3_CON_CLOCK_VAR     0x20

#define SPDIF_CS4_MAX_WORDLEN_24    0x01    /* 0 = 20 bit, 1 = 24 bit */
#define SPDIF_CS4_WORDLEN_MASK      0x0e
#define SPDIF_CS4_WORDLEN_UNSPEC    0x00
#define SPDIF_CS4_WORDLEN_20_16     0x02
#define SPDIF_CS4_WORDLEN_22_18     0x04
#define SPDIF_CS4_WORDLEN_23_19     0x08
#define SPDIF_CS4_WORDLEN_24_20     0x0a
#define SPDIF_CS4_WORDLEN_21_17     0x0c


void spdif_encoder_init(struct spdif_encoder *spdif);
void spdif_encoder_set_channel_status(struct spdif_encoder *spdif,
                                      const void *cs, size_t len);

void spdif_encode_frame_generic(struct spdif_encoder *spdif,
				void *encoded,
				int left_shifted, int right_shifted);

static inline void spdif_encode_frame_s24le(struct spdif_encoder *spdif,
			      void *encoded,
			      const void *frame)
{
	const uint32_t *f= frame;
	int left = (f[0] & 0x00ffffff)<<4;
	int right= (f[1] & 0x00ffffff)<<4;
	spdif_encode_frame_generic(spdif, encoded, left, right);
}

static inline void spdif_encode_frame_s24le_packed(struct spdif_encoder *spdif,
                                                   void *encoded,
                                                   const void *frame)
{
	const uint8_t *f = frame;
	int left = (f[0]|(f[1]<<8)|(f[2]<<16)) << 4;
	int right= (f[3]|(f[4]<<8)|(f[5]<<16)) << 4;
	spdif_encode_frame_generic(spdif, encoded, left, right);
}

static inline void spdif_encode_frame_s16le(struct spdif_encoder *spdif,
					    void *encoded,
					    const void *frame)
{
	const uint16_t *f = frame;
	int left = (f[0] & 0x0000ffff)<<12;
	int right= (f[1] & 0x0000ffff)<<12;
	spdif_encode_frame_generic(spdif, encoded, left, right);
}

static inline void spdif_encode_frame_zero(struct spdif_encoder *spdif, void *encoded) {
	spdif_encode_frame_generic(spdif, encoded, 0, 0);
}

#endif
