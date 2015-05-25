/*
 * SPDIF encoder
 *
 * Copyright (C) 2015 Kiffie van Haash <kiffie.vanhaash@gmail.com>
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

struct spdif_encoder {
	uint16_t first_byte[256];
	uint16_t byte[256];

	int last; /* value of last encoded bit */

	int frame_ctr;
};


#define SPDIF_PREAMBLE_X	0x00   /* channel A (left) */
#define SPDIF_PREAMBLE_Y	0x01   /* channel B (right) */
#define SPDIF_PREAMBLE_Z	0x02   /* channel A, start of block */
#define SPDIF_PREAMBLE_MASK	0x0f

void spdif_init(struct spdif_encoder *spdif);

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

static inline void spdif_encode_frame_s16le(struct spdif_encoder *spdif,
					    void *encoded,
					    const void *frame)
{
	const uint16_t *f= frame;
	int left = (f[0] & 0x0000ffff)<<12;
	int right= (f[1] & 0x0000ffff)<<12;
	spdif_encode_frame_generic(spdif, encoded, left, right);
}

#endif
