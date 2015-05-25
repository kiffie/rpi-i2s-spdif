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

#include "spdif-encoder.h"

static uint16_t spdif_biphase_encode(int last, uint8_t data){
	int i;
	uint16_t result=0;

	for( i=0; i< 8; i++){
		result<<=2;
		if( data&1 ){
			result|= 0b10;
		}else{
			result|= 0b11;
		}
		if( last ){
			result^= 0b11;
		}
		last= result&1;
		data>>=1;
	}
	return result;
}

static uint16_t spdif_preamble_encode(int last, uint8_t data){

	uint16_t result=0;
	int i;
	switch( data & SPDIF_PREAMBLE_MASK ){
		case SPDIF_PREAMBLE_X:
			result= 0b11100010;
			break;
		case SPDIF_PREAMBLE_Y:
			result= 0b11100100;
			break;
		case SPDIF_PREAMBLE_Z:
			result= 0b11101000;
			break;
		}
	if( last ){
		result^= 0xff;
	}
	last= result&1;
	data>>=4;
	for(i=0; i<4; i++){
		result<<=2;
		if( data&1 ){
			result|= 0b10;
		}else{
			result|= 0b11;
		}
		if( last ){
			result^= 0b11;
		}
		last= result&1;
		data>>=1;
	}
	return result;
}

static void spdif_init_bytes(struct spdif_encoder *spdif){
	int i;
	for(i=0; i< 256; i++){
		spdif->first_byte[i]= spdif_preamble_encode(0, i);
		spdif->byte[i]= spdif_biphase_encode(0, i);
	}
}


static void spdif_fast_encode(struct spdif_encoder *spdif,
			      void *encoded_buf, uint32_t subframe)
{
	uint8_t *bytes= (uint8_t *)&subframe;
	uint32_t parity, tmp;
	uint16_t data;
	uint16_t *encoded= (uint16_t *)encoded_buf;

	tmp= (subframe & ~SPDIF_PREAMBLE_MASK)<<16; /* exclude preamble bits */
	parity= subframe^tmp;
	tmp= parity << 8;
	parity^= tmp;
	tmp= parity << 4;
	parity^= tmp;
	tmp= parity << 2;
	parity^= tmp;
	tmp= parity << 1;
	parity^= tmp;
	subframe|= (parity&0x80000000);

	data= spdif->first_byte[bytes[0]];
	if( spdif->last ){
		data^= 0xffff;
	}
	spdif->last= data&1;
	encoded[1]=data;

	data= spdif->byte[bytes[1]];
	if( spdif->last ){
		data^= 0xffff;
	}
	spdif->last= data&1;
	encoded[0]=data;

	data= spdif->byte[bytes[2]];
	if( spdif->last ){
		data^= 0xffff;
	}
	spdif->last= data&1;
	encoded[3]=data;

	data= spdif->byte[bytes[3]];
	if( spdif->last ){
		data^= 0xffff;
	}
	spdif->last= data&1;
	encoded[2]=data;
}


void spdif_encode_frame_generic(struct spdif_encoder *spdif,
				void *encoded,
				int left_shifted, int right_shifted)
{
	uint8_t *p= encoded;
	uint32_t subframe;
	subframe= spdif->frame_ctr == 0 ? SPDIF_PREAMBLE_Z : SPDIF_PREAMBLE_X;
	subframe|= left_shifted;
	spdif_fast_encode(spdif, p, subframe);
	p+= SPDIF_FRAMESIZE/2;
	subframe= SPDIF_PREAMBLE_Y | right_shifted;
	spdif_fast_encode(spdif, p, subframe);
	if( ++spdif->frame_ctr >= SPDIF_BLOCKSIZE ){
		spdif->frame_ctr= 0;
	}
}

void spdif_init(struct spdif_encoder *spdif){
	spdif_init_bytes(spdif);
}

/* end of SPDIF encoder */

