// SPDX-License-Identifier: GPL-2.0
/*
 * Speck: a lightweight block cipher
 *
 * Copyright (c) 2018 Google, Inc
 *
 * Speck has 10 variants, including 5 block sizes.  For now we only implement
 * the variants Speck128/128, Speck128/192, Speck128/256, Speck64/96, and
 * Speck64/128.   Speck${B}/${K} denotes the variant with a block size of B bits
 * and a key size of K bits.  The Speck128 variants are believed to be the most
 * secure variants, and they use the same block size and key sizes as AES.  The
 * Speck64 variants are less secure, but on 32-bit processors are usually
 * faster.  The remaining variants (Speck32, Speck48, and Speck96) are even less
 * secure and/or not as well suited for implementation on either 32-bit or
 * 64-bit processors, so are omitted.
 *
 * Reference: "The Simon and Speck Families of Lightweight Block Ciphers"
 * https://eprint.iacr.org/2013/404.pdf
 *
 * In a correspondence, the Speck designers have also clarified that the words
 * should be interpreted in little-endian format, and the words should be
 * ordered such that the first word of each block is 'y' rather than 'x', and
 * the first key word (rather than the last) becomes the first round key.
 */

//#include <asm/unaligned.h>
#include "speck.h"
#include <stdio.h>
//#include <linux/bitops.h>
//#include <linux/crypto.h>
//#include <linux/init.h>
//#include <linux/module.h>


/* Speck64 */

static inline void speck64_round(u32 *x, u32 *y, u32 k)
{
	*x = ror32(*x, 8);
	*x += *y;
	*x ^= k;
	*y = rol32(*y, 3);
	*y ^= *x;
}

static inline void speck64_unround(u32 *x, u32 *y, u32 k)
{
	*y ^= *x;
	*y = ror32(*y, 3);
	*x ^= k;
	*x -= *y;
	*x = rol32(*x, 8);
}

void crypto_speck64_encrypt(const struct speck64_tfm_ctx *ctx,
			    u8 *out, const u8 *in)
{
	u32 y = get_unaligned_le32(in);
	u32 x = get_unaligned_le32(in + 4);
	int i;

	for (i = 0; i < ctx->nrounds; i++)
		speck64_round(&x, &y, ctx->round_keys[i]);

	put_unaligned_le32(y, out);
	put_unaligned_le32(x, out + 4);
}

void crypto_speck64_decrypt(const struct speck64_tfm_ctx *ctx,
			    u8 *out, const u8 *in)
{
	u32 y = get_unaligned_le32(in);
	u32 x = get_unaligned_le32(in + 4);
	int i;

	for (i = ctx->nrounds - 1; i >= 0; i--)
		speck64_unround(&x, &y, ctx->round_keys[i]);

	put_unaligned_le32(y, out);
	put_unaligned_le32(x, out + 4);
}

int crypto_speck64_setkey(struct speck64_tfm_ctx *ctx, const u8 *key,
			  unsigned int keylen)
{
	u32 l[3];
	u32 k;
	int i;

	switch (keylen) {
	case SPECK64_96_KEY_SIZE:
		k = get_unaligned_le32(key);
		l[0] = get_unaligned_le32(key + 4);
		l[1] = get_unaligned_le32(key + 8);
		ctx->nrounds = SPECK64_96_NROUNDS;
		for (i = 0; i < ctx->nrounds; i++) {
			ctx->round_keys[i] = k;
			speck64_round(&l[i % 2], &k, i);
		}
		break;
	case SPECK64_128_KEY_SIZE:
		k = get_unaligned_le32(key);
		l[0] = get_unaligned_le32(key + 4);
		l[1] = get_unaligned_le32(key + 8);
		l[2] = get_unaligned_le32(key + 12);
		ctx->nrounds = SPECK64_128_NROUNDS;
		for (i = 0; i < ctx->nrounds; i++) {
			ctx->round_keys[i] = k;
			speck64_round(&l[i % 3], &k, i);
		}
		break;
	default:
		return -1;
	}

	return 0;
}

