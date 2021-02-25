// SPDX-License-Identifier: GPL-2.0
/*
 * Common values for the Speck algorithm
 */

#ifndef _CRYPTO_SPECK_H
#define _CRYPTO_SPECK_H

//#include <linux/types.h>

#include <stdint.h>

typedef uint32_t u32;
typedef uint64_t u64;
typedef uint16_t u16;
typedef uint8_t u8;
typedef uint32_t __u32;

/**
 * rol32 - rotate a 32-bit value left
 * @word: value to rotate
 * @shift: bits to roll
 */
static inline __u32 rol32(__u32 word, unsigned int shift)
{
	return (word << shift) | (word >> ((-shift) & 31));
}

/**
 * ror32 - rotate a 32-bit value right
 * @word: value to rotate
 * @shift: bits to roll
 */
static inline __u32 ror32(__u32 word, unsigned int shift)
{
	return (word >> shift) | (word << (32 - shift));
}

static inline u32 get_unaligned_le32(const u8 *p)
{
	return p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24;
}

static inline void __put_unaligned_le16(u16 val, u8 *p)
{
	*p++ = val;
	*p++ = val >> 8;
}

static inline void put_unaligned_le32(u32 val, u8 *p)
{
	__put_unaligned_le16(val >> 16, p + 2);
	__put_unaligned_le16(val, p);
}


/* Speck64 */

#define SPECK64_BLOCK_SIZE	8

#define SPECK64_96_KEY_SIZE	12
#define SPECK64_96_NROUNDS	26

#define SPECK64_128_KEY_SIZE	16
#define SPECK64_128_NROUNDS	27

typedef struct speck64_tfm_ctx {
	u32 round_keys[SPECK64_128_NROUNDS];
	int nrounds;
} speck64_t;

void crypto_speck64_encrypt(const struct speck64_tfm_ctx *ctx,
			    u8 *out, const u8 *in);

void crypto_speck64_decrypt(const struct speck64_tfm_ctx *ctx,
			    u8 *out, const u8 *in);

int crypto_speck64_setkey(struct speck64_tfm_ctx *ctx, const u8 *key,
			  unsigned int keysize);

#endif /* _CRYPTO_SPECK_H */
