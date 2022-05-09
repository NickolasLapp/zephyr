/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "ccm_mode2_soft.h"

#include <string.h>

#include "aes.h"

/* How it all works:
 * Generation of the MIC and encryption are two separate procedures.
 *
 * Everything in AES-CCM happens in 16 byte blocks.
 *
 * Encryption of the payload is done by making a cipher stream S and xoring this with the payload:
 * enc_data = (S[1..N] xor data), where N = (data_len + 15) / 16 + 1. Note that we're skipping S[0].
 * S is made up of 16 byte blocks, where S[i] = AES(A[i]) and
 *
 *        | 1 byte    | 16 - L_LEN bytes | L_LEN bytes |
 * A[i] = | L_LEN - 1 | Nonce            | i           |
 *
 * To generate the MIC, we make a chain of 16 byte blocks X[i], i=0..M, where
 *
 * M = (a_len + 15) / 16 + (data_len + 15) / 16 + 1
 *
 * X[0] = AES(B[0])
 * X[i] = enc(X[i-1] xor B[i]).
 *
 * B is made up of some metadata, the additional data and the data:
 *
 *           | 16 bytes | 2 bytes | a_len bytes | 16-(a_len % 16) | data_len | 16-(data_len % 16) |
 * B[0..M] = | Metadata | a_len   | a_data ...  | 000000000000000 | data     | 000000000000000000 |
 *
 * The metadata (B[0]) is made up of 1 byte of flags, the (16 - L_LEN - 1) byte nonce and mic_len.
 *
 * Finally, the MIC = (X[M] xor S[0]), truncated to fit the desired MIC-length. NOTE: X[M] is named
 * T in the spec and code.
 *
 * To decrypt, we first calculate data = (S[1..N] xor enc_data), then insert this clear text data
 * into B, calculate the MIC, and compare it.
 */

/* All multibyte numbers are in big endian. Nonces, keys and data are represented as byte streams,
 * and are not encoded. */
#define L_LEN CCM_LENGTH_FIELD_LENGTH
#define CCM_BLOCK_SIZE CCM_KEY_LENGTH

#define BE2LE16(n) ((((n) << 8) & 0xff00) | (((n) >> 8) & 0x00ff)) /**< Converts a 16-bit big-endian number to little-endian. */
#define LE2BE16(n) BE2LE16(n)                                      /**< Converts a 16-bit little-endian number to big-endian. */

typedef struct
{
    uint8_t len_field_len;
    uint8_t nonce[CCM_NONCE_LENGTH];
    uint16_t counter;
} a_block_t;

/**
 * Bytewise XOR for an array.
 *
 * XORs size amount of data from p_src1 and p_src2 and stores it in p_dst.
 *
 * @note p_dst may be equal to one or more of the sources.
 *
 * @param p_dst  Destination address.
 * @param p_src1 First source address.
 * @param p_src2 Secound source address.
 * @param size   Number of bytes to XOR.
 */
static inline void utils_xor(uint8_t * p_dst, const uint8_t * p_src1, const uint8_t * p_src2, uint16_t size)
{
    while (0 != size)
    {
        size--;
        p_dst[size] = p_src1[size] ^ p_src2[size];
    }
}

/**
 * Encrypt all data. Assumes p_aes_data already has key set and cleartext=A[0]
 */
static void ccm_soft_crypt(ccm_soft_data_t * p_data, aes_data_t * p_aes_data)
{
    uint16_t i = 1;
    uint16_t octets_m = p_data->m_len;

    a_block_t * p_a = (a_block_t *) p_aes_data->cleartext;

    while (octets_m)
    {
        /* Just alter the already created A-block */
        p_a->counter = LE2BE16(i);
        /* S[i] = AES(A[i]) */
        aes_encrypt((nrf_ecb_hal_data_t *) p_aes_data);

        uint8_t block_size = (octets_m > CCM_BLOCK_SIZE ? CCM_BLOCK_SIZE : octets_m);
        /* enc_data = (S xor data) */
        utils_xor(&p_data->p_out[CCM_BLOCK_SIZE * (i - 1)],
                  &p_data->p_m[CCM_BLOCK_SIZE * (i - 1)],
                  p_aes_data->ciphertext,
                  block_size);
        octets_m -= block_size;
        i++;
    }
}

static inline void build_a_block(const uint8_t * p_nonce, void * A0, uint16_t i)
{
    a_block_t * p_a_block = (a_block_t *) A0;
    p_a_block->len_field_len = (L_LEN - 1); /* encoded */
    memcpy(p_a_block->nonce, p_nonce, CCM_NONCE_LENGTH);
    p_a_block->counter = LE2BE16(i);
}

void ccm_mode2_soft_encrypt(ccm_soft_data_t * p_data)
{
    aes_data_t aes_data;

    memcpy(aes_data.key, p_data->p_key, CCM_BLOCK_SIZE);

    build_a_block(p_data->p_nonce, aes_data.cleartext, 0);

    /* aes_data.cleartext now contains A0, no need to regenerate it. */
    ccm_soft_crypt(p_data, &aes_data);
}

void ccm_mode2_soft_decrypt(ccm_soft_data_t * p_data)
{
    aes_data_t aes_data;

    memcpy(aes_data.key, p_data->p_key, CCM_BLOCK_SIZE);

    if (p_data->m_len > 0)
    {
        /* Try to decrypt data with ciphers. */
        build_a_block(p_data->p_nonce, aes_data.cleartext, 0);
        ccm_soft_crypt(p_data, &aes_data);
    }
}
