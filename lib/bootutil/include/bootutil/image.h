#ifndef H_IMAGE_
#define H_IMAGE_

#include <inttypes.h>


struct flash_area;

#define IMAGE_MAGIC                 0x96f3b83d
#define IMAGE_MAGIC_V1              0x96f3b83c
#define IMAGE_MAGIC_NONE            0xffffffff
#define IMAGE_TLV_INFO_MAGIC        0x6907

#define IMAGE_HEADER_SIZE           32
#define PACKED      __attribute__((packed))

/*
 * Image header flags.
 */
#define IMAGE_F_PIC                      0x00000001 /* Not supported. */
#define IMAGE_F_NON_BOOTABLE             0x00000010 /* Split image app. */
/*
 * Indicates that this image should be loaded into RAM instead of run
 * directly from flash.  The address to load should be in the
 * ih_load_addr field of the header.
 */
#define IMAGE_F_RAM_LOAD                 0x00000020

/*
 * ECSDA224 is with NIST P-224
 * ECSDA256 is with NIST P-256
 */

/*
 * Image trailer TLV types.
 *
 * Signature is generated by computing signature over the image hash.
 * Currently the only image hash type is SHA256.
 *
 * Signature comes in the form of 2 TLVs.
 *   1st on identifies the public key which should be used to verify it.
 *   2nd one is the actual signature.
 */
#define IMAGE_TLV_KEYHASH           0x01   /* hash of the public key */
#define IMAGE_TLV_SHA256            0x10   /* SHA256 of image hdr and body */
#define IMAGE_TLV_RSA2048_PSS       0x20   /* RSA2048 of hash output */
#define IMAGE_TLV_ECDSA224          0x21   /* ECDSA of hash output */
#define IMAGE_TLV_ECDSA256          0x22   /* ECDSA of hash output */
#define IMAGE_TLV_ED25519           0x23   /* ED25519 of hash output */

struct PACKED image_version {
    uint8_t iv_major;
    uint8_t iv_minor;
    uint16_t iv_revision;
    uint32_t iv_build_num;
};

/** Image header.  All fields are in little endian byte order. */
struct PACKED image_header {
    uint32_t ih_magic;
    uint32_t ih_load_addr;
    uint16_t ih_hdr_size; /* Size of image header (bytes). */
    uint16_t _pad1;
    uint32_t ih_img_size; /* Does not include header. */
    uint32_t ih_flags;    /* IMAGE_F_[...]. */
    struct image_version ih_ver;
    uint32_t _pad2;
};

/** Image TLV header.  All fields in little endian. */
struct PACKED image_tlv_info {
    uint16_t it_magic;
    uint16_t it_tlv_tot;  /* size of TLV area (including tlv_info header) */
};

/** Image trailer TLV format. All fields in little endian. */
struct PACKED image_tlv {
    uint8_t  it_type;   /* IMAGE_TLV_[...]. */
    uint8_t  _pad;
    uint16_t it_len;    /* Data length (not including TLV header). */
};

int bootutil_img_validate(struct image_header *hdr,
                          const struct flash_area *fap,
                          uint8_t *tmp_buf, uint32_t tmp_buf_sz,
                          uint8_t *seed, int seed_len, uint8_t *out_hash);

#endif
