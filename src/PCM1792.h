#ifndef PCM1792_H
#define PCM1792_H

#define PCM1792_I2C_ADDR 0x4C

/* Register 18 */
#define PCM1792A_ATLD_MASK		0x80
#define PCM1792A_ATLD_DISABLED	0x00
#define PCM1792A_ATLD_ENABLED	0x80

#define PCM1792A_FMT_MASK		(0x07 << 4)
#define PCM1792A_FMT_16BR		(0x00 << 4)	// 16-bit standard, right-justified format data
#define PCM1792A_FMT_20BR		(0x01 << 4)	// 20-bit standard, right-justified format data
#define PCM1792A_FMT_24BR		(0x02 << 4)	// 24-bit standard, right-justified format data
#define PCM1792A_FMT_24BL		(0x03 << 4)	// 24-bit MSB-first, left-justified format data
#define PCM1792A_FMT_16BI2S		(0x04 << 4)	// 16-bit I 2 S format data
#define PCM1792A_FMT_24BI2S		(0x05 << 4)	// 24-bit I 2 S format data (default)

#define PCM1792A_DMF_MASK		(0x03 << 2)
#define PCM1792A_DMF_DISABLE	(0x00 << 3)
#define PCM1792A_DMF_48KHZ		(0x01 << 3)
#define PCM1792A_DMF_44_1KHZ	(0x02 << 3)
#define PCM1792A_DMF_32KHZ		(0x03 << 3)

#define PCM1792A_DME_MASK		(0x01 < 1)
#define PCM1792A_DME_DISABLED	(0x00 < 1)
#define PCM1792A_DME_ENABLED	(0x01 < 1)

#define PCM1792A_MUTE_MASK		1
#define PCM1792A_MUTE_DISABLED	0
#define PCM1792A_MUTE_ENABLED	1


/* register 19 */
#define PCM1792A_REV_MASK		0x80
#define PCM1792A_REV_ENABLED	0x00 // Normal output
#define PCM1792A_REV_DISABLED	0x80 // Inverted output

#define PCM1792A_ATS_MASK		(0x03 << 5)
#define PCM1792A_ATS_LRCK_1		(0x00 << 5) // Attenuation Rate = LRCK/1
#define PCM1792A_ATS_LRCK_2		(0x01 << 5) // Attenuation Rate = LRCK/2
#define PCM1792A_ATS_LRCK_4		(0x02 << 5) // Attenuation Rate = LRCK/4
#define PCM1792A_ATS_LRCK_8		(0x03 << 5) // Attenuation Rate = LRCK/8

#define PCM1792A_OPE_MASK		0x10
#define PCM1792A_OPE_ENABLED	0x10
#define PCM1792A_OPE_DISABLED	0x00

#define PCM1792A_DFMS_MASK			0x04
#define PCM1792A_DFMS_MONAURAL		0x00
#define PCM1792A_DFMS_STEREO_INPUT	0x04

#define PCM1792A_FLT_MASK		0x02
#define PCM1792A_FLT_SHARP		0x00
#define PCM1792A_FLT_SLOW		0x02

#define PCM1792A_INZD_MASK		0x01 // Infinit zero detect mute
#define PCM1792A_INZD_ENABLED	0x01
#define PCM1792A_INZD_DISABLED	0x00

// XXX: Other registers to be implemented.

/* Low level API */
void pcm1792_write_register(uint8_t reg, uint8_t data);
void pcm1792_write_register_field(uint8_t reg, uint8_t mask, uint8_t value, int write_to_device);

/* High level API */
void pcm1792_set_volume_left(uint8_t volume);
void pcm1792_set_volume_right(uint8_t volume);
void pcm1792_set_mute(uint8_t mute);
void pcm1792_set_atld(uint8_t atld);
void pcm1792_set_fmt(uint8_t fmt);
void pcm1792_set_dmf(uint8_t dmf);
void pcm1792_set_dme(uint8_t dme);

#endif // PCM1792_H
