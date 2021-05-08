// Wire Master Writer
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Writes data to an I2C/TWI slave device
// Refer to the "Wire Slave Receiver" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

//
// Source code for application to transmit image from ov7670 to PC via USB
// By Siarhei Charkes in 2015
// http://privateblog.info 
//

#include <stdint.h>
#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#define F_CPU 16000000UL
#define vga   0
#define qvga  1
#define qqvga   2
#define yuv422  0
#define rgb565  1
#define bayerRGB  2
#define camAddr_WR  0x42
#define camAddr_RD  0x43

/* Registers */
#define REG_GAIN    0x00  /* Gain lower 8 bits (rest in vref) */
#define REG_BLUE    0x01  /* blue gain */
#define REG_RED       0x02  /* red gain */
#define REG_VREF    0x03  /* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1    0x04  /* Control 1 */
#define COM1_CCIR656  0x40    /* CCIR656 enable */

#define REG_BAVE    0x05  /* U/B Average level */
#define REG_GbAVE   0x06  /* Y/Gb Average level */
#define REG_AECHH   0x07  /* AEC MS 5 bits */
#define REG_RAVE    0x08  /* V/R Average level */
#define REG_COM2    0x09  /* Control 2 */
#define COM2_SSLEEP         0x10  /* Soft sleep mode */
#define REG_PID           0x0a  /* Product ID MSB */
#define REG_VER           0x0b  /* Product ID LSB */
#define REG_COM3    0x0c  /* Control 3 */
#define COM3_SWAP         0x40  /* Byte swap */
#define COM3_SCALEEN          0x08  /* Enable scaling */
#define COM3_DCWEN          0x04  /* Enable downsamp/crop/window */
#define REG_COM4    0x0d  /* Control 4 */
#define REG_COM5    0x0e  /* All "reserved" */
#define REG_COM6    0x0f  /* Control 6 */
#define REG_AECH    0x10  /* More bits of AEC value */
#define REG_CLKRC   0x11  /* Clocl control */
#define CLK_EXT           0x40  /* Use external clock directly */
#define CLK_SCALE   0x3f  /* Mask for internal clock scale */
#define REG_COM7    0x12  /* Control 7 */ //REG mean address.
#define COM7_RESET          0x80  /* Register reset */
#define COM7_FMT_MASK         0x38
#define COM7_FMT_VGA          0x00
#define COM7_FMT_CIF          0x20  /* CIF format */
#define COM7_FMT_QVGA         0x10  /* QVGA format */
#define COM7_FMT_QCIF         0x08  /* QCIF format */
#define COM7_RGB          0x04  /* bits 0 and 2 - RGB format */
#define COM7_YUV          0x00  /* YUV */
#define COM7_BAYER          0x01  /* Bayer format */
#define COM7_PBAYER         0x05  /* "Processed bayer" */
#define REG_COM8    0x13  /* Control 8 */
#define COM8_FASTAEC          0x80  /* Enable fast AGC/AEC */
#define COM8_AECSTEP          0x40  /* Unlimited AEC step size */
#define COM8_BFILT    0x20  /* Band filter enable */
#define COM8_AGC    0x04  /* Auto gain enable */
#define COM8_AWB    0x02  /* White balance enable */
#define COM8_AEC    0x01  /* Auto exposure enable */
#define REG_COM9    0x14  /* Control 9- gain ceiling */
#define REG_COM10   0x15  /* Control 10 */
#define COM10_HSYNC         0x40  /* HSYNC instead of HREF */
#define COM10_PCLK_HB         0x20  /* Suppress PCLK on horiz blank */
#define COM10_HREF_REV          0x08  /* Reverse HREF */
#define COM10_VS_LEAD         0x04  /* VSYNC on clock leading edge */
#define COM10_VS_NEG          0x02  /* VSYNC negative */
#define COM10_HS_NEG          0x01  /* HSYNC negative */
#define REG_HSTART    0x17  /* Horiz start high bits */
#define REG_HSTOP   0x18  /* Horiz stop high bits */
#define REG_VSTART    0x19  /* Vert start high bits */
#define REG_VSTOP   0x1a  /* Vert stop high bits */
#define REG_PSHFT   0x1b  /* Pixel delay after HREF */
#define REG_MIDH    0x1c  /* Manuf. ID high */
#define REG_MIDL    0x1d  /* Manuf. ID low */
#define REG_MVFP    0x1e  /* Mirror / vflip */
#define MVFP_MIRROR         0x20  /* Mirror image */
#define MVFP_FLIP   0x10  /* Vertical flip */

#define REG_AEW           0x24  /* AGC upper limit */
#define REG_AEB           0x25    /* AGC lower limit */
#define REG_VPT           0x26  /* AGC/AEC fast mode op region */
#define REG_HSYST   0x30  /* HSYNC rising edge delay */
#define REG_HSYEN   0x31  /* HSYNC falling edge delay */
#define REG_HREF    0x32  /* HREF pieces */
#define REG_TSLB    0x3a  /* lots of stuff */
#define TSLB_YLAST    0x04  /* UYVY or VYUY - see com13 */
#define REG_COM11   0x3b  /* Control 11 */
#define COM11_NIGHT         0x80  /* NIght mode enable */
#define COM11_NMFR          0x60  /* Two bit NM frame rate */
#define COM11_HZAUTO          0x10  /* Auto detect 50/60 Hz */
#define COM11_50HZ          0x08  /* Manual 50Hz select */
#define COM11_EXP   0x02
#define REG_COM12   0x3c  /* Control 12 */
#define COM12_HREF          0x80  /* HREF always */
#define REG_COM13   0x3d  /* Control 13 */
#define COM13_GAMMA         0x80  /* Gamma enable */
#define COM13_UVSAT         0x40  /* UV saturation auto adjustment */
#define COM13_UVSWAP          0x01  /* V before U - w/TSLB */
#define REG_COM14   0x3e  /* Control 14 */
#define COM14_DCWEN         0x10  /* DCW/PCLK-scale enable */
#define REG_EDGE    0x3f  /* Edge enhancement factor */
#define REG_COM15   0x40  /* Control 15 */
#define COM15_R10F0         0x00  /* Data range 10 to F0 */
#define COM15_R01FE         0x80  /*      01 to FE */
#define COM15_R00FF         0xc0  /*      00 to FF */
#define COM15_RGB565          0x10  /* RGB565 output */
#define COM15_RGB555          0x30  /* RGB555 output */
#define REG_COM16   0x41  /* Control 16 */
#define COM16_AWBGAIN         0x08  /* AWB gain enable */
#define REG_COM17   0x42  /* Control 17 */
#define COM17_AECWIN          0xc0  /* AEC window - must match COM4 */
#define COM17_CBAR          0x08  /* DSP Color bar */
/*
* This matrix defines how the colors are generated, must be
* tweaked to adjust hue and saturation.
*
* Order: v-red, v-green, v-blue, u-red, u-green, u-blue
* They are nine-bit signed quantities, with the sign bit
* stored in0x58.Sign for v-red is bit 0, and up from there.
*/
#define REG_CMATRIX_BASE  0x4f
#define CMATRIX_LEN           6
#define REG_CMATRIX_SIGN  0x58
#define REG_BRIGHT    0x55  /* Brightness */
#define REG_CONTRAS         0x56  /* Contrast control */
#define REG_GFIX    0x69  /* Fix gain control */
#define REG_REG76   0x76  /* OV's name */
#define R76_BLKPCOR         0x80  /* Black pixel correction enable */
#define R76_WHTPCOR         0x40  /* White pixel correction enable */
#define REG_RGB444          0x8c  /* RGB 444 control */
#define R444_ENABLE         0x02  /* Turn on RGB444, overrides 5x5 */
#define R444_RGBX   0x01  /* Empty nibble at end */
#define REG_HAECC1    0x9f  /* Hist AEC/AGC control 1 */
#define REG_HAECC2    0xa0  /* Hist AEC/AGC control 2 */
#define REG_BD50MAX         0xa5  /* 50hz banding step limit */
#define REG_HAECC3    0xa6  /* Hist AEC/AGC control 3 */
#define REG_HAECC4    0xa7  /* Hist AEC/AGC control 4 */
#define REG_HAECC5    0xa8  /* Hist AEC/AGC control 5 */
#define REG_HAECC6    0xa9  /* Hist AEC/AGC control 6 */
#define REG_HAECC7    0xaa  /* Hist AEC/AGC control 7 */
#define REG_BD60MAX         0xab  /* 60hz banding step limit */


#define MTX1            0x4f  /* Matrix Coefficient 1 */
#define MTX2            0x50  /* Matrix Coefficient 2 */
#define MTX3            0x51  /* Matrix Coefficient 3 */
#define MTX4            0x52  /* Matrix Coefficient 4 */
#define MTX5            0x53  /* Matrix Coefficient 5 */
#define MTX6            0x54  /* Matrix Coefficient 6 */

#define MTXS            0x58  /* Matrix Coefficient Sign */
#define AWBC7           0x59  /* AWB Control 7 */
#define AWBC8           0x5a  /* AWB Control 8 */
#define AWBC9           0x5b  /* AWB Control 9 */
#define AWBC10            0x5c  /* AWB Control 10 */
#define AWBC11            0x5d  /* AWB Control 11 */
#define AWBC12            0x5e  /* AWB Control 12 */
#define REG_GFI           0x69  /* Fix gain control */
#define GGAIN           0x6a  /* G Channel AWB Gain */
#define DBLV            0x6b  
#define AWBCTR3           0x6c  /* AWB Control 3 */
#define AWBCTR2           0x6d  /* AWB Control 2 */
#define AWBCTR1           0x6e  /* AWB Control 1 */
#define AWBCTR0           0x6f  /* AWB Control 0 */

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(115200);
  
//writeReg( REG_COM7, COM7_RESET );
  cameraDefaultConfig();

  writeReg(REG_CLKRC, 0x0f);
  writeReg(REG_COM3, 4); // REG_COM3 enable scaling
  writeReg( REG_COM7, COM7_FMT_QVGA);
//  
//  writeReg(0x0c, 4); // dcw scaling
//
//  writeReg( REG_COM14, 0x1f ); //enable scaling pclk and set pclk divider // scales too much clk
 
  writeReg( 0x72, 0x11 );
  writeReg( 0x73, 0xf1 );

  writeReg( REG_HSTART, 0x16 );
  writeReg( REG_HSTOP, 0x04 );
  writeReg( REG_HREF, 0xa4 );
  writeReg( REG_VSTART, 0x02 );
  writeReg( REG_VSTOP, 0x7a );
  writeReg( REG_VREF, 0x0a );
  writeReg(0x13, 0  );
  writeReg(0x3d, 0);
  
}

byte x = 0;


void loop() {
  Wire.beginTransmission(0x21); // transmit to device #8
  Wire.write(0x11);        // sends five bytes
  Wire.requestFrom(0x21, 1);
  Wire.endTransmission();    // stop transmitting

  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
  }
  
  x++;
  delay(500);
}

void writeReg(uint8_t reg, uint8_t val){
  Wire.beginTransmission(0x21); // transmit to device #8
  Wire.write(reg);        // sends five bytes
  Wire.write(val);
  Wire.endTransmission();    // stop transmitting
  delay(1);
}

void cameraDefaultConfig(void){
writeReg( REG_COM7, COM7_RESET );
writeReg( REG_TSLB, 0x04 ); /* OV */
writeReg( REG_COM7, 0 );  /* VGA */
  /*
  * Set the hardware window.  These values from OV don't entirely
  * make sense - hstop is less than hstart.  But they work...
  */
writeReg( REG_HSTART, 0x13 );
writeReg( REG_HSTOP, 0x01 );
writeReg( REG_HREF, 0xb6 );
writeReg( REG_VSTART, 0x02 );
writeReg( REG_VSTOP, 0x7a );
writeReg( REG_VREF, 0x0a );

writeReg( REG_COM3, 0 );
writeReg( REG_COM14, 0 );
  /* Mystery scaling numbers */
writeReg( 0x70, 0x3a );
writeReg( 0x71, 0x35 );
writeReg( 0x72, 0x11 );
writeReg( 0x73, 0xf0 );
writeReg( 0xa2,/* 0x02 changed to 1*/1 );
writeReg( REG_COM10, 0x0 );
  /* Gamma curve values */
writeReg( 0x7a, 0x20 );
writeReg( 0x7b, 0x10 );
writeReg( 0x7c, 0x1e );
writeReg( 0x7d, 0x35 );
writeReg( 0x7e, 0x5a );
writeReg( 0x7f, 0x69 );
writeReg( 0x80, 0x76 );
writeReg( 0x81, 0x80 );
writeReg( 0x82, 0x88 );
writeReg( 0x83, 0x8f );
writeReg( 0x84, 0x96 );
writeReg( 0x85, 0xa3 );
writeReg( 0x86, 0xaf );
writeReg( 0x87, 0xc4 );
writeReg( 0x88, 0xd7 );
writeReg( 0x89, 0xe8 );
  /* AGC and AEC parameters.  Note we start by disabling those features,
  then turn them only after tweaking the values. */
writeReg( REG_COM8, COM8_FASTAEC | COM8_AECSTEP );
writeReg( REG_GAIN, 0 );
writeReg( REG_AECH, 0 );
writeReg( REG_COM4, 0x40 ); /* magic reserved bit */
writeReg( REG_COM9, 0x18 ); /* 4x gain + magic rsvd bit */
writeReg( REG_BD50MAX, 0x05 );
writeReg( REG_BD60MAX, 0x07 );
writeReg( REG_AEW, 0x95 );
writeReg( REG_AEB, 0x33 );
writeReg( REG_VPT, 0xe3 );
writeReg( REG_HAECC1, 0x78 );
writeReg( REG_HAECC2, 0x68 );
writeReg( 0xa1, 0x03 ); /* magic */
writeReg( REG_HAECC3, 0xd8 );
writeReg( REG_HAECC4, 0xd8 );
writeReg( REG_HAECC5, 0xf0 );
writeReg( REG_HAECC6, 0x90 );
writeReg( REG_HAECC7, 0x94 );
writeReg( REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC );
writeReg( 0x30, 0 );
writeReg( 0x31, 0 );//disable some delays
  /* Almost all of these are magic "reserved" values.  */
writeReg( REG_COM5, 0x61 );
writeReg( REG_COM6, 0x4b );
writeReg( 0x16, 0x02 );
writeReg( REG_MVFP, 0x07 );
writeReg( 0x21, 0x02 );
writeReg( 0x22, 0x91 );
writeReg( 0x29, 0x07 );
writeReg( 0x33, 0x0b );
writeReg( 0x35, 0x0b );
writeReg( 0x37, 0x1d );
writeReg( 0x38, 0x71 );
writeReg( 0x39, 0x2a );
writeReg( REG_COM12, 0x78 );
writeReg( 0x4d, 0x40 );
writeReg( 0x4e, 0x20 );
writeReg( REG_GFIX, 0 );
  /*{0x6b, 0x4a);*/
writeReg(0x74, 0x10 );
writeReg( 0x8d, 0x4f );
writeReg( 0x8e, 0 );
writeReg( 0x8f, 0 );
writeReg( 0x90, 0 );
writeReg( 0x91, 0 );
writeReg( 0x96, 0 );
writeReg( 0x9a, 0 );
writeReg( 0xb0, 0x84 );
writeReg( 0xb1, 0x0c );
writeReg( 0xb2, 0x0e );
writeReg( 0xb3, 0x82 );
writeReg( 0xb8, 0x0a );

  /* More reserved magic, some of which tweaks white balance */
writeReg( 0x43, 0x0a );
writeReg( 0x44, 0xf0 );
writeReg( 0x45, 0x34 );
writeReg( 0x46, 0x58 );
writeReg( 0x47, 0x28 );
writeReg( 0x48, 0x3a );
writeReg( 0x59, 0x88 );
writeReg( 0x5a, 0x88 );
writeReg( 0x5b, 0x44 );
writeReg( 0x5c, 0x67 );
writeReg( 0x5d, 0x49 );
writeReg( 0x5e, 0x0e );
writeReg( 0x6c, 0x0a );
writeReg( 0x6d, 0x55 );
writeReg( 0x6e, 0x11 );
writeReg( 0x6f, 0x9e ); /* it was 0x9F "9e for advance AWB" */
writeReg( 0x6a, 0x40 );
writeReg( REG_BLUE, 0x40 );
writeReg( REG_RED, 0x60 );
writeReg( REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_AGC | COM8_AEC | COM8_AWB );

  /* Matrix coefficients */
writeReg( 0x4f, 0x80 );
writeReg( 0x50, 0x80 );
writeReg( 0x51, 0 );  writeReg( 0x52, 0x22 );
writeReg( 0x53, 0x5e );
writeReg( 0x54, 0x80 );
writeReg( 0x58, 0x9e );

writeReg( REG_COM16, COM16_AWBGAIN );
writeReg( REG_EDGE, 0 );
writeReg( 0x75, 0x05 );
writeReg( REG_REG76, 0xe1 );
writeReg( 0x4c, 0 );   writeReg( 0x77, 0x01 );
writeReg( REG_COM13, /*0xc3*/0x48 );
writeReg( 0x4b, 0x09 );
writeReg( 0xc9, 0x60 );   /*{REG_COM16, 0x38);*/
writeReg( 0x56, 0x40 );

writeReg( 0x34, 0x11 );
writeReg( REG_COM11, COM11_EXP | COM11_HZAUTO );
writeReg( 0xa4, 0x82/*Was 0x88*/ );
writeReg( 0x96, 0 );
writeReg( 0x97, 0x30 );
writeReg( 0x98, 0x20 );
writeReg( 0x99, 0x30 );
writeReg( 0x9a, 0x84 );
writeReg( 0x9b, 0x29 );
writeReg( 0x9c, 0x03 );
writeReg( 0x9d, 0x4c );
writeReg( 0x9e, 0x3f );
writeReg( 0x78, 0x04 );

  /* Extra-weird stuff.  Some sort of multiplexor register */
writeReg( 0x79, 0x01 );
writeReg( 0xc8, 0xf0 );
writeReg( 0x79, 0x0f );
writeReg( 0xc8, 0x00 );
writeReg( 0x79, 0x10 );
writeReg( 0xc8, 0x7e );
writeReg( 0x79, 0x0a );
writeReg( 0xc8, 0x80 );
writeReg( 0x79, 0x0b );
writeReg( 0xc8, 0x01 );
writeReg( 0x79, 0x0c );
writeReg( 0xc8, 0x0f );
writeReg( 0x79, 0x0d );
writeReg( 0xc8, 0x20 );
writeReg( 0x79, 0x09 );
writeReg( 0xc8, 0x80 );
writeReg( 0x79, 0x02 );
writeReg( 0xc8, 0xc0 );
writeReg( 0x79, 0x03 );
writeReg( 0xc8, 0x40 );
writeReg( 0x79, 0x05 );
writeReg( 0xc8, 0x30 );
writeReg( 0x79, 0x26 );
writeReg( 0xff, 0xff ); /* END MARKER */
}
