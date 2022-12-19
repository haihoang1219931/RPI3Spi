 // Original code - FREE WING
 //  http://www.neko.ne.jp/~freewing/
 //  https://github.com/FREEWING-JP/RaspberryPi_KeDei_35_lcd_v50.git
 //  Mon Dec 19, 2022
 // Modified by Hai Nguyen Hoang, haihoangsoftware@gmail.com
 //
 // RPI3 Linux spidev c++ version
 // g++ -o main_spi_dev main_spi_dev.cpp -lwiringPi -lwiringPiDev
 // sudo ./main_spi_dev
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>

// wiringPi pinout
#define SPI_CS  10
#define SPI_RST 6
#define SPI_RS  5
#define SPI_LED 0

#define LCD_DEVICE "/dev/spidev0.0"
#define LCD_CS 0
#define TOUCH_CS 1
#define LCD_WIDTH  480
#define LCD_HEIGHT 320

#define uint8_t unsigned char
#define uint16_t unsigned int
#define uint32_t unsigned long

static const uint32_t mode = 0;
static const uint8_t bits = 8;
static const uint32_t speed = 1000000;

uint8_t lcd_rotations[4] = {
    0b11101010,	//   0
    0b01001010,	//  90
    0b00101010,	// 180
    0b10001010	// 270
};

uint8_t color;
uint8_t lcd_rotation;
uint16_t lcd_h;
uint16_t lcd_w;
int spih;

uint16_t colors[16] = {
    0b0000000000000000,				/* BLACK	000000 */
    0b0000000000010000,				/* NAVY		000080 */
    0b0000000000011111,				/* BLUE		0000ff */
    0b0000010011000000,				/* GREEN	009900 */
    0b0000010011010011,				/* TEAL		009999 */
    0b0000011111100000,				/* LIME		00ff00 */
    0b0000011111111111,				/* AQUA		00ffff */
    0b1000000000000000,				/* MAROON	800000 */
    0b1000000000010000,				/* PURPLE	800080 */
    0b1001110011000000,				/* OLIVE	999900 */
    0b1000010000010000,				/* GRAY		808080 */
    0b1100111001111001,				/* SILVER	cccccc */
    0b1111100000000000,				/* RED		ff0000 */
    0b1111100000011111,				/* FUCHSIA	ff00ff */
    0b1111111111100000,				/* YELLOW	ffff00 */
    0b1111111111111111				/* WHITE	ffffff */
};

void delayms(int ms);
int lcd_open(void);
int spi_transmit(int devsel, uint8_t *tx, uint8_t *rx, unsigned int len);
void lcd_rst(void);
void lcd_cmd(uint8_t cmd);
void lcd_data(uint16_t data);
int lcd_close(void);
void lcd_color(uint16_t col);
uint16_t colorRGB(uint8_t r, uint8_t g, uint8_t b);
void lcd_colorRGB(uint8_t r, uint8_t g, uint8_t b);
void lcd_setrotation(uint8_t m);
void lcd_init(void);
void lcd_setframe(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void lcd_fillframe(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t col);
void lcd_fill(uint16_t col);
void lcd_fillframeRGB(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b);
void lcd_fillRGB(uint8_t r, uint8_t g, uint8_t b);
void lcd_img(char *fname, uint16_t x, uint16_t y);
void loop();
void lcd_info();

int main(int argc,char *argv[]) {
    printf("New version updated\r\n");
    lcd_open();

    lcd_init();

    lcd_fill(255);
    sleep(1);
    // 24bit Bitmap only
    printf("loop\r\n");
    for(int i=0; i<16; i++)
        loop();
    lcd_close();
    return 0;
}

void delayms(int ms) {

    struct timespec req;
    req.tv_sec  = 0;
    req.tv_nsec = ms * 1000000L;

    nanosleep(&req, NULL);
}

int lcd_open(void) {
    int ret;

    spih = open(LCD_DEVICE, O_RDWR);
    if(spih<0) return -1;

    ret = ioctl(spih, SPI_IOC_WR_MODE, &mode);
    if (ret<0) return -1;

    ret = ioctl(spih, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret<0) return -1;

    ret = ioctl(spih, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret<0) return -1;

//    ret = ioctl(spih, SPI_IOC_RD_MODE, &mode);
//    if (ret<0) return -1;

//    ret = ioctl(spih, SPI_IOC_RD_BITS_PER_WORD, &bits);
//    if (ret<0) return -1;

//    ret = ioctl(spih, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
//    if (ret<0) return -1;

    wiringPiSetup();
    pinMode(SPI_CS,OUTPUT);
    pinMode(SPI_RS,OUTPUT);
    pinMode(SPI_RST,OUTPUT);
    return 0;
}

int spi_transmit(int devsel, uint8_t *tx, uint8_t *rx, unsigned int len) {

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.delay_usecs = 0;
    tr.speed_hz = speed;
    tr.bits_per_word = bits;
    tr.cs_change = 0;
    return ioctl(spih, SPI_IOC_MESSAGE(1), &tr);
}

void lcd_rst(void) {
    digitalWrite(SPI_CS,1);
    delayms(5);
    digitalWrite(SPI_RST,1);
    delayms(5);
    digitalWrite(SPI_RST,0);
    delayms(5);
    digitalWrite(SPI_RST,1);
    delayms(5);
}

void lcd_cmd(uint8_t cmd) {
    uint8_t b1[1];

    b1[0] = cmd;

    digitalWrite(SPI_RS,LOW);
    digitalWrite(SPI_CS,LOW);
    spi_transmit(LCD_CS, &b1[0], nullptr, sizeof(b1));
    digitalWrite(SPI_CS,HIGH);
}

void lcd_data(uint16_t data) {
    uint8_t b1[2];

    b1[0] = static_cast<uint8_t>(data>>8);
    b1[1] = static_cast<uint8_t>(data);
    digitalWrite(SPI_RS,HIGH);
    digitalWrite(SPI_CS,LOW);
    spi_transmit(LCD_CS, &b1[0], nullptr, sizeof(b1));
    digitalWrite(SPI_CS,HIGH);
}

int lcd_close(void) {
    lcd_cmd( 0x2a );
    lcd_data( 0x0 );
    lcd_data( 0xa );
    lcd_data( 0x0 );
    lcd_data( 0x6e );
    lcd_cmd( 0x2b );
    lcd_data( 0x0 );
    lcd_data( 0x4a );
    lcd_data( 0x0 );
    lcd_data( 0x49 );
    lcd_cmd( 0x2c );
    close(spih);

    return 0;
}

void lcd_color(uint16_t col) {
    lcd_data(col);
}

uint16_t colorRGB(uint8_t r, uint8_t g, uint8_t b) {

    uint16_t col = ((r<<8) & 0xF800) | ((g<<3) & 0x07E0) | ((b>>3) & 0x001F);
//	printf("%02x %02x %02x %04x\n", r, g, b, col);

    return col;
}

// 18bit color mode
void lcd_colorRGB(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t col = ((r<<8) & 0xF800) | ((g<<3) & 0x07E0) | ((b>>3) & 0x001F);

    lcd_data(col);
}

void lcd_setrotation(uint8_t m) {
    lcd_cmd(0x36); lcd_data(lcd_rotations[m]);
    if (m&1) {
        lcd_h = LCD_WIDTH;
        lcd_w = LCD_HEIGHT;
    } else {
        lcd_h = LCD_HEIGHT;
        lcd_w = LCD_WIDTH;
    }
}

void lcd_init(void) {
    //reset display
    lcd_rst();

    // Read Display MADCTL
    lcd_cmd( 0xb );
        lcd_data( 0x0 );
        lcd_data( 0x0 );

    // Sleep OUT
    lcd_cmd( 0x11 );

    // Interface Pixel Format
    lcd_cmd( 0x3a );
        lcd_data( 0x55 );

    // Memory Access Control
    lcd_cmd( 0x36 );
        lcd_data( 0x28 );

    // Power Control 3 (For Normal Mode)
    lcd_cmd( 0xc2 );
        lcd_data( 0x44 );

    // VCOM Control
    lcd_cmd( 0xc5 );
        lcd_data( 0x0 );
        lcd_data( 0x0 );
        lcd_data( 0x0 );
        lcd_data( 0x0 );

    // PGAMCTRL(Positive Gamma Control)
    lcd_cmd( 0xe0 );
        lcd_data( 0xf );
        lcd_data( 0x1f );
        lcd_data( 0x1c );
        lcd_data( 0xc );
        lcd_data( 0xf );
        lcd_data( 0x8 );
        lcd_data( 0x48 );
        lcd_data( 0x98 );
        lcd_data( 0x37 );
        lcd_data( 0xa );
        lcd_data( 0x13 );
        lcd_data( 0x4 );
        lcd_data( 0x11 );
        lcd_data( 0xd );
        lcd_data( 0x0 );

    // NGAMCTRL (Negative Gamma Correction)
    lcd_cmd( 0xe1 );
        lcd_data( 0xf );
        lcd_data( 0x32 );
        lcd_data( 0x2e );
        lcd_data( 0xb );
        lcd_data( 0xd );
        lcd_data( 0x5 );
        lcd_data( 0x47 );
        lcd_data( 0x75 );
        lcd_data( 0x37 );
        lcd_data( 0x6 );
        lcd_data( 0x10 );
        lcd_data( 0x3 );
        lcd_data( 0x24 );
        lcd_data( 0x20 );
        lcd_data( 0x0 );

    // Digital Gamma Control 1
    lcd_cmd( 0xe2 );
        lcd_data( 0xf );
        lcd_data( 0x32 );
        lcd_data( 0x2e );
        lcd_data( 0xb );
        lcd_data( 0xd );
        lcd_data( 0x5 );
        lcd_data( 0x47 );
        lcd_data( 0x75 );
        lcd_data( 0x37 );
        lcd_data( 0x6 );
        lcd_data( 0x10 );
        lcd_data( 0x3 );
        lcd_data( 0x24 );
        lcd_data( 0x20 );
        lcd_data( 0x0 );

    // Sleep OUT
    lcd_cmd( 0x11 );

    // Display ON
    lcd_cmd( 0x29 );

    // Read display info
    lcd_info();
}

void lcd_setframe(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    lcd_cmd(0x2A);
    lcd_data(x>>8);
    lcd_data(x&0xFF);
    lcd_data(((w+x)-1)>>8);
    lcd_data(((w+x)-1)&0xFF);
    lcd_cmd(0x2B);
    lcd_data(y>>8);
    lcd_data(y&0xFF);
    lcd_data(((h+y)-1)>>8);
    lcd_data(((h+y)-1)&0xFF);
    lcd_cmd(0x2C);
}

//lcd_fillframe
//fills an area of the screen with a single color.
void lcd_fillframe(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t col) {
    int span=h*w;
    lcd_setframe(x,y,w,h);
    int q;
    for(q=0;q<span;q++) { lcd_color(col); }
}

void lcd_fill(uint16_t col) {
    lcd_fillframe(0, 0, lcd_w, lcd_h, col);
}

void lcd_fillframeRGB(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint8_t g, uint8_t b) {
    int span=h*w;
    lcd_setframe(x,y,w,h);
    int q;
    for(q=0;q<span;q++) { lcd_colorRGB(r, g, b); }
}

void lcd_fillRGB(uint8_t r, uint8_t g, uint8_t b) {
    lcd_fillframeRGB(0, 0, lcd_w, lcd_h, r, g, b);
}

void lcd_img(char *fname, uint16_t x, uint16_t y) {

    uint8_t buf[54];
    uint16_t p, c;
    uint32_t isize, ioffset, iwidth, iheight, ibpp, fpos, rowbytes;

    FILE *f = fopen(fname, "rb");
    if (f != NULL) {
        fseek(f, 0L, SEEK_SET);
        fread(buf, 30, 1, f);

        isize =	 buf[2] + (buf[3]<<8) + (buf[4]<<16) + (buf[5]<<24);
        ioffset = buf[10] + (buf[11]<<8) + (buf[12]<<16) + (buf[13]<<24);
        iwidth =	buf[18] + (buf[19]<<8) + (buf[20]<<16) + (buf[21]<<24);
        iheight = buf[22] + (buf[23]<<8) + (buf[24]<<16) + (buf[25]<<24);
        ibpp =		buf[28] + (buf[29]<<8);

        printf("\n\n");
        printf("File Size: %u\nOffset: %u\nWidth: %u\nHeight: %u\nBPP: %u\n\n",isize,ioffset,iwidth,iheight,ibpp);

        lcd_setframe(x,y,iwidth,iheight); //set the active frame...

        // Wrong calculationg bytes in row for bitmap. #1
        rowbytes = (iwidth*3);
        uint8_t d = (iwidth*3)%4;
        if (d>0) { rowbytes += 4-d; }

        for (p=iheight-1;p>0;p--) {
            // p = relative page address (y)
            //fpos = ioffset+(p*rowbytes);
            //fseek(f, fpos, SEEK_SET);

            for (c=0;c<iwidth;c++) {
                // c = relative column address (x)
                //fread(buf, 3, 1, f);

                // B buf[0]
                // G buf[1]
                // R buf[2]
                // 18bit color mode
                buf[2] = 0;
                buf[1] = y;
                buf[0] = 0;
                lcd_colorRGB(buf[2], buf[1], buf[0]);
//                delayms(10);
                //printf(".");
                //fflush(stdout);
            }
        }

        fclose(f);
    }
}

void loop() {

    //Update rotation
    lcd_setrotation(lcd_rotation);

    //Fill entire screen with new color
    lcd_fillframe(0,0,lcd_w,lcd_h,colors[color]);

    //Make a color+1 box, 5 pixels from the top-left corner, 20 pixels high, 95 (100-5) pixels from right border.
    lcd_fillframe(5,5,lcd_w-100,20,colors[(color+1) & 0xF]);

    //increment color
    color++;
    //if color is overflowed, reset to 0
    if (color==16) {color=0;}

    //increment rotation
    lcd_rotation++;

    //if rotation is overflowed, reset to 0
    if (lcd_rotation==4) lcd_rotation=0;

    delayms(500);
}

void lcd_info()
{
    uint8_t tx[4];
    uint8_t rx[4];
    memset(tx,0x00,sizeof(tx));
    memset(rx,0x00,sizeof(rx));
//    lcd_cmd(0xD3);
    tx[0] = 0x04;
    digitalWrite(SPI_RS,LOW);
    digitalWrite(SPI_CS,LOW);
    int res = spi_transmit(LCD_CS, tx, rx, 4);
    digitalWrite(SPI_CS,HIGH);
    printf("Info return %d: [%02X %02X %02X %02X]\r\n",
           res,
           rx[0],rx[1],rx[2],rx[3]);
    lcd_cmd(0xD3);
    delayms(1);
    for(int i=0; i< 4; i++)
    {
        digitalWrite(SPI_RS,HIGH);
        digitalWrite(SPI_CS,LOW);
        res = spi_transmit(LCD_CS,nullptr,rx,sizeof(rx));
        digitalWrite(SPI_CS,HIGH);
        printf("Info return %d: [%02X %02X %02X %02X] \r\n",
               res,
               rx[0],rx[1],rx[2],rx[3]);
        delayms(1);
    }
}
