// Original code - Bartosz
//  https://github.com/bkosciow/gfxlcd.git
//  Mon Dec 19, 2022
// Modified by Hai Nguyen Hoang, haihoangsoftware@gmail.com
//
// RPI3 Linux spidev version
// Linux spidev c++ version
// g++ -o main_touch_screen main_touch_screen.cpp -lwiringPi -lwiringPiDev
// sudo ./main_touch_screen
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
#include <iostream>
#include <vector>

// wiringPi pinout
#define TOUCH_IRQ 0
#define TOUCH_CS 11

#define TOUCH_DEVICE "/dev/spidev0.0"

#define LCD_WIDTH  480
#define LCD_HEIGHT 320

#define uint8_t unsigned char
#define uint16_t unsigned int
#define uint32_t unsigned long

static const uint32_t mode = 0;
static const uint8_t bits = 8;
static const uint32_t speed = 1000000;


static const float tc_correct_x = 540.0f;
static const float tc_correct_y = 50.0f;
static const float tc_correct_ratio_x = 0.94f;
static const float tc_correct_ratio_y = 1.26f;

static int spih;

void delayms(int ms);
void convert_xy(int inputX, int inputY, int& outputX, int& outputY);
bool in_bound(int posX, int posY);
int lcd_open(void);
int spi_transmit(int devsel, uint8_t *tx, uint8_t *rx, unsigned int len);
int lcd_close(void);

int main()
{
    lcd_open();
    while(1)
    {
        int touched = digitalRead(TOUCH_IRQ);
        if(touched == 0)
        {
            std::vector<int>bufferX = {};
            std::vector<int>bufferY = {};
            int fuse = 40;
            int tc_rx,tc_ry,tc_rz;
            int pos_x, pos_y;
            uint8_t tx[4];
            uint8_t recv[4];
            while (bufferX.size() < 20 && fuse > 0)
            {
                digitalWrite(TOUCH_CS,LOW);


                tx[0] = 0x80 | 0x08 | 0x30;
                spi_transmit(TOUCH_CS,tx,nullptr,1);
                read(spih,recv,1);
                tc_rz = recv[0] & 0x7f;

                tx[0] = (0x80 | 0x08 | 0x40);
                spi_transmit(TOUCH_CS,tx,nullptr,1);
                read(spih,recv,1);
                tc_rz += (255-recv[0] & 0x7f);

                tx[0] = (0x80 | 0x10);
                spi_transmit(TOUCH_CS,tx,nullptr,1);
                read(spih,recv,2);
                tc_rx = 1023-((recv[0] << 2)|(recv[1] >> 6));

                tx[0] = (0x80 | 0x50);
                spi_transmit(TOUCH_CS,tx,nullptr,1);
                read(spih,recv,2);
                tc_ry = ((recv[0] << 2)|(recv[1] >> 6));

                digitalWrite(TOUCH_CS,HIGH);

                if (tc_rz > 10)
                {
                    convert_xy(tc_rx,tc_ry,pos_x,pos_y);
                    if(in_bound(pos_x, pos_y))
                    {
                        bufferX.push_back(pos_x);
                        bufferY.push_back(pos_y);
                    }
                }
                fuse --;
            }

            // calculate average
            pos_x = 0;
            pos_y = 0;
            for(int i=0 ;i < bufferX.size(); i++)
            {
                pos_x += bufferX[i];
                pos_y += bufferY[i];
            }
            if(bufferX.size() > 0)
            {
                pos_x /= bufferX.size();
                pos_y /= bufferY.size();
                printf("pos[%d,%d] / [%dx%d]\r\n",
                       pos_x,pos_y,
                       LCD_WIDTH,LCD_HEIGHT);
            }
            else
                printf("None\r\n");
        }
        delayms(16);
    }
    return 0;
}

void delayms(int ms) {

    struct timespec req;
    req.tv_sec  = 0;
    req.tv_nsec = ms * 1000000L;

    nanosleep(&req, NULL);
}

void convert_xy(int inputX, int inputY, int& outputX, int& outputY)
{
    outputX = (int)(((float)inputX - tc_correct_x)/tc_correct_ratio_x);
    outputY = LCD_HEIGHT - (int)(((float)inputY - tc_correct_y)/tc_correct_ratio_y);
}

bool in_bound(int posX, int posY)
{
    return 0 <= posX && posX <= LCD_WIDTH &&
            0 <= posY && posY <= LCD_HEIGHT;
}
int lcd_open(void) {
    int ret;

    spih = open(TOUCH_DEVICE, O_RDWR);
    if(spih<0) return -1;

    ret = ioctl(spih, SPI_IOC_WR_MODE, &mode);
    if (ret<0) return -1;

    ret = ioctl(spih, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret<0) return -1;

    ret = ioctl(spih, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret<0) return -1;

    wiringPiSetup();
    pinMode(TOUCH_IRQ,INPUT);
    pinMode(TOUCH_CS,OUTPUT);
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

int lcd_close(void) {
    close(spih);

    return 0;
}
