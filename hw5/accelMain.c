/* 
 * File:   disp.c
 * Author: Matt
 *
 * Created on April 13, 2015, 12:08 AM
 */

#include<xc.h> // processor SFR definitions
#include<sys/attribs.h> // __ISR macro

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "i2c_master_int.h"
#include "i2c_display.h"
#include "accel.h"

#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // not boot write protect
#pragma config CP = OFF // no code protect
#pragma config FNOSC = FRCPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = ON // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_20 // multiply clock after FPLLIDIV
#pragma config UPLLIDIV = DIV_2 // divide clock after FPLLMUL
#pragma config UPLLEN = ON // USB clock on
#pragma config FPLLODIV = DIV_2 // divide clock by 2 to output on pin
#pragma config USERID = 0 // some 16bit userid
#pragma config PMDL1WAY = ON // not multiple reconfiguration, check this
#pragma config IOL1WAY = ON // not multimple reconfiguration, check this
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // controlled by USB module


static const char ASCII[96][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00} // 20  (space)
    ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
    ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
    ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
    ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
    ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
    ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
    ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
    ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
    ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
    ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
    ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
    ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
    ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
    ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
    ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
    ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
    ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
    ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
    ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
    ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
    ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
    ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
    ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
    ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
    ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
    ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
    ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
    ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
    ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
    ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
    ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
    ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
    ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
    ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
    ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
    ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
    ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
    ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
    ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
    ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
    ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
    ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
    ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
    ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
    ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
    ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
    ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
    ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
    ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
    ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
    ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
    ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
    ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
    ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
    ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
    ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
    ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
    ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
    ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
    ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
    ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
    ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
    ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
    ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
    ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
    ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
    ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
    ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
    ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
    ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
    ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
    ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
    ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
    ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
    ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
    ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
    ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
    ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
    ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
    ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
    ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
    ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
    ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
    ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
    ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
    ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
    ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
    ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
    ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
    ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
    ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
    ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
    ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
    ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
    ,{0x00, 0x06, 0x09, 0x09, 0x06} // 7f ?
}; // end char ASCII[96][5]

void dispChar(char msg[], int row, int col);
int getBit (int ltr, int jj, int kk);
void screenReset();


int main(int argc, char** argv) {

__builtin_disable_interrupts();
__builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

BMXCONbits.BMXWSDRM = 0x0;
INTCONbits.MVEC = 0x1;
DDPCONbits.JTAGEN = 0;
__builtin_enable_interrupts();

    ANSELBbits.ANSB3 = 0; // B3 for digital
    ANSELBbits.ANSB14 = 0; //B14 is digital
    DDPCONbits.JTAGEN = 0;
    OSCCONbits.SOSCEN = 0;
    ANSELBbits.ANSB13 = 0; // 0 for digital, 1 for analog
    ANSELBbits.ANSB15 = 0; // 0 for digital, 1 for analog
    TRISBbits.TRISB13 = 1; // make pin 13 an input
    TRISBbits.TRISB7 = 0;// make pin 7 an output
    RPB15Rbits.RPB15R = 0b0101; // set B15 to OC1
    T2CONbits.TCKPS = 2; // Timer2 prescaler N=4 (1:4)
    PR2 = 19999; // period = (PR2+1) * N * 12.5 ns = 1000 us, 1 kHz
    TMR2 = 0; // initial TMR2 count is 0
    OC1RS = 10000;
    OC1CONbits.OCTSEL = 0; // use Timer2 for OC1
    OC1CONbits.OCM = 0b110; // PWM mode with fault pin disabled
    T2CONbits.ON = 1; // turn on Timer2
    OC1CONbits.ON = 1; // turn on OC1
    ANSELAbits.ANSA0 = 1;
    AD1CON3bits.ADCS = 3;
    AD1CHSbits.CH0SA = 0;
    AD1CON1bits.ADON = 1;

    //Reset Screen

    ANSELBbits.ANSB2 = 0; //Reset pin
    TRISBbits.TRISB2 = 0; //Set Pin as Output
    LATBbits.LATB2 = 0; //Turn off B2

    //Turn LED back on
    _CP0_SET_COUNT(0);
    int elapsed = 0;
    while (elapsed<2000000){elapsed = _CP0_GET_COUNT();}
    LATBbits.LATB2 = 1;
    _CP0_SET_COUNT(0);
    elapsed = 0;
    while (elapsed<2000000){elapsed = _CP0_GET_COUNT();}

    //Initialize devices
    acc_setup();
    display_init();
    acc_write_register(CTRL2, 0x0); //sets accelerometer defaul tol to be +-2g
    display_clear();

    //Initialize Accelerometer Data Holders
    short accels[3]; // 0 = X, 1 = Y, 2 = Z
    short mags[3];
    short temp;

    int yMid = 64/2; //Set middle row
    int xMid = 128/2; //Set middle Column
    //Initialize bounds
    int xlBound = 0;
    int xhBound = 0;
    int ylBound = 0;
    int yhBound = 0;

    //Pixel Counters
    int xPix = 0;
    int yPix = 0;

    while(1) {

        screenReset(); //Toggle LED Power for positioning
        
        //Read Accelerometer
        acc_read_register(OUT_X_L_A, (unsigned char *) accels,6);
        acc_read_register(OUT_X_L_M, (unsigned char *) mags,6);
        acc_read_register(TEMP_OUT_L, (unsigned char *) &temp, 2);

        char str[]=""; //Initialize Str to debug

        //Set length of pixels
        int xVal = (accels[0]*60)/16000;
        int yVal = (accels[1]*60)/16000;

        //sprintf(str, "X:%d Y:%d    reading: %d", xVal, yVal, accels[0]);

        //dispChar(str,63-28,127-32);

        //Set Bounds
        if (xVal < 0) { //xMid = 128/2
            xhBound = xMid-xVal;
            xlBound = xMid;
        }

        else {
            xlBound = xMid-xVal;
            xhBound = xMid;
        }

        if (yVal < 0) { //yMid = 64/2
            yhBound = yMid-yVal;
            ylBound = yMid;
        }
        else {
            ylBound = yMid-yVal;
            yhBound = yMid;
        }


        //Set image
        for (xPix = 0; xPix < 128; xPix++) {
            if (xPix > xlBound && xPix < xhBound){
                display_pixel_set(yMid,xPix,1);
                display_pixel_set(yMid+1,xPix,1);
                display_pixel_set(yMid-1,xPix,1);
            }
        }
        for (yPix = 0; yPix < 64; yPix++) {
            if (yPix > ylBound && yPix < yhBound){
                display_pixel_set(yPix,xMid,1);
                display_pixel_set(yPix,xMid-1,1);
                display_pixel_set(yPix,xMid+1,1);
            }
        }
        display_draw();

        //Chill
        _CP0_SET_COUNT(0);
        int elapsed = 0;
        while (elapsed<2000000){elapsed = _CP0_GET_COUNT();}

        //Start it up again
        display_clear();
    
    }
    return (EXIT_SUCCESS);
}

void dispChar(char msg[], int row, int col){
    int len = strlen(msg);
    int ltr;

    int ii=0;
    int jj=0;
    int kk=0;
    int numlet = 0;
    int crow = 0;
    
    for (ii=0; ii<len; ii++) { //for each letter in string
        ltr = (int) msg[ii] - 0x20; //set character ref
        for (jj=0; jj<5; jj++){ //cycle through columns of ascii table
            for (kk=0; kk<8; kk++) { //cycle through rows
                int bit = getBit(ltr,jj,kk);
                if(bit){
                    display_pixel_set(row-kk-crow*8,col-jj-5*numlet,1);
                }
                else {
                    display_pixel_set(row-kk-crow*8,col-jj-5*numlet,0);
                }
            }
        }
        numlet++;
        if (-numlet*5+col< 5 || msg[ii] == "\n") {
            numlet = 0;
            crow++;
        }
    }
    display_draw();
}

int getBit(int ltr, int jj, int kk) {
    return (ASCII[ltr][jj] & (1 << (kk-1))) >> (kk-1);
}

void screenReset() {
    LATBbits.LATB2 = 0;
    LATBbits.LATB2 = 1;
}