/*
 * File:   main.c
 * Author: prapul
 *
 * Created on 11 April, 2015, 4:57 PM
 */

/******************************************************************************
 * Software License Agreement
 *
 * Copyright � 2011 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED ìAS ISî WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 *****************************************************************************/


#include <xc.h>
#include "i2c_master_int.h"
#include "i2c_display.h"
#include "OLED.h"
#include "accel.h"
#include <math.h>

#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF// not boot write protect
#pragma config CP = OFF// no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins by turning sosc off
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 40MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_20 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 40MHz
#pragma config UPLLIDIV = DIV_2 // divide 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = ON // allow only one reconfiguration
#pragma config IOL1WAY = ON // allow only one reconfiguration
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // controlled by USB module


//	Function Prototypes
int main(void);
int readADC(void);
void print_string_oled(char *str);

void gen_setup();


int main(void) {
        int val,x_acc, y_acc,i,j,cur_row, cur_column;
        int cons=1337;
        char string[50];
        short accels[3]; // accelerations for the 3 axes
        short mags[3]; // magnetometer readings for the 3 axes
        short temp;
	// startup
        gen_setup();
        display_init();
        acc_setup();
        screen_on();
        
        //oled_pos_set(28,32);
        //sprintf(string,"Hello world!%d",cons);
        //print_string_oled(string);
    //display_draw();


	while (1) {
		// invert pin every 0.5s, set PWM duty cycle % to the pot voltage output %
		_CP0_SET_COUNT(0); // set core timer to 0, remember it counts at half the CPU clock
		LATBINV = 0b10000000; // invert a pin
		// wait for half a second, setting LED brightness to pot angle while waiting
		while (_CP0_GET_COUNT() < 10000000) {
			val = readADC();
			//val = (val/3);
			//if(val>1) val = 1;
			//if(val<0) val = 0;
			OC1RS = val * PR2 /1024 ;
                        
                        // read the accelerometer from all three axes
                        // the accelerometer and the pic32 are both little endian by default (the lowest address has the LSB)
                        
                        // the accelerations are 16-bit twos compliment numbers, the same as a short
                        acc_read_register(OUT_X_L_A, (unsigned char *) accels, 6);
                        
                        // need to read all 6 bytes in one transaction to get an update.

                        acc_read_register(OUT_X_L_M, (unsigned char *) mags, 6);
                        // read the temperature data. Its a right justified 12 bit two's compliment number

                        acc_read_register(TEMP_OUT_L, (unsigned char *) &temp, 2);
                        cur_row = 64;
                        cur_column = 32;//x_acc, y_acc
                        //sprintf(string,"acc_x:%d acc_y:%d acc_z:%d mag_x:%d mag_y:%d mag_z:%d temp:%d",accels[0],accels[1],accels[2],mags[0],mags[1],mags[2],temp);
                        //print_string_oled(string);
                        x_acc = (int)((double)accels[0]*64/16000);
                        y_acc = (int)((double)accels[1]*32/16000);
                        if(x_acc>=64)x_acc=63;
                        if(x_acc<=-64)x_acc=-63;
                        if(y_acc>=32)y_acc=31;
                        if(y_acc<=-32)y_acc=-31;
                        display_clear();
                        if(x_acc>0){
                        for(i=64;i<=64+x_acc;i++){
                            for(j=0;j<=4;j++){
                                display_pixel_set(j+cur_column, i,0b1);
                            }
                       }
                        }
                         if(x_acc<0){
                        for(i=64+x_acc;i<=64;i++){
                            for(j=0;j<=4;j++){
                                display_pixel_set(j+cur_column, i,0b1);
                            }
                       }
                        }

                        if(y_acc>0){
                        for(i=0;i<=4;i++){
                            for(j=32;j<=32+y_acc;j++){
                                display_pixel_set(j, i+cur_row,0b1);
                            }
                       }
                        }

                         if(y_acc<0){
                        for(i=0;i<=4;i++){
                            for(j=32+y_acc;j<=32;j++){
                                display_pixel_set(j, i+cur_row,0b1);
                            }
                       }
                        }
 
                        
                        display_draw();
			if (PORTBbits.RB13 == 1) {
				;// nothing
			} else {
				LATBINV = 0b10000000;
			}
		}
	}

    
}
int readADC(void) {
	int elapsed = 0;
	int finishtime = 0;
	int sampletime = 20;
	int a = 0;

	AD1CON1bits.SAMP = 1;
	elapsed = _CP0_GET_COUNT();
	finishtime = elapsed + sampletime;
	while (_CP0_GET_COUNT() < finishtime) {
	}
	AD1CON1bits.SAMP = 0;
	while (!AD1CON1bits.DONE) {
	}
	a = ADC1BUF0;
	return a;
}
void gen_setup(){

	__builtin_disable_interrupts();

	// set the CP0 CONFIG register to indicate that
	// kseg0 is cacheable (0x3) or uncacheable (0x2)
	// see Chapter 2 "CPU for Devices with M4K Core"
	// of the PIC32 reference manual
	__builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

	// no cache on this chip!

	// 0 data RAM access wait states
	BMXCONbits.BMXWSDRM = 0x0;

	// enable multi vector interrupts
	INTCONbits.MVEC = 0x1;

	// disable JTAG to be able to use TDI, TDO, TCK, TMS as digital
	DDPCONbits.JTAGEN = 0;



	// set up USER pin as input
	ANSELBbits.ANSB13 = 0;
	TRISBbits.TRISB13 = 1;

	// set up LED1 pin as a digital output
	TRISBbits.TRISB7 = 0;

	// set up LED2 as OC1 using Timer2 at 1kHz
	PR2 = 40000 - 1; // Timer2 is the base for OC1, PR2 defines PWM frequency, 1 kHz
	TMR2 = 0; // initialize value of Timer2
	T2CONbits.ON = 1; // turn Timer2 on, all defaults are fine (1:1 divider, etc.)
        T2CONbits.TCKPS = 0; // timer 2 prescaler  = 1
        T2CONbits.TGATE = 0;
        OC1CONbits.OCTSEL = 0; // use Timer2 for OC1
	OC1CONbits.OCM = 0b110; // PWM mode with fault pin disabled
	OC1CONbits.ON = 1; // Turn OC1 on
	OC1R = 40000-1;
	OC1RS = 40000-1;
	RPB15Rbits.RPB15R = 0b0101; // set B15 to U1TX

	// set up A0 as AN0
	ANSELAbits.ANSA0 = 1;
	AD1CON3bits.ADCS = 3;
	AD1CHSbits.CH0SA = 0;
	AD1CON1bits.ADON = 1;
       // U1RXRbits.U1RXR = 0b0000; // set U1RX to pin A2

	__builtin_enable_interrupts();
}