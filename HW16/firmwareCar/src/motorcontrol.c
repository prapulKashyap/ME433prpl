//Included header files
#include "motorcontrol.h"
#include <xc.h>


//functions
void motor_init(void) {


        // set up B15 (phase for left wheel)pin as output
        ANSELBbits.ANSB15=0;
	TRISBbits.TRISB15 = 0;

        // set up B14 (phase for right wheel) pin as output
        ANSELBbits.ANSB14=0;//setting B14 to digital
	TRISBbits.TRISB14 = 0;

	// set up B5 as OC2(right wheel) using Timer2 at 1kHz
	PR2 = 40000 - 1; // Timer2 is the base for OC1, PR2 defines PWM frequency, 1 kHz
	TMR2 = 0; // initialize value of Timer2
	T2CONbits.ON = 1; // turn Timer2 on, all defaults are fine (1:1 divider, etc.)
        T2CONbits.TCKPS = 0; // timer 2 prescaler  = 1
        T2CONbits.TGATE = 0;
        OC2CONbits.OCTSEL = 0; // use Timer2 for OC1
	OC2CONbits.OCM = 0b110; // PWM mode with fault pin disabled
	OC2CONbits.ON = 1; // Turn OC1 on
	OC2R = 40000-1;
	OC2RS = 40000-1;
	RPB5Rbits.RPB5R = 0b0101; // set B5 to OC2

        // set up B4 as OC1(left wheel) using Timer2 at 1kHz
        OC1CONbits.OCTSEL = 0; // use Timer2 for OC3
	OC1CONbits.OCM = 0b110; // PWM mode with fault pin disabled
	OC1CONbits.ON = 1; // Turn OC3 on
	OC1R = 40000-1;
	OC1RS = 40000-1;
	RPB4Rbits.RPB4R = 0b0101; // set B4 to OC1

}

void left_motor_pwm_set(int duty_percent){  // Specifies the user's PWM value
	if(duty_percent>0){// clockwise rotation
		OC1RS = (unsigned int) ((float)duty_percent*PR2/100);//PWM for the required duty cycle is set
		 LATBbits.LATB15=1;//phase is set to 1
	}
	else{// counter clockwise rotation
		OC1RS = (unsigned int) ((float)duty_percent*PR2*-1/100);//PWM for the required duty cycle is set
		LATBbits.LATB15=0;//phase is set to 0
	}
}
void right_motor_pwm_set(int duty_percent){  // Specifies the user's PWM value
	if(duty_percent>0){// clockwise rotation
		OC2RS = (unsigned int) ((float)duty_percent*PR3/100);//PWM for the required duty cycle is set
		 LATBbits.LATB14=1;//phase is set to 1
	}
	else{// counter clockwise rotation
		OC2RS = (unsigned int) ((float)duty_percent*PR3*-1/100);//PWM for the required duty cycle is set
		 LATBbits.LATB14=0;//phase is set to 0
	}
}

void vehicle_travel(int travel){
    static float prev_error=0;
    float error= (travel-320)*100/320;
    int speed;
    float            kp = 0.3,kd = 0.1;
    int spminerr,spplerr;
   // speed = abs(abs(error)-110);
    //if(abs(error)>85)speed = 100;
    //if(speed<10)speed = 10;
   // if(speed>100)speed = 100;
    speed = 100-(abs(error));
    
    
    error=(kp*error )+( kd*((error-prev_error)));
    if(error>100)error=100;
    if(error<-100)error=-100;
    prev_error=error;
    spminerr = speed-error;
    spplerr = speed+error;
    if(spminerr>100){spminerr=100;}
    if(spminerr<-100){spminerr=-100;}
    if(spplerr>100){spplerr=100;}
    if(spplerr<-100){spplerr=-100;}
    if(error<0){
    right_motor_pwm_set(spminerr);
    left_motor_pwm_set(spplerr);
    }
    else{
    {
    left_motor_pwm_set(spplerr);
    right_motor_pwm_set(spminerr);
    }
    }
    
    

}


