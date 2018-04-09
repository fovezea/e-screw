/*
to do 
limit switch stop

*/

/*
reading the hardware encoder part was inspired from 
https://forum.arduino.cc/index.php?topic=140205.30
*/


#include <math.h>
#include <U8glib.h>


//this block should be changed with defines
const int quad_A = 2;   //Pin for the encoder A pin
const int quad_B = 13;  //Pin for the encoder B pin
const int z = 10;       //Pin for the encoder Index

#define directionPin  12    // controles the direction of rotation and pin 5 controles the steps
#define STEP TC_Start(TC2, 0)


U8GLIB_ST7920_128X64_4X u8g(63,62, 4);          // LCD control pins

volatile signed int z_Total=0;               //keeps the total rotation
volatile signed long accumulatedPulses=0;  // keeps the total pulses
volatile int precision=1000;                  //equals with 1 , 10, 100 etc depending on how many decimals after the comma you want to consider in the division ration calculation 
volatile int breseham_demult_zecimala=0;    //keeps the zecimal part of the division ratio
volatile int error_accum=0;                  //used to keep the accumulated error from bresenham 
volatile int breseham_err=0;                //used to add one count to spindle counter when error_accumulator is bigger then "precision"
volatile int breseham_demult_int=12;        // keeps the integer part of the division ratio
enum State_machine{
                    motor_stop,
                    motor_at_speed,
                    motor_accel,
                    motor_deccel
                }state_machine; 
volatile int lost_step=0;
//volatile boolean motor_at_speed=true;
//volatile boolean motor_stop=false; 
//volatile boolean motor_deccel=false;
//volatile boolean motor_enable=true;  
//volatile boolean motor_accel=false;
volatile boolean output;
volatile boolean timing_err=false;
volatile signed long spindlePosition = 0;
volatile signed int oldSpindlePosition = 0;
volatile signed long saddlePosition = 0;
volatile boolean step_on=false;
volatile boolean step_off=false;
volatile long total_spindle;               
float encoderPPR = 1024*4;
float stepperPPR=200*8;
float e_gear_ratio;
float gear_ratio=6;     //existing lead screw ratio and /or gears between motor and lead screw (my lathe has 6 on B 6/2 on A si 6/4 on C
volatile boolean direction;
volatile boolean motorStart = false;

volatile boolean sincStarted = false;  //avoid adding encoderPPR  on incomplete rotation (first start) 

volatile int motor=0;  //just for debugging shows how the pulses for the motor are accumulating



// this rutine is starting a timer interupt for the given channel and with the given freqv value
void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
       pmc_set_writeprotect(false);
       pmc_enable_periph_clk((uint32_t)irq);
       TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
       uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
       TC_SetRA(tc, channel, rc/2); //50% high, 50% low
       TC_SetRC(tc, channel, rc);
       TC_Start(tc, channel);
       tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
       tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
       NVIC_EnableIRQ(irq);
}

//this is a hardcoded routine to set up a single shot timer for the step control.
//output pin is hardcoded for tc6
void startTimer_single_shot(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
        pmc_set_writeprotect(false);
        pmc_enable_periph_clk((uint32_t)irq);

        TC_Configure(tc, channel,
               TC_CMR_WAVE |
               TC_CMR_WAVSEL_UP_RC |
               TC_CMR_TCCLKS_TIMER_CLOCK4 |
              TC_CMR_ACPA_SET |     // RA compare sets TIOA
              TC_CMR_ACPC_CLEAR |  // RC compare clears TIOA
              TC_CMR_CPCSTOP                  // if is 1: Counter clock is stopped when counter reaches RC.
              );                             
        uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
        TC_SetRA(tc, channel, 1); //100/% on - can play with the value to make sure have  enough delay between steps
        TC_SetRC(tc, channel, rc);

         PIO_Configure(PIOC,
                       PIO_PERIPH_B,
                       PIO_PC25B_TIOA6,
                       PIO_DEFAULT);

         TC_Start(tc, channel); 
             
        //set up interupts 
        tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
       tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
        NVIC_EnableIRQ(irq);
}

//floating point calculation for the divider
//this function is slow but is calculated only once 
//using a double float, it returns 2 integer values

void divider_calculation( float pitch){
  
  float f;
  
  f=(encoderPPR)/(stepperPPR/(gear_ratio/pitch));
  Serial.print("partea cu virgula");Serial.println(f);
  breseham_demult_int= (int)f; 
  breseham_demult_zecimala = (int)((f - breseham_demult_int)* precision);
  
}


//fast pin reading and writing routines
inline void digitalWriteDirect(int pin, boolean val){
  if(val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

inline int digitalReadDirect(int pin){
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}
/*
 * routine for redrawing the screen
 * TODO implement one buffer to be updated from other routines
 */
void draw(){
  char buff[12];
  u8g.drawStr( 0, 20, "E-gearbox");
  u8g.drawStr(0, 35, itoa(spindlePosition, buff, 10)); 
   u8g.drawStr(0, 50, itoa(z_Total, buff, 10));
 u8g.drawStr(80, 50, itoa(accumulatedPulses, buff, 10));
}

void setup() {


  state_machine = motor_at_speed;
  
  Serial.begin(115200); 
  delay(100);
  
//display
  u8g.setFont(u8g_font_unifont);
  u8g.setColorIndex(1); // Instructs the display to draw with a pixel on. 

    //pin 12 set to output
    pinMode(directionPin, OUTPUT); //for direction

    //pinMode(2,INPUT_PULLUP);
    //pinMode(13,INPUT_PULLUP);

     // set pullups on inputs
 pinMode(2, OUTPUT);
 digitalWrite(2, 1);
 pinMode(13, OUTPUT);
 digitalWrite(13, 1);
 pinMode(A6, OUTPUT);
 digitalWrite(A6, 1);

  // Setup Quadrature Encoder

  REG_PMC_PCER0 = (1<<27)|(1<<28)|(1<<29); // activate clock for TC0,TC1,TC2

  // select XC0 as clock source and set capture mode
      //REG_TC0_CMR0 = 5;
      //Resets the count every index. Rem out the line above,and use the line below instead.
      REG_TC0_CMR0 = (1<<0)|(1<<2)|(1<<8)|(1<<10);

  REG_TC0_CMR2 = (1<<0)|(1<<1); // Set to TCLK4

  // activate quadrature encoder and position measure mode, no filters
  REG_TC0_BMR = (1<<8)|(1<<9)|(1<<12);

  // Enable the clock (CLKEN=1) and reset the counter (SWTRG=1)
  REG_TC0_CCR0 = 5; 
  REG_TC0_CCR1 = 5;
  REG_TC0_CCR2 = 5;

          //Remark out these 4 lines to disable Index interrupts
            REG_TC0_CMR1 = (1<<8); // Set rising edge of Z
            REG_TC0_IER1=0b10000001; // enable interrupt on Z (bit7) and on couter overflow (bit0)
            REG_TC0_IDR1=0b01111110; // disable other interrupts
            NVIC_EnableIRQ(TC1_IRQn);

      //intrerupere la schimbarea de pin pentru A:
     //pinMode(z, INPUT);       
     //attachInterrupt(quad_A, A_rising, RISING);  


//start the timer for the control loop
startTimer(TC1, 0, TC3_IRQn, 1000); //TC1 channel 0, the IRQ for that channel and the desired frequency
//startTimer(TC1, 1, TC4_IRQn, 50000); //TC1 channel 1, the IRQ for that channel and the desired frequency
//startTimer(TC1, 2, TC5_IRQn, 2); //TC1 channel 0, the IRQ for that channel and the desired frequency

//single shot timer but the output pin is hardcoded for pin 5 of TC6
//with a 500 for frequency the step lenght is 1 ms and with 700 is abt. 1.4
startTimer_single_shot(TC2, 0, TC6_IRQn,700);

//debug
divider_calculation(3);//request calculation of the e-gear ratio

//Serial.print("partea intreaga");Serial.println(breseham_demult_int);
//Serial.print("partea zecimala");Serial.println(breseham_demult_zecimala);
}

void loop() {
  
  //start the saddle
  motorStart=true;
  //REG_TC0_CV0 Stores count from encoder
  //(REG_TC0_QISR >> 8) & 0x1; gives 0 or 1 depends of direction
  if ((REG_TC0_QISR >> 8) & 0x1)Serial.println("stinga");
  else Serial.print("dreapta");
  Serial.print("accumulatedPulses= "); Serial.println(accumulatedPulses);
  Serial.print("Spindle index = ");Serial.println(spindlePosition);//Serial.print(" ");//Serial.println(breseham_demult_int);
  Serial.print("Motor index = ");Serial.println(saddlePosition);
  //Serial.println(accumulatedPulses);
  Serial.print("total_spindle= ");Serial.println(total_spindle);
/*

  //REG_TC0_CV1 Stores count from index if interrupts are off
    //if interrupts are on, CV1 will always reset every cycle to zero.
 if (REG_TC0_CV1 == 0) 
  {
   // Serial.print(z_Total);
   // Serial.println("         -With Int");
  }
  else
  {
    //Serial.print(REG_TC0_CV1);
    //Serial.println("         -No Int");
  }
  
*/  
  //TPR holds the quantity of ticks every Index. Index interrupts must be on.
  //TCLK4 is set so 128 is the divisor.
  //Serial.print((F_CPU/128/TPR)*60);Serial.println(" RPM");
  

 // if (timing_err){
 //   Serial.println("buffer overflow");
 //  timing_err=false; 
 // }
//  Serial.println("-------");


  //LCD drawing routine
  u8g.firstPage();
  do {  
    draw();
  } while( u8g.nextPage() );

  delay(200);

}


// void A_rising(){
  
// }


void TC1_Handler() {
  //This won't fire unless the Index interrupt is enabled above
  

  volatile long status = REG_TC0_SR1; // vital - reading this clears some flag
                            // otherwise you get infinite interrupts
                        

// need  to avoid adding encoderPPR  on incomplete rotation (first start) 
 if (sincStarted){
if((REG_TC0_QISR >> 8) & 0x1){
 accumulatedPulses-=encoderPPR;
}else {
  accumulatedPulses+=encoderPPR;
  //accumulatedPulses=REG_TC0_CV2;
}
//need to add here cod to start the acceleration in order to be in sinc with the spindle
if (motorStart){ 
  
  state_machine = motor_accel;

  state_machine=motor_at_speed;   //this needs to change when the code is ready to motor_accel;
motorStart=false;
}
 }else{
   sincStarted = true; 
 }



}

//TC1 ch 0
void TC3_Handler()
{
      
      volatile long dummy=REG_TC1_SR0;
     
  switch (state_machine){

  case (motor_stop): 
           sincStarted = false;
           total_spindle = 0;
           accumulatedPulses = 0;
           oldSpindlePosition = 0;
           saddlePosition = 0;
           spindlePosition = 0;
           break;

  case (motor_at_speed):
    //check if we are not during a step already in which case return
      if (step_on) break;
      if (!sincStarted) break;
      //store the encoder position 
        spindlePosition = REG_TC0_CV0;
        if(accumulatedPulses){
           total_spindle += (accumulatedPulses - oldSpindlePosition) + spindlePosition;
           oldSpindlePosition = spindlePosition;
           accumulatedPulses = 0;
        }else{
          total_spindle +=spindlePosition - oldSpindlePosition;
          oldSpindlePosition = spindlePosition;
        }
       //check spindle rotating direction
       if((REG_TC0_QISR >> 8) & 0x1){   //gives 0 or 1 depending on the direction
              //if rotation is pozitive

              //store the encoder position 
              //spindlePosition = REG_TC0_CV0;
              //just for test fire the step pin
              if(total_spindle > saddlePosition  ){
                digitalWriteDirect(directionPin, true );  //set the direction pin
                step_on=true;
                saddlePosition ++;
                STEP; // this fires the single shot timer fore the step 
            
              }
       }else{
            //if rotation is negative
              
            if(total_spindle < saddlePosition ){
              digitalWriteDirect(directionPin, false);
              step_on = true;
              saddlePosition --;
              STEP; // this fires the single shot timer fore the step 
            }
       }
       break;
       case ( motor_accel):break;  //code to be added
       case (motor_deccel):break;
  }


     /*
      switch (state_machine){
      
        
        case 0:     if((spindlePosition+breseham_err+lost_step)>=breseham_demult_int){
                    timing_err=true;
                    REG_TC0_CCR0 = 5; //Reset counter
                    }
                    if((pozspindlePositionitie+breseham_err+lost_step)==breseham_demult_int){
                   // state_machine=1;   //pentru ca registru de spindle nu se reseteaza decit la urmatorul puls de la encoder si pe care il pierde
                    REG_TC0_CCR0 = 5; //Reset counter
                    breseham_err=0;
                    total_spindle +=breseham_demult_int;
                    lost_step=1;
                    error_accum += breseham_demult_zecimala;  //incrementam partea zecimala
                     if (error_accum > precision){
                        breseham_err=1;
                        total_spindle++;
                        error_accum-=precision; //reducem accumulatorul de eroare
                     }
                    
                    
                    //debug
                    //Serial.print("Spindle index = ");Serial.print(spindlePosition);Serial.print(" ");Serial.println(breseham_demult_int);
                    step_on=true;
                    }
                    break; 
        case 1:  if(pozitispindlePositione==0)state_machine=0;
                 break; 
    
      
      
      }*/
}  

//intrerupere de tastatura
void TC5_Handler()
{
       volatile long dummy=TC_GetStatus(TC1, 2);
       
}

//at the end of the step interrupt is triggered to notify that a new step can be done
void TC6_Handler()
{
        TC_GetStatus(TC2, 0);  
        step_on=false; //sigals that the step was finished and a new step can be done
        
}

