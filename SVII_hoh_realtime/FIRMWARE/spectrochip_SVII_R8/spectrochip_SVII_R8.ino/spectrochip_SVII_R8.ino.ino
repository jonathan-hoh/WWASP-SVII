

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                           Spectrochip SVII Driver Version 1.20
// Written By: Adrian Tang  UCLA
// Date: Mar 18, 2017
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// A little basic revision control never hurt 
#include <EEPROM.h>
#define version_code  "3.11"


//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                         I/O and Pin Configuration
//-----------------------------------------------------------------------------------------------------------------------------------------------
//Interface to SVII SOC

int LED = 13; // Pin 13 is the monitor/debug LED 

// Sequence control interface
int RODONE = 3;  //IC Pin 1 AR 3
int ROSTAR = 4;  //IC Pin 2 AR 4 
int CDONE =  5;  //IC Pin 9 AR 5
int CSTAR =  6;  //IC Pin 10 AR 6

//SPI Interface for Config and Control
int UMX = 7;  //IC Pin 11  AR Pin  D7
int USX = 8;  //IC Pin 12  AR Pin  D8
int UCK = 9;  //IC Pin 13  AR Pin  D9
int RES = 10; //IC Pin 14  AR Pin  D10

//Analog sensor and monitor interfaces
int CM_mon    = A0; //Pin 23 AR Analog Pin 0  //D Digital clock phase monitor
int clk_vctrl = A1; //Pin 24 AR Analog Pin 1  // Control voltage from internal clock PLL
int DP_mon    = A2; //Pin 25 AR Analog Pin 2  // THA & Preamp common-mode monitor feedback
int AP_mon    = A3; //Pin 26 AR Analog Pin 3  // Analog clock phase monitor

//Debug led




//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                         SVII SOC Control Registers and Structs
//-----------------------------------------------------------------------------------------------------------------------------------------------
//Analog Dynamic Calibration

int CM_opt=214;  //Optimal THA CM Value (determined emperically)
int PE_opt=155;  //Optimal Pre_Amp CM Value (determined emperically)

// --------- ADC Configuration Registers----------- //

//Analog Section
int CM_select;  // 1 bit  address 0L         //Selects which voltage is displayed at the common mode output monitor pin
int THA;        // 8 bit  address 0H         //Sets the THA input bias voltage
int PREAMP;     // 8 bit  address 1L         //Sets the comparator-preamplifier bias current

//Clock Config
int delay_B;    // 6 bit  address 1H         //Sets the coarse tune clock delay (positive) of ADC 2's THA and comparator
int delay_BN;   // 6 bit  address 2L         //Sets the coarse tune clock delay (negative) of ADC 2's THA and comparator
int delay_A;    // 6 bit  address 2H         //Sets the coarse tune clock delay (positive) of ADC 1's THA and comparator
int delay_AN;   // 6 bit  address 3L         //Sets the coarse tune clock delay (negative) of ADC 1's THA and comparator
int dfine_CD;   // 8 bit  address 3H         //Sets the fine tune clock delay of ADC 2's THA and comparator
int dfine_AB;   // 8 bit  address 4L         //Sets the fine tune clock delay of ADC 1's THA and comparator
int dtune_CD;   // 4 bit  address 4H         //Sets the coarse tune clock delay of DEMUX CD (data coming from ADC 2) 
int dtune_AB;   // 4 bit  address 5L         //Sets the coarse tune clock delay of DEMUX AB (data coming from ADC 1)
int ADC_ref6;   // 8 bit  address 5H         //Sets the ADC reference level 
int ADC_ref5;   // 8 bit  address 6L         //Sets the ADC reference level 
int ADC_ref4;   // 8 bit  address 6H         //Sets the ADC reference level 
int ADC_ref3;   // 8 bit  address 7L         //Sets the ADC reference level 
int ADC_ref2;   // 8 bit  address 7H         //Sets the ADC reference level 
int ADC_ref1;   // 8 bit  address 8L         //Sets the ADC reference level 
int ADC_ref0;   // 8 bit  address 8H         //Sets the ADC reference level 
int ADC_REF_R_BIAS;   // 8 bit  address 9L         //Sets the ADC reference level 
int ADC_REF_B_BIAS;   // 8 bit  address 9H         //Sets the ADC reference level 
int ADC_REF_A_BIAS;   // 8 bit  address 10L         //Sets the ADC reference level 
int clock_sel;  // 4 bit  address 10H               //Sets the clock rate 1 = full speed, 2 = half speed, 3 = quarter speed, 4 = external clock
int retime_AB2; // 8 bit  address 11L         //Sets the fine tune clock delay of retimer AB stage 2 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX and previous stage retimer over PVT)
int retime_AB1; // 8 bit  address 11H         //Sets the fine tune clock delay of retimer AB stage 1 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX over PVT)
int retime_CD2; // 8 bit  address 12L         //Sets the fine tune clock delay of retimer CD stage 2 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX and previous stage retimer over PVT)
int retime_CD1; // 8 bit  address 12H         //Sets the fine tune clock delay of retimer CD stage 1 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX over PVT)
int demuxf_AB;  // 8 bit  address 13L         //Sets the fine tune clock delay of DEMUX CD (data coming from ADC 2)
int demuxf_CD;  // 8 bit  address 13H         //Sets the fine tune clock delay of DEMXU AB (data coming from ADC 1)
int divider_res; // 1 bit  address 14L        //Reset the high speed clock divider on boot-up



int dec_coarse;  // 5 bit  address 15H        //Sets the coarse tune clock delay of decoder
int dec_fine;    // 8 bit  address 16L        //Sets the fine tune clock delay of decoder
int demux_reset; // 1 bit address 16H        //Reset of the Yan's demux
int vco_on;      // 1 bit address 17L        //Turns the VCO on and off
int trimcap;     // 4 bit address 17H        //Tunes the VCO
int DAC_CD_SEL;  // 1 bit address 18L        //Aux dac channel select
int DAC_AB_SEL;  // 1 bit address 18H        //Aux dac channel select

// -------------- Readout Config Registers ----------------- //
int RO_clk_div_enab; // 1 bit address 14H     //Enables the high-speed to readout-speed clock divider
int RO_clk_div_fact; // 4 bit address 15L    //Sets the division ratio to 2^(setting)
int multi_pixel;     // 1 bit address 19L    //Turns on and off the multi-pixel serial readout format


// -------------- Processor Config Registers --------------- //
int debug_en;  // address 19H  1 bit for debug mode
int hnw_en;    // address 20L 1 bit hardware enable
int averagesLL; //[D00-D07] // 8 bit address 22L   //Controls the amount of observation time before the core flushes to the readout
int averagesLH; //[D08-D15] // 8 bit address 22H   //NOTE! there is no internal overflow protection
int averagesHL; //[D16-D23] // 8 bit address 23L
int averagesHH; //[D24-D31] // 8 bit address 23H

//---------------Software Stuctures--------------------------
char avg[2];            // Average factor accepted from host
int SOC_RETURN[3];      // SPI buffer for SoC->Host




//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                   ATMEL Controller Configuration and Initialization
//-----------------------------------------------------------------------------------------------------------------------------------------------
void setup() {  
  
 byte check[3];  //Data register for SPI contents returned from the SOC

// initialize the serial port
  Serial.begin(57600);
  
// initialize the digital pins for the correct direction
  //SPI pins
  pinMode(UCK, OUTPUT);  
  pinMode(UMX, OUTPUT);  
  pinMode(RES, OUTPUT);  
  pinMode(USX, INPUT);    

  //Sequence Control Pins  
  pinMode(RODONE, INPUT);    
  pinMode(ROSTAR, OUTPUT);    
  pinMode(CDONE, INPUT);    
  pinMode(CSTAR, OUTPUT);    

  //Debug Pins
  pinMode(LED, OUTPUT);  

  //Wait for a serial connection
  while (!Serial) {}

//Reset the SVII core
digitalWrite(ROSTAR,LOW);
digitalWrite(CSTAR,LOW);
  
  //Reset the SOC-ASIC Controller with the FSMRES pin
  digitalWrite(UMX, LOW);
  delay(100);
  digitalWrite(RES, LOW);
  delay(100);
  digitalWrite(RES, HIGH);
  delay(100); 

//-----------------------------------------------------------
//                  Identify ourselves to the host
//-----------------------------------------------------------
Serial.println(" ");
Serial.print("SVII Release R8 Version: ");
Serial.println(version_code);

//plot the ready prompt (normally we have the host hook on the ">"
Serial.println("SVII Ready>");



//-----------------------------------------------------------
//                  Default Configuration
//-----------------------------------------------------------
CM_select    = 0;  // 1 bit  address 0L         //Selects which voltage is displayed at the common mode output monitor pin
THA          = 47;  // 8 bit  address 0H         //Sets the THA input bias voltage
PREAMP       = 71;  // 8 bit  address 1L         //Sets the comparator-preamplifier bias current

//Clock Config     
delay_B      = 30;  // 6 bit  address 1H         //Sets the coarse tune clock delay (positive) of ADC 2's THA and comparator
delay_BN     = 30;  // 6 bit  address 2L         //Sets the coarse tune clock delay (negative) of ADC 2's THA and comparator
delay_A      = 30;  // 6 bit  address 2H         //Sets the coarse tune clock delay (positive) of ADC 1's THA and comparator
delay_AN     = 30;  // 6 bit  address 3L         //Sets the coarse tune clock delay (negative) of ADC 1's THA and comparator
dfine_CD     = 50; // 8 bit  address 3H         //Sets the fine tune clock delay of ADC 2's THA and comparator
dfine_AB     = 50; // 8 bit  address 4L         //Sets the fine tune clock delay of ADC 1's THA and comparator
dtune_CD     = 6; // 4 bit  address 4H         //Sets the coarse tune clock delay of DEMUX CD (data coming from ADC 2)
dtune_AB     = 6; // 4 bit  address 5L         //Sets the coarse tune clock delay of DEMUX AB (data coming from ADC 1)
clock_sel    = 1; // 4 bit  address 5H         //Sets the clock rate 1 = full speed, 2 = half speed, 4 = quarter speed, 8 = external clock
retime_AB2   = 41; // 8 bit  address 6L         //Sets the fine tune clock delay of retimer AB stage 2 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX and previous stage retimer over PVT)
retime_AB1   = 45; // 8 bit  address 6H         //Sets the fine tune clock delay of retimer AB stage 1 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX over PVT)
retime_CD2   = 45; // 8 bit  address 7L         //Sets the fine tune clock delay of retimer CD stage 2 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX and previous stage retimer over PVT)
retime_CD1   = 45; // 8 bit  address 7H         //Sets the fine tune clock delay of retimer CD stage 1 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX over PVT)
demuxf_AB    = 50; // 8 bit  address 8L         //Sets the fine tune clock delay of DEMUX CD (data coming from ADC 2)
demuxf_CD    = 50; // 8 bit  address 8H         //Sets the fine tune clock delay of DEMXU AB (data coming from ADC 1)
divider_res  =  1; // 1 bit  address 9L         //Reset the high speed clock divider on boot-up
dec_coarse   = 10; // 5 bit  address 10H        //Sets the coarse tune clock delay of decoder //default 15
dec_fine     = 50; // 8 bit  address 11L        //Sets the fine tune clock delay of decoder /default 50

demux_reset =1;   // 1 bit address 12L    // high speed demux before core reset

vco_on = 1;      // 1 bit address XXX        //Turns the VCO on and off
trimcap = 8;     // 4 bit address XXX        //Tunes the VCO
DAC_CD_SEL = 0;  // 1 bit address XXX        //Aux dac channel select
DAC_AB_SEL = 0;  // 1 bit address XXX        //Aux dac channel select


//----------------ADC Control Select-------------------
ADC_ref6 =187;   // 8 bit  address XX         //Sets the ADC reference level 
ADC_ref5 =181;   // 8 bit  address XX         //Sets the ADC reference level 
ADC_ref4 =174;   // 8 bit  address XX         //Sets the ADC reference level 
ADC_ref3 =168;   // 8 bit  address XX         //Sets the ADC reference level 
ADC_ref2 =162;   // 8 bit  address XX         //Sets the ADC reference level 
ADC_ref1 =155;   // 8 bit  address XX         //Sets the ADC reference level 
ADC_ref0 =149;   // 8 bit  address XX         //Sets the ADC reference level 
ADC_REF_R_BIAS = 102;   // 8 bit  address XX         //Sets the ADC reference level 
ADC_REF_B_BIAS = 102;   // 8 bit  address XX         //Sets the ADC reference level 
ADC_REF_A_BIAS = 102;   // 8 bit  address XX         //Sets the ADC reference level 




// -------------- Readout Config Registers ----------------- //
RO_clk_div_enab =1; // 1 bit address 9H     //Enables the high-speed to readout-speed clock divider
RO_clk_div_fact =6; // 3 bit address 10L    //Sets the division ratio to 2^(setting)
multi_pixel     =0;     // 1 bit address 13L    //Turns on and off the multi-pixel serial readout format

// -------------- Processor Config Registers --------------- //
averagesLL = 0; //[D00-D07] // 8 bit address 14L   //Controls the amount of observation time before the core flushes to the readout
averagesLH = 0; //[D08-D15] // 8 bit address 14H   //NOTE! there is no internal overflow protection
averagesHL = 16; //[D16-D23] // 8 bit address 15L
averagesHH = 0; //[D24-D31] // 8 bit address 15H
debug_en = 0;  // 1 bit for debug mode
hnw_en = 1;    // 1 bit hardware enable




//Update the config registers on the SoC with this data

write_USART(5,0,THA,CM_select,&check[0]);
write_USART(5,1,delay_B,PREAMP,&check[0]);
write_USART(5,2,delay_A,delay_BN,&check[0]);
write_USART(5,3,dfine_CD,delay_AN,&check[0]);
write_USART(5,4,dtune_CD,dfine_AB,&check[0]);
write_USART(5,5,ADC_ref6,dtune_AB,&check[0]);
write_USART(5,6,ADC_ref4,ADC_ref5,&check[0]);
write_USART(5,7,ADC_ref2,ADC_ref3,&check[0]);
write_USART(5,8,ADC_ref0,ADC_ref1,&check[0]);
write_USART(5,9,ADC_REF_B_BIAS,ADC_REF_R_BIAS,&check[0]);
write_USART(5,10,clock_sel,ADC_REF_A_BIAS,&check[0]);
write_USART(5,11,retime_AB1,retime_AB2,&check[0]);
write_USART(5,12,retime_CD1,retime_CD2,&check[0]);
write_USART(5,13,demuxf_CD,demuxf_AB,&check[0]);
write_USART(5,14,RO_clk_div_enab,divider_res,&check[0]);
write_USART(5,15,dec_coarse,RO_clk_div_fact,&check[0]);
write_USART(5,16,demux_reset,dec_fine,&check[0]);
write_USART(5,17,trimcap,vco_on,&check[0]);
write_USART(5,18,DAC_AB_SEL, DAC_CD_SEL,&check[0]);
write_USART(5,19,debug_en,multi_pixel,&check[0]);
write_USART(5,20,0,hnw_en,&check[0]);
write_USART(5,22,averagesLH,averagesLL,&check[0]);
write_USART(5,23,averagesHH,averagesHL,&check[0]);  
}



//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                                   ATMEL Controller Operations Handling
//-----------------------------------------------------------------------------------------------------------------------------------------------
void loop() { 

  
 byte check[3];  // Data returned from the SOC via the SPI
 char ASCII_cmd; // Command accepted from host    
  int ctr;       // counter for the register dump
  int readvalue; // Sensor Read value for the status dump

  int phase0;    //Phase detector values during sweep (cmd 5)
  int phase1;
  int phase2;
  int phase3;
  int phase4;
  int phase_th;
  int phase_start;
  int phase_stop;
  int phase_sel;
  
  // Wait for the serial data to arrive from the PC. Packet length should be 1 byte
  while (Serial.available() < 1) {
  }
   // Get the command from the USB buffer
  ASCII_cmd=Serial.read();



 //--------------HOST COMMAND TABLE------------------------
 //   a)   Configuration / Status Dump
 //   b)   Configuation Update
 //   c)   Sequencer reset
 //   d)   Core Start
 //   e)   Readout Start
 //   f)   Full Acqusition
 //   g)   Readout clock divider up
 //   h)   Readout clock divider down
 //   i)   Primary Clock 3G
 //   j)   Primary Clock 1.5G
 //   k)   Primary Clock 750M
 //   l)   THA up
 //   m)   THA down
 //   n)   PREAMP up
 //   o)   PREAMP down
 //   p)   Dtune AB up
 //   q)   Dtune AB down
 //   r)   Dtune CD up
 //   s)   Dtune CD down 
 //   t)   PLL Bypass
 //   u)   Hanning window off
 //   v)   Hanning window on
 //   w)   debug mode off
 //   x)   debug mode on
 //   z)   set averages
 //   1)   trimcap up
 //   2)   trimcap down
 //   3)   vco off
 //   4)   vco on
 //   6)   interleave cal
 //   7)   analog cal
 //   8)   clock cal
 //   9)   power down
 
 
//-------------------------------------------------------------------------------------------------
//                             CONTROL REGISTER COMMANDS
//-------------------------------------------------------------------------------------------------
 
 // If the ASCII command is('a') then do a configuration status dump
       if (ASCII_cmd == 'a') {
                      
                Serial.println(" ");
                Serial.println(" ");
                Serial.println("SOC Reg dump:");
                Serial.println("ADD MSB LSB");
                for (ctr = 0;ctr<16;ctr++){
                    write_USART(6,ctr,0,0,&check[0]);
                    Serial.print(ctr,DEC);
                    Serial.print(" ");
                    Serial.print(check[1],DEC);
                    Serial.print(" ");
                    Serial.println(check[0],DEC);                   
                }               
                Serial.println(" ");
                Serial.println(" ");
                Serial.println(" ");                
                Serial.println("Analog Status");
                CM_select    = 0;  // Selects CM Mode Readout              
                write_USART(5,0,THA,CM_select,&check[0]);
                delay(500);                
                readvalue=analogRead(clk_vctrl);
                Serial.print(" VCTRL: ");
                Serial.print(readvalue,DEC);
                readvalue=analogRead(CM_mon);
                Serial.print(" CM: ");
                Serial.print(readvalue,DEC);
                readvalue=analogRead(AP_mon);
                Serial.print(" AP: ");
                Serial.print(readvalue,DEC);
                readvalue=analogRead(DP_mon);
                Serial.print(" DP: ");
                Serial.print(readvalue,DEC);     
                CM_select    = 1;  // Selects PREAMP Mode Readout              
                write_USART(5,0,THA,CM_select,&check[0]);
                delay(500);
                readvalue=analogRead(CM_mon);
                Serial.print(" PRE: ");
                Serial.print(readvalue,DEC);               
                Serial.println(" ");
                Serial.println(" ");
                Serial.println(" ");
               Serial.println("Digital Status");               
               readvalue = digitalRead(CDONE);  
               Serial.print(" CD: ");
               Serial.print(readvalue,DEC);                    
               readvalue = digitalRead(RODONE);  
               Serial.print(" RD: ");
               Serial.print(readvalue,DEC);                                   
               Serial.println(">");                                   
        }

// If the ASCII command is('b') then do a configuration update
       if (ASCII_cmd == 'b') {
       write_USART(5,0,THA,CM_select,&check[0]);
       write_USART(5,1,delay_B,PREAMP,&check[0]);
       write_USART(5,2,delay_A,delay_BN,&check[0]);
       write_USART(5,3,dfine_CD,delay_AN,&check[0]);
       write_USART(5,4,dtune_CD,dfine_AB,&check[0]);
       write_USART(5,5,ADC_ref6,dtune_AB,&check[0]);
       write_USART(5,6,ADC_ref4,ADC_ref5,&check[0]);
       write_USART(5,7,ADC_ref2,ADC_ref3,&check[0]);
       write_USART(5,8,ADC_ref0,ADC_ref1,&check[0]);
       write_USART(5,9,ADC_REF_B_BIAS,ADC_REF_R_BIAS,&check[0]);
       write_USART(5,10,clock_sel,ADC_REF_A_BIAS,&check[0]);
       write_USART(5,11,retime_AB1,retime_AB2,&check[0]);
       write_USART(5,12,retime_CD1,retime_CD2,&check[0]);
       write_USART(5,13,demuxf_CD,demuxf_AB,&check[0]);
       write_USART(5,14,RO_clk_div_enab,divider_res,&check[0]);
       write_USART(5,15,dec_coarse,RO_clk_div_fact,&check[0]);
       write_USART(5,16,demux_reset,dec_fine,&check[0]);
       write_USART(5,17,trimcap,vco_on,&check[0]);
       write_USART(5,18,DAC_AB_SEL, DAC_CD_SEL,&check[0]);
       write_USART(5,19,debug_en,multi_pixel,&check[0]);
       write_USART(5,20,0,hnw_en,&check[0]);
       write_USART(5,22,averagesLH,averagesLL,&check[0]);
       write_USART(5,23,averagesHH,averagesHL,&check[0]);  
          Serial.println("CFG OK*");                          
       }
       


//-------------------------------------------------------------------------------------------------
//                           READOUT CONFIGURATION COMMANDS
//-------------------------------------------------------------------------------------------------

// If the ASCII command is('g') then adjust readout clock rate up
       if (ASCII_cmd == 'g') {
       if (RO_clk_div_fact+1 < 7){         
       RO_clk_div_fact = RO_clk_div_fact +1;  
       write_USART(5,15,dec_coarse,RO_clk_div_fact,&check[0]);
       Serial.print("CLK: ");                                 
       Serial.println(RO_clk_div_fact,DEC);                                 
       }       
       }

// If the ASCII command is('h') then adjust readout clock rate down
       if (ASCII_cmd == 'h') {
       if (RO_clk_div_fact-1 > -1){
       RO_clk_div_fact = RO_clk_div_fact - 1;           
       write_USART(5,15,dec_coarse,RO_clk_div_fact,&check[0]);
       Serial.print("CLK: ");                                 
       Serial.println(RO_clk_div_fact,DEC);                                        
       }
       }       



// If the ASCII command is('1') then increase VCO tuning cap
       if (ASCII_cmd == '1') {
       if (trimcap +1 < 16){         
       trimcap = trimcap  +1;  
       write_USART(5,17,trimcap,vco_on,&check[0]);
       Serial.print("trim: ");                                 
       Serial.println(trimcap ,DEC);                                 
       }       
       }

// If the ASCII command is('2') then decrease VCO tuning cap
       if (ASCII_cmd == '2') {
       if (trimcap -1 > -1){
       trimcap  = trimcap  - 1;           
       write_USART(5,17,trimcap,vco_on,&check[0]);
       Serial.print("trim: ");                                 
       Serial.println(trimcap ,DEC);                                                 
       }
       }       

// If the ASCII command is('3') then turn the VCO off
       if (ASCII_cmd == '3') {
       vco_on = 0;       
       write_USART(5,17,trimcap,vco_on,&check[0]);
       Serial.println("vco_off");                                   
       }             

// If the ASCII command is('4') then turn the VCO on
       if (ASCII_cmd == '4') {
       vco_on = 1;       
       write_USART(5,17,trimcap,vco_on,&check[0]);
       Serial.println("vco_on");                                                 
       }            

// If the ASCII command is('i') then set the 3G clock mode
       if (ASCII_cmd == 'i') {
       clock_sel = 1;
       write_USART(5,10,clock_sel,ADC_REF_A_BIAS,&check[0]);           
       Serial.println("3G Clock");                                 
       }       

// If the ASCII command is('j') then set the 1.5G clock mode
       if (ASCII_cmd == 'j') {
       clock_sel = 2;
       write_USART(5,10,clock_sel,ADC_REF_A_BIAS,&check[0]);
       Serial.println("1.5G Clock");                                 
       }       

// If the ASCII command is('k') then set the 0.75G clock mode
       if (ASCII_cmd == 'k') {
       clock_sel = 4;
       write_USART(5,10,clock_sel,ADC_REF_A_BIAS,&check[0]);
       Serial.println("0.75G Clock");  
       dtune_CD = 13;
       dtune_AB = 13;
       write_USART(5,5,ADC_ref6,dtune_AB,&check[0]);
       write_USART(5,4,dtune_CD,dfine_AB,&check[0]);  
       }       

// If the ASCII command is('t') then bypass the PLL
       if (ASCII_cmd == 't') {
       clock_sel = 8;
       write_USART(5,10,clock_sel,ADC_REF_A_BIAS,&check[0]);
       Serial.println("EXT Clock");                                 
       }       


// If the ASCII command is('t') then bypass the PLL
       if (ASCII_cmd == '8') {
             Serial.println("SYNTH CAL");
             trimcap=0;
             vco_on =1;
             digitalWrite(CSTAR,HIGH);
             delay(10);
             write_USART(5,17,trimcap,vco_on,&check[0]);
             readvalue=analogRead(clk_vctrl);             
             while(((readvalue>290) || (readvalue<30)) && (trimcap<16)){
             trimcap=trimcap+1;
             write_USART(5,17,trimcap,vco_on,&check[0]);
             readvalue=analogRead(clk_vctrl);             
             Serial.print("cap:");
             Serial.print(trimcap,DEC);                                        
             Serial.print(" vctrl:");
             Serial.println(readvalue,DEC);                                            
             }
             digitalWrite(CSTAR,LOW);
       Serial.println("CLK CAL DONE.");
       Serial.print("VCO:  ");
       Serial.print(trimcap,DEC);   
       Serial.println("*");
       }       

  
//-------------------------------------------------------------------------------------------------
//                           ANALOG DYNAMICS
//-------------------------------------------------------------------------------------------------
if (ASCII_cmd == '7') {
  Serial.println(" ");
  Serial.println("----ANALOG CAL THA-----");

                //Select ADC THA Common mode sensor monitor
                CM_select    = 0;  // Selects CM Mode Readout              
                write_USART(5,0,THA,CM_select,&check[0]);                                
                //Start at 255 on the THA
                THA=255;
                write_USART(5,0,THA,CM_select,&check[0]);
                delay(1000);
                readvalue=analogRead(CM_mon);    
                readvalue=analogRead(CM_mon);    
                readvalue=analogRead(CM_mon);    
                readvalue=analogRead(CM_mon);                    
                readvalue=analogRead(CM_mon);    
                          
                      

                while ((readvalue > CM_opt) && (THA >0)){
                THA=THA-1;
                write_USART(5,0,THA,CM_select,&check[0]);
                delay(1);
                readvalue=analogRead(CM_mon);            
                 Serial.print("THA:");
                Serial.print(THA,DEC);
                 Serial.print(" CM:");
                Serial.println(readvalue,DEC);
                }

  Serial.println("----ANALOG CAL PRE-----");
                //Select ADC PreAmp
                CM_select    = 1;  // Selects PREAMP Mode Readout              
                write_USART(5,0,THA,CM_select,&check[0]);
                
                
                //Start at 0 on the PREAMP
                PREAMP=0;
                write_USART(5,1,delay_B,PREAMP,&check[0]);
                readvalue=analogRead(CM_mon);            

                while ((readvalue > PE_opt) && (PREAMP < 255)){
                PREAMP=PREAMP+1;
                write_USART(5,1,delay_B,PREAMP,&check[0]);
                //delay(1);
                readvalue=analogRead(CM_mon);            
                Serial.print("PRE:");
                Serial.print(PREAMP,DEC);
                Serial.print(" PCM:");
                Serial.println(readvalue,DEC);              
           
                }
                Serial.println(" ");
                Serial.println("ANA CAL DONE.");
                Serial.print("THA:");
                Serial.print(THA,DEC);
                Serial.print(" PRE:");
                Serial.print(PREAMP,DEC);
                Serial.println("*");
               
                

}
   
//-------------------------------------------------------------------------------------------------
//                           ANALOG POWER DOWN
//-------------------------------------------------------------------------------------------------
       if (ASCII_cmd == '9') {
       write_USART(5,17,0,0,&check[0]);
       write_USART(5,0,0,0,&check[0]);
       write_USART(5,1,0,0,&check[0]);
       write_USART(5,2,0,0,&check[0]);
       write_USART(5,3,0,0,&check[0]);
       write_USART(5,4,0,0,&check[0]);
       write_USART(5,5,0,0,&check[0]);
       write_USART(5,6,0,0,&check[0]);
       write_USART(5,7,0,0,&check[0]);
       write_USART(5,8,0,0,&check[0]);
       write_USART(5,9,0,0,&check[0]);
       write_USART(5,10,0,0,&check[0]);
       write_USART(5,11,0,0,&check[0]);
       write_USART(5,12,0,0,&check[0]);
       write_USART(5,13,0,0,&check[0]);
       write_USART(5,14,0,0,&check[0]);
       write_USART(5,16,0,0,&check[0]);      
       Serial.println("POWER DOWN");                                 
       }      



//-------------------------------------------------------------------------------------------------
//                           PROCESSOR CONFIGURATION COMMANDS
//-------------------------------------------------------------------------------------------------
// If the ASCII command is('u') then turn on hanning window
       if (ASCII_cmd == 'u') {
       hnw_en = 0;
       write_USART(5,20,0,hnw_en,&check[0]);
       Serial.println("HAN off");                                 
       }       

// If the ASCII command is('v') then turn on hanning window
       if (ASCII_cmd == 'v') {
       hnw_en = 1;
       write_USART(5,20,0,hnw_en,&check[0]);
       Serial.println("HAN on");                                 
       }       

// If the ASCII command is('w') then turn off debug mode
       if (ASCII_cmd == 'v') {
       debug_en = 0;
       write_USART(5,19,debug_en,multi_pixel,&check[0]);
       Serial.println("DEBUG off");                                 
       }       

// If the ASCII command is('x') then turn on debug mode
       if (ASCII_cmd == 'x') {
       debug_en = 1;
       write_USART(5,19,debug_en,multi_pixel,&check[0]);
       Serial.println("DEBUG on");                                 
       }       

//-------------------------------------------------------------------------------------------------
//                           EXTERNAL TIMING LOAD FOR CALIBRATION
//-------------------------------------------------------------------------------------------------
// If the ASCII command is('y') then update the clock configuration
  if (ASCII_cmd == 'y') {                    
                   while (Serial.available() < 12) {
                   }                     
                    
              dfine_CD     = Serial.read(); // 8 bit  address 3H         //Sets the fine tune clock delay of ADC 2's THA and comparator
              dfine_AB     = Serial.read(); // 8 bit  address 4L         //Sets the fine tune clock delay of ADC 1's THA and comparator
              dtune_CD     = Serial.read(); // 4 bit  address 4H         //Sets the coarse tune clock delay of DEMUX CD (data coming from ADC 2)
              dtune_AB     = Serial.read(); // 4 bit  address 5L         //Sets the coarse tune clock delay of DEMUX AB (data coming from ADC 1)
              retime_AB2   = Serial.read(); // 8 bit  address 6L         //Sets the fine tune clock delay of retimer AB stage 2 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX and previous stage retimer over PVT)
              retime_AB1   = Serial.read(); // 8 bit  address 6H         //Sets the fine tune clock delay of retimer AB stage 1 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX over PVT)
              retime_CD2   = Serial.read(); // 8 bit  address 7L         //Sets the fine tune clock delay of retimer CD stage 2 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX and previous stage retimer over PVT)
              retime_CD1   = Serial.read(); // 8 bit  address 7H         //Sets the fine tune clock delay of retimer CD stage 1 (ideally same as DEMUX fine tune setting, there is fixed clock delay stage to match the delay of DEMUX over PVT)
              demuxf_AB    = Serial.read(); // 8 bit  address 8L         //Sets the fine tune clock delay of DEMUX CD (data coming from ADC 2)
              demuxf_CD    = Serial.read(); // 8 bit  address 8H         //Sets the fine tune clock delay of DEMXU AB (data coming from ADC 1)              
              dec_coarse   = Serial.read(); // 5 bit  address 10H        //Sets the coarse tune clock delay of decoder //default 15
              dec_fine     = Serial.read(); // 8 bit  address 11L        //Sets the fine tune clock delay of decoder /default 50
                      
              write_USART(5,3,dfine_CD,delay_AN,&check[0]);
              write_USART(5,4,dtune_CD,dfine_AB,&check[0]);
              write_USART(5,5,ADC_ref6,dtune_AB,&check[0]);
              write_USART(5,11,retime_AB1,retime_AB2,&check[0]);
              write_USART(5,12,retime_CD1,retime_CD2,&check[0]);
              write_USART(5,13,demuxf_CD,demuxf_AB,&check[0]);
              write_USART(5,15,dec_coarse,RO_clk_div_fact,&check[0]);
              write_USART(5,16,demux_reset,dec_fine,&check[0]);
                                              
                    }



//-------------------------------------------------------------------------------------------------
//                           EEPROM CALIBRATION SAVE
//-------------------------------------------------------------------------------------------------
// If the ASCII command ! then flash the EEPROM with the timing information

  if (ASCII_cmd == '#') {                    
                   if (clock_sel==1){
                    EEPROM.update(1,THA);
                    EEPROM.update(2,PREAMP);
                    EEPROM.update(3,delay_B);
                    EEPROM.update(4,delay_BN);
                    EEPROM.update(5,delay_A);
                    EEPROM.update(6,delay_AN);
                    EEPROM.update(7,dfine_CD);
                    EEPROM.update(8,dfine_AB);
                    EEPROM.update(9,dtune_CD);
                    EEPROM.update(10,dtune_AB);
                    EEPROM.update(11,retime_AB2);
                    EEPROM.update(12,retime_AB1);
                    EEPROM.update(13,retime_CD2);
                    EEPROM.update(14,retime_CD1);
                    EEPROM.update(15,demuxf_AB);
                    EEPROM.update(16,demuxf_CD);
                    EEPROM.update(17,dec_coarse);
                    EEPROM.update(18,dec_fine);
                    EEPROM.update(19,ADC_ref6);
                    EEPROM.update(20,ADC_ref5);
                    EEPROM.update(21,ADC_ref4);
                    EEPROM.update(22,ADC_ref3);
                    EEPROM.update(23,ADC_ref2);
                    EEPROM.update(24,ADC_ref1);
                    EEPROM.update(25,ADC_ref0);
                    EEPROM.update(26,ADC_REF_R_BIAS);
                    EEPROM.update(27,ADC_REF_B_BIAS);
                    EEPROM.update(28,ADC_REF_A_BIAS);
                    EEPROM.update(29,trimcap);                    
                    }
                   if (clock_sel==2){
                    EEPROM.update(30,THA);
                    EEPROM.update(31,PREAMP);
                    EEPROM.update(32,delay_B);
                    EEPROM.update(33,delay_BN);
                    EEPROM.update(34,delay_A);
                    EEPROM.update(35,delay_AN);
                    EEPROM.update(36,dfine_CD);
                    EEPROM.update(37,dfine_AB);
                    EEPROM.update(38,dtune_CD);
                    EEPROM.update(39,dtune_AB);
                    EEPROM.update(40,retime_AB2);
                    EEPROM.update(41,retime_AB1);
                    EEPROM.update(42,retime_CD2);
                    EEPROM.update(43,retime_CD1);
                    EEPROM.update(44,demuxf_AB);
                    EEPROM.update(45,demuxf_CD);
                    EEPROM.update(46,dec_coarse);
                    EEPROM.update(47,dec_fine);
                    EEPROM.update(48,ADC_ref6);
                    EEPROM.update(49,ADC_ref5);
                    EEPROM.update(50,ADC_ref4);
                    EEPROM.update(51,ADC_ref3);
                    EEPROM.update(52,ADC_ref2);
                    EEPROM.update(53,ADC_ref1);
                    EEPROM.update(54,ADC_ref0);
                    EEPROM.update(55,ADC_REF_R_BIAS);
                    EEPROM.update(56,ADC_REF_B_BIAS);
                    EEPROM.update(57,ADC_REF_A_BIAS);
                    EEPROM.update(58,trimcap);                    
                    }
                   if (clock_sel==4){
                    EEPROM.update(59,THA);
                    EEPROM.update(60,PREAMP);
                    EEPROM.update(61,delay_B);
                    EEPROM.update(62,delay_BN);
                    EEPROM.update(63,delay_A);
                    EEPROM.update(64,delay_AN);
                    EEPROM.update(65,dfine_CD);
                    EEPROM.update(66,dfine_AB);
                    EEPROM.update(67,dtune_CD);
                    EEPROM.update(68,dtune_AB);
                    EEPROM.update(69,retime_AB2);
                    EEPROM.update(70,retime_AB1);
                    EEPROM.update(71,retime_CD2);
                    EEPROM.update(72,retime_CD1);
                    EEPROM.update(73,demuxf_AB);
                    EEPROM.update(74,demuxf_CD);
                    EEPROM.update(75,dec_coarse);
                    EEPROM.update(76,dec_fine);
                    EEPROM.update(77,ADC_ref6);
                    EEPROM.update(78,ADC_ref5);
                    EEPROM.update(79,ADC_ref4);
                    EEPROM.update(80,ADC_ref3);
                    EEPROM.update(81,ADC_ref2);
                    EEPROM.update(82,ADC_ref1);
                    EEPROM.update(83,ADC_ref0);
                    EEPROM.update(84,ADC_REF_R_BIAS);
                    EEPROM.update(85,ADC_REF_B_BIAS);
                    EEPROM.update(86,ADC_REF_A_BIAS);
                    EEPROM.update(87,trimcap);                    
                    }
                    delay(1000);         
                    
                    
                    digitalWrite(LED, HIGH);
                    delay(300);
                    digitalWrite(LED, LOW);
                    
                    delay(1000);         
                    
                    digitalWrite(LED, HIGH);
                    delay(300);
                    digitalWrite(LED, LOW);
                    delay(300);
                    digitalWrite(LED, HIGH);
                    delay(300);
                    digitalWrite(LED, LOW);

                    delay(1000);         
                    digitalWrite(LED, HIGH);
                    delay(300);
                    digitalWrite(LED, LOW);
                    delay(300);
                    digitalWrite(LED, HIGH);
                    delay(300);
                    digitalWrite(LED, LOW);                    
                    delay(300);
                    digitalWrite(LED, HIGH);
                    delay(300);
                    digitalWrite(LED, LOW);
                    
  }
       

 if (ASCII_cmd == '$') {                    
                   if (clock_sel==1){
                    THA            = EEPROM.read(1);
                    PREAMP         = EEPROM.read(2);
                    delay_B        = EEPROM.read(3);
                    delay_BN       = EEPROM.read(4);
                    delay_A        = EEPROM.read(5);
                    delay_AN       = EEPROM.read(6);
                    dfine_CD       = EEPROM.read(7);
                    dfine_AB       = EEPROM.read(8);
                    dtune_CD       = EEPROM.read(9);
                    dtune_AB       = EEPROM.read(10);
                    retime_AB2     = EEPROM.read(11);
                    retime_AB1     = EEPROM.read(12);
                    retime_CD2     = EEPROM.read(13);
                    retime_CD1     = EEPROM.read(14);
                    demuxf_AB      = EEPROM.read(15);
                    demuxf_CD      = EEPROM.read(16);
                    dec_coarse     = EEPROM.read(17);
                    dec_fine       = EEPROM.read(18);
                    ADC_ref6       = EEPROM.read(19);
                    ADC_ref5       = EEPROM.read(20);
                    ADC_ref4       = EEPROM.read(21);
                    ADC_ref3       = EEPROM.read(22);
                    ADC_ref2       = EEPROM.read(23);
                    ADC_ref1       = EEPROM.read(24);
                    ADC_ref0       = EEPROM.read(25);
                    ADC_REF_R_BIAS = EEPROM.read(26);
                    ADC_REF_B_BIAS = EEPROM.read(27);
                    ADC_REF_A_BIAS = EEPROM.read(28);
                    trimcap        = EEPROM.read(29);
                   }
                                                    
                   if (clock_sel==2){
                    THA            = EEPROM.read(30);
                    PREAMP         = EEPROM.read(31);
                    delay_B         = EEPROM.read(32);
                    delay_BN        = EEPROM.read(33);
                    delay_A         = EEPROM.read(34);
                    delay_AN        = EEPROM.read(35);
                    dfine_CD       = EEPROM.read(36);
                    dfine_AB       = EEPROM.read(37);
                    dtune_CD       = EEPROM.read(38);
                    dtune_AB       = EEPROM.read(39);
                    retime_AB2     = EEPROM.read(40);
                    retime_AB1     = EEPROM.read(41);
                    retime_CD2     = EEPROM.read(42);
                    retime_CD1     = EEPROM.read(43);
                    demuxf_AB      = EEPROM.read(44);
                    demuxf_CD      = EEPROM.read(45);
                    dec_coarse     = EEPROM.read(46);
                    dec_fine       = EEPROM.read(47);
                    ADC_ref6       = EEPROM.read(48);
                    ADC_ref5       = EEPROM.read(49);
                    ADC_ref4       = EEPROM.read(50);
                    ADC_ref3       = EEPROM.read(51);
                    ADC_ref2       = EEPROM.read(52);
                    ADC_ref1       = EEPROM.read(53);
                    ADC_ref0       = EEPROM.read(54);
                    ADC_REF_R_BIAS = EEPROM.read(55);
                    ADC_REF_B_BIAS = EEPROM.read(56);
                    ADC_REF_A_BIAS = EEPROM.read(57);
                    trimcap        = EEPROM.read(58);                    
                    }

                  if (clock_sel==4){
                    THA            = EEPROM.read(59);
                    PREAMP         = EEPROM.read(60);
                    delay_B         = EEPROM.read(61);
                    delay_BN        = EEPROM.read(62);
                    delay_A         = EEPROM.read(63);
                    delay_AN        = EEPROM.read(64);
                    dfine_CD       = EEPROM.read(65);
                    dfine_AB       = EEPROM.read(66);
                    dtune_CD       = EEPROM.read(67);
                    dtune_AB       = EEPROM.read(68);
                    retime_AB2     = EEPROM.read(69);
                    retime_AB1     = EEPROM.read(70);
                    retime_CD2     = EEPROM.read(71);
                    retime_CD1     = EEPROM.read(72);
                    demuxf_AB      = EEPROM.read(73);
                    demuxf_CD      = EEPROM.read(74);
                    dec_coarse     = EEPROM.read(75);
                    dec_fine       = EEPROM.read(76);
                    ADC_ref6       = EEPROM.read(77);
                    ADC_ref5       = EEPROM.read(78);
                    ADC_ref4       = EEPROM.read(79);
                    ADC_ref3       = EEPROM.read(80);
                    ADC_ref2       = EEPROM.read(81);
                    ADC_ref1       = EEPROM.read(82);
                    ADC_ref0       = EEPROM.read(83);
                    ADC_REF_R_BIAS = EEPROM.read(84);
                    ADC_REF_B_BIAS = EEPROM.read(85);
                    ADC_REF_A_BIAS = EEPROM.read(86);
                    trimcap        = EEPROM.read(87);                    
                    }

       write_USART(5,0,THA,CM_select,&check[0]);
       write_USART(5,1,delay_B,PREAMP,&check[0]);
       write_USART(5,2,delay_A,delay_BN,&check[0]);
       write_USART(5,3,dfine_CD,delay_AN,&check[0]);
       write_USART(5,4,dtune_CD,dfine_AB,&check[0]);
       write_USART(5,5,ADC_ref6,dtune_AB,&check[0]);
       write_USART(5,6,ADC_ref4,ADC_ref5,&check[0]);
       write_USART(5,7,ADC_ref2,ADC_ref3,&check[0]);
       write_USART(5,8,ADC_ref0,ADC_ref1,&check[0]);
       write_USART(5,9,ADC_REF_B_BIAS,ADC_REF_R_BIAS,&check[0]);
       write_USART(5,10,clock_sel,ADC_REF_A_BIAS,&check[0]);
       write_USART(5,11,retime_AB1,retime_AB2,&check[0]);
       write_USART(5,12,retime_CD1,retime_CD2,&check[0]);
       write_USART(5,13,demuxf_CD,demuxf_AB,&check[0]);
       write_USART(5,14,RO_clk_div_enab,divider_res,&check[0]);
       write_USART(5,15,dec_coarse,RO_clk_div_fact,&check[0]);
       write_USART(5,16,demux_reset,dec_fine,&check[0]);
       write_USART(5,17,trimcap,vco_on,&check[0]);
       write_USART(5,18,DAC_AB_SEL, DAC_CD_SEL,&check[0]);
       write_USART(5,19,debug_en,multi_pixel,&check[0]);
       write_USART(5,20,0,hnw_en,&check[0]);
       write_USART(5,22,averagesLH,averagesLL,&check[0]);
       write_USART(5,23,averagesHH,averagesHL,&check[0]);  
                    
  }




// If the ASCII command is('z') then update the averages setting
  if (ASCII_cmd == 'z') {                    
                   while (Serial.available() < 4) {
                   }                     
                      averagesHH = Serial.read(); //[D00-D07] // 8 bit address 14L   //Controls the amount of observation time before the core flushes to the readout
                      averagesHL = Serial.read(); //[D08-D15] // 8 bit address 14H   //NOTE! there is no internal overflow protection
                      averagesLH = Serial.read(); //[D16-D23] // 8 bit address 15L
                      averagesLL = Serial.read(); //[D24-D31] // 8 bit address 15H                
                      write_USART(5,22,averagesLH,averagesLL,&check[0]);
                      write_USART(5,23,averagesHH,averagesHL,&check[0]);  
                      Serial.print("AVG: ");
                      Serial.print(averagesHH,DEC);
                      Serial.print(" ");
                      Serial.print(averagesHL,DEC);
                      Serial.print(" ");
                      Serial.print(averagesLH,DEC);
                      Serial.print(" ");
                      Serial.println(averagesLL,DEC);                     
                      }
//-------------------------------------------------------------------------------------------------
//                           Interleave Alignment
//-------------------------------------------------------------------------------------------------

 if (ASCII_cmd == '6') {                    
                   phase_th=150;                                        
                   phase0=0;
                   phase1=0;
                   phase2=0;
                   phase3=0;
                   phase4=0;
                   phase_start=0;
                   phase_stop=0;
                   for (ctr = 0;ctr<64;ctr++){                   
                   delay_A=ctr;  
                   delay_AN=ctr; 
                   write_USART(5,2,delay_A,delay_BN,&check[0]);
                   write_USART(5,3,dfine_CD,delay_AN,&check[0]);
                   
                   phase0=phase1;
                   phase1=phase2;
                   phase2=phase3;
                   phase3=phase4;
                   phase4=analogRead(AP_mon);    

                  if ((phase0<=phase_th) && (phase1<=phase_th) && (phase2<=phase_th) && (phase3>=phase_th) && (phase4>=phase_th)){
                    phase_start = ctr;
                   }
                                  
                   if ((phase0>=phase_th) && (phase1>=phase_th) && (phase2>=phase_th) && (phase3<=phase_th) && (phase4<=phase_th)){
                    phase_stop = ctr;
                   }                                                      
                   }
                   Serial.print("P-start:");
                   Serial.println(phase_start,DEC);  
                   Serial.print("P-start:");
                   Serial.println(phase_stop,DEC);  
                   phase_sel = phase_start + (phase_stop-phase_start)/2;
                   Serial.print("P-sel:");
                   Serial.print(phase_sel,DEC);  
                   Serial.println("*");
                   }









//-------------------------------------------------------------------------------------------------
//                           PROCESSOR SEQEUNCING COMMANDS
//-------------------------------------------------------------------------------------------------

// If the ASCII command is('c') then do a sequencer reset
       if (ASCII_cmd == 'c') {
       digitalWrite(ROSTAR,LOW);
       digitalWrite(CSTAR,LOW);
       Serial.println("SEQ RESET");                                 
       }
       
// If the ASCII command is('d') then do core start
       if (ASCII_cmd == 'd') {
       digitalWrite(CSTAR,HIGH);
       Serial.println("SEQ CSTART");                                 
       }

// If the ASCII command is('e') then do readout start
       if (ASCII_cmd == 'e') {
       digitalWrite(ROSTAR,HIGH);
       Serial.println("SEQ RSTART");                                 
       }       

// If the ASCII command is('f') then do one full acqusition
       if (ASCII_cmd == 'f') {

        // Reset the sequencer 
        digitalWrite(ROSTAR,LOW);
        digitalWrite(CSTAR,LOW);
        Serial.println("S-RES");                                         

        //Wait until CDONE =0  (core is ready)
        readvalue = digitalRead(CDONE);
        while (readvalue == HIGH){
        readvalue = digitalRead(CDONE);                         
        }       
       
        //Start the SIV core
        digitalWrite(CSTAR,HIGH);
        Serial.println("S-CS");                                           

        //Wait until CDONE =1  (core is done)
        readvalue = digitalRead(CDONE);
        while (readvalue == LOW){
        readvalue = digitalRead(CDONE);                         
        }       
        Serial.println("S-CD");                                           
  
       //Start the SIV readout
        digitalWrite(ROSTAR,HIGH);  
        Serial.println("S-RS");                                           

        //Wait until RODONE =1  (readout is done)
        readvalue = digitalRead(RODONE);
        while (readvalue == LOW){
        readvalue = digitalRead(RODONE);                         
        }       
        Serial.println("S-RD");                                                                           
        
                
        // Reset the sequencer 
        digitalWrite(ROSTAR,LOW);
        digitalWrite(CSTAR,LOW);
        Serial.println("S-RES");                                                          
        }       
              
       
     
}
         
                   
//-----------------------------------------------------------------------------------------------------------------------------------------------
//                                             SPI Function Which Updates all the SVII Config Registers
//-----------------------------------------------------------------------------------------------------------------------------------------------                   
void update_SOC_config(){
byte check[3];  //Data register for SPI contents returned from the SOC  
       write_USART(5,0,THA,CM_select,&check[0]);
       write_USART(5,1,delay_B,PREAMP,&check[0]);
       write_USART(5,2,delay_A,delay_BN,&check[0]);
       write_USART(5,3,dfine_CD,delay_AN,&check[0]);
       write_USART(5,4,dtune_CD,dfine_AB,&check[0]);
       write_USART(5,5,ADC_ref6,dtune_AB,&check[0]);
       write_USART(5,6,ADC_ref4,ADC_ref5,&check[0]);
       write_USART(5,7,ADC_ref2,ADC_ref3,&check[0]);
       write_USART(5,8,ADC_ref0,ADC_ref1,&check[0]);
       write_USART(5,9,ADC_REF_B_BIAS,ADC_REF_R_BIAS,&check[0]);
       write_USART(5,10,clock_sel,ADC_REF_A_BIAS,&check[0]);
       write_USART(5,11,retime_AB1,retime_AB2,&check[0]);
       write_USART(5,12,retime_CD1,retime_CD2,&check[0]);
       write_USART(5,13,demuxf_CD,demuxf_AB,&check[0]);
       write_USART(5,14,RO_clk_div_enab,divider_res,&check[0]);
       write_USART(5,15,dec_coarse,RO_clk_div_fact,&check[0]);
       write_USART(5,16,demux_reset,dec_fine,&check[0]);
       write_USART(5,17,trimcap,vco_on,&check[0]);
       write_USART(5,18,DAC_AB_SEL, DAC_CD_SEL,&check[0]);
       write_USART(5,19,debug_en,multi_pixel,&check[0]);
       write_USART(5,20,0,hnw_en,&check[0]);
       write_USART(5,22,averagesLH,averagesLL,&check[0]);
       write_USART(5,23,averagesHH,averagesHL,&check[0]);  
}                   







//----------------------------------------------------------LOW LEVEL USART3 FUNCTION-----------------------------------------------------------------------------------------
void write_USART(byte command, byte address, byte dataH, byte dataL, byte *resultdata) {

    
digitalWrite(LED, HIGH);    // turn the LED on to show we have USART activity

//Variables and Constants
const int dfact=0;         //Delay time between transitions on the USART interface
int RX_bit=0;              //Start bit flag for slave from the SoC

int SOC_DATA_OUT[33];   //integer format //Master uC-to-SoC Packet Structure
boolean SOC_OUT[33];    //binary format

int SOC_DATA_IN[25];   //integer format //Master SoC-to-Uc Packet Structure
boolean SOC_IN[25];    //binary format

int ctr = 0; // Counter for packet copying between interger and binary format
  

//DATA PACKET FORMAT 
//USART  FFFFFFFF AAAAAAAA DDDDDDDD DDDDDDDD
// F-Command A-Address D-Data


//----------------Convert from byte format to an integer format.
//Put the Start Bit on the Front of the Packet
SOC_DATA_OUT[33]=1;



//Function Select
SOC_DATA_OUT[32]=(command>>7) & (1);
SOC_DATA_OUT[31]=(command>>6) & (1);
SOC_DATA_OUT[30]=(command>>5) & (1);
SOC_DATA_OUT[29]=(command>>4) & (1);
SOC_DATA_OUT[28]=(command>>3) & (1);
SOC_DATA_OUT[27]=(command>>2) & (1);
SOC_DATA_OUT[26]=(command>>1) & (1);
SOC_DATA_OUT[25]=(command) & (1);


//Address Select
SOC_DATA_OUT[24]=(address>>7) & (1);
SOC_DATA_OUT[23]=(address>>6) & (1);
SOC_DATA_OUT[22]=(address>>5) & (1);
SOC_DATA_OUT[21]=(address>>4) & (1);
SOC_DATA_OUT[20]=(address>>3) & (1);
SOC_DATA_OUT[19]=(address>>2) & (1);
SOC_DATA_OUT[18]=(address>>1) & (1);
SOC_DATA_OUT[17]=(address) & (1);

//Data High Bit
SOC_DATA_OUT[16]=(dataH>>7) & (1);
SOC_DATA_OUT[15]=(dataH>>6) & (1);
SOC_DATA_OUT[14]=(dataH>>5) & (1);
SOC_DATA_OUT[13]=(dataH>>4) & (1);
SOC_DATA_OUT[12]=(dataH>>3) & (1);
SOC_DATA_OUT[11]=(dataH>>2) & (1);
SOC_DATA_OUT[10]=(dataH>>1) & (1);
SOC_DATA_OUT[9]=(dataH) & (1);

//Data Low bit
SOC_DATA_OUT[8]=(dataL>>7) & (1);
SOC_DATA_OUT[7]=(dataL>>6) & (1);
SOC_DATA_OUT[6]=(dataL>>5) & (1);
SOC_DATA_OUT[5]=(dataL>>4) & (1);
SOC_DATA_OUT[4]=(dataL>>3) & (1);
SOC_DATA_OUT[3]=(dataL>>2) & (1);
SOC_DATA_OUT[2]=(dataL>>1) & (1);
SOC_DATA_OUT[1]=(dataL) & (1);



//CONVERT each of the integer formats in SOC_DATA_OUT[] to binary formats SOC_OUT[] for writing to the pins
for (ctr = 0; ctr<33; ctr++) {
if (SOC_DATA_OUT[ctr] == 0)
  {SOC_OUT[ctr]=LOW;}
  else
  {SOC_OUT[ctr]=HIGH;}

}




//-----uC-to-SoC-PACKET-STARTS-HERE------------------------------------------------------------------------------------------------------------

//Reset the slave start big flag to receive a new packet
RX_bit=0;


//Idle for a few clock cycles before the packet begins

 digitalWrite(UMX, LOW); 
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
  digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 
 
 
 // Begin sending the packet

 //-----------------------START BIT-------------------------
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, HIGH);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 

  
 //-----------------------Function Command------------------------  
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[32]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact);
  
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[31]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact); 
  
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[30]);
// delay(dfact); 
 digitalWrite(UCK, HIGH); 
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[29]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[28]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact); 
  
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[27]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact);
  
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[26]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact); 
  
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[25]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
   
   
 //-----------------------Address Code-------------------------


digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[24]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[23]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[22]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[21]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);

  digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[20]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[19]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[18]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[17]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 //-----------------------------DATA Upper Byte-----------------------------
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[16]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[15]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[14]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[13]);
// delay(dfact);
 digitalWrite(UCK, HIGH);
// delay(dfact);
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[12]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact); 
  
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[11]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact);
  
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[10]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact); 
  
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[9]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);


 //-----------------------------DATA lower Byte-----------------------------
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[8]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact); 
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[7]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact); 
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[6]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact);

 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[5]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact); 
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[4]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact);
  
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[3]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact); 
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[2]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact); 
 
 digitalWrite(UCK, LOW);
// delay(dfact);
 digitalWrite(UMX, SOC_OUT[1]);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact); 
 
digitalWrite(UCK, LOW);
// delay(dfact);
digitalWrite(UMX, LOW);
// delay(dfact); 
digitalWrite(UCK, HIGH);
// delay(dfact); 
 
 
 //---------------------SoC-to-uC PACKET STARTS HERE------------------------------------
 //DATA PACKET FORMAT (3 bytes total)
//USART   S FFFFFFFF DDDDDDDD DDDDDDDD
//S-Start F-Function D-Data
 
 
 //First wait for the packet to start indicated by the first "1" on 
 //the USX pin. Until we see the start bit, just keep sending clocks

 while (RX_bit == LOW) {
 digitalWrite(UCK, HIGH);
// delay(dfact);
 RX_bit= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
    }  






 //------------Receive the function Packet (byte 2)-----------
 digitalWrite(UCK, HIGH);
  SOC_IN[24]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[23]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[22]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[21]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[20]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[19]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[18]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[17]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 //--------------------Receive the upper data Packet (byte 1)-----------
 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[16]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[15]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[14]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[13]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[12]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[11]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[10]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[9]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 


 //--------------------Receive the lower data Packet (byte 0)-----------
 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[8]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[7]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[6]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[5]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[4]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[3]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[2]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[1]= digitalRead(USX);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 

 digitalWrite(UCK, HIGH);
// delay(dfact);
 SOC_IN[0]= digitalRead(USX);
// delay(dfact);




//Put extra clocks after the packet
for (ctr=0;ctr<50;ctr++){
 digitalWrite(UCK, LOW);
// delay(dfact); 
 digitalWrite(UCK, HIGH);
// delay(dfact);
 digitalWrite(UCK, LOW);
// delay(dfact); 
}


//CONVERT each of the binary formats in SOC_IN[] to integer formats SOC_DATA_IN[] for writing to ASCII back to the PC
 for (ctr = 0; ctr<25; ctr++) {
  if (SOC_IN[ctr] == LOW)
  {SOC_DATA_IN[ctr]=0;}
  else
  {SOC_DATA_IN[ctr]=1;}
}

//For packet debugging
//Serial.println(" ");
//for (ctr = 24; ctr>0; ctr--) {
//Serial.print(SOC_DATA_IN[ctr]);
//}
//Serial.println(" ");



//Assemble the returned integer representations of each bit back into bytes for return
SOC_RETURN[2]=SOC_DATA_IN[24]*128 + SOC_DATA_IN[23]*64 + SOC_DATA_IN[22]*32 + SOC_DATA_IN[21]*16 + SOC_DATA_IN[20]*8 + SOC_DATA_IN[19]*4 + SOC_DATA_IN[18]*2 + SOC_DATA_IN[17];
SOC_RETURN[1]=SOC_DATA_IN[16]*128 + SOC_DATA_IN[15]*64 + SOC_DATA_IN[14]*32 + SOC_DATA_IN[13]*16 + SOC_DATA_IN[12]*8 + SOC_DATA_IN[11]*4 + SOC_DATA_IN[10]*2 + SOC_DATA_IN[9];
SOC_RETURN[0]=SOC_DATA_IN[8]*128 + SOC_DATA_IN[7]*64 + SOC_DATA_IN[6]*32 + SOC_DATA_IN[5]*16 + SOC_DATA_IN[4]*8 + SOC_DATA_IN[3]*4 + SOC_DATA_IN[2]*2 + SOC_DATA_IN[1];

digitalWrite(LED, LOW);    // turn off the light once the communication is over

//return the data to the calling function
resultdata[0] = SOC_RETURN[0];
resultdata[1] = SOC_RETURN[1];
resultdata[2] = SOC_RETURN[2];

}
                   
