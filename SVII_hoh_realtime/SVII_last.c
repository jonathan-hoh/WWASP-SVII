// -----------------------------------------------------------
//              Acqusition Support for SVII-C
//              Adrian Tang JPL/UCLA 2018 R7 V3.00
//              Written Apr 6 2018
// -----------------------------------------------------------

// Standard RPI Include Libraries
#include <inttypes.h>
#include <stdio.h>
#include <tgmath.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include "./ftd2xx.h"




//Defines the baud rate of the USB FTDI Link for the control port
#define BAUDRATE B57600

//Define the buffer size for the FTL interface
#define BUF_SIZE 0x100000 // 1 Megabyte Data Buffer



//Define the GNUplot program
#define GNUPLOT "gnuplot"




//this function is for printing to GNU plot with 64 bit integers unsigned
void gnuprint(FILE *gp, uint64_t x[], int N)
{
int i;
fprintf(gp, "set logscale y \n");
fprintf(gp, "plot '-' with lines \n");
for (i=0;i<N;i++){fprintf(gp, "%llu $g\n", x[i]);}
fflush(gp);
fprintf(gp, "e\n");
}








//----------------------------------------------------------
//      
// Main initialization code that runs on execute
//
//----------------------------------------------------------

int main(int argc, char ** argv) {
printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
printf("******************************************************************************************************************************\n");
printf("                                     SpectroChip SVII Support R7 V3.00 ajtang Apr 2018                                        \n");
printf("                                                               Here we go!...                                                 \n");
printf("******************************************************************************************************************************\n");



FILE *pagefile;    //page file containing setup values

char buffer [1000];  //Empty buffer for header reading




char data_file [1000];  //Filename for saving spectrometer data
char ctrl_serial[9]; //Control Serial Number
char ctrl_tty [70];  //Control TTY Port

char data_serial[9]; //Data Serial Number



//Averaging and range variables
int HH_average;
int HL_average;
int LH_average;
int LL_average;
int SOFT_average;
double range;
int WIN_enabled;
int clock_source;
int cap_trim;
int RO_prescale;
int debugg;

//This is the data_port for this pixel
int s_data_port;


//----------------------------------------------------------READ PAGE FILE SECTION------------------------------------------------
//
//
//
//----------------------------------------------------------READ PAGE FILE SECTION------------------------------------------------
printf("\n\n");
printf("******************************************************************************************************************************\n");
printf("                                          SYSTEM STATUS                                                                       \n");
printf("******************************************************************************************************************************\n");
printf("Our program is now in kernel state(%i)                                                                                        \n",argc);



printf("\n");
printf("******************************************************************************************************************************\n");
printf("                                          SVII CONFIGURATION                                                                  \n");
printf("******************************************************************************************************************************\n");
//---------Open the config file--------------------
printf("opening: config file: (%s) for read\n",argv[1]);
pagefile = fopen(argv[1],"r");

//---------Read all the header material------------
fscanf(pagefile,"%s",buffer);
//printf("\n  %s",buffer);
fscanf(pagefile,"%s",ctrl_serial);
fscanf(pagefile,"%s",ctrl_tty);
fscanf(pagefile,"%s",buffer);
fscanf(pagefile,"%s",data_serial);
fscanf(pagefile,"%s",buffer);
fscanf(pagefile,"%i %i %i %i %i",&HH_average,&HL_average,&LH_average,&LL_average, &SOFT_average);
fscanf(pagefile,"%s",buffer);
fscanf(pagefile,"%lf",&range);
fscanf(pagefile,"%s",buffer);
fscanf(pagefile,"%s",data_file);
fscanf(pagefile,"%s",buffer);
fscanf(pagefile,"%i",&WIN_enabled);
fscanf(pagefile,"%s",buffer);
fscanf(pagefile,"%i",&clock_source);
fscanf(pagefile,"%s",buffer);
fscanf(pagefile,"%i",&cap_trim);
fscanf(pagefile,"%s",buffer);
fscanf(pagefile,"%i",&RO_prescale);
fscanf(pagefile,"%s",buffer);
fscanf(pagefile,"%i",&debugg);
fclose(pagefile);

printf("Control Port Chip Serial No: %s\n",ctrl_serial);
printf("Control Port TTY: %s\n",ctrl_tty);
printf("Data Port Chip Serial No: %s\n",data_serial);
printf("Averages: %i %i %i %i %i\n",HH_average,HL_average,LH_average,LL_average,SOFT_average);
printf("Range: %lf \n",range);
printf("data file: %s\n",data_file);
printf("clock source: %i\n",clock_source);
printf("Win Enabled: %i\n",WIN_enabled);
printf("Vco trim: %i\n",cap_trim);
printf("Readout Rate: %i\n",RO_prescale);
printf("Debug Mode: %i\n",debugg);



//----------------------------------------------------------FIND THE RIGHT DATA PORT SECTION--------------------------------------
//
//
//
//----------------------------------------------------------FIND THE RIGHT DATA PORT SECTION--------------------------------------







DWORD i;

printf("\n");
printf("******************************************************************************************************************************\n");
printf("                           Searching available devices for the right data port with specified serial                          \n");
printf("******************************************************************************************************************************\n");

//parameters related to searching the USB interfaces
FT_HANDLE ftHandle;
FT_DEVICE ftDevice;
FT_STATUS ftStatus;
DWORD deviceID;

//parameters returned from each device
char SerialNumber[9];
char Description[64];

//number of available devices
int numDevs;

// create the device information list. This function figures out how many devices total are connected to the USB network
ftStatus = FT_CreateDeviceInfoList(&numDevs);
   if (ftStatus == FT_OK) {
   printf("Number of FTDIxxx devices connected is %d\n\n\n",numDevs);
   }



// if more than one device is connected
if (numDevs > 0) {         
           //cycle through each connected device
           for (i = 0; i < numDevs; i++) {
           
		ftStatus = FT_Open(i, &ftHandle);
	        if(ftStatus == FT_OK) { 	   
        	   ftStatus = FT_GetDeviceInfo(ftHandle,&ftDevice,&deviceID,SerialNumber,	Description,NULL);
                   if (ftStatus == FT_OK) {
                          //check against the desired serial number
                             if ((strcmp(SerialNumber,data_serial))==0) {
                             s_data_port = i;
                             printf("*");
                             }//check
                          printf("Device:%i  Chip serial no:%s\n",i,&SerialNumber); 
                          printf("have: %i %i %i %i %i %i %i %i %i\n",SerialNumber[0],SerialNumber[1],SerialNumber[2],SerialNumber[3],SerialNumber[4],SerialNumber[5],SerialNumber[6],SerialNumber[7],SerialNumber[8]);
                          printf("need: %i %i %i %i %i %i %i %i %i\n\n",data_serial[0],data_serial[1],data_serial[2],data_serial[3],data_serial[4],data_serial[5],data_serial[6],data_serial[7],data_serial[8]);
                          }//ok for get device 
                   }//okay for open		       
                 
                else {
                  printf("Device:%i refused to open\n\n",i);
                }//not ok for open           	
                FT_Close(ftHandle);                
           }//for
           
}//device more than 0

printf("Based on (%s), device (%i) is the right data port for (%s)                                                            \n",argv[1],s_data_port,&ctrl_tty);












//----------------------------------------------------------ENVIRONMENT SETUP--------------------------------------
//
//
//
//----------------------------------------------------------ENVIRONMENT SETUP--------------------------------------








printf("\n");
printf("******************************************************************************************************************************\n");
printf("                                Setting up realtime GNU Plotting                                                              \n");
printf("******************************************************************************************************************************\n");
printf("set yrange [0:%lf]\n",range);
printf("plot '-' with lines \n");

    //Open a GNUplot pipe
        FILE *gp;
        gp = popen(GNUPLOT,"w");  /* open the pipe to GNUplot */	
	if (gp==NULL) {
	printf("GNU plot has an issue");
	return(0);
	}
        printf("GNU plot now open with handle %d\n",*gp);
	fprintf(gp,"set term x11\n");
        fflush(gp);


printf("\n");
printf("******************************************************************************************************************************\n");
printf("                                Setting up real-time clock and storage                                                        \n");
printf("******************************************************************************************************************************\n");

//Variables related to timestamp capture from the system
static int seconds_last=99;
char TimeString[128];
time_t curTime;


//Set the path of the output file and open it
FILE *savefile;

savefile = fopen(data_file,"w" );
printf("State: File %s opened for write\n",&data_file);
printf("State: Realtime clock is started\n");






printf("\n");
printf("******************************************************************************************************************************\n");
printf("                                Setting up control communications: %s                                                         \n",ctrl_tty);
printf("******************************************************************************************************************************\n");

// Create a Port Structure for USB COM port
int USB;



  // Open the Port regardless of what DCTS is doing
      USB = open(ctrl_tty, O_RDWR | O_NOCTTY );
       if (USB == -1) {
       perror("open_port: Unable to open -");
       return(-1);
      }
      printf("State: FTDI PORT %s open\n",ctrl_tty);




//define the tty port stucture
    struct termios options;



//get the current port settings
     if (tcgetattr(USB, &options) != 0) {
     perror("get attribute: Unable to get -");
}

      printf("State: PORT attributes loaded\n");



//Set the FTDI Baud Rate
  cfsetispeed(&options, BAUDRATE);
  cfsetospeed(&options, BAUDRATE);
  printf("State: Baudrate set to (8'H%i)\n",BAUDRATE);



  //Set USB port to to 8n1-FTDI configuration
  options.c_cflag &= ~PARENB;   // No Parity
  options.c_cflag &= ~CSTOPB;   // 1 stopbit
  options.c_cflag &= ~CSIZE;    // no masked bits
  options.c_cflag |= CS8;       // 8n1 config

  options.c_cflag &= ~CRTSCTS;  // Turn of DCTS
  options.c_cc[VMIN] = 0;      //blocking disabled
  options.c_cc[VTIME] =10;     //inter-character time delay
  
  options.c_cflag |= CREAD | CLOCAL;   //Local mode ... ignore CTS

  cfmakeraw(&options);                 //Clear any other settings in the termios
  printf("State: makeraw FTDI pipe for direct access set\n");


  tcflush(USB, TCIFLUSH);                //flush the current port
  printf("State: TCIFLUSH: success\n");

  if (tcsetattr(USB, TCSANOW, &options)) {      //update the attributes
     perror("could not set serial port");
     return -99;}
else {
  printf("State: TCsetattr direct pipe config okay\n");
      }
  printf("State: Waiting to hear from the SVII SoC chip \n");
 


 //---------------------------------------SOC INITIALIZATION SECTION ON USART------------------------------------------------

  // Wait for the spectrochip SVII to respond with its happy header (terminates on >)


  char buf[800];
  int n;
  int ctr=0;
  char inchar;
  
   //read from the serial port until we find the terminator
   while (inchar != '>'){
        n = read(USB, (void*)&inchar, 1);                  
        if (n>0){
        buf[ctr] = inchar;}
        ctr=ctr+1;        
        }
        buf[ctr] = '\0';
        printf("\n\n data: %s \n\n",buf);

  

       //wait awhile after the spectrochip SVII header is received so the buffer clears out
       printf("\nState: SVII SoC Detected\n");

          
//----------------------------------------------------------APPLY CONFIG TO THE SVII--------------------------------------
//
//
//
//----------------------------------------------------------APPLY CONFIG TO THE SVII--------------------------------------
sleep(1);
  //---------------------Configure Averages------------------------------------------------
      char averages[5];
      int avgLL;
      int avgLH;
      int avgHL;
      int avgHH;
      avgHH = HH_average;
      avgHL = HL_average;
      avgLH = LH_average;
      avgLL = LL_average;

      averages[0] = 'z';
      averages[1] = avgHH;
      averages[2] = avgHL;     
      averages[3] = avgLH;
      averages[4] = avgLL;
      
      
      n = write(USB,averages,5);
      if (n<0){
      perror("send command failed -");
      }
      printf("State: setting averages: %i %i %i %i\n",avgHH,avgHL,avgLH,avgLL);

sleep(1);
  //---------------------Configure Clock Chain------------------------------------------------
//FULL RATE CLOCK
if(clock_source==0){
 n = write(USB,"i",1);
    if (n<0){
    perror("send command failed :(");
    }  
 n = write(USB,"4",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: setting full rate clock\n");
}
//HALF_RATE_CLOCK
if(clock_source==1){
 n = write(USB,"j",1);
    if (n<0){
    perror("send command failed :(");
    }  
 n = write(USB,"4",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: setting half rate clock\n");
}

//QUARTER_RATE_CLOCK
if(clock_source==2){
 n = write(USB,"k",1);
    if (n<0){
    perror("send command failed :(");
    }  
 n = write(USB,"4",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: setting quarter rate clock\n");
}
//EXTERNAL CLOCK
if(clock_source==3){
 n = write(USB,"t",1);
    if (n<0){
    perror("send command failed :(");
    }  
 n = write(USB,"3",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: setting external clock\n");
}



  //---------------------Configure Readout Clock Chain------------------------------------------------
sleep(1);
if(RO_prescale==0){
    printf("State: standard readout\n");
}
if(RO_prescale==1){
 n = write(USB,"h",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: 2X readout\n");
}
if(RO_prescale==2){
 n = write(USB,"h",1);
    if (n<0){
    perror("send command failed :(");
    }  
 n = write(USB,"h",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: 4X readout\n");
}
if(RO_prescale==3){
 n = write(USB,"h",1);
    if (n<0){
    perror("send command failed :(");
    }  
 n = write(USB,"h",1);
    if (n<0){
    perror("send command failed :(");
    }  
 n = write(USB,"h",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: 8X readout\n");
}


 //---------------------Window Configuration------------------------------------------------
sleep(1);
if(WIN_enabled==1){
    n = write(USB,"v",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: Hanning Window\n");
}
else
{
    n = write(USB,"u",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: Box Window\n");
}




 //---------------------Debug Configuration------------------------------------------------
sleep(1);
if(debugg==1){
    n = write(USB,"x",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: Debug on\n");
}
else
{
    n = write(USB,"w",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: Debug Off\n");
}



 //---------------------Initialization-----------------------------------------------


sleep(1);
//Perform Clock Alignment
n = write(USB,"8",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: Clock Cal\n");
sleep(1);
//Perform Analog Alignment
n = write(USB,"7",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: Analog Cal\n");

sleep(1);
//Perform Analog Alignment
n = write(USB,"6",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: Interleave Cal\n");


sleep(1);
n = write(USB,"c",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: Clean Init\n");

          

sleep(1);
n = write(USB,"$",1);
    if (n<0){
    perror("send command failed :(");
    }  
    printf("State: Timing Loaded from EEPROM\n");

          

sleep(1);
printf("\n");
printf("******************************************************************************************************************************\n");
printf("                                Setting up data pipe communications USB                                                       \n");
printf("******************************************************************************************************************************\n");


 //---------------------------------------SOC INITIALIZATION FOR FTL PORT------------------------------------------------
int iport;
	
	
		iport = s_data_port;   // FTL should be the second port
	
        //OPEN THE FTL PORT
	ftStatus = FT_Open(iport, &ftHandle);
	if(ftStatus != FT_OK) {
	        printf("FT_Open(%d) failed\n", iport);
		return 1;
	        }
                if(ftStatus == FT_OK) {
                printf("State: FTL PORT (%i) supposedly opened okay\n",iport);  
                }

      



 //---------------------------------------DATA COLLECTION PART------------------------------------------------

long queue_size = 0;   // Amount of data currently captured
char databuffer[24576];
long actualread;
int loop = 0;

unsigned int data0;
unsigned int data1;
unsigned int data2;
unsigned int data3;
unsigned int data4;
unsigned int data5;

uint64_t data_out;
uint64_t data_out_accum[4096];
int dctr;

//for some reason the buffer needs to be initialized to zero before we can read from it....
for(dctr=0;dctr<24577;dctr++){
databuffer[dctr]=0;
}


//initialize the software accumulator to zero
for(dctr=1;dctr<4096;dctr++){        //run from 1 to 4096 to avoid the DC
data_out_accum[4096]=0;
}











printf("\n");
printf("******************************************************************************************************************************\n");
printf("                                Start Data Capture Thread                                                                     \n");
printf("******************************************************************************************************************************\n");

while(loop < SOFT_average){



        //Reset the FTL interface 
        ftStatus = FT_ResetDevice(ftHandle);
	        if(ftStatus != FT_OK) {
	        printf("FT_REsetDevice failed\n", iport);
		return 1;
	        }
                if(ftStatus == FT_OK) {
                printf("State: Resync the FTL port\n");  
                }



    //Request one spectrum from the SOC (ASCII command F)
    n = write(USB,"f",1);
    if (n<0){
    perror("send command failed :(");
    }   

    printf("State: Data capture and accumulation in progress\n");
   
     
       //Wait for the full 4096 X 6 bit spectrum to be returned
       while (queue_size < 24576){      
       if (FT_GetQueueStatus(ftHandle,&queue_size) != FT_OK){
       printf("FT_SetGetQueue (%d) failed\n", iport);
       }
   }


      //Pull the data in
      if (FT_Read(ftHandle,&databuffer,queue_size,&actualread) != FT_OK){
      printf("FT_Read (%d) failed\n", iport);
      }         
    

       //postprocess the returned data back into normal numbers
       printf(" State: Post-processing spectrum\n");  
   


       // Parse the buffer and split up each 48 bit section
       for(dctr=1;dctr<4096;dctr++){        //run from 1 to 1023 to avoid the DC
               data0 = databuffer[dctr*6];
               data1 = databuffer[dctr*6+1];
               data2 = databuffer[dctr*6+2];
               data3 = databuffer[dctr*6+3];
               data4 = databuffer[dctr*6+4];
               data5 = databuffer[dctr*6+5];

	   
               // Compute the integer equivalent
                 data_out = 0;   
                 data_out = data_out + (uint64_t)data5;
                 data_out = data_out + (uint64_t)data4 * 256llu;
                 data_out = data_out + (uint64_t)data3 * 256llu * 256;
                 data_out = data_out + (uint64_t)data2 * 256llu * 256 * 256;
                 data_out = data_out + (uint64_t)data1 * 256llu * 256 * 256 * 256;
                 data_out = data_out + (uint64_t)data0 * 256llu * 256 * 256 * 256 * 256;


               // store the integer equivalent to the plot buffer
               data_out_accum[dctr] = data_out_accum[dctr] + data_out;
              
               //debug print indiviudal bytes
               //printf("%i,%i,%i,%i,%i,%i\n",data0,data1,data2,data3,data4,data5);
               //printf("%llu\n",data_out);

       } 

     
        // Set the DC to zero for the plot and remove those pesky interleaving spurs
       data_out_accum[0] = 0;
       data_out_accum[1] = 0;
       data_out_accum[2] = 0;
       data_out_accum[3] = 0;
       data_out_accum[4] = 0;
       data_out_accum[5] = 0;
       data_out_accum[1023] =data_out_accum[1022] ;
       data_out_accum[2047] =data_out_accum[2046] ;
       data_out_accum[2048] =data_out_accum[2049] ;
       data_out_accum[3071] =data_out_accum[3070] ;


   
       //Update the GNU plot
       gnuprint(gp,data_out_accum,4095);
 
//increment the soft averaging counter
loop = loop +1;    

 }




//write the final values to the output file
for(dctr=1;dctr<4096;dctr++){        //run from 1 to 511 to avoid the DC
fprintf(savefile,"%llu\n",data_out_accum[dctr]);
}
fprintf(savefile,"\n");
fflush(savefile);  



//----------------------------------------------------------ON EXIT--------------------------------------
//
//
//
//----------------------------------------------------------ON EXIT--------------------------------------



    //POWER DOWN
    n = write(USB,"9",1);
    if (n<0){
    perror("send command failed :(");
    }   

 



  printf("\n\nExiting the SVII program!\n\n");
  // Close down the USB at the end of the program
  close(USB);
          if (FT_Close(ftHandle) != FT_OK){
          printf("FT_close (%d) failed\n", iport);
          }

  fclose(savefile);





  return 0;
}

