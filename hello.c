// -------------------------
// ----- SETUP USART 0 -----
// -------------------------
// At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD 
// (ie the alt0 function) respectively
// OPEN THE UART
// The flags (defined in fcntl.h):
// Access modes (use 1 of these):
//   O_RDONLY - Open for reading only.
//   O_RDWR - Open for reading and writing.
//   O_WRONLY - Open for writing only.
//
// O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. 
//    When set read requests on the file can return immediately with a failure status
//    if there is no input immediately available (instead of blocking). 
//    Likewise, write requests can also return immediately with a failure status 
//    if the output can't be written immediately.
//
// O_NOCTTY - When set and path identifies a terminal device, 
//    open() shall not cause the terminal device to become 
//    the controlling terminal for the process.
// CONFIGURE THE UART
// The flags (defined in /usr/include/termios.h 
//     - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
// Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, 
//             B115200, B230400, B460800, B500000, B576000, B921600, 
//             B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, 
//             B3500000, B4000000
// CSIZE:- CS5, CS6, CS7, CS8
// CLOCAL - Ignore modem status lines
// CREAD - Enable receiver
// IGNPAR = Ignore characters with parity errors
// ICRNL - Map CR to NL on input 
//         (Use for ASCII comms where you want to auto correct 
//         end of line characters - don't use for bianry comms!)
// PARENB - Parity enable
// PARODD - Odd parity (else even)

#include <stdio.h>
#include <unistd.h> //Used for UART
#include <fcntl.h> //Used for UART
#include <string.h> 

#include <termios.h> //Used for UART

int main(){

int uart0_filestream[4] = {-1,};
unsigned char rx_init_buffer[2] = {0, };

int read_byte = 34;
int initial_flag = 0;
int cnt = 0;

uart0_filestream[0] = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY ); //Open in non blocking read/write mode
uart0_filestream[1] = open("/dev/ttyUSB1", O_RDONLY | O_NOCTTY | O_NONBLOCK); //Open in non blocking read/write mode
uart0_filestream[2] = open("/dev/ttyUSB2", O_RDONLY | O_NOCTTY | O_NONBLOCK); //Open in non blocking read/write mode
uart0_filestream[3] = open("/dev/ttyUSB3", O_RDONLY | O_NOCTTY | O_NONBLOCK); //Open in non blocking read/write mode


for(int i=0;i<4;i++){

  if (uart0_filestream[i] == -1) {     // ERROR - CAN'T OPEN SERIAL PORT
    printf("Error - Unable to open UART. Ensure it is not in use by another application\n");
  }

}

struct termios options;
tcgetattr(uart0_filestream[0], &options);
options.c_cflag = B921600 | CS8 | CLOCAL | CREAD; //<Set baud rate
options.c_iflag = IGNPAR;
options.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
options.c_iflag &= ~IGNBRK;         // disable break processing

options.c_oflag = 0;
options.c_lflag = 0;
tcflush(uart0_filestream[0], TCIFLUSH);

tcsetattr(uart0_filestream[0], TCSANOW, &options);

//----- CHECK FOR ANY RX BYTES -----
while (uart0_filestream[0] != -1) {

  // Read up to 255 characters from the port if they are there
  unsigned char rx_buffer[34];



 // if(initial_flag == 0){

  //  for(int i = 0 ;i<33;i++){

  //   rx_buffer[34-i] = rx_buffer[32-i];

  //  }

  int rx_length = read(uart0_filestream[0], (void*)rx_buffer, read_byte); //Filestream, buffer to store in, number of bytes to read (max)

    
    // printf("%x %x", rx_buffer[0], rx_buffer[1]);
    // printf("\n"); 


    // if(rx_buffer[0] == 0x55 && rx_buffer[1] == 0x55){
      
    //   printf("%x %x", rx_buffer[0], rx_buffer[1]);
    //   printf("\n"); 

    //     for(int i=0;i<34;i++){
    //     //  if(rx_buffer[i] == 55 && rx_buffer[i+1] == 55)
    //         printf("%x ", rx_buffer[i]);
    //     }
    //         printf("\n"); 



//      initial_flag = 1;

    //}
 // }

//   if(rx_buffer[32] == 0x55 && rx_buffer[33] == 0x55)
// {
  if (rx_length < 0) {

    //An error occured (will occur if there are no bytes)
  } else if (rx_length == 0) {
    //No data waiting

  } else {
    //Bytes received
    printf("%i Bytes read :", rx_length);

    for(int i=0;i<rx_length;i++){
    //  if(rx_buffer[i] == 55 && rx_buffer[i+1] == 55)
        printf("%x ", rx_buffer[i]);
    }
        printf("\n"); 
  }
// }else{
//   continue;
// }


////////////////////////////////////////////////////////parse rx_buffer
// [0][1] : 0x55 0x55 - SOP
// [2] : Ch
// [3] : ID
// [4][5] [6][7] [8][9] : Euler RPY
// [10][11] [12][13] [14][15] : Gyro XYZ
// [16][17] [18][19] [20][21] : Accel XYZ
// [22][23] [24][25] [26][27] : Magnet XYZ
// [28][29] : Battery
// [30][31] : timestamp
// [32][33] : checksum

// unsigned char Ch = rx_buffer[2];
// unsigned char ID = rx_buffer[3];
// float Euler[3], Gyro[3], Accel[3], Magnet[3];
// short Battery;
// unsigned short Timestamp, Checksum, chksum;

//   for(int i = 0; i<6 ;i = i + 2){
//     Euler[i] = ((float)(rx_buffer[i+5] << 8 | rx_buffer[i+4])/100);
//     Gyro[i]  = ((float)(rx_buffer[i+11] << 8 | rx_buffer[i+10])/10);
//     Accel[i] = ((float)(rx_buffer[i+17] << 8 | rx_buffer[i+16])/1000);
//     Magnet[i]= ((float)(rx_buffer[i+23] << 8 | rx_buffer[i+22])/10);
//   }

//     Battery = rx_buffer[29] << 8 | rx_buffer[28];
//     Timestamp = rx_buffer[31] | rx_buffer[30];

//   for(int i = 0 ; i < read_byte - 2 ; i++){
//       chksum += rx_buffer[i];
//   }

//   if ((rx_buffer[33] << 8 | rx_buffer[32]) != chksum){
//    //   printf("CHK False");
//   }

// printf("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d\r\n" ,cnt, Euler[0],Euler[1],Euler[2], Gyro[0], Gyro[1], Gyro[2], Accel[0],Accel[1], Accel[2], Magnet[0],Magnet[1],Magnet[2], Battery);

//     cnt ++;
  
  
}


}