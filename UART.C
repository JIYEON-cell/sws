#include <stdio.h>
#include <unistd.h> //Used for UART
#include <fcntl.h> //Used for UART
#include <termios.h> //Used for UART
int main(){
// -------------------------
// ----- SETUP USART 0 -----
// -------------------------
// At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD 
// (ie the alt0 function) respectively
int uart0_filestream = -1;
int read_byte = 2;
int initial_flag = 0;
int cnt = 0;
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
uart0_filestream = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK); //Open in non blocking read/write mode
if (uart0_filestream == -1) {     // ERROR - CAN'T OPEN SERIAL PORT
  printf("Error - Unable to open UART. Ensure it is not in use by another application\n");
}
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
struct termios options;
tcgetattr(uart0_filestream, &options);
options.c_cflag = B921600 | CS8 | CLOCAL | CREAD; //<Set baud rate
options.c_iflag = IGNPAR;
options.c_oflag = 0;
options.c_lflag = 0;


options.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
options.c_iflag &= ~IGNBRK;         // disable break processing
tcflush(uart0_filestream, TCIFLUSH);
tcsetattr(uart0_filestream, TCSANOW, &options);



//----- CHECK FOR ANY RX BYTES -----
while (uart0_filestream != -1) {
  // Read up to 255 characters from the port if they are there
  char rx_buffer[35];
  int rx_length = read(uart0_filestream, (void*)rx_buffer, read_byte); //Filestream, buffer to store in, number of bytes to read (max)
  if(initial_flag == 0){
    if(rx_buffer[0] == 0x55 && rx_buffer[1] == 0x55){
      printf("%x %x", rx_buffer[0], rx_buffer[1]);
      read_byte = 34;
      initial_flag = 1;
    }
  }else{
        if(rx_buffer[32] != 0x55 && rx_buffer[33] != 0x55){
            initial_flag = 0;
            read_byte = 2;

        }
  }

  if (rx_length < 0) {
    //An error occured (will occur if there are no bytes)
  } else if (rx_length == 0) {
    //No data waiting
  } else {
    //Bytes received
  //  printf("%i Bytes read :", rx_length);

        if(rx_buffer[32] == 0x55 && rx_buffer[33] == 0x55){

    // for(int i=0;i<rx_length;i++){


    //     printf("%x ", rx_buffer[i]);

    // }
    //       printf("\n"); 

////////////////////////////////////////////////////////parse rx_buffer


// [0] : Ch
// [1] : ID
// [2][3] [4][5] [6][7] : Euler RPY
// [8][9] [10][11] [12][13] : Gyro XYZ
// [14][15] [16][17] [18][19] : Accel XYZ
// [20][21] [22][23] [24][25] : Magnet XYZ
// [26][27] : Battery
// [28][29] : timestamp
// [30][31] : checksum
// [32][33] : 0x55 0x55 - SOP

            unsigned char Ch = rx_buffer[0];
            unsigned char ID = rx_buffer[1];
            float Euler[3], Gyro[3], Accel[3], Magnet[3];
            short Battery;
            unsigned short Timestamp, Checksum, chksum;

            for(int i = 0; i<6 ;i = i + 2){
                Euler[i] = ((float)(rx_buffer[i+3] << 8 | rx_buffer[i+2])/100);
                Gyro[i]  = ((float)(rx_buffer[i+9] << 8 | rx_buffer[i+8])/10);
                Accel[i] = ((float)(rx_buffer[i+15] << 8 | rx_buffer[i+14])/1000);
                Magnet[i]= ((float)(rx_buffer[i+21] << 8 | rx_buffer[i+20])/10);
            }

            Battery = rx_buffer[27] << 8 | rx_buffer[26];
            Timestamp = rx_buffer[29] | rx_buffer[28];

            for(int i = 0 ; i < read_byte - 2 ; i++){
                chksum += rx_buffer[i];
            }

            if ((rx_buffer[31] << 8 | rx_buffer[30]) != chksum){
               printf("CHK False");
            }else{
       

            }
       
         printf("%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %d\r\n" ,cnt, Euler[0],Euler[2],Euler[4], Gyro[0], Gyro[2], Gyro[4], Accel[0],Accel[2], Accel[4], Magnet[0],Magnet[2],Magnet[4], Battery);
                printf("\n"); 
                                cnt ++;

    }

  }
}







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


//   else {
//     //Bytes received
//     rx_buffer[rx_length] = '\0';
//     printf("%i bytes read : %s\n", rx_length, rx_buffer);
//   }
// }
}