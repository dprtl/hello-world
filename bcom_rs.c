// BCOM_RS
// Daniel Pratlong - mai 2020

#include <stdio.h>    
#include <unistd.h>   
#include <fcntl.h>    
#include <errno.h>    
#include <termios.h>  
#include <string.h>  
#include <sys/ioctl.h>
#include <stdint.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
 
int serialport_init(const char* serialport,int baud)
// serialport: string name of the serial port (e.g. "/dev/ttyUSB0","COM1")
// baud: baud rate (bps)
// Connects to that port at that speed and 8N1.
// Opens the port in fully raw mode so you can send binary data.
{
  struct termios options;
  int fd;
  speed_t brate=B1200;
     
  // ouvre le port en mode non-bloquant
  fd = open(serialport,O_RDWR | O_NONBLOCK);    
  if (fd == -1)  {
    perror("serialport_init: Unable to open port ");
    return -1;
  }
  // recupere les options
  if (tcgetattr(fd, &options) < 0) {
    perror("serialport_init: Couldn't get term attributes");
    return -1;
  }
  switch(baud) {
    case 9600:   brate=B9600;   break;
    case 19200:  brate=B19200;  break;
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
  }
  options.c_cflag &= ~PARENB;  // no parity
  options.c_cflag &= ~CSTOPB;  // 1 stop bit
  options.c_cflag &= ~CSIZE;   // 8 data bits
  options.c_cflag |= CS8;      // 8 data bits
  options.c_cflag |= CRTSCTS;  // RTS/CTS flow control
  //options.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset
  options.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  options.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  options.c_oflag &= ~OPOST; // make raw
  // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  options.c_cc[VMIN]  = 1;  // wait for one character before return from read
  options.c_cc[VTIME] = 20; // wait 20 ms before return from read
  // on set les parametres :
  cfsetspeed(&options,brate);
  tcsetattr(fd,TCSANOW,&options);
  if(tcsetattr(fd,TCSAFLUSH,&options) < 0) {
    perror("init_serialport: Couldn't set term attributes");
    return -1;
  }
  return fd;
}

int serialport_write(int fd, const char* str)
{
  int len = strlen(str);
  int n = write(fd, str, len);
  
  if( n != len ) {
    perror("serialport_write: couldn't write whole string\n");
    return -1;
  }
  return 0;
}

int serialport_read_until(int fd,char* buf,char until,int buf_max,int timeout)
{
  char b[1];   // read expects an array, so we give it a 1-byte array
  int i = 0,n = 0;
  do { 
    n = read(fd,b,1);        // read a char at a time
    if(n == -1) return -1;   // couldn't read
    if(n == 0) {
       usleep(1000);       // wait 1 msec try again
       timeout--;
       if(timeout == 0) return -2;
       continue;
    }
    buf[i] = b[0]; 
    i++;
  } while(b[0]!=until && i<buf_max && timeout>0);
  buf[i] = 0;  // null terminate the string
  return 0;
}

int serialport_flush_close(int fd)
{
  int err;
  
  sleep(2);   //required to make flush work, for some reason
  err = tcflush(fd,TCIOFLUSH);
  close(fd);
  return err;
}

size_t file_len (char *filename) 
{ 
  struct stat buf;
  
  if (-1 == stat(filename,&buf)) return -1; 
  return buf.st_size; 
}

int main(int argc, char **argv)
{
  FILE* fichier = NULL;
  int fd = 0;
  int car,success,percent1 = 0,percent2 = 0;
  long count,retry,taille;
  const char p = '%';
  
  fichier = fopen(argv[1], "r");
  if (fichier == NULL) {
    printf("Impossible d'ouvrir le fichier\n");
    getchar();
    return -1;
  }
  taille = file_len(argv[1]);
  printf("Fichier %s ouvert (%ld octets)\n",argv[1],taille);
  printf("Ouverture du port serie\n");
  /* fd = serialport_init("aux",38400); */
  fd = serialport_init("/dev/ttyUSB0",38400);
  if (fd == -1) {
    printf("Impossible d'ouvrir le port serie\n");
    getchar();
    return -1;
  }
  serialport_write(fd,"atd\"192.168.1.5:1024\"\n");
  // lecture caractere par caractere
  retry = 0;
  car = fgetc(fichier);
  count = 1;
  while (car != EOF) {  // EOF fin de fichier
    do {
      success = write(fd,&car,1);
      if (success != 1) {
        sleep(1);
	printf("retry: %ld\n",retry);
	retry++;
      }
    } while (success != 1);
    percent1 = count*100/taille;
    if (percent1 != percent2) {
      printf("%d%c count: %ld/%ld  retry: %ld\n",percent1,p,count,taille,retry);
      percent2=percent1;
    }
    car = fgetc(fichier);
    count++;
    // 38400 bauds = 4800 bytes/s = 500 bytes/105 ms en theorie
    // 2400 bauds = 300 bytes/s = 3.3 ms par byte en theorie
    // 2 ms : NOK
    // 3 ms : NOK
    usleep(4000);    // 4 ms
  } 
  // fermeture du fichier
  fclose(fichier);
  // fermeture du port serie
  serialport_flush_close(fd);
  printf("Transfert terminé, touche [Enter]\n");
  getchar();
  return 0;
}
