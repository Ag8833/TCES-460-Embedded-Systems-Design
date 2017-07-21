#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>
#include <mcp3004.h>
#include <softPwm.h>

#define BASE 100
#define SPI_CHAN 0
#define ANALOG_READ_SIZE 5
#define MOTORT 1	//GPIO 18
#define MOTORI 4	//GPIO 23
#define MOTORM 5	//GPIO 24
#define MOTORR 6	//GPIO 25
#define MOTORP 26	//GPIO 12

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

int main(int argc, char *argv[])
{
     wiringPiSetup();
     mcp3004Setup(BASE,SPI_CHAN);
     char snum[10];
     int x = 0, i = 0, j = 0, k = 0;;
     char readBuffer[256];
     char writeBuffer[256];
     char catArray[3]; 
     int resultArray[256];

     pinMode(MOTORT, OUTPUT);
     pinMode(MOTORI, OUTPUT);
     pinMode(MOTORM, OUTPUT);
     pinMode(MOTORR, OUTPUT);
     pinMode(MOTORP, OUTPUT);
     softPwmCreate(MOTORT,0,25);
     softPwmCreate(MOTORI,0,25);
     softPwmCreate(MOTORM,0,25);
     softPwmCreate(MOTORR,0,25);
     softPwmCreate(MOTORP,0,25);

     //SOCKET STUFF ----------

     int sockfd, newsockfd, portno;
     socklen_t clilen;
     struct sockaddr_in serv_addr, cli_addr;
     int n;
     if (argc < 2) {
         fprintf(stderr,"ERROR, no port provided\n");
         exit(1);
     }
     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0) 
        error("ERROR opening socket");
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = atoi(argv[1]);
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0) 
              error("ERROR on binding");
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     if (newsockfd < 0) 
          error("ERROR on accept");

     //END SOCKET STUFF ----------
     
     for(;;)
     {
	bzero(readBuffer,256);
	bzero(writeBuffer,256);
	bzero(catArray,3);
	bzero(resultArray,256);

	/*
        softPwmWrite(MOTORT, 100);
	softPwmWrite(MOTORI, 100);
	softPwmWrite(MOTORM, 100);
	softPwmWrite(MOTORR, 100);
	softPwmWrite(MOTORP, 100);
	*/

	if(x == 1)
	{
		bzero(readBuffer,256);
		k = 0;

		n = read(newsockfd,readBuffer,255);
		if(n < 0) error("ERROR reading from socket");

		for(i = 0; i < strlen(readBuffer); i++)
		{
			while(readBuffer[i] != ',')
			{
				catArray[j] = readBuffer[i];
				j++;
				i++;
			}
			resultArray[k] = atoi(catArray);

			k++;
			j = 0;
			bzero(catArray,3);
		}

		printf("Received: ");
		for(k = 0; k < ANALOG_READ_SIZE; k++)
		{
			printf("%d ", resultArray[k]);
		}
		
		softPwmWrite(MOTORT, resultArray[0]);
		softPwmWrite(MOTORI, resultArray[1]);
		softPwmWrite(MOTORM, resultArray[2]);
		softPwmWrite(MOTORR, resultArray[3]);
		softPwmWrite(MOTORP, resultArray[4]);

		printf("\n");
		delay(5);
		x = 0;
	}
	else
	{
		bzero(snum,10);
		bzero(writeBuffer,256);

		for(i = 0; i < ANALOG_READ_SIZE; i++)
		{
			int analogResult = analogRead(100 + i)/4;	
			snprintf(snum, 10, "%d", analogResult);
			strcat(writeBuffer, snum);
			strcat(writeBuffer, ",");
		}

	    n = write(newsockfd,writeBuffer,strlen(writeBuffer));
		if(n < 0) error("ERROR writing to socket");

		delay(5);
		x = 1;
	}
     }
     close(newsockfd);
     close(sockfd);
     return 0; 
}
