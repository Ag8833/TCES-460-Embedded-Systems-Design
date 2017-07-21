#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include <wiringPi.h>
#include <wiringSerial.h>
#include <mcp3004.h>
#include <softPwm.h>

#define BASE 100
#define SPI_CHAN 0
#define ANALOG_READ_SIZE 5
#define MOTORT 8	//GPIO 2
#define MOTORI 9	//GPIO 3
#define MOTORM 7	//GPIO 4
#define MOTORR 0	//GPIO 17
#define MOTORP 2	//GPIO 27

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main(int argc, char *argv[])
{
    wiringPiSetup();
    mcp3004Setup(BASE,SPI_CHAN);
    char snum[10];
    int x = 0, i = 0, j = 0, k = 0;   
    char readBuffer[256];
    char writeBuffer[256];
    char catArray[3];
    int resultArray[256];
    int mappedResult[4];

    pinMode(MOTORT, OUTPUT);
    pinMode(MOTORI, OUTPUT);
    pinMode(MOTORM, OUTPUT);
    pinMode(MOTORR, OUTPUT);
    pinMode(MOTORP, OUTPUT);
    softPwmCreate(MOTORT,0,200);
    softPwmCreate(MOTORI,0,200);
    softPwmCreate(MOTORM,0,200);
    softPwmCreate(MOTORR,0,200);
    softPwmCreate(MOTORP,0,200);

    //SOCKET STUFF ----------

    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    if (argc < 3) {
       fprintf(stderr,"usage %s hostname port\n", argv[0]);
       exit(0);
    }
    portno = atoi(argv[2]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");

    //END SOCKET STUFF ----------

    for(;;)
    {
    	bzero(readBuffer,256);
	bzero(writeBuffer,256);
	bzero(catArray,3);
	bzero(resultArray,256);

	if(x == 1)
	{
		bzero(snum,10);
		bzero(readBuffer,256);

		for(i = 0; i < ANALOG_READ_SIZE; i++)
		{
			int analogResult = analogRead(100 + i)/4;
			snprintf(snum, 10, "%d", analogResult);
			strcat(writeBuffer, snum);
			strcat(writeBuffer, ",");
		}

		n = write(sockfd,writeBuffer,strlen(writeBuffer));
		//printf("Writing = %s\n", writeBuffer);
		if(n < 0) error("ERROR writing to socket");

		delay(5);
		x = 0;
	}
	else
	{
		bzero(readBuffer,256);
		k = 0;

		n = read(sockfd,readBuffer,255);
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
			if(k == 0)
			{
				resultArray[k] += 3;
			}
			mappedResult[k] = map(resultArray[k], 12, 23, 22, 4);
			printf("%d ", resultArray[k]);
		}

		softPwmWrite(MOTORT, mappedResult[0]);
		softPwmWrite(MOTORI, mappedResult[1]);
		softPwmWrite(MOTORM, mappedResult[2]);
		softPwmWrite(MOTORR, mappedResult[3]);
		softPwmWrite(MOTORP, mappedResult[4]);

		printf("\n");
		delay(5);
		x = 1;
	}
    }

    close(sockfd);
    return 0;
}