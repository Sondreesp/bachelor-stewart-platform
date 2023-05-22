#include <cstdio>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <time.h>    
#include <sys/time.h>
#include "StewartPlatform.h"
//#include "sendRecieve.h"
//#include "circularBuffer.h"
#include "pi2c.h"



#define PORT "8080"
#define BILLION  1000000000L;

using namespace std;

int main(int argc, char *argv[]){


    // I2C Setup for the multiple nodes
    const int i2c_slaves[] = {0x08, 0x09, 0x10, 0x11, 0x12, 0x13};
    long positionAndTime[2] = {0}; 
    char *data = (char*)positionAndTime;
    int length = sizeof(long)*2;
    
    
    char ip[16], name[32];
    char buffer[BUFSIZ];
    unsigned char actuatorStatus[6][3] = {0};
    float floatsToSend[7] = {0}, floatsToReceive[12] = {0};
    int serverSocket, portno, n, check, maxLoop = 100, numFloats=7;

    int numberOfElements = 1000, elementSize = 14;
    const char* fileName = "dataDT.dat";
    const char elementType[] = "double"; 
    const char localIP[] = {"127.0.0.1"};
    const char* formatSpecifier = "Controller,   DigitalTwin\nX, Y, Z, th1, th2, th3, time, X, Y, Z, th1, th2, th3, time\n";

    double timestep = 0.5, dt=0, c0[6] = {0};
    double dataToSave[elementSize] = {0};
    double angV[3] = {0};
    double XYZ_V[3] = {0};
    double actuatorLength[6] = {0};
    double feedBackLengths[6] = {0};
    double feedbackTime = 0, feedback_dt=0;;
    double t=0, omg=M_PI/30;

    struct timespec preTime, time, startTime;
    double accum;
    
// Initiate platform object
    StewartPlatform platform(timestep);	

// Initiate I2C master
    Pi2c rpi(0x00);


    //argvChecker(argc, argv, ip, name);
    portno = atoi(PORT);
    /*
    serverSocket = connectToServer(portno,ip);
    if(serverSocket ==-1){
    * cout << "Error connect to server" << endl;
    * return -1;
    * }
    n = read(serverSocket,buffer,255);
    * printf("Andver was %s \n",buffer);
    * check = strncmp(buffer,"200",3);
    * if(check){
    * printf(\n server error with code %s:\n",buffer);
    * return -1;
    * }

    */
// get the real time
    if( clock_gettime( CLOCK_REALTIME, &startTime) == -1 )
    {
        perror( "clock gettime" );
        return EXIT_FAILURE;
    }

    if( clock_gettime( CLOCK_REALTIME, &time) == -1 )
    {
        perror( "clock gettime" );
        return EXIT_FAILURE;
    }

    if( clock_gettime( CLOCK_REALTIME, &preTime) == -1 )
    {
        perror( "clock gettime" );
        return EXIT_FAILURE;
    }

    while(dt<timestep){

        if( clock_gettime( CLOCK_REALTIME, &time) == -1 )
        {
            perror( "clock gettime" );
            return EXIT_FAILURE;
        }

        dt = ( time.tv_sec - preTime.tv_sec )
	        + (double)( time.tv_nsec - preTime.tv_nsec ) / (double)BILLION;
        
    }
    preTime.tv_sec = time.tv_sec;
    preTime.tv_nsec = time.tv_nsec;







for(int i=0;i<maxLoop;i++){
  /* ------This needs to be rewritten to I2C comunication------

    if(getFeedbackFromTwin(serverSocket, floatsToReceive,actuatorStatus)){
		std::cout << "Error in getFeedbackFromTwin in controller" << std::endl;
		return 1;	
	}
    for(int j=0;j<6;j++){
        feedBackLengths[j] = (double)floatsToReceive[j];
    }
    feedbackTime = (double)floatsToReceive[6];

    for(int j=0;j<7;j++){
        dataToSave[j] = (double)floatsToReceive[j];
    }
    
    */
    
    for (int j = 0; j < 6; j++){
        rpi.i2cChangeSlave(i2c_slaves[j]);
        rpi.i2cRead((char*)&floatsToReceive + 8*j, 8);
        feedBackLengths[j] = (double)floatsToReceive[j*2];
    }

    platform.getActuatorLengths(actuatorLength);
    for(int j=0;j<6;j++){
        dataToSave[j+7] = actuatorLength[j];  
    }
    dataToSave[13] = t;
// A save function. Not attatched with this code yet
    //myCircularBuffer.push(dataToSave,elementSize);
    
    t = ( time.tv_sec - startTime.tv_sec )
	    + (double)( time.tv_nsec - startTime.tv_nsec ) / (double)BILLION + timestep;

    feedback_dt = t- feedbackTime;

	angV[0] = 0;
	angV[1] = 0;
	angV[2] = 0;
	
	XYZ_V[0] = 0;
	XYZ_V[1] = 0;
	XYZ_V[2] = 100*sin(10*t*omg);

	platform.calculateNewTopPlatePosition(angV, XYZ_V);
	platform.calculateActuatorLength();
	platform.calculateActuatorAcceleration();
    platform.calculateStepperAcceleration();
    platform.calculateC0fromStepperAccel();
    platform.getC0(c0);
    
	cout << "Feet length = " ; 
    for(int k = 0; k<12; k++){
        cout << (long)floatsToReceive[k] << ", ";
    
    }
    cout << endl;
    for(int j=0; j<6;j++){
        floatsToSend[j] = (float)c0[j];
    }
    floatsToSend[6] = (float) t;
    
    for (int i = 0; i < 6; i++){
        rpi.i2cChangeSlave(i2c_slaves[i]);
        rpi.i2cWrite((char*)&floatsToSend + 4*i, 4);
    }
    
/* Rewrite to I2C comunication
    if(sendAccToTwin(serverSocket,floatsToSend)){
		std::cout << "Error in sendAccToTwin in platformControll" << std::endl;
		return 1;	
        * 
        * 
	}
*/
    do{

        if( clock_gettime( CLOCK_REALTIME, &time) == -1 ){
            perror( "clock gettime" );
            return EXIT_FAILURE;
        }

        dt = ( time.tv_sec - preTime.tv_sec )
	        + (double)( time.tv_nsec - preTime.tv_nsec ) / (double)BILLION;
        
    }while(dt<timestep);

    preTime.tv_sec = time.tv_sec;
    preTime.tv_nsec = time.tv_nsec;
}//for



if( clock_gettime( CLOCK_REALTIME, &time) == -1 ){
    perror( "clock gettime" );
    return EXIT_FAILURE;
}

accum = ( time.tv_sec - startTime.tv_sec ) + (double)( time.tv_nsec - startTime.tv_nsec ) / (double)BILLION;
printf( "acum = %lf\n", accum );
printf( "average loop time = %lf\n", accum/maxLoop );
//myCircularBuffer.shutdown();

    return 0;
}
