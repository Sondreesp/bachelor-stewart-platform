#include <cstdio>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <time.h>    
#include <sys/time.h>
#include "./../lib/platformControll.h"



#define PORT "8080"
#define BILLION  1000000000L;

using namespace std;

int main(int argc, char *argv[]){



	char ip[16], name[32];
	char buffer[BUFSIZ];
    unsigned char actuatorStatus[6][3] = {0};
	float floatsToSend[7] = {0}, floatsToReceive[7] = {0};
    int serverSocket, portno, n, check, maxLoop = 100, numFloats=7;

    int numberOfElements = 1000, elementSize = 14;
    const char* fileName = "dataDT.dat";
    const char elementType[] = "double"; 
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
	
	XYZ_V[0] = 100*sin(t*omg);
	XYZ_V[1] = 0;
	XYZ_V[2] = 0;

	platform.calculateNewTopPlatePosition(angV, XYZ_V);
	platform.calculateActuatorLength();
	platform.calculateActuatorAcceleration(feedBackLengths);
    platform.calculateStepperAcceleration();
    platform.calculateC0fromStepperAccel();
    platform.getC0(c0);
	
    for(int j=0; j<6;j++){
        floatsToSend[j] = (float)c0[j];
    }
    floatsToSend[6] = (float) t;
    
/* Rewrite to I2C comunication
    if(sendAccToTwin(serverSocket,floatsToSend)){
		std::cout << "Error in sendAccToTwin in platformControll" << std::endl;
		return 1;	
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