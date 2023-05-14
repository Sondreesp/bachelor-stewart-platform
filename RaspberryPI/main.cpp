

#include <math.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <string>
#include <vector>
#include <cstring>
#include "pi2c.h"
//#include "./../lib/matplotlibcpp.h"





float getAcc(time_t time);



using namespace std;

//namespace plt = matplotlibcpp;

int main(int argc, char *argv[])
{
	
	int address = 0x08;
	long positionAndTime[2] = {0}; 
	char *data = (char*)positionAndTime;
	int length = sizeof(long)*2;
	float c0 = 0.0, acc = 0.0, oldAcc = 0.0, pos = 0.0, pos2 = 0.0, vel = 0.0, encoderTime = 0.0, encoderPosition = 0.0;
	float omg = M_PI/20, wantPos = 0.0, wantPosPre = 0.0, currPos = 0.0, prePos = 0.0, stpPS = 0.0, timestep = 0.1, dtsq = timestep*timestep;
	float error = 0, cumError = 0, rateError = 0, lastError = 0, kp = 5.0/(dtsq), kd = 1.0/(dtsq), kc = 1.0/(dtsq);
	clock_t start, end;
	start = clock();
	float t = 0.0, dt = 0.0, preT = 0.0, nextT = 0.0;
	vector<float> wanted, current, timeVec;

	bool runFlag = true;

	if(argc >= 4){

		kp = stof(argv[1])/dtsq;
		kd = stof(argv[2])/dtsq;
		kc = stof(argv[3])/dtsq;
	}
	else if(argc == 2){
		if(strcmp(argv[1],"-h")){
			cout << "kp, kd, kc, all inputs need a value and are treated as floats\n standard input is 1, 1, 1" << endl;
			return 0;
		}
	}


	ofstream DataFile("encoderAndStepperTest.dat");
	DataFile << "Time      encoder Position      Calculated Position" << endl;


	Pi2c pi2c(address);
	
	cout << "connected...\n";


	while(runFlag){
		while(dt<timestep){
		end = clock();
		t = float(end-start)/CLOCKS_PER_SEC;
		dt = t-preT;
		}
		

		wantPos = -1000*cos((t+dt)*omg) + 1000;

		if(pi2c.i2cRead(data,length)<0){
			cout << " error reading from arduino" << endl;
			cout << "time was: " << data[1]<< "pos was: " << data[0]<< endl;
			//return -1;
		};
		currPos = (float) positionAndTime[0]*0.6; //from encoder to stepper steps =  (300.0/500.0);
		
		acc = (wantPos-2*currPos+prePos)/(dt*dt);

		error = wantPosPre - currPos;
		cumError += error*dt;
		rateError = (error - lastError)/dt;

		acc += error*kp + cumError*kc + rateError*kd;
		// acc = getAcc(t);
		if(acc!=oldAcc){
		if(acc==0){
			c0 = 0;
		}else if( acc<0 ){
			c0 = 676000*sqrt(2/-acc);
		}else{
			c0 = -676000*sqrt(2/acc);
		}
		pi2c.i2cWrite((char*)&c0,4);
		}

		encoderTime = (float) positionAndTime[1]*0.001;
		encoderPosition = (float) positionAndTime[0]*2*M_PI/500;

		//pos = -1000*cos(encoderTime*omg)*2*M_PI/300;
		pos2 += (0.5*acc*dt*dt )*2*M_PI/300 + vel*dt;
		vel += acc*dt*2*M_PI/300;

		DataFile << encoderTime << "    " << encoderPosition << "     " <<   wantPos*(2*M_PI/300)<< endl;

		cout << "Time: " << encoderTime <<"  encoder: " << currPos << 
		"  calculated: " << wantPos << " Local time: " << t << " acc = " << acc << 
		"  error =  " << error << " rateErr  = " << rateError << " cumError = " << cumError << endl;
		

		wanted.push_back(wantPos*(2*M_PI/300));
		current.push_back(encoderPosition);
		timeVec.push_back(t);
        const char* myTitle = "Encoder Test";
        const char* myLegendEncoder = "Encoder";
		const char* myLegendWanted = "Wanted";


		lastError = error;
		wantPosPre = wantPos;
		oldAcc = acc;
		prePos = currPos;
		dt = 0.0;
		preT = t;
		if(t>60){
			runFlag = false;
		}
	}	
	
	return 0;
}



float getAcc(time_t time){
	
if(time<5){
	return (float) 100;
}
else if(time <10){
	return (float) -100;
}
else if(time <15){
	return (float) 100;
}
else if(time <20){
	return (float) -100;
}
else{
	return 0.0;
}
}









