#ifndef StewartPlatform_h
#define StewartPlatform_h

#include "myMatrixMath.h"





int sendAccToTwin(int serverSocket, float floatsToSend[7]);

int getFeedbackLengthFromTwin(int serverSocket, float floatsToGet[7], unsigned char charsToGet[6][3], unsigned char* charBuffer);

int getFeedbackTopPlatePosFromTwin(int serverSocket, float floatsToGet[7], unsigned char charsToGet[6][3], unsigned char* charBuffer);

int shutdownTwin(int serverSocket);

class StewartPlatform{
public:
	//Constructor for running with unknown axis of rotation
	StewartPlatform(double timeStep);

	//Constructor for running with known axis of rotations
	StewartPlatform(double timeStep, int axisOfRotations[3]);

	void setLokalPosTopLinearActuator(double inputArray[6][3]);
	void setLokalPosBottomLinearActuator(double inputArray[6][3]);
	void setInitialRotPointPos(double rotPointPos[3]);
	void getRotPointPos(double rotPointPos[3]);
	void setdt(double timeStep);
	void calculateNewTopPlatePosition(double angularVelocities[3], double translationXYZ[3]);
	void getNewTopPlatePosition(double dataOut[6]);

	void calculateActuatorLength(void);
	void getActuatorLengths(double actuatorLengthsOut[6]);
	
	void calculateActuatorAcceleration(void);
	void calculateActuatorAcceleration(double feedbackLengths[7]);
	void calculateStepperAcceleration(void);
	void calculateC0fromStepperAccel(void);
	void resetPositionAndOrientation(void);


	void getActuatorAccelerations(double vec[6]);
	void getC0(double vec[6]);
private:

	void calculateNewRotation(double angularVelocities[3]);
	void setNewRotationMatrices(double angles[3]);
	
	double newLokalPosTopLinearActuator[6][3];
	double tempPosArray[6][3];

	double lokalPosTopLinearActuator[6][3] = {{ -40.00,-144.62,-50.0},
						  {  40.00,-144.62,-50.0},
						  { 145.22,  37.67,-50.0},
						  { 105.25, 106.95,-50.0},
						  {-105.25, 106.95,-50.0},
						  {-145.22,  37.67,-50.0}};


	double lokalPosBottomLinearActuator[6][3] = {{-432.625,-331.18,54.61},
						     { 432.625,-331.18,54.61},
						     { 503.125,-209.07,54.61},
						     {  70.500, 540.25,54.61},
						     { -70.500, 540.25,54.61},
						     {-503.125,-209.07,54.61}};

	double actuatorLengths[6];
	double currentActuatorLengths[6];
	double currentActuatorVelocities[6];
	double actuatorAccelerations[6];
	double stepperAccelerations[6];
	double c0ForSteppers[6];
	double initialRotPointPos[3] = {0,0,680.0};
	double newRotPointPos[3];

	//Used if unknown rotational axis is chosen
	double rotationMatrix[3][3];
	double prevRotationMatrix[3][3];
	double tempRotation[3][3];

	// Used for known rotation axis:
	int axisOfRotations[3];
	double angle[3];
	int stepperDirection[6];
	
	double rotationMatrix1[3][3];
	double rotationMatrix2[3][3];
	double rotationMatrix3[3][3];
	
	
	int rotationFlag;

	//Feed constant of stepper with 200steps/rev and 5 mm/rev: 5/200 mm/step
	const int feedConst = 40;// step/mm
	double dt;
	double dtsq;	


};


#endif
