
#include <iostream>

#include "myMatrixMath.h"
//#include "sendReceive.h"
#include "StewartPlatform.h"
/*

int sendAccToTwin(int serverSocket, float floatsToSend[7]){
	int numFloats = 7;

	if(SendInt32(serverSocket,1)){
		std::cout << "Error in SendInt32 in sendAccToTwin" << std::endl;
		return 1;	
	}

	if(SendNFloats(serverSocket,floatsToSend,numFloats)){
		std::cout << "Error in SendNFloats in sendAccToTwin" << std::endl;
		return 1;	
	}

	return 0;
}

int getFeedbackLengthFromTwin(int serverSocket, float floatsToGet[7], unsigned char charsToGet[6][3], unsigned char* charBuffer){
	int numFloats = 7, numChars= 18;
	int numCharsToGet = numFloats*sizeof(float)+numChars*sizeof(char);

	if(SendInt32(serverSocket,2)){
		std::cout << "Error in SendInt32 in getFeedbackFromTwin" << std::endl;
		return 1;	
	}

	if(ReceiveNUnsignedChars(serverSocket,  charBuffer, numCharsToGet)){
		std::cout << "Error in ReceiveNUnsignedChars in getFeedbackFromTwin" << std::endl;
		return 1;	

	}
	for(int i=0;i<numFloats;i++){
		*(((char*)floatsToGet)+i*4 +0) = *(charBuffer + i*4 + 0);
		*(((char*)floatsToGet)+i*4 +1) = *(charBuffer + i*4 + 1);
		*(((char*)floatsToGet)+i*4 +2) = *(charBuffer + i*4 + 2);
		*(((char*)floatsToGet)+i*4 +3) = *(charBuffer + i*4 + 3);
	}
	for(int i=0; i<6;i++){
		charsToGet[i][0] = *(charBuffer + numFloats*4 + i*3 + 0);
		charsToGet[i][1] = *(charBuffer + numFloats*4 + i*3 + 1);
		charsToGet[i][2] = *(charBuffer + numFloats*4 + i*3 + 2);
	}

	return 0;
}

int getFeedbackTopPlatePosFromTwin(int serverSocket, float floatsToGet[7], unsigned char charsToGet[6][3], unsigned char* charBuffer){
	int numFloats = 7, numChars= 18;
	int numCharsToGet = numFloats*sizeof(float)+numChars*sizeof(char);

	if(SendInt32(serverSocket,3)){
		std::cout << "Error in SendInt32 in getFeedbackFromTwin" << std::endl;
		return 1;	
	}

	if(ReceiveNUnsignedChars(serverSocket,  charBuffer, numCharsToGet)){
		std::cout << "Error in ReceiveNUnsignedChars in getFeedbackFromTwin" << std::endl;
		return 1;	

	}

	for(int i=0;i<numFloats;i++){
		*(((char*)floatsToGet)+i*4 +0) = *(charBuffer + i*4 + 0);
		*(((char*)floatsToGet)+i*4 +1) = *(charBuffer + i*4 + 1);
		*(((char*)floatsToGet)+i*4 +2) = *(charBuffer + i*4 + 2);
		*(((char*)floatsToGet)+i*4 +3) = *(charBuffer + i*4 + 3);
	}
	for(int i=0; i<6;i++){
		charsToGet[i][0] = *(charBuffer + numFloats*4 + i*3 + 0);
		charsToGet[i][1] = *(charBuffer + numFloats*4 + i*3 + 1);
		charsToGet[i][2] = *(charBuffer + numFloats*4 + i*3 + 2);
	}

	return 0;
}

int shutdownTwin(int serverSocket){
	
	if(SendInt32(serverSocket,42)){
		std::cout << "Error in SendInt32 in getFeedbackFromTwin" << std::endl;
		return 1;	
	}
	return 0;
}

*/
StewartPlatform::StewartPlatform(double timeStep){


// Initialize rotation matrices
	myMatrixMath::eye3by3(rotationMatrix);
	myMatrixMath::eye3by3(prevRotationMatrix);	

// set dt
	dt = timeStep;
	dtsq = timeStep*timeStep;

// Set rotation flag for unknown axis of rotation
	rotationFlag = 0;

// Initialize needed initial values
	myMatrixMath::zeroVector(currentActuatorVelocities,6);	

// Calculate initial top plate position
	double vec1[3]={0},vec2[3]={0};
	calculateNewTopPlatePosition(vec1, vec2);

// Calcualte initial actuator lengths 
	calculateActuatorLength();
}




StewartPlatform::StewartPlatform(double timeStep, int axisOfRotation[3]){

// Initialize rotation matrices

	myMatrixMath::eye3by3(rotationMatrix1);
	myMatrixMath::eye3by3(rotationMatrix2);	
	myMatrixMath::eye3by3(rotationMatrix3);	

// set dt

	dt = timeStep;
	dtsq = timeStep*timeStep;

// Set axis of rotations and rotation flag:
	axisOfRotations[0] = axisOfRotation[0];
	axisOfRotations[1] = axisOfRotation[1];
	axisOfRotations[2] = axisOfRotation[2];

	rotationFlag = 1;
	

// Initialize needed initial values
	myMatrixMath::zeroVector(currentActuatorVelocities,6);	

// Calculate initial top plate position
	double vec1[3]={0},vec2[3]={0};
	calculateNewTopPlatePosition(vec1, vec2);

// Calcualte initial actuator lengths 
	calculateActuatorLength();
}

void StewartPlatform::setdt(double timeStep){

	dt = timeStep;
	dtsq = timeStep*timeStep;
}

void StewartPlatform::resetPositionAndOrientation(){

	if(rotationFlag){
	// Reset rotation matrices
		myMatrixMath::eye3by3(rotationMatrix1);
		myMatrixMath::eye3by3(rotationMatrix2);	
		myMatrixMath::eye3by3(rotationMatrix3);	
	}else{
	// Reset rotation matrices
		myMatrixMath::eye3by3(rotationMatrix);
		myMatrixMath::eye3by3(prevRotationMatrix);	
	}

	// Reset needed initial values
		myMatrixMath::zeroVector(currentActuatorVelocities,6);	

	// Reset initial top plate position
		double vec1[3]={0},vec2[3]={0};
		calculateNewTopPlatePosition(vec1, vec2);

	// Reset initial actuator lengths 
		calculateActuatorLength();
}

void StewartPlatform::calculateNewRotation(double angularVelocities[3]){

	myMatrixMath::expMRodriguez(tempRotation,angularVelocities[0],
			angularVelocities[1], angularVelocities[2],dt);	

	myMatrixMath::MatMatMult3by3(rotationMatrix,prevRotationMatrix,tempRotation);
	myMatrixMath::copyMatrix3by3(prevRotationMatrix, rotationMatrix);


}

void StewartPlatform::setNewRotationMatrices(double angles[3]){

	myMatrixMath::getRotation(rotationMatrix1,axisOfRotations[0],angles[0]);	
	myMatrixMath::getRotation(rotationMatrix2,axisOfRotations[1],angles[1]);	
	myMatrixMath::getRotation(rotationMatrix3,axisOfRotations[2],angles[2]);

	myMatrixMath::MatMatMult3by3(tempRotation,rotationMatrix1,rotationMatrix2);
	myMatrixMath::MatMatMult3by3(rotationMatrix,tempRotation,rotationMatrix3);	

}



void StewartPlatform::calculateNewTopPlatePosition(double angularVelocities[3], 
							double translationXYZ[3]){
	if(rotationFlag){
		/*For known rotation axis */
		setNewRotationMatrices(angularVelocities);
	}else{
		/* For unknown rotational axis */
		calculateNewRotation(angularVelocities);
	};

	/*Update the frame position*/
	myMatrixMath::VecVecAdd3(newRotPointPos,translationXYZ,initialRotPointPos);

	for(int i=0;i<6;i++){
		/* Rotate initial local positions to new location */
		myMatrixMath::MatVecMult3by3(tempPosArray[i],
					rotationMatrix,lokalPosTopLinearActuator[i]);
					
		/* Add the new position of the moving frame origin */
		myMatrixMath::VecVecAdd3(newLokalPosTopLinearActuator[i],tempPosArray[i],
									newRotPointPos);
	}	
}



void StewartPlatform::calculateActuatorLength(){

	for(int i=0;i<6;i++){
		/* Save the previous lengths for acceleration calculation */
		currentActuatorLengths[i] = actuatorLengths[i];

		/* Get the relative position from joint to joint */
		myMatrixMath::VecVecSub3(tempPosArray[i],newLokalPosTopLinearActuator[i],
							lokalPosBottomLinearActuator[i]);
		/* Calculate the length of each vector */
		actuatorLengths[i] = myMatrixMath::vecNorm(tempPosArray[i]);
	}

}

void StewartPlatform::getActuatorLengths(double actuatorLengthsOut[6]){

	actuatorLengthsOut[0] = actuatorLengths[0];
	actuatorLengthsOut[1] = actuatorLengths[1];
	actuatorLengthsOut[2] = actuatorLengths[2];
	actuatorLengthsOut[3] = actuatorLengths[3];
	actuatorLengthsOut[4] = actuatorLengths[4];
	actuatorLengthsOut[5] = actuatorLengths[5];

}


void StewartPlatform::calculateActuatorAcceleration(){

/* Calculate linear accelerations for each actuator
 * ac = (wantedPosition-currentPosition-currentVelocity*dt)/dt²*/

	actuatorAccelerations[0] = (actuatorLengths[0] - 
			(currentActuatorLengths[0]+currentActuatorVelocities[0]*dt ))/dtsq; 
	
	actuatorAccelerations[1] = (actuatorLengths[1] - 
			(currentActuatorLengths[1]+currentActuatorVelocities[1]*dt ))/dtsq; 
	
	actuatorAccelerations[2] = (actuatorLengths[2] - 
			(currentActuatorLengths[2]+currentActuatorVelocities[2]*dt ))/dtsq; 
	
	actuatorAccelerations[3] = (actuatorLengths[3] - 
			(currentActuatorLengths[3]+currentActuatorVelocities[3]*dt ))/dtsq; 
	
	actuatorAccelerations[4] = (actuatorLengths[4] - 
			(currentActuatorLengths[4]+currentActuatorVelocities[4]*dt ))/dtsq; 
	
	actuatorAccelerations[5] = (actuatorLengths[5] - 
			(currentActuatorLengths[5]+currentActuatorVelocities[5]*dt ))/dtsq; 


// Update actuator current velocities:
	currentActuatorVelocities[0] += actuatorAccelerations[0]*dt;
	currentActuatorVelocities[1] += actuatorAccelerations[1]*dt;
	currentActuatorVelocities[2] += actuatorAccelerations[2]*dt;
	currentActuatorVelocities[3] += actuatorAccelerations[3]*dt;
	currentActuatorVelocities[4] += actuatorAccelerations[4]*dt;
	currentActuatorVelocities[5] += actuatorAccelerations[5]*dt;

}



void StewartPlatform::calculateActuatorAcceleration(double feedbackLengths[7]){

/* Calculate linear accelerations for each actuator
 * ac = (wantedPosition-currentPosition-currentVelocity*dt)/dt²*/

	actuatorAccelerations[0] = (actuatorLengths[0] -  
			(feedbackLengths[0]+currentActuatorVelocities[0]*dt ))/dtsq; 
	
	actuatorAccelerations[1] = (actuatorLengths[1] -  
			(feedbackLengths[1]+currentActuatorVelocities[1]*dt ))/dtsq; 
	
	actuatorAccelerations[2] = (actuatorLengths[2] - 
			(feedbackLengths[2]+currentActuatorVelocities[2]*dt ))/dtsq; 
	
	actuatorAccelerations[3] = (actuatorLengths[3] - 
			(feedbackLengths[3]+currentActuatorVelocities[3]*dt ))/dtsq; 
	
	actuatorAccelerations[4] = (actuatorLengths[4] - 
			(feedbackLengths[4]+currentActuatorVelocities[4]*dt ))/dtsq; 
	
	actuatorAccelerations[5] = (actuatorLengths[5] - 
			(feedbackLengths[5]+currentActuatorVelocities[5]*dt ))/dtsq; 


// Update actuator current velocities:
	currentActuatorVelocities[0] += actuatorAccelerations[0]*dt;
	currentActuatorVelocities[1] += actuatorAccelerations[1]*dt;
	currentActuatorVelocities[2] += actuatorAccelerations[2]*dt;
	currentActuatorVelocities[3] += actuatorAccelerations[3]*dt;
	currentActuatorVelocities[4] += actuatorAccelerations[4]*dt;
	currentActuatorVelocities[5] += actuatorAccelerations[5]*dt;

}


void StewartPlatform::calculateStepperAcceleration(){

	stepperAccelerations[0] = actuatorAccelerations[0]*feedConst;
	stepperAccelerations[1] = actuatorAccelerations[1]*feedConst;
	stepperAccelerations[2] = actuatorAccelerations[2]*feedConst;
	stepperAccelerations[3] = actuatorAccelerations[3]*feedConst;
	stepperAccelerations[4] = actuatorAccelerations[4]*feedConst;
	stepperAccelerations[5] = actuatorAccelerations[5]*feedConst;

}

void StewartPlatform::calculateC0fromStepperAccel(){
	double temp;

	if(stepperAccelerations[0] < 0){
		temp = -stepperAccelerations[0];
		c0ForSteppers[0] = -676000*sqrt(2/temp);
	}else if(stepperAccelerations[0] == 0){
		c0ForSteppers[0] = 0;
	}else{
		c0ForSteppers[0] = 676000*sqrt(2/stepperAccelerations[0]);
	}

	if(stepperAccelerations[1] < 0){
		temp = -stepperAccelerations[1];
		c0ForSteppers[1] = -676000*sqrt(2/temp);
	}else if(stepperAccelerations[1] == 0){
		c0ForSteppers[1] = 0;
	}else{
		c0ForSteppers[1] = 676000*sqrt(2/stepperAccelerations[1]);
	}

	if(stepperAccelerations[2] < 0){
		temp = -stepperAccelerations[2];
		c0ForSteppers[2] = -676000*sqrt(2/temp);
	}else if(stepperAccelerations[2] == 0){
		c0ForSteppers[2] = 0;
	}else{
		c0ForSteppers[2] = 676000*sqrt(2/stepperAccelerations[2]);
	}

	if(stepperAccelerations[3] < 0){
		temp = -stepperAccelerations[3];
		c0ForSteppers[3] = -676000*sqrt(2/temp);
	}else if(stepperAccelerations[3] == 0){
		c0ForSteppers[3] = 0;
	}else{
		c0ForSteppers[3] = 676000*sqrt(2/stepperAccelerations[3]);
	}

	if(stepperAccelerations[4] < 0){
		temp = -stepperAccelerations[4];
		c0ForSteppers[4] = -676000*sqrt(2/temp);
	}else if(stepperAccelerations[4] == 0){
		c0ForSteppers[4] = 0;
	}else{
		c0ForSteppers[4] = 676000*sqrt(2/stepperAccelerations[4]);
	}

	if(stepperAccelerations[5] < 0){
		temp = -stepperAccelerations[5];
		c0ForSteppers[5] = -676000*sqrt(2/temp);
	}else if(stepperAccelerations[5] == 0){
		c0ForSteppers[5] = 0;
	}else{
		c0ForSteppers[5] = 676000*sqrt(2/stepperAccelerations[5]);
	}

}

void StewartPlatform::setInitialRotPointPos(double rotationPointPos[3]){

	initialRotPointPos[0] = rotationPointPos[0];
	initialRotPointPos[1] = rotationPointPos[1];
	initialRotPointPos[2] = rotationPointPos[2];

}

void StewartPlatform::setLokalPosTopLinearActuator(double inputArray[6][3]){


	lokalPosTopLinearActuator[0][0] = inputArray[0][0];
	lokalPosTopLinearActuator[0][1] = inputArray[0][1];
	lokalPosTopLinearActuator[0][2] = inputArray[0][2];
	
	lokalPosTopLinearActuator[1][0] = inputArray[1][0];
	lokalPosTopLinearActuator[1][1] = inputArray[1][1];
	lokalPosTopLinearActuator[1][2] = inputArray[1][2];
	
	lokalPosTopLinearActuator[2][0] = inputArray[2][0]; 
	lokalPosTopLinearActuator[2][1] = inputArray[2][1];
	lokalPosTopLinearActuator[2][2] = inputArray[2][2];

	lokalPosTopLinearActuator[3][0] = inputArray[3][0]; 
	lokalPosTopLinearActuator[3][1] = inputArray[3][1];
	lokalPosTopLinearActuator[3][2] = inputArray[3][2]; 

	lokalPosTopLinearActuator[4][0] = inputArray[4][0]; 
	lokalPosTopLinearActuator[4][1] = inputArray[4][1];
	lokalPosTopLinearActuator[4][2] = inputArray[4][2]; 

	lokalPosTopLinearActuator[5][0] = inputArray[5][0]; 
	lokalPosTopLinearActuator[5][1] = inputArray[5][1];
	lokalPosTopLinearActuator[5][2] = inputArray[5][2]; 


}

void StewartPlatform::setLokalPosBottomLinearActuator(double inputArray[6][3]){

	lokalPosBottomLinearActuator[0][0] = inputArray[0][0];
	lokalPosBottomLinearActuator[0][1] = inputArray[0][1];
	lokalPosBottomLinearActuator[0][2] = inputArray[0][2];

	lokalPosBottomLinearActuator[1][0] = inputArray[1][0];
	lokalPosBottomLinearActuator[1][1] = inputArray[1][1];
	lokalPosBottomLinearActuator[1][2] = inputArray[1][2];

	lokalPosBottomLinearActuator[2][0] = inputArray[2][0];
	lokalPosBottomLinearActuator[2][1] = inputArray[2][1];
	lokalPosBottomLinearActuator[2][2] = inputArray[2][2];

	lokalPosBottomLinearActuator[3][0] = inputArray[3][0];
	lokalPosBottomLinearActuator[3][1] = inputArray[3][1];
	lokalPosBottomLinearActuator[3][2] = inputArray[3][2];

	lokalPosBottomLinearActuator[4][0] = inputArray[4][0];
	lokalPosBottomLinearActuator[4][1] = inputArray[4][1];
	lokalPosBottomLinearActuator[4][2] = inputArray[4][2];

	lokalPosBottomLinearActuator[5][0] = inputArray[5][0];
	lokalPosBottomLinearActuator[5][1] = inputArray[5][1];
	lokalPosBottomLinearActuator[5][2] = inputArray[5][2];

}


void StewartPlatform::getRotPointPos(double rotPointPos[3]){

	rotPointPos[0] = newRotPointPos[0];
	rotPointPos[1] = newRotPointPos[1];
	rotPointPos[2] = newRotPointPos[2];

}

void StewartPlatform::getActuatorAccelerations(double vec[6]){

	for(int i=0;i<6;i++){
		vec[i]=actuatorAccelerations[i];
	}

}


void StewartPlatform::getC0(double vec[6]){

	for(int i=0;i<6;i++){
		vec[i]=c0ForSteppers[i];
	}

}

	void StewartPlatform::getNewTopPlatePosition(double dataOut[6]){



	}
