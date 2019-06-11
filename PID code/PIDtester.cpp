#include "Code/DepthPid.h"
#include <iostream>
#include <cstdlib>

int main()
{
	DepthPIDController pid = DepthPIDController(0.1, 1, 1 ,0);
	const int arrayLength = 5;
	float pseudoPos = 1000;
	float inertiaArray[arrayLength] = { 0 };
	float inertia = 0;
	float pidReturn = 0;
	
	for (int i = 0; i < 1000; i++)
	{
		std::cout << pseudoPos << "\n";
		pidReturn = pid.getMotorValues(pseudoPos);
		inertiaArray[i % arrayLength] = pidReturn;
		for (int j = 0; j < arrayLength; ++j)
			inertia += inertiaArray[j];
		pseudoPos += pidReturn + inertia/arrayLength;
	}

	return 0;
}