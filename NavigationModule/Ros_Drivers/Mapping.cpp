#include "Mapping.h"
#include <cmath>
#include <iostream>
#include <math.h>


#define PI 3.14159265
#define DEBGUG true
void Mapping::getCoordinates(double x_1, double y_1, double x_2, double y_2, double x_3, double y_3, double d_1, double d_2, double d_3)
{
	//change from doubles to int to be use take the last 3 decimals
//makes the first x_1 and y_1 the shortest values and subtract those from the distances. 
	//comes in as millimetters just change into an int for the rest of run, could use casting 
	int X_1 = x_1 ;
	int Y_1 = y_1 ;
	int X_2 = x_2 ;
	int Y_2 = y_2 ;
	int X_3 = x_3 ;
	int Y_3 = y_3 ;
	
	//change system so that we can find the distance easier
	X_3 -= X_1;
	Y_3 -= Y_1;
	X_2 -= X_1;
	Y_2 -= Y_1;
	X_1 -= X_1;
	Y_1 -= Y_1;

	if(DEBGUG)
	{
		printf("For the First ESP the values are x= %d, y = %d, and distance = %f\n",X_1, Y_1,d_1);

		printf("For the Second ESP the values are x= %d, y = %d, and distance = %f\n",X_2, Y_2,d_2);

		printf("For the Third ESP the values are x= %d, y = %d, and distance = %f\n",X_3, Y_3,d_3);
	}
	//needs confirmation that location of X_2  and Y_2 is never zero if so swap 
	//Formula is now
	// x^2 +y^2 = d_1^2
	// (x-X_2)^2 + y^2 = d_2^2
	//
	//found by deriving 
	if (X_2 != 0 && Y_2 !=0) //check - not entirely following this section
	{
		//this means swap X_2 and X_3
		int place = X_2;
		X_2 = X_3;
		X_3 = place;
		place = Y_2;
		Y_2 = Y_3;
		Y_3 = place;
		//need to swap distances as well
		double distance = d_2;
		d_2 = d_3;
		d_3 = distance; 
		std::cout << "Swapped";
	}
	if(Y_2 == 0)
	{
		if(DEBGUG)
		{

		}
		//subtract out the x and solve for y first then x

		
		double foundX = (pow(d_2, 2) - pow(d_1, 2) - pow((double)X_2, 2)) / (-2 * ((double)X_2));



		//use teh 3rd point to find y
		double foundY = sqrt(pow(d_2, 2) - pow(foundX - (double)X_2 , 2));
		this->found_x_1 = foundX;

		if(DEBGUG)
		{
			printf("Found X value = %f\n",foundX);	
			printf("Found Y value = %f\n",foundY);
		}
		//oundY = pow(d_2, 2) - pow(foundX + (double)X_2/1000, 2);
		//found1 is +- in the kit 
		//test it out using point 3
		double distance1 = sqrt(pow(foundX - x_3, 2) + (pow(foundY - y_3, 2)));
		double offFactor1 = abs(((distance1 - d_3) / (d_3)) * 100);
		double distance2 = sqrt(pow(foundX - x_3, 2) + (pow(-foundY - y_3, 2)));
		double offFactor2 = abs((((distance2 - d_3) / (d_3)) * 100));


		if(DEBGUG)
		{
			printf("Negative off factor is %f\n",offFactor2);
			printf("Positve off factor is %f\n",offFactor1);
			printf("Found distance positive is  %f\n",distance1);
			printf("Found distance negative is  %f\n",distance2);
			
		}

		//check to confirm proper logic here
		if (offFactor1 <= (offFactor2))
		{
			this->found_y_1 = foundY;
		
		}
		else
		{
			this->found_y_1 = -foundY;
		}
	}
	else
	{
		
		//x should be zero procede
		double foundY = (pow(d_2, 2) - pow(d_1, 2) - pow((double)Y_2, 2)) / (-2 * ((double)Y_2));

		//use teh 3rd point to find y
		double foundX = sqrt(pow(d_2, 2) - pow(foundY - (double)Y_2, 2));
		this->found_y_1 = foundY;
		if(DEBGUG)
		{
			printf("Found X value = %f\n",foundX);	
			printf("Found Y value = %f\n",foundY);
		}
		//oundY = pow(d_2, 2) - pow(foundX + (double)X_2/1000, 2);
		//found1 is +- in the kit 
		//test it out using point 3
		double distance1 = sqrt(pow(foundX - (double)X_3, 2) + (pow(foundY - (double)Y_3, 2)));
		double offFactor1 = abs(((distance1 - d_3) / (d_3)) * 100);
		double distance2 = sqrt(pow(-foundX - (double)X_3 , 2) + (pow(foundY - (double)Y_3, 2)));
		double offFactor2 = abs((((distance2 - d_3) / (d_3)) * 100));

		if(DEBGUG)
		{
			printf("Negative off factor is %f\n",offFactor2);
			printf("Positve off factor is %f\n",offFactor1);
		}

		//check to confirm proper logic here
		if (offFactor1 <= (offFactor2))
		{
			this->found_x_1 = foundX;
		
		}
		else
		{
			this->found_x_1 = -foundX;
		}
	}
	//if doesnt work part it out and check
	

}

double Mapping::getAngleFromCenter() // Gotta be careful here because the angle is ependent on where it is being taken from. REMOVE ABSOLUTE
{
	// we already have the x, y coordinates for the values.
	//the value must be taken by an absolute to proceded with the function
	double xAngle = atan2(this->found_y_1, this->found_x_1);
	if (this->found_y_1 < 0)
	{
		//add in half pi
		xAngle += PI / 2;
		if (this->found_x_1 > 0)
		{
			xAngle += PI / 2;
			//add in half pi
		}
	}
	return xAngle;
}
