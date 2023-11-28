#include <math.h>

namespace Mapping 
{
    // The value of the mathematical constant pi.
    const double PI = 3.14159265;

    // Represents a pair of coordinates.
    typedef struct
    {
        // The x-coordinate.
        double x;
        // The y-coordinate.
        double y;
    } CoordinatePair;

    /*
     * Produces the approximate coordinates formed by the intersection of the three circles : (x - x_1) ^ 2 + (y - y_1) ^ 2 = d_1 ^ 2,
     * (x - x_2)^2 + (y - y_2)^2 = d_2^2,and (x - x_3)^2 + (y - y_3)^2 = d_3^2.
     *
     */CoordinatePair findIntersection(double x_1, double y_1, double x_2, double y_2, double x_3, double y_3, double d_1, double d_2, double d_3)
    {
        bool isOneValid = false;;
        bool isTwoValid = false;
        CoordinatePair pairs[2];

        //change from doubles to int to be use take the last 3 decimals
        //makes the first x_1 and y_1 the shortest values and subtract those from the distances. 
        //comes in as millimeters just change into an int for the rest of run, could use casting 
        int X_1 = x_1;
        int Y_1 = y_1;
        int X_2 = x_2;
        int Y_2 = y_2;
        int X_3 = x_3;
        int Y_3 = y_3;

        // Shift coordinate system so that (x_1, y_1) is center. 
        X_3 -= X_1;
        Y_3 -= Y_1;
        X_2 -= X_1;
        Y_2 -= Y_1;
        X_1 -= X_1;
        Y_1 -= Y_1;

        //needs confirmation that location of X_2  and Y_2 is never zero if so swap 
        //Formula is now
        // x^2 +y^2 = d_1^2
        // (x-X_2)^2 + y^2 = d_2^2
        //
        //found by deriving

        // Use the closer coordinate first.
        if(X_2 != 0)
        {
            //subtract out the x and solve for y first then x
            double squareD_1 = d_1 * d_1;
            double squareD_2 = d_2 * d_2;
            double foundX = (squareD_1 - squareD_2 + (double)X_2 * (double)X_2) / (2 * ((double)X_2));
            pairs[0].x = foundX;

            // Plug back in to x^2+y^2=d_1^2 solve for y.
            double squareFoundX = foundX * foundX;
            double foundY_1 = sqrt(squareD_1 - squareFoundX);

            if(foundY_1 == 0)
            {
                // If zero, this is the only point.
                pairs[0].y = foundY_1;
                isOneValid = true;
            }
            else
            {
                double foundY_2 = -foundY_1;

                //found1 is +- in the kit 
                //test it out using point 3
                double xDifference = foundX - (double)X_3;
                double yDifference_1 = foundY_1 - (double)Y_3;
                double yDifference_2 = foundY_2 - (double)Y_3;
                double xDifferenceSquared = xDifference * xDifference;
                double firstRoot = xDifferenceSquared + yDifference_1 * yDifference_1;
                double secondRoot = xDifferenceSquared + yDifference_2 * yDifference_2;

                //check to confirm proper logic here.
                if(firstRoot >= 0 && 
                   secondRoot >= 0) // Both are valid square roots requires comparison.
                {
                    double distance1 = sqrt(firstRoot);
                    double offFactor1 = abs(distance1 - d_3);
                    double distance2 = sqrt(secondRoot);
                    double offFactor2 = abs(distance2 - d_3);
                    if(offFactor1 < offFactor2)
                    {
                        pairs[0].y = foundY_1;

                    }
                    else
                    {
                        pairs[0].y = foundY_2;
                    }
                    isOneValid = true;
                }
                else if(firstRoot >= 0) // Only one valid square root must be the answer.
                {
                    pairs[0].y = foundY_1;
                    isOneValid = true;
                }
                else if(secondRoot >= 0) // Only one valid square root must be the answer.
                {
                    pairs[0].y = foundY_2;
                    isOneValid = true;
                }
            }
        }
        else // In this case, Y_2 must be zero since the point is not the same point.
        {
            //subtract out the y and solve for x first then y
            double squareD_1 = d_1 * d_1;
            double squareD_2 = d_2 * d_2;
            double foundY = (squareD_1 - squareD_2 + (double)Y_2 * (double)Y_2) / (2 * ((double)Y_2));
            pairs[0].y = foundY;

            // Plug back in to x^2+y^2=d_1^2 solve for x.
            double squareFoundY = foundY * foundY;
            double foundX_1 = sqrt(squareD_1 - squareFoundY);

            if(foundX_1 == 0)
            {
                // If zero, this is the only point.
                pairs[0].x = foundX_1;
                isOneValid = true;
            }
            else
            {
                double foundX_2 = -foundX_1;

                //found1 is +- in the kit 
                //test it out using point 3
                double yDifference = foundY - (double)Y_3;
                double xDifference_1 = foundX_1 - (double)X_3;
                double xDifference_2 = foundX_2 - (double)X_3;
                double yDifferenceSquared = yDifference * yDifference;
                double firstRoot = yDifferenceSquared + xDifference_1 * xDifference_1;
                double secondRoot = yDifferenceSquared + xDifference_2 * xDifference_2;

                //check to confirm proper logic here
                if(firstRoot >= 0 &&
                   secondRoot >= 0) // Both are valid square roots requires comparison.
                {
                    double distance1 = sqrt(firstRoot);
                    double offFactor1 = abs(distance1 - d_3);
                    double distance2 = sqrt(secondRoot);
                    double offFactor2 = abs(distance2 - d_3);
                    if(offFactor1 < offFactor2)
                    {
                        pairs[0].x = foundX_1;

                    }
                    else
                    {
                        pairs[0].x = foundX_2;
                    }
                    isOneValid = true;
                }
                else if(firstRoot >= 0) // Only one valid square root must be the answer.
                {
                    pairs[0].x = foundX_1;
                    isOneValid = true;
                }
                else if(secondRoot >= 0) // Only one valid square root must be the answer.
                {
                    pairs[0].x = foundX_2;
                    isOneValid = true;
                }
            }
        }

        // Repeat with the farther coordinate.
        if(X_3 != 0)
        {
            //subtract out the x and solve for y first then x
            double squareD_1 = d_1 * d_1;
            double squareD_3 = d_3 * d_3;
            double foundX = (squareD_1 - squareD_3 + (double)X_3 * (double)X_3) / (2 * ((double)X_3));
            pairs[1].x = foundX;

            // Plug back in to x^2+y^2=d_1^2 solve for y.
            double squareFoundX = foundX * foundX;
            double foundY_1 = sqrt(squareD_1 - squareFoundX);

            if(foundY_1 == 0)
            {
                // If zero, this is the only point.
                pairs[1].y = foundY_1;
                isTwoValid = true;
            }
            else
            {
                double foundY_2 = -foundY_1;

                //found1 is +- in the kit 
                //test it out using point 2
                double xDifference = foundX - (double)X_2;
                double yDifference_1 = foundY_1 - (double)Y_2;
                double yDifference_2 = foundY_2 - (double)Y_2;
                double xDifferenceSquared = xDifference * xDifference;
                double firstRoot = xDifferenceSquared + yDifference_1 * yDifference_1;
                double secondRoot = xDifferenceSquared + yDifference_2 * yDifference_2;

                //check to confirm proper logic here.
                if(firstRoot >= 0 &&
                   secondRoot >= 0) // Both are valid square roots requires comparison.
                {
                    double distance1 = sqrt(firstRoot);
                    double offFactor1 = abs(distance1 - d_3);
                    double distance2 = sqrt(secondRoot);
                    double offFactor2 = abs(distance2 - d_3);
                    if(offFactor1 < offFactor2)
                    {
                        pairs[1].y = foundY_1;

                    }
                    else
                    {
                        pairs[1].y = foundY_2;
                    }
                    isTwoValid = true;
                }
                else if(firstRoot >= 0) // Only one valid square root must be the answer.
                {
                    pairs[1].y = foundY_1;
                    isTwoValid = true;
                }
                else if(secondRoot >= 0) // Only one valid square root must be the answer.
                {
                    pairs[1].y = foundY_2;
                    isTwoValid = true;
                }
            }
        }
        else // In this case, Y_3 must be zero since the point is not the same point.
        {
            //subtract out the y and solve for x first then y
            double squareD_1 = d_1 * d_1;
            double squareD_3 = d_3 * d_3;
            double foundY = (squareD_1 - squareD_3 + (double)Y_3 * (double)Y_3) / (2 * ((double)Y_3));
            pairs[1].y = foundY;

            // Plug back in to x^2+y^2=d_1^2 solve for x.
            double squareFoundY = foundY * foundY;
            double foundX_1 = sqrt(squareD_1 - squareFoundY);

            if(foundX_1 == 0)
            {
                // If zero, this is the only point.
                pairs[1].x = foundX_1;
                isTwoValid = true;
            }
            else
            {
                double foundX_2 = -foundX_1;

                //found1 is +- in the kit 
                //test it out using point 3
                double yDifference = foundY - (double)Y_2;
                double xDifference_1 = foundX_1 - (double)X_2;
                double xDifference_2 = foundX_2 - (double)X_2;
                double yDifferenceSquared = yDifference * yDifference;
                double firstRoot = yDifferenceSquared + xDifference_1 * xDifference_1;
                double secondRoot = yDifferenceSquared + xDifference_2 * xDifference_2;

                //check to confirm proper logic here
                if(firstRoot >= 0 &&
                    secondRoot >= 0) // Both are valid square roots requires comparison.
                {
                    double distance1 = sqrt(firstRoot);
                    double offFactor1 = abs(distance1 - d_3);
                    double distance2 = sqrt(secondRoot);
                    double offFactor2 = abs(distance2 - d_3);
                    if(offFactor1 < offFactor2)
                    {
                        pairs[1].x = foundX_1;

                    }
                    else
                    {
                        pairs[1].x = foundX_2;
                    }
                    isTwoValid = true;
                }
                else if(firstRoot >= 0) // Only one valid square root must be the answer.
                {
                    pairs[1].x = foundX_1;
                    isTwoValid = true;
                }
                else if(secondRoot >= 0) // Only one valid square root must be the answer.
                {
                    pairs[1].x = foundX_2;
                    isTwoValid = true;
                }
            }
        }

        // Get the average of the two measurements.
        CoordinatePair result { };
        if(!isOneValid && 
           !isTwoValid)
        {
            // Give the current position when no position can be found.
            result.x = 0;
            result.y = 0;
        }
        else if(!isOneValid) 
        {
            // Only the second coordinate must be considered.
            result.x = pairs[1].x;
            result.y = pairs[1].y;
        }
        else if(!isTwoValid) 
        {
            // Only the first coordinate must be considered.
            result.x = pairs[0].x;
            result.y = pairs[0].y;
        }
        else 
        {
            result.x = (pairs[0].x + pairs[1].x) / 2.0;
            result.y = (pairs[0].y + pairs[1].y) / 2.0;
        }

        // Convert back to original coordinate system.
        result.x += X_1;
        result.y += Y_1;

        return result;
    }
}