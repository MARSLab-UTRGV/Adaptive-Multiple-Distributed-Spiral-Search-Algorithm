#include "Region.h"

/*****
 * The  Region needs to keep track of four things:
 *
 * [1] location of the center
 * [2] top left corner point of the region
 * [3] bottom right corner point of the region
 *
 *****/
Region::Region(argos::CVector2 posC, argos::CVector2 posTL, argos::CVector2 posBR)
{
    /* required initializations */
	center = newLocation;
	topLeft = posTL;
	bottomRight = posBR;
	spiralPoints = GenerateSpiralPoints();
}

/*****
 * Return the Region's center.
 *****/
argos::CVector2 Region::GetRegionCenter() {
    return center;
}


argos::CVector2 Region::GetTopLeft(){
	return topLeft;
}


argos::CVector2 Region::GetBottomRight(){
	return bottomRight;
}

std::vector<argos::CVector2> Region::GenerateSpiralPoints() {
    return spiralPoints;
}

std::vector<argos::CVector2> Region::GetSpiralPoints() {
    return spiralPoints;
}

