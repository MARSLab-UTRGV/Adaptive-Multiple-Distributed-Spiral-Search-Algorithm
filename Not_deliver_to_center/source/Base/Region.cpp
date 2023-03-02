#include "Region.h"

/*****
 * The  Region needs to keep track of four things:
 *
 * [1] location of the center
 * [2] top left corner point of the region
 * [3] bottom right corner point of the region
 *
 *****/
Region::Region(CVector2 posC, CVector2 posTL, CVector2 posBR)
{
    /* required initializations */
	center = posC;
	//topLeft = posTL;
	//bottomRight = posBR;
	//spiralPoints = GenerateSpiralPoints();
	//points.push_back(CVector2(0, 0));
}

/*****
 * Return the Region's center.
 *****/
//CVector2 Region::GetCenter() {
//    return center;
//}


//CVector2 Region::GetTopLeft(){
//	return topLeft;
//}


//CVector2 Region::GetBottomRight(){
//	return bottomRight;
//}

//vector<CVector2> Region::GenerateSpiralPoints() {
//	vector<CVector2> pts;
//    return pts;
//}

//vector<CVector2> Region::GetSpiralPoints() {
//    return points;
//}

