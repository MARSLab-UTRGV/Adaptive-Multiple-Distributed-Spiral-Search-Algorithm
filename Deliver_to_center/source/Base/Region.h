#ifndef IANT_REGION_H
#define IANT_REGION_H

#include <argos3/core/utility/math/vector2.h>
using namespace argos;
using namespace std;
/*****
 * Implementation of the regions for distributed robots.
 * The Region includes the center location, the boundary of the region. 
 * The boundary is defined by the top left corner and bottom right corner. qilu 02/2023 
 *****/
class Region {

    public:

        /* constructor function */
		Region(CVector2 posC, CVector2 posTL, CVector2 posBR);

        /* public helper functions */
        //CVector2            GetCenter();
        //CVector2 			GetTopLeft();
		//CVector2            GetBottomRight();
		//vector<CVector2>  	GenerateSpiralPoints();
		//vector<CVector2> 	GetSpiralPoints();
		CVector2              center;
		

	private:

        /* Region position variables */
		
		//argos::CVector2              topLeft;
		//argos::CVector2              bottomRight;
		vector<CVector2>			 points;
		
     
};

#endif /* IANT_REGION_H */
