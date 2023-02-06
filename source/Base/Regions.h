#ifndef IANT_REGION_H
#define IANT_REGION_H

#include <argos3/core/utility/math/vector2.h>

/*****
 * Implementation of the regions for distributed robots.
 * The Region includes the center location, the boundary of the region. 
 * The boundary is defined by the top left corner and bottom right corner. qilu 02/2023 
 *****/
class Region {

    public:

        /* constructor function */
		Region(argos::CVector2 posC, argos::CVector2 posTL, argos::CVector2 posBR);

        /* public helper functions */
        argos::CVector2             GetCenter();
        argos::CVector2 			GetTopLeft();
		argos::Real                 GetBottomRight();
		

	private:

        /* Region position variables */
		argos::CVector2              center;
		argos::CVector2              topLeft;
		argos::CVector2              bottomRight;
		
     
};

#endif /* IANT_REGION_H */
