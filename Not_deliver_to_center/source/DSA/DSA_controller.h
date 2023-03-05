#ifndef DSA_CONTROLLER_H
#define DSA_CONTROLLER_H

#include <source/Base/BaseController.h>
#include <source/DSA/DSA_loop_functions.h>
//#include <source/Base/Region.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <cmath>

using namespace argos;
using namespace std;

class DSA_loop_functions;

static unsigned int num_targets_collected = 0;

class DSA_controller : public BaseController {

    public:

        DSA_controller();

        // CCI_Controller inheritence functions
        void Init(TConfigurationNode& node);
        void ControlStep();
        void Reset();

        bool   IsHoldingFood();
        bool   IsInTheNest(); //qilu 02/2023
        bool   IsInTheRegion();
        void   getSpiralPath(size_t regID);
        bool   GetSpiralPath();
        int    calcDistanceToTravel(int ith_robot, int i_circuit, int N_robots, char direction);
        void   calRegions(); //qilu 12/2022
		void   writePatternToFile(vector<char>&, int N_robots);
		void   addDirectionToPattern(char direction);
		void   printPath(vector<char>&);

		void SetLoopFunctions(DSA_loop_functions* lf) { loopFunctions = lf; }

		argos::Real SimTimeInSeconds();

    private:
    
		//Region region; //qilu 02/2023
		string 	controllerID;
		size_t	RobotID; // start from 0 qilu 12/2022
		size_t  RegionID;
		size_t  PreRegionID;
		bool    firstAssigned;

        size_t NumberOfRobots;
        size_t NumberOfSpirals;
        size_t NumOfRegions;

        /* Robot DSA state variable */
        enum DSA { START = 0, SEARCHING = 1, RETURN_TO_REGION = 2, RETURN_TO_SEARCH = 3, IDLE = 4 } DSA;

        /* robot internal variables & statistics */
        CRandom::CRNG*      RNG;
        DSA_loop_functions* loopFunctions;

        CVector2            ReturnPosition;
        CVector2            ReturnSpiralPosition;

        vector<CRay3>       myTrail;
        CColor              TrailColor;

		Real                ProbTargetDetection;
        Real                SearcherGap;
        Real                FoodDistanceTolerance;
        Real                SquaredFoodDistanceTolerance;
       	CVector2            previous_position;
		CVector2            previous_target;
		CVector2            newTarget;
        CVector3            startPosition;
        
        vector<char>        pattern;
        vector<CVector2>    spiral; //qilu 2/2023
        vector<char>        tempPattern;
        vector<string>      rPattern;
        vector<CVector2>	robotSpiralPoints; //qilu 2/2023
        int                 levels;
        bool                isHoldingFood;
        bool                goingHome;
        bool                ResetReturnPosition;
        CRange<CRadians>    AngleToleranceInRadians;
        CRange<CRadians>    Tolerance;
        size_t              stopTimeStep;
        size_t              collisionDelay;
	    char 				direction_last;
	    CVector2			nextSpiralPoint; //qilu 2/2023
        
        size_t targetsPerMin;

        /* movement functions */
        CDegrees angleInDegrees;
    
        /* movement helper functions */
        void ReachSpiralTargets();
        void CopyPatterntoTemp();
        bool TargetHit();
        void SetHoldingFood(); 
        
	string results_path;
	string results_full_path;
		/* Pointer to the LEDs actuator */
        CCI_LEDsActuator* m_pcLEDs;
};

#endif /* DSA_CONTROLLER_H */
