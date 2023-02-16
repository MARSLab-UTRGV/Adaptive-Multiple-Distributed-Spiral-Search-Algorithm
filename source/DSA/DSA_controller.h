#ifndef DSA_CONTROLLER_H
#define DSA_CONTROLLER_H

#include <source/Base/BaseController.h>
#include <source/DSA/DSA_loop_functions.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

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
        void   GetPattern(string ith_Pattern, vector<CVector2> spiralPoints);
        void   SetRobotPath(string path);
		void generatePattern(int N_circuits, int N_robots);
		int    calcDistanceToTravel(int ith_robot, int i_circuit, int N_robots, char direction);
		void calRegions(int num_regions); //qilu 12/2022
		void   writePatternToFile(vector<char>&, int N_robots);
		void   addDirectionToPattern(char direction);
		void   printPath(vector<char>&);

		void SetLoopFunctions(DSA_loop_functions* lf) { loopFunctions = lf; }

		argos::Real SimTimeInSeconds();

    private:
  string 			controllerID;
  size_t	 RobotNumber; //qilu 12/2022

        size_t NumberOfRobots;
        size_t NumberOfSpirals;

        /* Robot DSA state variable */
        enum DSA { SEARCHING = 1, RETURN_TO_NEST = 2, RETURN_TO_SEARCH = 3, IDLE = 4 } DSA;

        /* robot internal variables & statistics */
        CRandom::CRNG*      RNG;
        DSA_loop_functions* loopFunctions;

        CVector2            ReturnPosition;
        CVector2            ReturnPatternPosition;

        vector<CRay3>       myTrail;
        CColor              TrailColor;

	Real                ProbTargetDetection;
        Real                SearcherGap;
        Real                FoodDistanceTolerance;
       	CVector2            previous_position;
	CVector2            previous_target;
	CVector2            newTarget;
        CVector3            startPosition;
        vector<CVector2>    centers; //qilu 12/2022
        vector<CVector2>    topLeftPts; //qilu 2/2023
        vector<CVector2>    bottomRightPts; //qilu 2/2023
        
        vector<char>        pattern;
        vector<CVector2>    spiral; //qilu 2/2023
        vector<char>        tempPattern;
        vector<string>      rPattern;
        vector<CVector2>	tempSpiralPoints; //qilu 2/2023
        int                 levels;
        bool                isHoldingFood;
        bool                goingHome;
        bool                ResetReturnPosition;
        CRange<CRadians>    AngleToleranceInRadians;
        CRange<CRadians>    Tolerance;
        size_t              stopTimeStep;
        size_t              collisionDelay;
	    char 				direction_last;
	    CVector2			spiral_last; //qilu 2/2023

        /* movement functions */
        CDegrees angleInDegrees;

        void SetTargetN(char x);
        void SetTargetS(char x);
        void SetTargetE(char x);
        void SetTargetW(char x);
        void SetTargetO(char x);
    
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
