#ifndef DSA_LOOP_FUNCTIONS_H
#define DSA_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/DSA/DSA_controller.h>
//#include <source/Base/Region.h>
using namespace argos;
using namespace std;

class DSA_loop_functions : public argos::CLoopFunctions {

	friend class DSA_controller;
	friend class DSA_qt_user_functions;

	public:

		DSA_loop_functions();

		void Init(TConfigurationNode& node);
		void PreStep();

		void PostExperiment();

		/* Calculates the performance of the robot in a trial */
		Real Score();
	

        void SetFoodDistribution();
        
        void calRegions(int num_regions); //qilu 12/2022
        void generatePattern(int N_robots);
		int    calcDistanceToTravel(int ith_robot, int i_circuit, int N_robots, char direction);
        size_t NumOfRobots;
        vector<vector<CVector2>> spiralPoints; //qilu 02/2023	

	argos::Real getSimTimeInSeconds();

	protected:
	
	

	void setScore(double s);

        argos::CRandom::CRNG* RNG;

	size_t sim_time;
	size_t ticks_per_second;
        size_t MaxSimTime;
        size_t ResourceDensityDelay;
        size_t RandomSeed;
        size_t SimCounter;
        size_t MaxSimCounter;
        size_t VariableFoodPlacement;
        size_t OutputData;
        size_t DrawDensityRate;
        size_t DrawIDs;
        size_t DrawTrails;
        size_t DrawTargetRays;
        size_t FoodDistribution;
        size_t FoodItemCount;
        size_t NumberOfClusters;
        size_t ClusterWidthX;
        size_t ClusterLengthY;
        size_t PowerRank;

		CVector2 NestPosition;

        /* physical robot & world variables */
        argos::Real FoodRadius;
        argos::Real FoodRadiusSquared;
        argos::Real NestRadius;
        argos::Real NestRadiusSquared;
        argos::Real NestElevation;
        argos::Real SearchRadiusSquared;

	argos::Real FoodBoundsWidth;
	argos::Real FoodBoundsHeight;
	/* regions qilu 02/2023*/
	//std::vector<Region>   RegionList; 
	
	
        /* list variables for food */
        std::vector<argos::CVector2> FoodList;

        std::vector<argos::CColor>   FoodColoringList;
        std::vector<argos::CVector2> FidelityList;

        std::vector<argos::CRay3>    TargetRayList;
        std::vector<argos::CColor>   TargetRayColorList;

        argos::CRange<argos::Real>   ForageRangeX;
        argos::CRange<argos::Real>   ForageRangeY;

    private:

        /* private helper functions */
        void RandomFoodDistribution();
        void ClusterFoodDistribution();
        void PowerLawFoodDistribution();
	void FindClusterLengthWidth();
        bool IsOutOfBounds(argos::CVector2 p, size_t length, size_t width);
        bool IsCollidingWithNest(argos::CVector2 p);
        bool IsCollidingWithFood(argos::CVector2 p);
        bool CanGenerateSpiralPoint(int idx_robot, CVector2 point);

	double score;
	int PrintFinalScore;
	
	vector<CVector2>    centers; //qilu 2/2023
        vector<CVector2>    topLeftPts; //qilu 2/2023
        vector<CVector2>    bottomRightPts; //qilu 2/2023
        size_t NumberOfSpirals;
       size_t	RobotID; // start from 0 qilu 12/2022
         Real                SearcherGap;
        
};

#endif /* DSA_LOOP_FUNCTIONS_H */
