#ifndef DSA_LOOP_FUNCTIONS_H
#define DSA_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <source/DSA/DSA_controller.h>

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
	
        /* list variables for food & pheromones */
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

	double score;
	int PrintFinalScore;
};

#endif /* DSA_LOOP_FUNCTIONS_H */
