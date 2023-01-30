#include "DSA_loop_functions.h"

DSA_loop_functions::DSA_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
    //MaxSimTime(3600 * GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick()),
    ResourceDensityDelay(0),
    //RandomSeed(GetSimulator().GetRandomSeed()),
    SimCounter(0),
    MaxSimCounter(1),
    VariableFoodPlacement(0),
    OutputData(0),
    DrawDensityRate(4),
//    DrawIDs(1),
    DrawIDs(0),
    DrawTrails(0),
    DrawTargetRays(0),
    FoodDistribution(1),
//    FoodDistribution(9),
    FoodItemCount(256),
    NumberOfClusters(4),
    ClusterWidthX(8),
    ClusterLengthY(8),
    PowerRank(4),
    FoodRadius(0.05),
    FoodRadiusSquared(0.0025),
    NestRadius(0.25),
    NestRadiusSquared(0.0625),
    NestElevation(0.01),
  SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
  ticks_per_second(0),
  sim_time(0),
  score(0),
  PrintFinalScore(0)
{}

void DSA_loop_functions::Init(TConfigurationNode& node) {
CSimulator     *simulator     = &GetSimulator();
  CPhysicsEngine *physicsEngine = &simulator->GetPhysicsEngine("default");
  ticks_per_second = physicsEngine->GetInverseSimulationClockTick();
 argos::TConfigurationNode DDSA_node = argos::GetNode(node, "DDSA");
 argos::GetNodeAttribute(DDSA_node, "PrintFinalScore",                   PrintFinalScore);
 argos::GetNodeAttribute(DDSA_node, "FoodDistribution",                  FoodDistribution);
 argos::GetNodeAttribute(DDSA_node, "FoodItemCount",                  FoodItemCount);
 argos::GetNodeAttribute(DDSA_node, "NestRadius",                 NestRadius);
 //argos::GetNodeAttribute(DDSA_node, "FoodBoundsWidth",                 FoodBoundsWidth);
 //argos::GetNodeAttribute(DDSA_node, "FoodBoundsHeight",                 FoodBoundsHeight); qilu 12/2022

 NestRadiusSquared = NestRadius*NestRadius;

    //Calculate the forage range 
    //The robot's radius is 0.085m
	//The half of wall width: 0.05/2 meter 
	//The proximity range is 10cm (or 0.1m) qilu 12/2022 
	
    argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
    argos::Real rangeX = ArenaSize.GetX() / 2.0;
	argos::Real rangeY = ArenaSize.GetY() / 2.0;
	
    //argos::Real rangeX = FoodBoundsWidth/2.0;//(ArenaSize.GetX() / 2.0) - 0.085;
    //argos::Real rangeY = FoodBoundsHeight/2.0;//(ArenaSize.GetY() / 2.0) - 0.085;  
    ForageRangeX.Set(-rangeX, rangeX);
    ForageRangeY.Set(-rangeY, rangeY);

    CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    CSpace::TMapPerType::iterator it;
 
	for(it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
        DSA_controller& c2 = dynamic_cast<DSA_controller&>(c);

        c2.SetLoopFunctions(this);
	}
    
	SetFoodDistribution();
}


double DSA_loop_functions::Score()
{  
  return score;
}


void DSA_loop_functions::setScore(double s)
{
  score = s;
  if (score >= FoodItemCount) 
    {
      PostExperiment();
      exit(0);
    }
}

void DSA_loop_functions::PostExperiment() 
{
  if (PrintFinalScore == 1) printf("%f, %f\n", getSimTimeInSeconds(), score);
}


void DSA_loop_functions::PreStep() 
{
    sim_time++;
}

argos::Real DSA_loop_functions::getSimTimeInSeconds()
{
  return sim_time/ticks_per_second;
}


/*****
 *
 *****/
void DSA_loop_functions::SetFoodDistribution() {
    switch(FoodDistribution) {
        case 0:
            RandomFoodDistribution();
            break;
        case 1:
            ClusterFoodDistribution();
            break;
        case 2:
            PowerLawFoodDistribution();
            break;
        default:
            argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
    }
}

/*****
 *
 *****/
void DSA_loop_functions::RandomFoodDistribution() {
    FoodList.clear();

    argos::CVector2 placementPosition;

    for(size_t i = 0; i < FoodItemCount; i++) {
        placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

        while(IsOutOfBounds(placementPosition, 1, 1)) {
            placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
        }

        FoodList.push_back(placementPosition);
        FoodColoringList.push_back(argos::CColor::BLACK);
    }
}

void DSA_loop_functions::ClusterFoodDistribution() {
    FoodList.clear();
    
	argos::Real     foodOffset  = 3.0 * FoodRadius;
	size_t          foodToPlace = FoodItemCount;//Wayne: Changed since no longer necessary
	size_t          foodPlaced = 0;
	argos::CVector2 placementPosition;

    FindClusterLengthWidth();//Wayne: sets cluster sides (X,Y)

    //-----Wayne: Creates vector of number of food in each cluster
    size_t index = 0;
    size_t ClusterFoodCount = 0;
    size_t foodCount = 0;
    vector <size_t> FoodClusterCount;
    
    //initialize vector
    for (int i = 0; i < NumberOfClusters; i++){
        FoodClusterCount.push_back(0);
    }
    
    //add food
    while (foodCount < FoodItemCount){
        FoodClusterCount[index] = FoodClusterCount[index]+ 1;
        foodCount++;
        index++;
        if (index == NumberOfClusters) index = 0;
        
    }
    
    //make vector cumulative in food
    for (int i = 1; i < NumberOfClusters; i++){
        FoodClusterCount[i] += FoodClusterCount[i - 1];
    }
    //------Wayne: end of vector creation
    
	for(size_t i = 0; i < NumberOfClusters; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, ClusterLengthY, ClusterWidthX)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}
        
        /*Wayne: Modified to break from loops if food count reached.
         Provides support for unequal clusters and odd food numbers.
         Necessary for DustUp and Jumble Distribution changes. */
        
		for(size_t j = 0; j < ClusterLengthY; j++) {
			for(size_t k = 0; k < ClusterWidthX; k++) {
				foodPlaced++;
				/*
				#include <argos3/plugins/simulator/entities/box_entity.h>

				string label("my_box_");
				label.push_back('0' + foodPlaced++);

				CBoxEntity *b = new CBoxEntity(label,
					CVector3(placementPosition.GetX(),
					placementPosition.GetY(), 0.0), CQuaternion(), true,
					CVector3(0.1, 0.1, 0.001), 1.0);
				AddEntity(*b);
				*/

				FoodList.push_back(placementPosition);
				FoodColoringList.push_back(argos::CColor::BLACK);
				placementPosition.SetX(placementPosition.GetX() + foodOffset);
                if (foodPlaced == FoodClusterCount[i]) break;
			}

			placementPosition.SetX(placementPosition.GetX() - (ClusterWidthX * foodOffset));
			placementPosition.SetY(placementPosition.GetY() + foodOffset);
            if (foodPlaced == FoodClusterCount[i]) break;
		}
        if (foodPlaced == FoodItemCount) break;
	}
}

void DSA_loop_functions::PowerLawFoodDistribution() {
	FoodList.clear();
    
    argos::Real foodOffset     = 3.0 * FoodRadius;
	size_t      foodPlaced     = 0;
	size_t      powerLawLength = 1;
	size_t      maxTrials      = 200;
	size_t      trialCount     = 0;

	std::vector<size_t> powerLawClusters;
	std::vector<size_t> clusterSides;
	argos::CVector2     placementPosition;
    
    //-----Wayne: Dertermine PowerRank and food per PowerRank group
    size_t priorPowerRank = 0;
    size_t power4 = 0;
    size_t FoodCount = 0;
    size_t diffFoodCount = 0;
    size_t singleClusterCount = 0;
    size_t otherClusterCount = 0;
    size_t modDiff = 0;
    
    //Wayne: priorPowerRank is determined by what power of 4
    //plus a multiple of power4 increases the food count passed required count
    //this is how powerlaw works to divide up food into groups
    //the number of groups is the powerrank
    while (FoodCount < FoodItemCount){
        priorPowerRank++;
        power4 = pow (4.0, priorPowerRank);
        FoodCount = power4 + priorPowerRank * power4;
    }
    
    //Wayne: Actual powerRank is prior + 1
    PowerRank = priorPowerRank + 1;
    
    //Wayne: Equalizes out the amount of food in each group, with the 1 cluster group taking the
    //largest loss if not equal, when the powerrank is not a perfect fit with the amount of food.
    diffFoodCount = FoodCount - FoodItemCount;
    modDiff = diffFoodCount % PowerRank;
    
    if (FoodItemCount % PowerRank == 0){
        singleClusterCount = FoodItemCount / PowerRank;
        otherClusterCount = singleClusterCount;
    }
    else {
        otherClusterCount = FoodItemCount / PowerRank + 1;
        singleClusterCount = otherClusterCount - modDiff;
    }
    //-----Wayne: End of PowerRank and food per PowerRank group
    
	for(size_t i = 0; i < PowerRank; i++) {
		powerLawClusters.push_back(powerLawLength * powerLawLength);
		powerLawLength *= 2;
	}
    
	for(size_t i = 0; i < PowerRank; i++) {
		powerLawLength /= 2;
		clusterSides.push_back(powerLawLength);
	}

    /*Wayne: Modified to break from loops if food count reached.
     Provides support for unequal clusters and odd food numbers.
     Necessary for DustUp and Jumble Distribution changes. */
    
	for(size_t h = 0; h < powerLawClusters.size(); h++) {
		for(size_t i = 0; i < powerLawClusters[h]; i++) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

			while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
				trialCount++;
				placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

				if(trialCount > maxTrials) {
					argos::LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
					break;
				}
			}

            trialCount = 0;
			for(size_t j = 0; j < clusterSides[h]; j++) {
				for(size_t k = 0; k < clusterSides[h]; k++) {
					foodPlaced++;
					FoodList.push_back(placementPosition);
					FoodColoringList.push_back(argos::CColor::BLACK);
					placementPosition.SetX(placementPosition.GetX() + foodOffset);
                    if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
				}

				placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
				placementPosition.SetY(placementPosition.GetY() + foodOffset);
                if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
            if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
		}
	}
    
}


//Wayne: makes as square a cluster as it can
void DSA_loop_functions::FindClusterLengthWidth(){
    
    size_t tempClusterAreaCount = 0;
    size_t tempFoodItemCount =  FoodItemCount;
    
    while (tempFoodItemCount % NumberOfClusters != 0){
        tempFoodItemCount++;
    }
    
    //Find number of seeds in cluster
    size_t ClusterAreaCount = tempFoodItemCount / NumberOfClusters;
    
    //Find square root (max for both sides)
    size_t x =  sqrt(ClusterAreaCount);
    
    if (ClusterAreaCount % x != 0 || (x == 1 && FoodItemCount > NumberOfClusters)){
        ClusterLengthY = x + 1;
        ClusterWidthX = x + 1;
    }
    else {
        ClusterWidthX = x;
        ClusterLengthY = x;
    }
}


/*****
 *
 *****/
bool DSA_loop_functions::IsOutOfBounds(argos::CVector2 p, size_t length, size_t width) {
    argos::CVector2 placementPosition = p;

    argos::Real foodOffset   = 3.0 * FoodRadius;
    argos::Real widthOffset  = 3.0 * FoodRadius * (argos::Real)width;
    argos::Real lengthOffset = 3.0 * FoodRadius * (argos::Real)length;

    argos::Real x_min = p.GetX() - FoodRadius;
    argos::Real x_max = p.GetX() + FoodRadius + widthOffset;

    argos::Real y_min = p.GetY() - FoodRadius;
    argos::Real y_max = p.GetY() + FoodRadius + lengthOffset;

    if((x_min < (ForageRangeX.GetMin() + FoodRadius)) ||
       (x_max > (ForageRangeX.GetMax() - FoodRadius)) ||
       (y_min < (ForageRangeY.GetMin() + FoodRadius)) ||
       (y_max > (ForageRangeY.GetMax() - FoodRadius))) {
        return true;
    }

    for(size_t j = 0; j < length; j++) {
        for(size_t k = 0; k < width; k++) {
            if(IsCollidingWithFood(placementPosition)) return true;
            if(IsCollidingWithNest(placementPosition)) return true;
            placementPosition.SetX(placementPosition.GetX() + foodOffset);
        }

        placementPosition.SetX(placementPosition.GetX() - (width * foodOffset));
        placementPosition.SetY(placementPosition.GetY() + foodOffset);
    }

    return false;
}

/*****
 *
 *****/
bool DSA_loop_functions::IsCollidingWithNest(argos::CVector2 p) {
    argos::Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
    argos::Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

    return ((p - NestPosition).SquareLength() < NRPB_squared);
}

/*****
 *
 *****/
bool DSA_loop_functions::IsCollidingWithFood(argos::CVector2 p) {
    argos::Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
    argos::Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

    for(size_t i = 0; i < FoodList.size(); i++) {
        if((p - FoodList[i]).SquareLength() < FRPB_squared) return true;
    }

    return false;
}

REGISTER_LOOP_FUNCTIONS(DSA_loop_functions, "DSA_loop_functions")
