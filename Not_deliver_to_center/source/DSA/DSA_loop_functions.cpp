#include "DSA_loop_functions.h"

DSA_loop_functions::DSA_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
    //MaxSimTime(3600 * GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick()),
        CollisionTime(0), 
        lastNumCollectedFood(0),
        currNumCollectedFood(0),
    ResourceDensityDelay(0),
    RandomSeed(GetSimulator().GetRandomSeed()),
    SimCounter(0),
    MaxSimCounter(1),
    VariableFoodPlacement(0),
    OutputData(0),
    DrawDensityRate(4),
    DrawIDs(1),
    //DrawIDs(0),
    DrawTrails(0),
    IdleCount(0),
    DrawTargetRays(0),
    FoodDistribution(1),
    FoodItemCount(256),
    NumberOfClusters(4),
    ClusterWidthX(8),
    ClusterLengthY(8),
    PowerRank(4),
    FoodRadius(0.05),
    FoodRadiusSquared(0.0025),
    NestRadius(0.25),
    NestRadiusSquared(0.0625),
    RegionRadius(0.25),
    RegionRadiusSquared(0.0625),
    NestElevation(0.01),
    SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
    ticks_per_second(0),
    sim_time(0),
    score(0),
    PrintFinalScore(0),
    FilenameHeader("\0"),
    scoreLastMinute(0)
{}

void DSA_loop_functions::Init(TConfigurationNode& node) {
	CSimulator     *simulator     = &GetSimulator();
	CPhysicsEngine *physicsEngine = &simulator->GetPhysicsEngine("dyn2d");
	ticks_per_second = physicsEngine->GetInverseSimulationClockTick();
	argos::TConfigurationNode DDSA_node = argos::GetNode(node, "DDSA");
	argos::GetNodeAttribute(DDSA_node, "PrintFinalScore",     PrintFinalScore);
	argos::GetNodeAttribute(DDSA_node, "FoodDistribution",    FoodDistribution);
	argos::GetNodeAttribute(DDSA_node, "FoodItemCount",       FoodItemCount);
	argos::GetNodeAttribute(DDSA_node, "NestRadius",          NestRadius);
	argos::GetNodeAttribute(DDSA_node, "RegionRadius",        RegionRadius);
	argos::GetNodeAttribute(DDSA_node, "SearcherGap",         SearcherGap);
	argos::GetNodeAttribute(DDSA_node, "NumOfRegions",        NumOfRegions);
    argos::GetNodeAttribute(DDSA_node, "FilenameHeader",      FilenameHeader);
   
	NestRadiusSquared = NestRadius*NestRadius;
	RegionRadiusSquared = RegionRadius*RegionRadius;
    //Calculate the forage range 
    //The robot's radius is 0.085m
	//The proximity range is 10cm (or 0.1m) qilu 12/2022 
	
    argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
    argos::Real rangeX = ArenaSize.GetX() / 2.0;
	argos::Real rangeY = ArenaSize.GetY() / 2.0;
	
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
	NumOfRobots = footbots.size();
    calRegions();
    generateSpiralPath();
	
}


void DSA_loop_functions::calRegions()
{
	int num_rows = sqrt(NumOfRegions);
	int num_cols = num_rows;
	
	double_t unit = ForageRangeX.GetMax()/num_rows;
	double_t rangeMax = ForageRangeX.GetMax();
	
	CVector2 location;
	CVector2 pos;
	vector<CVector2> tmpRegionCenters, tmpTopLeftPts, tmpBottomRightPts;
	vector<pair<Real, int>> dist;
	int count=0, idx;
	for(int i =0; i < num_rows; i++)
	{
		for(int j =0; j < num_cols; j++)
		{
		location = CVector2(rangeMax-(2*i+1)*unit, rangeMax-(2*j+1)*unit);
		dist.push_back(make_pair(location.SquareLength(), count));
		count++;
		
		tmpRegionCenters.push_back(location);
		//LOG << "region center["<<i<<","<<j<<"]="<<location<<endl;
		pos = CVector2(location.GetX()+unit, location.GetY()+unit);
		tmpTopLeftPts.push_back(pos);
		pos = CVector2(location.GetX()-unit, location.GetY()-unit);
		tmpBottomRightPts.push_back(pos);
		}
	}
	sort(dist.begin(), dist.end());
	for(int i=0; i < dist.size(); i++)
	{
		idx = dist[i].second;
		regionCenters.push_back(tmpRegionCenters[idx]);
		topLeftPts.push_back(tmpTopLeftPts[idx]);
		bottomRightPts.push_back(tmpBottomRightPts[idx]);
		//LOG << "New region center["<<i<<"]="<<tmpRegionCenters[idx]<<endl;
		
		}
	
	
}

void DSA_loop_functions::generateSpiralPath()
{
	vector<CVector2> pointVector;
	CVector2 point;
	argos::Real x, y;
    int n_steps_north, n_steps_east, n_steps_south, n_steps_west;
    int N_robot_per_region = 1;
    
    for (int i_region = 0; i_region < NumOfRegions; i_region++)
    {
		point = regionCenters[i_region];
        pointVector.push_back(point);
        
        for(int i_circuit=1; i_circuit < 50; i_circuit++) //assume the max is 50        
		{ 
            n_steps_north = calcDistanceToTravel(i_region, i_circuit, N_robot_per_region, 'N');
           
			while(n_steps_north >= 3) //create a point in every 3 searchGap
			{	
				x = point.GetX() + 3*SearcherGap;
				y = point.GetY();
				point = CVector2(x, y);
				pointVector.push_back(point);
				n_steps_north -= 3;
			}
			if(n_steps_north > 0)
			{
				x = point.GetX() + n_steps_north*SearcherGap;
				y = point.GetY();
				point = CVector2(x, y);
				pointVector.push_back(point);
			} 
            if(!canGenerateSpiralPoint(i_region, point)) break;
                
 
        
            n_steps_east = calcDistanceToTravel(i_region, i_circuit, N_robot_per_region, 'E');
            while(n_steps_east >= 3)
			{	
				x = point.GetX();
				y = point.GetY() - 3*SearcherGap;
				point = CVector2(x, y);
				pointVector.push_back(point);
				n_steps_east -= 3;
			}
			if(n_steps_east > 0)
			{
				x = point.GetX();
				y = point.GetY() - n_steps_east*SearcherGap;
				point = CVector2(x, y);
				pointVector.push_back(point);
			} 
            if(!canGenerateSpiralPoint(i_region, point)) break;
             
            
            n_steps_south = calcDistanceToTravel(i_region, i_circuit, N_robot_per_region, 'S');
            
            while(n_steps_south >= 3)
			{	
				x = point.GetX() - 3*SearcherGap;
				y = point.GetY();
				point = CVector2(x, y);
				pointVector.push_back(point);
				n_steps_south -= 3;
			}
			if(n_steps_south > 0)
			{
				x = point.GetX() - n_steps_south*SearcherGap;
				y = point.GetY();
				point = CVector2(x, y);
				pointVector.push_back(point);
			} 
            if(!canGenerateSpiralPoint(i_region, point)) break;
            
            
            
            n_steps_west = calcDistanceToTravel(i_region, i_circuit, N_robot_per_region, 'W');
           
            while(n_steps_west >= 3)
			{	
				x = point.GetX();
				y = point.GetY() + 3*SearcherGap;
				point = CVector2(x, y);
				pointVector.push_back(point);
				n_steps_west -= 3;
			}
			if(n_steps_west > 0)
			{
				x = point.GetX();
				y = point.GetY() + n_steps_west*SearcherGap;
				point = CVector2(x, y);
				pointVector.push_back(point);
			} 
            if(!canGenerateSpiralPoint(i_region, point)) break;
            
        } //end of generating a path
       //LOG<<"Generated all spiral paths ..."<<endl;
        spiralPoints.push_back(pointVector);
        pointVector.clear();   
        
        shareFlag.push_back(false);
        shareAssignUpdated.push_back(true); //do not allow updates at the beginning
        singleAssignFlag.push_back(false);
        currSpiralTarget.push_back(CVector2(0, 0));
    }
    /*for(int i=0; i< spiralPoints.size(); i++)
    {
        LOG<<"spiralPoints[" << i << "]="<<endl;
        for(int j=0; j< spiralPoints[i].size(); j++)
        LOG <<"["<<spiralPoints[i][j]<<"], ";
        LOG<<endl;
    }*/
}

bool DSA_loop_functions::canGenerateSpiralPoint(int idx_region, CVector2 point)
{
    if((point.GetX()> topLeftPts[idx_region].GetX() && point.GetY() > topLeftPts[idx_region].GetY()) || (point.GetX()< bottomRightPts[idx_region].GetX() && point.GetY() < bottomRightPts[idx_region].GetY()) )
        return false;
    else return true;
}
    
int DSA_loop_functions::calcDistanceToTravel(int ith_robot, int ith_circuit, int n_robots, char direction)
{
    ith_robot = 0; // qilu 2/2023 one robot in each region 
    
    int n_steps  = 0;

	// the following algorithm is based on Qi Lu's 2019 ICRA paper 
    if (direction == 'N' || direction == 'E')
    {
        if (ith_circuit == 1)
        {
            n_steps = ith_robot;
            return n_steps;
        }
        else 
        {
            n_steps = (2*ith_circuit -3)*n_robots + 2*ith_robot;
            return n_steps;
        }
    }

    else if (direction == 'S' || direction == 'W')
    {
        if (ith_circuit == 1)
        {
            n_steps = 2*ith_robot;
            return n_steps;
        }

        else
        {
            n_steps = calcDistanceToTravel(ith_robot, ith_circuit, n_robots, 'N') + n_robots;
            return n_steps;
        }
    }  
    
    return 0;
}

double DSA_loop_functions::Score()
{  
  return score;
}


void DSA_loop_functions::setScore(double s)
{
  score = s;
	
}

void DSA_loop_functions::PostExperiment() 
{
//   if (PrintFinalScore == 1) 
//   {		printf("Time(s), Collected, Total, Percentage\n");
// 	  printf("%0.2lf, \t %d, \t %d, \t %0.1f\%\n", getSimTimeInSeconds(), (int)score, (int)FoodItemCount, 100*score/FoodItemCount);
//   }

    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
        DSA_controller& c2 = dynamic_cast<DSA_controller&>(c);
        CollisionTime += c2.GetCollisionTime();
    }

    ofstream DataOut((FilenameHeader+"MDSA-D-Data.txt").c_str(), ios::app);
    if (DataOut.tellp()==0){

        DataOut << "Sim Time(s), Collected, Total, Percentage, Collision Time(s)\n";
        
    }
    DataOut << getSimTimeInSeconds() << "," << (int)score << "," << (int)FoodItemCount << "," << 100*score/FoodItemCount << "," << CollisionTime/(2*ticks_per_second) << endl;
		DataOut.close();
	LOG << "Sim Time(s), Collected, Total, Percentage, Collision Time(s)\n";
    LOG << getSimTimeInSeconds() << "," << (int)score << "," << (int)FoodItemCount << "," << 100*score/FoodItemCount << "%," << CollisionTime/(2*ticks_per_second) << endl;
    size_t tmp = 0;

    for (size_t fpm : foodPerMinute){
        tmp += fpm;
    }

    if (tmp > (int)FoodItemCount){
        LOGERR << "Total number of collected food items > number of items in the simulation..." << endl;
        LOGERR << "Number of collected food: " << tmp << ", Number of food in simulation: " << (int)FoodItemCount << endl;
    }

    // the food collected in the remaining simulation time is discarded if the remaining simulation time < 60 seconds (1 minute)
    ofstream DataOut2((FilenameHeader+"MDSA-D-TargetsPerMin.txt").c_str(), ios::app);
    if (DataOut2.tellp()==0){
		DataOut << "Collected per second\n";
		}
    for (size_t fpm : foodPerMinute){
            DataOut2 << fpm << ",";
		}
		DataOut2<<"\n";
	DataOut2.close();
}


void DSA_loop_functions::PreStep() 
{
    sim_time++;

    // get num collected for for each minute
    curr_time_in_minutes = getSimTimeInSeconds()/60.0;
    if(curr_time_in_minutes - last_time_in_minutes==1){      
        //LOG << "Minute Passed... getSimTimeInSeconds: " << getSimTimeInSeconds() << ", Food Collected: " << currNumCollectedFood - lastNumCollectedFood << endl;
        foodPerMinute.push_back(currNumCollectedFood - lastNumCollectedFood);
        lastNumCollectedFood = currNumCollectedFood;
        last_time_in_minutes++;
	}
        
    
    /*size_t FoodThisMinute;
    if (int(getSimTimeInSeconds())%60 == 0 && sim_time % ticks_per_second == 0){
        FoodThisMinute = score - scoreLastMinute;
        scoreLastMinute = score;
        foodPerMinute.push_back(FoodThisMinute);
        LOG << "Minute Passed... getSimTimeInSeconds: " << getSimTimeInSeconds() << ", Food Collected: " << FoodThisMinute << endl;
    }*/

    if(IdleCount >= NumOfRobots)
    {
      PostExperiment();
		exit(0); 
	}
   
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
