#include "DSA_controller.h"

/*****
 * Initialize most basic variables and objects here. Most of the setup should
 * be done in the Init(...) function instead of here where possible.
 *****/
DSA_controller::DSA_controller() :
    NumberOfRobots(0),
    NumberOfSpirals(0),
    DSA(SEARCHING),
    RNG(NULL),
    ResetReturnPosition(true),
    stopTimeStep(0),
    m_pcLEDs(NULL),
    isHoldingFood(false)
{}

/*****
 * Initialize the controller via the XML configuration file. ARGoS typically
 * wants objects & variables initialized here instead of in the constructor(s).
 *****/
void DSA_controller::Init(TConfigurationNode& node) {

    compassSensor   = GetSensor<argos::CCI_PositioningSensor>("positioning");
    wheelActuator   = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
    proximitySensor = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity");

    argos::TConfigurationNode settings = argos::GetNode(node, "settings");
    argos::GetNodeAttribute(settings, "NumberOfRobots",          NumberOfRobots);
    argos::GetNodeAttribute(settings, "NumberOfSpirals",         NumberOfSpirals);
    argos::GetNodeAttribute(settings, "SearchStepSize",          SearchStepSize);
    argos::GetNodeAttribute(settings, "NestDistanceTolerance", NestDistanceTolerance);
    argos::GetNodeAttribute(settings, "NestAngleTolerance", NestAngleTolerance);
    argos::GetNodeAttribute(settings, "NestAngleTolerance", NestAngleTolerance);
    argos::GetNodeAttribute(settings, "TargetDistanceTolerance", TargetDistanceTolerance);
    argos::GetNodeAttribute(settings, "TargetAngleTolerance",    TargetAngleTolerance);
    argos::GetNodeAttribute(settings, "SearcherGap",             SearcherGap);
    argos::GetNodeAttribute(settings, "FoodDistanceTolerance",   FoodDistanceTolerance);
    argos::GetNodeAttribute(settings, "RobotForwardSpeed",       RobotForwardSpeed);
    argos::GetNodeAttribute(settings, "RobotRotationSpeed",      RobotRotationSpeed);
    argos::GetNodeAttribute(settings, "ResultsDirectoryPath",      results_path);
    argos::GetNodeAttribute(settings, "DestinationNoiseStdev",      DestinationNoiseStdev);
    argos::GetNodeAttribute(settings, "PositionNoiseStdev",      PositionNoiseStdev);
    argos::GetNodeAttribute(settings, "ProbTargetDetection",      ProbTargetDetection);
    FoodDistanceTolerance *= FoodDistanceTolerance;

    //argos::CVector2 p(GetPosition());
    //SetStartPosition(argos::CVector3(p.GetX(), p.GetY(), 0.0));
    
    m_pcLEDs   = GetActuator<CCI_LEDsActuator>("leds");
		m_pcLEDs->SetAllColors(CColor::GREEN);
		controllerID= GetId();//qilu 07/26/2016

    RNG = CRandom::CreateRNG("argos");
    calRegions(4);
    generatePattern(NumberOfSpirals, NumberOfRobots);
   
    //LOG<<"RobotNumber="<<RobotNumber<<endl;
    //SetStartPosition(argos::CVector3(centers[RobotNumber].GetX(), centers[RobotNumber].GetY(), 0.0));
    TrailColor = CColor(std::rand()%150, std::rand()%150, std::rand()%150, 255); // we avoid the white or nearly white, so we do not mode the random number by 255 

    // Name the results file with the current time and date
 time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    stringstream ss;

    ss << "DSA-"<<GIT_BRANCH<<"-"<<GIT_COMMIT_HASH<<"-" 
       << (now->tm_year) << '-'
       << (now->tm_mon + 1) << '-'
       <<  now->tm_mday << '-'
       <<  now->tm_hour << '-'
       <<  now->tm_min << '-'
       <<  now->tm_sec << ".csv";

    string results_file_name = ss.str();
   results_full_path = results_path+"/"+results_file_name;        

    // Only the first robot should do this:


    if (GetId().compare("DSA_0") == 0)
      {
  
   ofstream results_output_stream;
 results_output_stream.open(results_full_path, ios::app);
 results_output_stream << "NumberOfRobots, "
		       << "NumberOfSpirals, "
		       << "TargetDistanceTolerance, "
		       << "TargetAngleTolerance, "
		       << "SearcherGap, "
		       << "FoodDistanceTolerance, "
		       << "RobotForwardSpeed, "
		       << "RobotRotationSpeed, "
		       << "RandomSeed" << endl
           << NumberOfRobots << ", "
		       << NumberOfSpirals << ", "
		       << TargetDistanceTolerance << ", "
		       << TargetAngleTolerance << ", "
		       << SearcherGap << ", "
		       << FoodDistanceTolerance << ", "
		       << RobotForwardSpeed << ", "
		       << RobotRotationSpeed << ", "
		       << CSimulator::GetInstance().GetRandomSeed() << endl;  
 results_output_stream.close();
      }

    cout << "Finished Initializing the DDSA" << endl;
}

void DSA_controller::calRegions(int num_regions)
{
	int num_rows = sqrt(num_regions);
	int num_cols = num_rows;
	
	double_t unit = ForageRangeX.GetMax()/num_rows;
	double_t rangeMax = ForageRangeX.GetMax();
	
	CVector2 location;
	CVector2 pos;
	for(int i =0; i < num_rows; i++)
	{
		for(int j =0; j < num_cols; j++)
		{
			location = CVector2(rangeMax-(2*i+1)*unit, rangeMax-(2*j+1)*unit);
			centers.push_back(location);
			LOG << "center["<<i<<","<<j<<"]="<<location<<endl;
			pos = CVector2(location.GetX()+unit, location.GetY()+unit);
			topLeftPts.push_back(pos);
			pos = CVector2(location.GetX()-unit, location.GetY()-unit);
			bottomRightPts.push_back(pos);
			
		}
	}
	
}
	
	
bool DSA_controller::IsInTheNest() {
    
	return ((GetPosition() - loopFunctions->NestPosition).SquareLength()
		< loopFunctions->NestRadiusSquared);
	}
	
void DSA_controller::generatePattern(int N_circuits, int N_robots) //Why is this function not in LoopFuction? qilu 2/2023
{
    string ID = GetId();
    string ID_number;
    LOG<<"ID = "<<ID<<endl;
      
    /*for(size_t i = 0; i < ID.size(); i++) {
        if(ID[i] >= '0' && ID[i] <= '9') {
            LOG<<"ID_number = "<<ID_number<<endl;
            LOG<<"ID["<< i<< "] = "<<ID[i]<<endl;
            ID_number += ID[i];
            LOG<<"after: ID_number = "<<ID_number<<endl;
            
        }
    }*/
    for(size_t i=4; i< ID.size(); i++){
      ID_number += ID[i];
    }
    RobotNumber = stoi(ID_number);
    vector<string> paths;
    string ith_robot_steps;
    vector<vector<CVector2>> spiralPoints; //qilu 02/2023
	vector<CVector2> pointVector;
	CVector2 point;
	argos::Real x, y;
    
    
    for (int i_robot = 1; i_robot <= N_robots; i_robot++)
    {
		ith_robot_steps +='O';
		pointVector.push_back(centers[RobotNumber]);
		point = centers[RobotNumber];
        for (int i_circuit = 1; i_circuit <= N_circuits; i_circuit++)
        { 
            int n_steps_north = calcDistanceToTravel(i_robot, i_circuit, N_robots, 'N');
            for (int j = 0; j < n_steps_north; j++)
            {
                ith_robot_steps += 'N';
            }
            x = point.GetX() + n_steps_north*SearcherGap;
            y = point.GetY();
            point = CVector2(x, y);
            pointVector.push_back(point);
            
            int n_steps_east = calcDistanceToTravel(i_robot, i_circuit, N_robots, 'E');
            for (int j = 0; j < n_steps_east; j++)
            {
                ith_robot_steps += 'E';
            }
            
            x = point.GetX();
            y = point.GetY() - n_steps_east*SearcherGap; 
            point = CVector2(x, y);
            pointVector.push_back(point);

            int n_steps_south = calcDistanceToTravel(i_robot, i_circuit, N_robots, 'S');
            for (int j = 0; j < n_steps_south; j++)
            {
                ith_robot_steps += 'S';
            }
            x = point.GetX() - n_steps_south*SearcherGap;
            y = point.GetY();
            point = CVector2(x, y);
            pointVector.push_back(point);
            
            int n_steps_west = calcDistanceToTravel(i_robot, i_circuit, N_robots, 'W');
            for (int j = 0; j < n_steps_west; j++)
            {
                ith_robot_steps += 'W';
            }
            x = point.GetX();
            y = point.GetY() + n_steps_west*SearcherGap;
            point = CVector2(x, y);
            pointVector.push_back(point);

        }
		paths.push_back(ith_robot_steps);
        ith_robot_steps.clear();
        spiralPoints.push_back(pointVector);
        pointVector.clear();
        
    }
    GetPattern(paths[RobotNumber], spiralPoints[RobotNumber]);
}

int DSA_controller::calcDistanceToTravel(int ith_robot, int ith_circuit, int n_robots, char direction)
{
	
    int Num_robots = 1; // one robot in each region qilu 12/2022
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
            n_steps = (2*ith_circuit -3)*Num_robots + 2*ith_robot;
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
            n_steps = calcDistanceToTravel(ith_robot, ith_circuit, Num_robots, 'N') + Num_robots;
            return n_steps;
        }

    }  
    
    return 0;
}

void DSA_controller::printPath(vector<char>& path)
{
    cout << path.size() << endl;
    for(int i = 0; i<path.size(); i++)
    { 
        cout << path.at(i) << endl;
    }
}

void DSA_controller::GetPattern(string ith_Pattern, vector<CVector2> spiralPoints)
{
    copy(ith_Pattern.begin(),ith_Pattern.end(),back_inserter(tempPattern));
    reverse(tempPattern.begin(), tempPattern.end());
	
	copy(spiralPoints.begin(),spiralPoints.end(),back_inserter(tempSpiralPoints));
    reverse(tempSpiralPoints.begin(), tempSpiralPoints.end());
    
}

// /*****
//  *
//  *****/
void DSA_controller::CopyPatterntoTemp() 
{
    copy(pattern.begin(),pattern.end(),back_inserter(tempPattern));
    reverse(tempPattern.begin(),tempPattern.end());/* Reverses the tempPattern */
    
    copy(spiral.begin(),spiral.end(),back_inserter(tempSpiralPoints));
    reverse(tempSpiralPoints.begin(),tempSpiralPoints.end());/* Reverses the tempSpiralPoints */
}

/*****
 * Primary control loop for this controller object. This function will execute
 * the DSA logic once per frame.
 *****/
void DSA_controller::ControlStep() 
{

  // To draw paths
  if (DSA == SEARCHING)
    {
      CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.00);
      CVector3 target3d(previous_position.GetX(), previous_position.GetY(), 0.00);
      CRay3 targetRay(target3d, position3d);
      myTrail.push_back(targetRay);
  
      loopFunctions->TargetRayList.push_back(targetRay);
      loopFunctions->TargetRayColorList.push_back(TrailColor);
    }

  //LOG << myTrail.size() << endl;
  previous_position = GetPosition();

  /* Continue in a sprial */
  if( DSA == SEARCHING )
  {
      SetIsHeadingToNest(false);
      //argos::LOG << "SEARCHING" << std::endl;
      SetHoldingFood();
      if (IsHoldingFood())
	  {
	    bool cpf = true; // why? qilu 12/2022
	    if (cpf)
	    {
	      ReturnPosition = GetPosition();
	      ReturnPatternPosition = GetTarget();
	      DSA = RETURN_TO_NEST;
	  SetIsHeadingToNest(true);
	  SetTarget(loopFunctions->NestPosition);
	  //LOG<<" after setting to the nest, target location ="<<GetTarget()<<endl;
		Stop();
	    }
	    else
	    {
			//LOG<<"Test ************, do we need this ?"<<endl;
	      num_targets_collected++;
	      loopFunctions->setScore(num_targets_collected);
	      isHoldingFood = false;
	    }
	    return;
	  } 
      else // not holding food
	  {
	    ReachSpiralTargets(); /* Initializes targets positions. */
	  }
  } 
  else if( DSA == RETURN_TO_NEST) 
  {
      //argos::LOG << "RETURN_TO_NEST" << std::endl;
      SetIsHeadingToNest(true);

      // Check if we reached the nest. If so record that we dropped food off and go back to the spiral
      if((GetPosition() - loopFunctions->NestPosition).SquareLength() < loopFunctions->NestRadiusSquared) 
      {
	  //DSA = RETURN_TO_SEARCH;
	  //SetIsHeadingToNest(false);
	  //SetTarget(ReturnPosition);

	     if (isHoldingFood)
	     {	
			//argos::LOG << "Holding food and drop it" << std::endl;
	      num_targets_collected++;
	      loopFunctions->setScore(num_targets_collected);
	      //qilu 12/2022
	      DSA = RETURN_TO_SEARCH;
	      SetIsHeadingToNest(false);
	      SetTarget(ReturnPosition);
	      isHoldingFood = false;
	      }
	      else // if the robot does not hold food, it means it finished the spiral search. qilu 12/2022
	      {
			DSA = IDLE;
			//SetTarget(GetPosition());
			argos::LOG << "Set to idle ..." << std::endl;
		  }
        //isHoldingFood = false;
	 
	  //ofstream results_output_stream;
	  //results_output_stream.open(results_full_path, ios::app);
	  //results_output_stream << loopFunctions->getSimTimeInSeconds() << ", " << ++num_targets_collected << ", " << "Col Count" << endl;	    
	  //results_output_stream.close();
	  
	}
    else
	{
	 //argos::LOG << "set to nest ...." << std::endl;	
	  //SetIsHeadingToNest(true);
	  //SetTarget(loopFunctions->NestPosition);
	  //LOG<<" after setting to the nest, target location ="<<GetTarget()<<endl;
	} 
  } 
  else if( DSA == RETURN_TO_SEARCH) 
  {
       //argos::LOG << "RETURN_TO_SEARCH" << std::endl;
      SetIsHeadingToNest(false);
      
      // Check if we have reached the return position
      if ( IsAtTarget() )
		{
	  //argos::LOG << "RETURN_TO_SEARCH: Pattern Point" << std::endl;
	  SetIsHeadingToNest(false);
	  SetTarget(ReturnPatternPosition);
	  DSA = SEARCHING;
		}
  }
  else if(DSA == IDLE)
  {
	// just wait for a new assignment qilu 12/2022
	LOG<<"Becomes idle ..."<<endl;
  } 
  
  Move();
}   

/*****
 * Sets target North of the robot's current target.
 *****/
void DSA_controller::SetTargetN(char x)
{
    CVector2 position = GetTarget();
    SetIsHeadingToNest(false);
    //SetTarget(CVector2(position.GetX()+SearcherGap,position.GetY()));
}

/*****
 * Sets target South of the robot's current target.
 *****/
void DSA_controller::SetTargetS(char x){
    CVector2 position = GetTarget();
    SetIsHeadingToNest(false);
    //SetTarget(CVector2(position.GetX()-SearcherGap,position.GetY()));
}

/*****
 * Sets target East of the robot's current target.
 *****/
void DSA_controller::SetTargetE(char x){
   CVector2 position = GetTarget();
   SetIsHeadingToNest(false);
   //SetTarget(CVector2(position.GetX(),position.GetY()-SearcherGap));
}

/*****
 * Sets target West of the robot's current target.
 *****/
void DSA_controller::SetTargetW(char x){
    CVector2 position = GetTarget();
    SetIsHeadingToNest(false);
    //SetTarget(CVector2(position.GetX(),position.GetY()+SearcherGap));
}

/*****
 * Sets target to the center point of the region.
 *****/
void DSA_controller::SetTargetO(char x){
    CVector2 position = GetTarget();
    SetIsHeadingToNest(false);
    SetTarget(centers[RobotNumber]);
}

/*****
 * Helper function that reads vector <char> pattern
 * and sets the target's direction base on the 
 * char at the current vector index.
 *****/
 void DSA_controller::ReachSpiralTargets(){

   /* If the robot hit target and the patter size >0
       then find the next direction. */
    //if(TargetHit() && tempPattern.size() > 0) {
   //   /* Finds the last direction of the pattern. */
   // direction_last = tempPattern[tempPattern.size() - 1]; 
    
    if(TargetHit() && tempSpiralPoints.size() > 0) {
      /* Finds the last direction of the spiral. */
    spiral_last = tempSpiralPoints[tempSpiralPoints.size() - 1]; 
    SetTarget(spiral_last);
    	
        switch(direction_last)
        {
            case 'N':
                SetTargetN('N');
                break;
            case 'S':
                SetTargetS('S');
                break;
            case 'E':
                SetTargetE('E');
                break;
            case 'W':
                SetTargetW('W');
                break;
            case 'O':
                SetTargetO('O');
                break;
	   }

	tempPattern.pop_back();
	tempSpiralPoints.pop_back();
    }
    
    else if(tempPattern.size() == 0 && tempSpiralPoints.size() == 0) 
      {
    	Stop();// can be modified and let it to idle. qilu 12/2022
      }
}

/*****
 * Returns a boolean based on weather the robot is with 0.01 
 * distance tolerance. Declares that the robot had reached 
 * current target.
 *****/
 bool DSA_controller::TargetHit() {
    CVector2 position = GetPosition() - GetTarget();
    bool hit = false;
     
    if(position.SquareLength() < TargetDistanceTolerance){
        hit = true;
    }
    return hit;
 }

/*****
 * Check if the Robot is finding food. This is defined as the Robot being within
 * the distance tolerance of the position of a food item. If the Robot has found
 * food then the appropriate boolean flags are triggered.
 *****/
void DSA_controller::SetHoldingFood(){
    if(IsHoldingFood() == false) 
      {
	if(rand()*1.0/RAND_MAX < ProbTargetDetection) {
	
        vector <CVector2> newFoodList; 
        size_t i = 0;
        for (i = 0; i < loopFunctions->FoodList.size(); i++)
	    {
         if ((GetPosition()-loopFunctions->FoodList[i]).SquareLength() < FoodDistanceTolerance && !isHoldingFood)
	      {
			isHoldingFood = true;
	      } 
	     else 
	      {
			newFoodList.push_back(loopFunctions->FoodList[i]);
	      }
        } 
        loopFunctions->FoodList = newFoodList;
      }
      }

}

/*****
 * Is this Robot_controller holding food?
 *     true  = yes
 *     false = no
 *****/
bool DSA_controller::IsHoldingFood() {
    return isHoldingFood;
}
/*****
 * After pressing the reset button in the GUI, this controller will be set to
 * default factory settings like at the start of a simulation.
 *****/
void DSA_controller::Reset() {
    collisionDelay  = 0;
    SetIsHeadingToNest(true);
    SetTarget(loopFunctions->NestPosition);
    tempPattern.clear();
    CopyPatterntoTemp();
    generatePattern(NumberOfSpirals, NumberOfRobots);
    
}

REGISTER_CONTROLLER(DSA_controller, "DSA_controller")
