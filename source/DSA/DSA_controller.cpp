#include "DSA_controller.h"

/*****
 * Initialize most basic variables and objects here. Most of the setup should
 * be done in the Init(...) function instead of here where possible.
 *****/
DSA_controller::DSA_controller():
    NumberOfRobots(0),
    NumberOfSpirals(0),
    DSA(START),
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
    argos::GetNodeAttribute(settings, "SearchStepSize",          SearchStepSize);
    argos::GetNodeAttribute(settings, "NestDistanceTolerance", NestDistanceTolerance);
    argos::GetNodeAttribute(settings, "NestAngleTolerance", NestAngleTolerance);
    argos::GetNodeAttribute(settings, "TargetDistanceTolerance", TargetDistanceTolerance);
    argos::GetNodeAttribute(settings, "TargetAngleTolerance",    TargetAngleTolerance);
    argos::GetNodeAttribute(settings, "FoodDistanceTolerance",   FoodDistanceTolerance);
    argos::GetNodeAttribute(settings, "RobotForwardSpeed",       RobotForwardSpeed);
    argos::GetNodeAttribute(settings, "RobotRotationSpeed",      RobotRotationSpeed);
    argos::GetNodeAttribute(settings, "ResultsDirectoryPath",      results_path);
    argos::GetNodeAttribute(settings, "DestinationNoiseStdev",      DestinationNoiseStdev);
    argos::GetNodeAttribute(settings, "PositionNoiseStdev",      PositionNoiseStdev);
    argos::GetNodeAttribute(settings, "ProbTargetDetection",      ProbTargetDetection);
    //FoodDistanceTolerance *= FoodDistanceTolerance;
    SquaredFoodDistanceTolerance = FoodDistanceTolerance * FoodDistanceTolerance; //squared distance

    //argos::CVector2 p(GetPosition());
    //SetStartPosition(argos::CVector3(p.GetX(), p.GetY(), 0.0));
    
    m_pcLEDs   = GetActuator<CCI_LEDsActuator>("leds");
		m_pcLEDs->SetAllColors(CColor::GREEN);
		controllerID= GetId();//qilu 07/26/2016

    RNG = CRandom::CreateRNG("argos");
    
     string ID = GetId();
    string ID_number;
    LOG<<"Robot ID string= "<<ID<<endl;
      
    for(size_t i=4; i< ID.size(); i++){
      ID_number += ID[i];
    }
    RobotID = stoi(ID_number);
 
    LOG<<"RobotID number="<<RobotID<<endl;
    //SetStartPosition(argos::CVector3(centers[RobotID].GetX(), centers[RobotID].GetY(), 0.0));
    TrailColor = CColor(std::rand()%100, std::rand()%150, std::rand()%200, 255); // we avoid the white or nearly white, so we do not mode the random number by 255 

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
	
bool DSA_controller::IsInTheNest() {
    
	return ((GetPosition() - loopFunctions->NestPosition).SquareLength()
		< loopFunctions->NestRadiusSquared);
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
	copy(spiralPoints.begin(),spiralPoints.end(),back_inserter(tempSpiralPoints));
    reverse(tempSpiralPoints.begin(), tempSpiralPoints.end());
}

void DSA_controller::CopyPatterntoTemp() 
{
    copy(spiral.begin(),spiral.end(),back_inserter(tempSpiralPoints));
    reverse(tempSpiralPoints.begin(),tempSpiralPoints.end());/* Reverses the tempSpiralPoints */
}

/*****
 * Primary control loop for this controller object. This function will execute
 * the DSA logic once per frame.
 *****/
void DSA_controller::ControlStep() 
{
    if (DSA == START)
    {
        LOG<<"Start ....."<< endl;
        GetPattern(loopFunctions->paths[RobotID], loopFunctions->spiralPoints[RobotID]);
        DSA = SEARCHING;
        }

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
		     Stop(); //stop to reach the target location on the spiral path
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
        //LOG<<"ReachSpiralTargets ***"<<endl;
	  }
  } 
  else if( DSA == RETURN_TO_NEST) 
  {
      //argos::LOG << "RETURN_TO_NEST" << std::endl;
      SetIsHeadingToNest(true);

      // Check if we reached the nest. If so record that we dropped food off and go back to the spiral
      if((GetPosition() - loopFunctions->NestPosition).SquareLength() < loopFunctions->NestRadiusSquared) 
      {
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
	      else 
	      {}
      
	  //ofstream results_output_stream;
	  //results_output_stream.open(results_full_path, ios::app);
	  //results_output_stream << loopFunctions->getSimTimeInSeconds() << ", " << ++num_targets_collected << ", " << "Col Count" << endl;	    
	  //results_output_stream.close();
	  
	}
    else
	{
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
	//LOG<<"Going to idle ..."<<endl;
	
      // Check if we reached the nest. If so record that we dropped food off and go back to the spiral
      if((GetPosition() - loopFunctions->centers[RobotID]).SquareLength() < loopFunctions->NestRadiusSquared) 
      { Stop(); }
  } 
  
  Move();
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
        //LOG<<"RobotID="<<RobotID<<", current spiral point ="<<spiral_last<<endl;
        SetIsHeadingToNest(false);
        SetTarget(spiral_last);
	    tempSpiralPoints.pop_back();
    }
    
    //else if(tempPattern.size() == 0 || tempSpiralPoints.size() == 0) 
      else if(TargetHit() && tempSpiralPoints.size() == 0) 
      {
        LOG<<"Robot "<< RobotID<< " complete the spiral search ***"<<endl;
		DSA == IDLE;
		SetIsHeadingToNest(false);
    	SetTarget(loopFunctions->centers[RobotID]);
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
         if ((GetPosition()-loopFunctions->FoodList[i]).SquareLength() < SquaredFoodDistanceTolerance && !isHoldingFood)
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
    //tempPattern.clear();
    CopyPatterntoTemp();
    //generatePattern(NumberOfSpirals, NumberOfRobots);
    
}

REGISTER_CONTROLLER(DSA_controller, "DSA_controller")
