#include <fub_local_structured_trajectory_planner/behaviour_config.h>

namespace fub
{
namespace behaviour
{
    
void behaviour::loadRosParamFromNodeHandle(const ros::NodeHandle& pnh)
{
    

    pnh.param<bool>("UseHookCheck", mUseHookCheck, false); //"enable or disable online hook check"
    pnh.param<bool>("UseDistanceCheck", mUseDistanceCheck, false); //enable or disable online distance check
    pnh.param<bool>("UseSubsequentLaneChanges", mUseSubsequentLaneChanges, true); //enable or disable subsequent lane change planning
    //distance
    pnh.param<float>("MinPlanDist", mMinPlanDist, 50.0); //minimum distance to expand plans in m
    pnh.param<float>("PlanDistFactor", mPlanDistFactor, 4.5 * MS_2_KMH); //distance factor for plans (times current speed in m/s)
    pnh.param<float>("SampleDist", mSampleDist, 1.0); //distance of each sampled point in m


    //shifts
    pnh.param<bool>("EnableShifts", mEnableShifts, false); //enable or disable lane shifts
    pnh.param<int>("NumShifts", mNumShifts, 1); //number of shifts to central plan to left and right side
    pnh.param<float>("MaxShiftPercentage", mMaxShiftPercentage, 0.65); //percentage of maximum shift in relation to laneWidth
    pnh.param<float>("MinLaneWidth", mMinLaneWidth, 2.5); //minimum lane width to compute shifts in m

    //swerves and lane changes
    pnh.param<bool>("EnableLaneChanges", mEnableLaneChanges, false); //enable or disable lane changes
    pnh.param<bool>("EnableLaneChangesInRoundabouts", mEnableLaneChangesInRoundabouts, false); //
    pnh.param<float>("DeferLaneChangeTime", mDeferLaneChangeTime, 1.7); //defer lane changes for seconds

    pnh.param<int>("MaxLeftLaneChanges", mMaxLeftLaneChanges, 1); //maximum allowed lane changes to the left
    pnh.param<int>("MaxRightLaneChanges", mMaxRightLaneChanges, 1); //maximum allowed lane changes to the right
    pnh.param<int>("MaxOncomingLaneChanges", mMaxOncomingLaneChanges, 0); //maximum allowed lane changes on oncoming lanes (to the left)
    pnh.param<bool>("UseBaseDirForSwerveGeneration", mUseBaseDirForSwerveGeneration, true); //use template base direction as start direction for swerve template generation
    pnh.param<bool>("UseVehicleDirForLaneChangeGeneration", mUseVehicleDirForLaneChangeGeneration, true); //use vehicle direction as start direction for lane change template generation

    pnh.param<float>("MinSwerveDist", mMinSwerveDist, 6.5); //minimum distance offset for swerves/nudges shifts in m //6            //for lane changes use mMinLaneChangeDist
    pnh.param<float>("SwerveDistFactor", mSwerveDistFactor, 2.25); //2.5//distance factor for swerves/nudges (times current speed in m/s) 2.5
    pnh.param<float>("MinLaneChangeDist", mMinLaneChangeDist, 9.0); //18//minimum distance offset for lane changes in m
    pnh.param<float>("LaneChangeDistFactorLowSpeed", mLaneChangeDistFactorLowSpeed, 4.0); //distance factor for lane changes (times current speed in m/s) at low speed
    pnh.param<float>("LaneChangeDistFactorHighSpeed", mLaneChangeDistFactorHighSpeed, 9.0); //7.6//distance factor for lane changes (times current speed in m/s) at high speed
    pnh.param<float>("LaneChangeLowSpeed", mLaneChangeLowSpeed, 30 * KMH_2_MS); //low speed for lane changes
    pnh.param<float>("LaneChangeHighSpeed", mLaneChangeHighSpeed, 100 * KMH_2_MS); //high speed for lane changes
 

    //blind spot
    pnh.param<float>("BlindSpotFrontCheckDistRatio", mBlindSpotFrontCheckDistRatio, 0.5); //distance ratio for forward blind spots (times lane change distance)
    pnh.param<float>("MinBlindSpotRearCheckDist", mMinBlindSpotRearCheckDist, 3.0); //minimum distance on target edge treated as blind spot from car rearward in m
    pnh.param<float>("BlindSpotRearCheckDistFactor", mBlindSpotRearCheckDistFactor, 9.0); //0.6 * MS_2_KMH  //0.5//distance factor for rear blind spots (times current speed in m/s)



    //drifts
    pnh.param<bool>("EnableDrifts", mEnableDrifts, false); //enable or disable drifts
    pnh.param<float>("MinObstacleDist", mMinObstacleDist, 1.0); //minimum obstacle distance in m
    pnh.param<float>("MaxDriftDistance", mMaxDriftDist,0.5); //maximum drift distance in m
    pnh.param<float>("MaxNearDriftDistance", mMaxNearDriftDist, 0.01); //maximum drift distance directly in front of the car in m
    pnh.param<float>("DriftFrontCheckDist", mDriftFrontCheckDist, 100.0); //front check distance for drift in m
    pnh.param<float>("DriftRearCheckDist", mDriftRearCheckDist,0.0); //rear check distance for drift in m
    pnh.param<float>("DriftCheckWidth", mDriftCheckWidth,3.0); //check width for drift in m
    pnh.param<bool>("UseDirectionCheckForDrifts", mUseDirectionCheckForDrifts,false); //use direction check

    //obstacles
    pnh.param<float>("ObstacleEvalScalar", mObstacleEvalScalar, 2.51); //changed from 2.501
    pnh.param<float>("ObstacleTrafficLightIgnoreDist", mObstacleTrafficLightIgnoreDist, 50.0); //ignore distance for obstacles before traffic lights in m
    pnh.param<int>("CyclesOvertakingConditionMetThreshold", mCyclesOvertakingConditionMetThreshold, 20); //number of cycles overtaking condition is needed before executing
    pnh.param<double>("MaxVelocityRatioToOvertake", mMaxVelocityRatioToOvertake, 0.6); //0.05//velocity percentage of max velocity to overtake



    //zone planning
    pnh.param<bool>("PlanParkingSpots", mPlanParkingSpots,false); //indicates whether simple zone planning expands parking spots in zones

    //lane preference
    pnh.param<bool>("PreferRight", mPreferRight,true); //prefer right lane
    pnh.param<bool>("PreferMiddle", mPreferMiddle,false); //prefer middle lane
    pnh.param<bool>("PreferRightOnHighway", mPreferRightOnHighway,false); //prefer right lane on highway
    pnh.param<bool>("PreferMiddleOnHighway", mPreferMiddleOnHighway,true); //prefer middle lane on highway

    //debug
    pnh.param<bool>("DebugMacroPlanGeneration", mDebugMacroPlanGeneration,false); //en- or disable debug output for macro plan generation
    pnh.param<bool>("DebugMicroPlanGeneration", mDebugMicroPlanGeneration,false); //en- or disable debug output for micro plan generation
    pnh.param<bool>("PrettyPrintDebugEvaluation", mPrettyPrintDebugEvaluation,false); //debug evaluation on console with prettyprint
    pnh.param<bool>("DebugEvaluation", mDebugEvaluation,false); //en- or disable debug evaluation with debug preferences

    pnh.param<int>("DebugPreferredLane", mDebugPreferredLane,0); //preferred lane index (from 0 to number of lanes -1) for debugging purposes
    pnh.param<int>("DebugPreferredPlan", mDebugPreferredPlan,0); //preferred plan index (from 0 to number of plans in lane -1) for debugging purposes
    pnh.param<int>("DebugPreferredShift", mDebugPreferredShift,0); //preferred shift index (from -mNumShifts to +mNumShifts) for debugging purposes

    pnh.param<int>("Counter", mCounter,0); //
    pnh.param<float>("CurPlanDist", mCurPlanDist,0.0); //
    pnh.param<float>("CurLeftObstacleDist", mCurLeftObstacleDist,0.0); //
    pnh.param<float>("CurRightObstacleDist", mCurRightObstacleDist,0.0); //
  
  
}

}
}