/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#ifndef TEB_CONFIG_H_
#define TEB_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>


// Definitions
#define USE_ANALYTIC_JACOBI // if available for a specific edge, use analytic jacobi 


namespace teb_local_planner
{

/**
 * @class TebConfig
 * @brief Config class for the teb_local_planner and its components.
 */  
class behaviour_config
{
public:
  
  bool mUseHookCheck;                         /** enable or disable online hook check */
  bool mUseDistanceCheck;               /** enable or disable online distance check */
  bool mUseSubsequentLaneChanges;           /** enable or disable subsequent lane change planning */

  float mMinPlanDist;                 /** minimum distance to expand plans in m */
  float mPlanDistFactor;                  /** distance factor for plans (times current speed in m/s) */
  float mSampleDist;                    /** distance of each sampled point in m */

  bool mEnableShifts;                 /** enable or disable lane shifts */
  int32_t mNumShifts;                   /** number of shifts to central plan to left and right side */
  float mMaxShiftPercentage;                /** percentage of maximum shift in relation to laneWidth */
  float mMinLaneWidth;                  /** minimum lane width to compute shifts in m */

  bool mEnableLaneChanges;                /** enable or disable lane changes */
  bool mEnableLaneChangesInRoundabouts;       /** enable or disable lane changes in roundabouts */
  float mDeferLaneChangeTime;             /** defer lane changes for seconds */

  int32_t mMaxLeftLaneChanges;                /** maximum allowed lane changes to the left */
  int32_t mMaxRightLaneChanges;             /** maximum allowed lane changes to the right */
  int32_t mMaxOncomingLaneChanges;              /** maximum allowed lane changes on oncoming lanes (to the left) */
  bool mUseBaseDirForSwerveGeneration;          /** use template base direction as start direction for swerve template generation */
  bool mUseVehicleDirForLaneChangeGeneration;     /** use vehicle direction as start direction for lane change template generation */

  float mMinSwerveDist;                 /** minimum distance offset for swerves/nudges shifts in m */
  float mSwerveDistFactor;                /** distance factor for swerves/nudges shifts (times current speed in m/s) */
  float mMinLaneChangeDist;               /** minimum distance offset for lane change shifts in m */
  float mLaneChangeDistFactorLowSpeed;          /** distance factor for lane changes (times current speed in m/s) at low speed */
  float mLaneChangeDistFactorHighSpeed;         /** distance factor for lane changes (times current speed in m/s) at high speed */
  float mLaneChangeLowSpeed;                /** low speed for lane changes */
  float mLaneChangeHighSpeed;             /** high speed for lane changes */

  float mBlindSpotFrontCheckDistRatio;          /** distance ratio for front blind spots (times lane change dist) */
  float mMinBlindSpotRearCheckDist;           /** minimum distance on target edge treated as blind spot from car rearward in m */
  float mBlindSpotRearCheckDistFactor;          /** distance factor for rear blind spots (times current speed in m/s) */

  RTT::PropertyBag mDriftPropertyBag;
  bool mEnableDrifts;                 /** enable or disable drifts */
  float mMinObstacleDist;               /** minimum obstacle distance in m */
  float mMaxDriftDist;                  /** maximum drift distance in m */
  float mMaxNearDriftDist;                /** maximum drift distance directly in front of the car in m */
  float mDriftFrontCheckDist;             /** front check distance for drift in m */
  float mDriftRearCheckDist;                /** rear check distance for drift in m */
  float mDriftCheckWidth;               /** check width for drift in m */
  bool mUseDirectionCheckForDrifts;         /** use direction check */



  RTT::PropertyBag mObstaclePropertyBag;
  float mObstacleEvalScalar;                /** factor for obstacle evaluation */
  float mObstacleTrafficLightIgnoreDist;          /** ignore distance for obstacles before traffic lights in m */
  int32_t mCyclesOvertakingConditionMetThreshold;
  RTT::Property<double> mMaxVelocityRatioToOvertake;

  bool mPlanParkingSpots;               /** indicates whether simple zone planning expands parking spots in zones */

  bool mPreferRightOnHighway;
  bool mPreferMiddleOnHighway;

    bool mPreferRight;
    bool mPreferMiddle;

  RTT::PropertyBag mDebugPropertyBag;
  bool mDebugMacroPlanGeneration;
  bool mDebugMicroPlanGeneration;
  bool mPrettyPrintDebugEvaluation;         /** debug evaluation on console with prettyprint */

  bool mDebugEvaluation;                /** en- or disable debug evaluation with debug preferences */
  int32_t mDebugPreferredLane;                /** preferred lane index (from 0 to number of lanes -1) for debugging purposes */
  int32_t mDebugPreferredPlan;                /** preferred plan index (from 0 to number of plans in lane -1) for debugging purposes */
  int32_t mDebugPreferredShift; 

  behaviour_config()
  {
    
    mUseHookCheck;                         /** enable or disable online hook check */
    mUseDistanceCheck;               /** enable or disable online distance check */
    mUseSubsequentLaneChanges;           /** enable or disable subsequent lane change planning */

    mMinPlanDist;                 /** minimum distance to expand plans in m */
    mPlanDistFactor;                  /** distance factor for plans (times current speed in m/s) */
    mSampleDist;                    /** distance of each sampled point in m */

    mEnableShifts;                 /** enable or disable lane shifts */
    mNumShifts;                   /** number of shifts to central plan to left and right side */
  float mMaxShiftPercentage;                /** percentage of maximum shift in relation to laneWidth */
  float mMinLaneWidth;                  /** minimum lane width to compute shifts in m */

  bool mEnableLaneChanges;                /** enable or disable lane changes */
  bool mEnableLaneChangesInRoundabouts;       /** enable or disable lane changes in roundabouts */
  float mDeferLaneChangeTime;             /** defer lane changes for seconds */

  int32_t mMaxLeftLaneChanges;                /** maximum allowed lane changes to the left */
  int32_t mMaxRightLaneChanges;             /** maximum allowed lane changes to the right */
  int32_t mMaxOncomingLaneChanges;              /** maximum allowed lane changes on oncoming lanes (to the left) */
  bool mUseBaseDirForSwerveGeneration;          /** use template base direction as start direction for swerve template generation */
  bool mUseVehicleDirForLaneChangeGeneration;     /** use vehicle direction as start direction for lane change template generation */

  float mMinSwerveDist;                 /** minimum distance offset for swerves/nudges shifts in m */
  float mSwerveDistFactor;                /** distance factor for swerves/nudges shifts (times current speed in m/s) */
  float mMinLaneChangeDist;               /** minimum distance offset for lane change shifts in m */
  float mLaneChangeDistFactorLowSpeed;          /** distance factor for lane changes (times current speed in m/s) at low speed */
  float mLaneChangeDistFactorHighSpeed;         /** distance factor for lane changes (times current speed in m/s) at high speed */
  float mLaneChangeLowSpeed;                /** low speed for lane changes */
  float mLaneChangeHighSpeed;             /** high speed for lane changes */

  float mBlindSpotFrontCheckDistRatio;          /** distance ratio for front blind spots (times lane change dist) */
  float mMinBlindSpotRearCheckDist;           /** minimum distance on target edge treated as blind spot from car rearward in m */
  float mBlindSpotRearCheckDistFactor;          /** distance factor for rear blind spots (times current speed in m/s) */

  RTT::PropertyBag mDriftPropertyBag;
  bool mEnableDrifts;                 /** enable or disable drifts */
  float mMinObstacleDist;               /** minimum obstacle distance in m */
  float mMaxDriftDist;                  /** maximum drift distance in m */
  float mMaxNearDriftDist;                /** maximum drift distance directly in front of the car in m */
  float mDriftFrontCheckDist;             /** front check distance for drift in m */
  float mDriftRearCheckDist;                /** rear check distance for drift in m */
  float mDriftCheckWidth;               /** check width for drift in m */
  bool mUseDirectionCheckForDrifts;         /** use direction check */



  RTT::PropertyBag mObstaclePropertyBag;
  float mObstacleEvalScalar;                /** factor for obstacle evaluation */
  float mObstacleTrafficLightIgnoreDist;          /** ignore distance for obstacles before traffic lights in m */
  int32_t mCyclesOvertakingConditionMetThreshold;
  RTT::Property<double> mMaxVelocityRatioToOvertake;

  bool mPlanParkingSpots;               /** indicates whether simple zone planning expands parking spots in zones */

  bool mPreferRightOnHighway;
  bool mPreferMiddleOnHighway;

    bool mPreferRight;
    bool mPreferMiddle;

  RTT::PropertyBag mDebugPropertyBag;
  bool mDebugMacroPlanGeneration;
  bool mDebugMicroPlanGeneration;
  bool mPrettyPrintDebugEvaluation;         /** debug evaluation on console with prettyprint */

  mDebugEvaluation;                /** en- or disable debug evaluation with debug preferences */
  mDebugPreferredLane;                /** preferred lane index (from 0 to number of lanes -1) for debugging purposes */
  mDebugPreferredPlan;                /** preferred plan index (from 0 to number of plans in lane -1) for debugging purposes */
  mDebugPreferredShift; 


  }
  
  /**
   * @brief Load parmeters from the ros param server.
   * @param nh const reference to the local ros::NodeHandle
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);
  

  
};


} // namespace teb_local_planner

#endif
