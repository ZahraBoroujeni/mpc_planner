#include "Behaviour.h"

#include <iostream>
#include <rtt/Logger.hpp>
#include <patterns/Singleton.h>
#include <fstream>
#include <boost/range/size.hpp>
#include <boost/foreach.hpp>

#include <util/TaskContextFactory.h>
#include <util/Templates.h>
#include <math/AutoMath.h>
#include <math/DiffGeom.h>
#include <math/PathSpline.h>
#include <data/VehicleData.h>
#include <util/OrocosHelperFunctions.h>
#include <util/PrettyPrint.h>

#include <aa/modules/nav/statemachine/StateMachine.h>

#define MAX_APSP_DISTANCE               1000000

#define FATAL_OBSTACLE_EVAL             1100000
#define OBSTACLE_EVAL                   42
#define LANE_PREF_EVAL                  0.5*0.6*OBSTACLE_EVAL
#define LANE_CHANGE_LEFT_EVAL           0.5*0.3*OBSTACLE_EVAL
#define LANE_CHANGE_RIGHT_EVAL          0.5*0.4*OBSTACLE_EVAL

behaviour::behaviour(ros::NodeHandle & nh): mNodeHandle(nh)
{
    // node handle in the private namespace
    ros::NodeHandle pnh = ros::NodeHandle("~");
    loadRosParamFromNodeHandle(pnh);
    mMacroPlanVecOut = mNodeHandle.advertise<nav_msgs::Odometry>("MacroPlanVecOut", 10);
    mSampledPointPlanVecOut = mNodeHandle.advertise<nav_msgs::Odometry>("SampledPointPlanVecOut", 10);
    mMicroPlanVecOut = mNodeHandle.advertise<nav_msgs::Odometry>("MicroPlanVecOut", 10);
    mTemplatePlanVecOut = mNodeHandle.advertise<nav_msgs::Odometry>("TemplatePlanVecOut", 10);
    mPlanOut = mNodeHandle.advertise<nav_msgs::Odometry>("PlanOut", 10);
    mBlindSpotDataOut = mNodeHandle.advertise<nav_msgs::Odometry>("BlindSpotDataOut", 10);
    mSideObstacleDistLeftOut = mNodeHandle.advertise<nav_msgs::Odometry>("SideObstacleDistLeftOut", 10);
    mSideObstacleDistRightOut = mNodeHandle.advertise<nav_msgs::Odometry>("SideObstacleDistRightOut", 10);
    mSideObstacleSpotDataOut = mNodeHandle.advertise<nav_msgs::Odometry>("SideObstacleSpotDataOut", 10);
    mSubscriberStateMachine = mNodeHandl.subscribe("statemachine", 32, behaviour::stateMachineCallback, this);


}

bool behaviour::stateMachineCallback(const std_msgs::Int16& msg)
{
    mStateMachine = msg.data;
}
bool behaviour::egoStateCallback(const std_msgs::Int16& msg)
{
    mEgoStateIn = msg.data;
}
bool behaviour::missionCallback(const std_msgs::Int16& msg)
{
    mMissionIn = msg.data;
}

bool behaviour::obstacleCallback(const std_msgs::Int16& msg)
{
    mObstaclesIn = msg.data;
}



bool behaviour::startHook()
{
    ROS_INFO("Behaviour");


    if (!mStateMachine) {
        ROS_ERORR("missing StateMachine peer");
        return false;
    }


    aGraph const & rGraph = mARNDGraph.getBoostGraph();

    if (num_vertices(rGraph) == 0) {
        ROS_ERORR("No vertices: Graph not loaded?");
        return false;
    }

    if (!mShortestPath.calcAPSP()) {
        ROS_ERORR("Calculating ShortestPaths failed!");
        return false;
    }

    // REQUIRED_PORT(mEgoStateIn);
    // REQUIRED_PORT(mMissionIn);
    // OPTIONAL_PORT(mObstaclesIn);
    // OPTIONAL_PORT(mReplanNowIn);

    // OPTIONAL_PORT(mMacroPlanVecOut);
    // OPTIONAL_PORT(mSampledPointPlanVecOut);
    // OPTIONAL_PORT(mMicroPlanVecOut);
    // OPTIONAL_PORT(mTemplatePlanVecOut);
    // OPTIONAL_PORT(mPlanOut);
    // OPTIONAL_PORT(mBlindSpotDataOut);
    // OPTIONAL_PORT(mSideObstacleDistLeftOut);
    // OPTIONAL_PORT(mSideObstacleDistRightOut);
    // OPTIONAL_PORT(mSideObstacleSpotDataOut);

    return true;
}


void Behaviour::buildMicroPlanVec()
{
    mMicroPlanVec.clear();

    //splinify point sampled plans
    for (uint i = 0; i < mSampledPointPlanVec.size(); i++) {

        vector< vector< Plan_ptr > > curMicroLane;

        for (uint j = 0; j < mSampledPointPlanVec[i].size(); j++) {

            vector< Plan_ptr > curMicroPlanList;

            for (uint k = 0; k < mSampledPointPlanVec[i][j].size(); k++) {

                Plan_ptr curPlanPtr = aa::modules::nav::controller::AutoPlan();
                curPlanPtr->clear();

                SampledPointList curSampledPointPlan = mSampledPointPlanVec[i][j][k];
                SampledActionList curSampledActionPlan = mSampledActionPlanVec[i][j][k];


                for (uint n = 0; n < curSampledPointPlan.size(); n++) {
                    SampledPoint sample = curSampledPointPlan[n];

                    Vec3 dir(0, 0, 0);

                    if (curSampledPointPlan.size() > 1) {
                        if (n == curSampledPointPlan.size() - 1) {
                            dir = normalized(sample.pos - mSampledPointPlanVec[i][j][k][n - 1].pos);
                        }
                        else {
                            dir = normalized(mSampledPointPlanVec[i][j][k][n + 1].pos - sample.pos);
                        }
                    }


                    bool hookfree = mUseHookCheck.get() ? verifyHook(curPlanPtr, sample.param, sample.pos, dir) : true;

                    if (hookfree) {
                        curPlanPtr->push_back(sample.param, sample.pos, dir);
//                      curPlanPtr->push_back(sample.param, sample.pos, sample.dir);
                        curPlanPtr->push_back_edge(sample.edge);
                    }
                }

                //fill actions
                for (uint n = 0; n < curSampledActionPlan.size(); n++) {
                    SampledAction sample = curSampledActionPlan[n];
                    curPlanPtr->push_back_action(sample.type, sample.startParam, sample.endParam, sample.meta);
                }

                curMicroPlanList.push_back(curPlanPtr);
            }

            curMicroLane.push_back(curMicroPlanList);
        }

        mMicroPlanVec.push_back(curMicroLane);
    }


    if (mDebugMicroPlanGeneration.get()) {
        std::cout << std::endl << "Micro plans:" << std::endl;

        for (uint i = 0; i < mMicroPlanVec.size(); i++) {
            for (uint j = 0; j < mMicroPlanVec[i].size(); j++) {
                for (uint k = 0; k < mMicroPlanVec[i][j].size(); k++) {
                    std::cout << i << "/" << j << "/" << k;
                    std::cout << "  " << (*mMicroPlanVec[i][j][k]).domain().first << " <-> " << (*mMicroPlanVec[i][j][k]).domain().second << std::endl;
                }

                std::cout << std::endl;
            }

            std::cout << std::endl;
        }

        std::cout << numMicroPlans() << " micro plans" << std::endl;
    }
}




std::pair<TemplatePlan, flt> Behaviour::buildSwerveTemplate(edge_descr const & edge, flt param, int s, Vec3 const & curPos, Vec3 const & curDir, flt swerveLength, flt curSpeed, swerve_type type)
{
    //create temporary plans for swerves/nudges for shifts
    TemplatePlan curTemplatePlan;
    curTemplatePlan.plan->clear();

    // access graph
    aGraph const & rGraph = mARNDGraph.getBoostGraph();
//  property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), rGraph);
    property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), rGraph);

    //get minimal shift lane width by looking sampling edges
    flt laneWidth = mARNDGraph.laneWidth(edge, param);

    /*
    for (uint i = vertexIndex; i < macroPlan.size(); i++) {
        vertex_descr const x = macroPlan[i];
        laneWidth = min(laneWidth, vertexDataMap[x].laneWidth);
    }
    */

    laneWidth = max(laneWidth, mMinLaneWidth.get());


    //shift base direction
    Vec3 const baseShiftDir = Vec3(1.0, 0.0,0.0);

    //calc shift distances
    LaneSpline3D const & laneSpline = *edgeDataMap[edge].driveSpline();

    flt alpha = ::math::angle(laneSpline.firstDerivative(param), curPos - laneSpline(param));
    flt const startShiftDist = sin(alpha) * (curPos - laneSpline(param)).norm();
    flt const targetShiftDist = (mMaxShiftPercentage.get() * -(flt)s / max((flt)mNumShifts.get(), 1.0) * 0.5 * laneWidth);


    //increase swerve dist with lane change width
    flt newShiftLength = swerveLength;
    flt const swerveWidthRatio = abs(targetShiftDist - startShiftDist) / laneWidth * 0.6;

    if (swerveWidthRatio > 1.0) {
        newShiftLength *= swerveWidthRatio;
    }

    //may need to defer
    flt deferLength = 0;

    if (type == LANE_CHANGE && mDeferLaneChangeTime.get() > 0) {
        deferLength = mDeferLaneChangeTime.get() + curSpeed;
        newShiftLength = newShiftLength + deferLength;
    }

    //aproximate param by linear approximation
    flt const ydiff = targetShiftDist - startShiftDist;
    flt const approxLen = sqrt(newShiftLength * newShiftLength + ydiff * ydiff);

    //define start and end
    Vec3 const startShiftPos(0, startShiftDist, curPos[2]);
    Vec3 const startShiftDir = ::math::rotateToNewBase(curDir, laneSpline.firstDerivative(param), baseShiftDir);

    Vec3 const targetShiftPos(newShiftLength, targetShiftDist, curPos[2]);
    Vec3 const targetShiftDir = baseShiftDir;

    //define support points position in local coordinate system
    map<flt, Vec3> supportPoints;
    supportPoints[0.0] = startShiftPos;
    supportPoints[1.0] = targetShiftPos;


    //defer it
    if (type == LANE_CHANGE && mDeferLaneChangeTime.get() > 0) {
//      for (flt i = 0.05; i * newShiftLength <= deferLength; i += 0.05) {
//            supportPoints[i] = Vec3(newShiftLength * i, startShiftDist, 0.f);
//      }
        supportPoints[(deferLength*0.5)/newShiftLength] = Vec3(deferLength*0.5, startShiftDist, 0.f);
        supportPoints[deferLength/newShiftLength] = Vec3(deferLength, startShiftDist, 0.f);
    }

//  supportPoints[0.5] = Vec2(newShiftLength * 0.5, startShiftDist + ydiff * 0.45);
//  supportPoints[0.7] = Vec2(newShiftLength * 0.7, startShiftDist + ydiff * 0.5);


    //add support points
    map<flt, Vec3>::const_iterator it = supportPoints.begin();
    map<flt, Vec3>::const_iterator next = supportPoints.begin();
    next++;

    for (; it != supportPoints.end(); it++, next++) {
        if (type == LANE_CHANGE && mUseVehicleDirForLaneChangeGeneration.get() && it == supportPoints.begin()) {
            curTemplatePlan.plan->push_back(approxLen * it->first, it->second, startShiftDir);
        }
        else if (type == SWERVE && mUseBaseDirForSwerveGeneration.get() &&  it == supportPoints.begin()) {
//          curTemplatePlan.plan->push_back(approxLen * it->first, it->second, startShiftDir);
//          curTemplatePlan.plan->push_back(approxLen * it->first, it->second, baseShiftDir);
            curTemplatePlan.plan->push_back(approxLen * it->first, it->second, normalized(0.3 * baseShiftDir + 0.7 * normalized(next->second - it->second)));

        }
        else if (next == supportPoints.end()) {
            curTemplatePlan.plan->push_back(approxLen, it->second, targetShiftDir);
        }
        else {
            curTemplatePlan.plan->push_back(approxLen * it->first, it->second, normalized(next->second - it->second));
        }
    }

    //add properties to template
    curTemplatePlan.mApproxLength = approxLen;
    curTemplatePlan.mApproxWidth = ydiff;
    curTemplatePlan.mTargetShiftDist = targetShiftDist;


    return make_pair(curTemplatePlan, newShiftLength);
}

/*
std::pair<TemplatePlan, flt> Behaviour::buildShiftTemplate(MacroPlan macroPlan, int vertexIndex, edge_descr const & edge, flt param, int s, Vec2 const & curPos, Vec2 const & curDir, flt shiftLength)
{
    //create temporary plans for swerves/nudges for shifts
    TemplatePlan curTemplatePlan;
    curTemplatePlan.plan->clear();

    // access graph
    aGraph const & rGraph = mRNDFGraph.getBoostGraph();
    property_map<aGraph, vertex_data_t>::const_type vertexDataMap = get(vertex_data_t(), rGraph);
    property_map<aGraph, edge_data_t>::const_type edgeDataMap = get(edge_data_t(), rGraph);

    //get minimal shift lane width by looking into future
    flt laneWidth = mRNDFGraph.laneWidth(edge, param);

    for (uint i = vertexIndex; i < macroPlan.size(); i++) {
        vertex_descr const x = macroPlan[i];
        laneWidth = min(laneWidth, vertexDataMap[x].laneWidth);
    }

    laneWidth = max(laneWidth, mMinLaneWidth.get());

    //shift base direction
    Vec2 const shiftBaseDir = Vec2(1.0, 0.0);

    //calc shift distances
    LaneSpline const & laneSpline = *edgeDataMap[edge].laneSpline;
    flt const startShiftDist = sin(::math::angle(laneSpline.firstDerivative(param), curPos - laneSpline(param))) * (curPos - laneSpline(param)).norm();
    flt const targetShiftDist = (mMaxShiftPercentage.get() * -(flt)s / max((flt)mNumShifts.get(), 1.0) * 0.5 * laneWidth);

//  std::cout<<targetShiftDistToMid << "  / " << targetShiftDist <<  "  " << (edgeDataMap[targetEdge].isConnection ? "C " : "") << edgeDataMap[targetEdge].name;
//  std::cout<<"curShiftDist " << curShiftDist << " targetShiftDist " << targetShiftDist;

    //increase swerve dist with lane change width
    flt newShiftLength = shiftLength;
    flt const swerveWidthRatio = abs(targetShiftDist - startShiftDist) / laneWidth * 0.6;

    if (swerveWidthRatio > 1.0) {
        newShiftLength *= swerveWidthRatio;
    }

    //aproximate param by linear approximation
    flt const ydiff = targetShiftDist - startShiftDist;
    flt const approxLen = sqrt(newShiftLength * newShiftLength + ydiff * ydiff);


    //define support points in local coordinate system
    Vec2 const shiftStartPos(0, startShiftDist);
    Vec2 const shiftEndPos(newShiftLength, targetShiftDist);

    map<flt, Vec2> supportPoints;
    supportPoints[0.0] = shiftStartPos;
    supportPoints[1.0] = shiftEndPos;

//  supportPoints[0.5] = Vec2(newShiftLength * 0.5, startShiftDist + ydiff * 0.45);
//  supportPoints[0.7] = Vec2(newShiftLength * 0.7, startShiftDist + ydiff * 0.5);

    if (isnan(targetShiftDist)) {
        std::cout<<shiftStartPos << "    " << shiftEndPos << "     " << laneWidth;
        assert(false);
    }

    //add support points
    map<flt, Vec2>::const_iterator it = supportPoints.begin();
    map<flt, Vec2>::const_iterator next = supportPoints.begin();
    next++;

    for (; it != supportPoints.end(); it++, next++) {
        if (next == supportPoints.end()) {
            curTemplatePlan.plan->push_back(approxLen, it->second, shiftBaseDir);
        }
        else {
            curTemplatePlan.plan->push_back(approxLen * it->first, it->second, normalized(next->second - it->second));
        }
    }

    curTemplatePlan.mApproxLength = approxLen;
    curTemplatePlan.mApproxWidth = ydiff;
    curTemplatePlan.mTargetShiftDist = targetShiftDist;


    return make_pair(curTemplatePlan, newShiftLength);
}
*/




/**
 * verify if hooks would arise if (param,pos,dir) would be inserted in plan (unfinished yet)
 * @return true if hook free
 */
bool Behaviour::verifyHook(Plan_ptr plan, flt param, Vec3 pos, Vec3 dir)
{
    Plan const & curPlan = *plan;

    if (curPlan.empty()) {
        return true;
    }

    std::pair<flt, flt> const dom(curPlan.domain());

    std::pair<Vec3, Vec3> lastPosAndDir;
//  std::cout<<dom.first << "  " << dom.second;
    curPlan.valueAndFirstDerivative(lastPosAndDir, max(dom.first, dom.second - 0.01));

    //check pos
    Vec3 lastDir =  lastPosAndDir.second.normalized();
    Vec3 directCon = (pos - lastPosAndDir.first).normalized();

    if (ssd(pos, lastPosAndDir.first) < sqr(0.5 * mSampleDist.get())) {
        return false;
    }

    if (ssd(pos, lastPosAndDir.first) > sqr(10 * mSampleDist.get())) {
        return false;
    }

//  std::cout<<"test: " << param << "   " << directCon.norm() << "  " << param << "-" << dom.second << " = " << param-dom.second;
//  if(directCon.norm() > param-dom.second+0.01) return false;

    flt angleDiff = ::math::angle(directCon, lastDir);

    std::cout << "dom: " << dom.first << "/" << dom.second << "  p:  " << param << "  diff: " << angleDiff * R2D
              << "  lastDir: " << lastDir[0] << " " << lastDir[1] << " directCon: " << directCon [0] << " " << directCon[1]
              << " pos: " << pos[0] << " " << pos[1] << "  lastPos: " << lastPosAndDir.first[0] << " " << lastPosAndDir.first[1];

    if (abs(angleDiff) > 90 * D2R) {
//      std::cout<<" directCon: " << directCon << " lastPos: " << lastPosAndDir.first << " lastDir: " << lastDir << " angleDiff: " << angleDiff << std::endl;
        return false;
    }

    return true;
}


Plan_ptr Behaviour::evaluatePlans()
{

    MicroPlanVec const & curMicroPlanVec = mMicroPlanVec;

    //get statemachine functions
    OperationCaller<bool(int)> isInState(mStateMachine->getOperation("isInState"));
//  OperationCaller<State(void)> getCurrentState(mStateMachine->getOperation("getCurrentState"));

    OperationCaller<string(void)> getOperatorDecision(mStateMachine->getOperation("getOperatorDecision"));
    OperationCaller<void(string)> setOperatorDecision(mStateMachine->getOperation("setOperatorDecision"));
    OperationCaller<int(void)> getCurrentStateId(mStateMachine->getOperation("getCurrentStateId"));



    if (numMicroPlans() <= 1) {
        setOperatorDecision("");
    }

    //get meta
    string metaString = getOperatorDecision();


    //determine evaluation functions
    vector<WeightedEvalFunction> evalFunctions;

    if ((isInState(STATE_WAITFOROPERATOR) || isInState(STATE_OPERATORDRIVE) || metaString == "left" || metaString == "right")) {

        //add eval functions
        evalFunctions.push_back(make_pair<EvalFunction, flt>(&Behaviour::evaluateOperatorBinaryInput, 1.0));
    }
    else {
        evalFunctions.push_back(make_pair<EvalFunction, flt>(&Behaviour::evaluateCollision, 1.0 /*0.5*/));
        evalFunctions.push_back(make_pair<EvalFunction, flt>(&Behaviour::evaluateBlindSpots, 1.0 /*0.5*/));
        evalFunctions.push_back(make_pair<EvalFunction, flt>(&Behaviour::evaluateLaneChange, 1.0 /*0.3*/));
        evalFunctions.push_back(make_pair<EvalFunction, flt>(&Behaviour::evaluateLanePreference, 1.0 /*0.3*/));
        evalFunctions.push_back(make_pair<EvalFunction, flt>(&Behaviour::evaluateRemainDist, 1.0 /*0.3*/));
    }


    //normalize, if not already normalized
//  flt sumWeights = 0.f;
//  for(vector<WeightedEvalFunction>::const_iterator iter = evalFunctions.begin(); iter != evalFunctions.end(); iter++) {
//      sumWeights += iter->second;
//  }
//  for(vector<WeightedEvalFunction>::iterator iter = evalFunctions.begin(); iter != evalFunctions.end(); iter++) {
//      iter->second = iter->second / sumWeights;
//  }



    int obstacleEvalIndex = -1;
    int remainDistEvalIndex = -1;

    for (uint e = 0; e < evalFunctions.size(); e++) {
        if (evalFunctions[e].first == &Behaviour::evaluateCollision) {
            obstacleEvalIndex = e;
        }
        else if (evalFunctions[e].first == &Behaviour::evaluateRemainDist) {
            remainDistEvalIndex = e;
        }
    }


    //reset blindspots
    mBlindSpotData.clear();

    // reset evaluation
    mEvaluationVec.clear();


    if (mLastPlan.get() != 0 && mInPreviousLaneChange) {
        return mLastPlan;
    }



    //start evaluation (part 1): for each plan calculate mini-evaluations
    vector< vector< vector< vector<flt> > > > allMiniEval;

    for (uint i = 0; i < mMicroPlanVec.size(); i++) {
        vector< vector< vector<flt> > > laneMiniEval;

        for (uint j = 0; j < mMicroPlanVec[i].size(); j++) {
            vector< vector<flt> > planMiniEval;

            for (uint k = 0; k < mMicroPlanVec[i][j].size(); k++) {
                Plan_ptr plan = mMicroPlanVec[i][j][k];

                Plan const & curPlan = *plan;
                vector<flt> miniEvals;

                //ignore empty plans
                if (curPlan.empty()) {
                    miniEvals.push_back(-1);

                }
                else {
                    for (uint e = 0; e < evalFunctions.size(); e++) {
                        flt partEval = evalFunctions[e].first(this, plan);

                        if (e == remainDistEvalIndex) {
                            partEval = roundMSD(partEval, 3);
                        }

                        miniEvals.push_back(partEval);
                    }
                }

                planMiniEval.push_back(miniEvals);
            }

            laneMiniEval.push_back(planMiniEval);
        }

        allMiniEval.push_back(laneMiniEval);
    }



    vector< vector< vector< flt > > > allShiftPenalties;
    vector< vector< flt > > allLaneChangePenalties;

    //continue evaluation (part 2): calculate penalities
    for (uint i = 0; i < mMicroPlanVec.size(); i++) {
        vector< vector< flt > > laneShiftPenalties;
        vector< flt > laneLaneChangePenalties;

        for (uint j = 0; j < mMicroPlanVec[i].size(); j++) {
            vector< flt > planShiftPenalties;

            //check for tie in obstalce evalutions
            bool obstacleEvalTie = true;

            if (obstacleEvalIndex < 0) {
                obstacleEvalTie = false;
            }
            else {
                for (uint k = 0; k < mMicroPlanVec[i][j].size(); k++) {
                    flt obstacleEval = allMiniEval[i][j][k][obstacleEvalIndex];

                    if (obstacleEval == 0) {
                        obstacleEvalTie = false;
                        break;
                    }
                    else if (abs(obstacleEval - allMiniEval[i][j][0][obstacleEvalIndex]) > epsilon) {
                        obstacleEvalTie = false;
                        break;
                    }
                }
            }

            //calc shift penalties: prefer last one when all obstacles eval are the same, else prefer middle one
            for (uint k = 0; k < mMicroPlanVec[i][j].size(); k++) {
                int shiftOffset = 0;

                if (obstacleEvalTie) {
                    shiftOffset = abs((int)k - mNumShifts.get() - mLastShift);
                }
                else {
                    shiftOffset = abs((int)k - mNumShifts.get());
                }

                flt shiftPenalty = rangeCut(-0.8, shiftOffset * 0.05, 0.8) * OBSTACLE_EVAL;

                planShiftPenalties.push_back(shiftPenalty);
            }

            laneShiftPenalties.push_back(planShiftPenalties);
            laneLaneChangePenalties.push_back(0);
        }

        allShiftPenalties.push_back(laneShiftPenalties);
        allLaneChangePenalties.push_back(laneLaneChangePenalties);
    }



    //continue evaluation (part 3): add it as weighted sum
    for (uint i = 0; i < mMicroPlanVec.size(); i++) {
        vector< vector<flt> > laneEval;

        for (uint j = 0; j < mMicroPlanVec[i].size(); j++) {
            vector<flt> planEval;

            for (uint k = 0; k < mMicroPlanVec[i][j].size(); k++) {
                Plan curPlan = *mMicroPlanVec[i][j][k];

                flt totalEval = 0.0;

                //ignore empty plans
                if (curPlan.empty()) {
                    totalEval = -1.0;

                }
                else {
                    totalEval = 0.0;

                    vector<flt> evals = allMiniEval[i][j][k];

                    for (uint e = 0; e < evalFunctions.size(); e++) {
                        totalEval += evals[e] * evalFunctions[e].second;
                    }

                    //add lane change penalty
                    totalEval += allLaneChangePenalties[i][j];

                    //add shift penalty
                    totalEval += allShiftPenalties[i][j][k];
                }

                planEval.push_back(totalEval);
            }

            laneEval.push_back(planEval);
        }

        mEvaluationVec.push_back(laneEval);
    }




    //debug evaluation
    if (mPrettyPrintDebugEvaluation.get()) {
//      cout << PrettyPrint::cGreen << "Evaluation of Micro plans:" << std::endl << std::flush;
        cout << prettyprint::cYellow << "Plan #" << prettyprint::cGreen << "\tCollision \tBlindSpot \tLaneChange \tLanePref \tRemainDist \tLC-P \tS-P" << prettyprint::cRed << "\t    Total" << std::endl;
        std::cout.precision(6);

        for (uint i = 0; i < mMicroPlanVec.size(); i++) {

            for (uint j = 0; j < mMicroPlanVec[i].size(); j++) {

                for (uint k = 0; k < mMicroPlanVec[i][j].size(); k++) {
                    Plan const & curPlan = *mMicroPlanVec[i][j][k];

                    //ignore empty plans
                    if (curPlan.empty()) {
                        std::cout << prettyprint::cYellow << i << "/" << j << "/" << k << ":\t";
                        std::cout << prettyprint::cRed << "disabled" << std::endl;

                    }
                    else {
                        std::cout << prettyprint::cYellow << i << "/" << j << "/" << k << ":\t";

                        vector<flt> evals = allMiniEval[i][j][k];

                        for (uint e = 0; e < evalFunctions.size(); e++) {
                            std::cout  << prettyprint::cBlue << "F(" << e << "): " << evals[e]*evalFunctions[e].second << ",\t";
                        }

                        std::cout << prettyprint::cWhite << (allLaneChangePenalties[i][j] >= 0 ? "+" : "") << allLaneChangePenalties[i][j] << ",\t";
                        std::cout << prettyprint::cWhite << (allShiftPenalties[i][j][k] >= 0 ? "+" : "") << allShiftPenalties[i][j][k] << ",\t";
                        std::cout << prettyprint::cRed << " -> " << mEvaluationVec[i][j][k] << std::endl;
                    }

                }
            }
        }

        std::cout << std::endl;
    }


    int prefLane = rangeCut<int>(0, mDebugPreferredLane.get(), mMicroPlanVec.size() - 1);
//      std::cout<<"prefLane: " << prefLane;
    int prefPlan = rangeCut<int>(0, mDebugPreferredPlan.get(), mMicroPlanVec[prefLane].size() - 1);
//      std::cout<<"prefPlan: " << prefPlan;
    int prefShift = rangeCut<int>(0, mDebugPreferredShift.get() + mNumShifts.get(), mMicroPlanVec[prefLane][prefPlan].size() - 1);

    if (mDebugEvaluation.get()) {
        mDebugPreferredLane.set(0);
        return mMicroPlanVec[prefLane][prefPlan][prefShift];
    }


    //select best one
    Plan_ptr bestPlan = mMicroPlanVec[prefLane][prefPlan][prefShift];
    flt bestPlanValue = std::numeric_limits<flt>::infinity();
    int bestShift = 0;

    for (uint i = 0; i < mEvaluationVec.size(); i++) {
        for (uint j = 0; j < mEvaluationVec[i].size(); j++) {
            for (uint k = 0; k < mEvaluationVec[i][j].size(); k++) {
                if (mEvaluationVec[i][j][k] < 0) {
                    continue;
                }

                if (mEvaluationVec[i][j][k] < bestPlanValue) {
                    bestPlanValue = mEvaluationVec[i][j][k];
                    bestPlan = mMicroPlanVec[i][j][k];
                    bestShift = k - mNumShifts.get();
                }
            }
        }
    }


    mLastShift = bestShift;

    return bestPlan;
}

