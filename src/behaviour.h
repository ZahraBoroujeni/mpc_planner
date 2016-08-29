#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <fub_cargate_msgs/Activation.h>
#include <fub_cargate_msgs/AuxDevicesData.h>
#include <fub_cargate_msgs/BrakePressure.h>
#include <fub_cargate_msgs/HeadLight.h>
#include <fub_cargate_msgs/Gear.h>
#include <fub_cargate_msgs/NormalizedSpeed.h>
#include <fub_cargate_msgs/NormalizedSteeringAngle.h>
#include <fub_cargate_msgs/Siren.h>
#include <fub_cargate_msgs/SteeringAngle.h>
#include <fub_cargate_msgs/ThrottleVoltage.h>
#include <fub_cargate_msgs/TurnSignal.h>
#include <fub_cargate_msgs/Wiper.h>



#include <autonomos_comm_transport/udp.h>

#include "CargatePacket.h"



namespace fub
{
namespace behaviour
{


class behaviour: public nodelet::Nodelet 
{
private:
	// the node handle
	ros::NodeHandle nh_;

	// node handle in the private namespace
	ros::NodeHandle priv_nh_;

	// subscribers


public:

	explicit behaviour(ros::NodeHandle nh, int argc,char** argv);
	virtual ~behaviour();
	virtual bool startHook();
	virtual void updateHook();
	virtual void stopHook();
typedef ::math::flt flt;
    typedef ::math::Vec2 Vec2;
    typedef ::math::Vec3 Vec3;

	typedef TimedData< Mission > TimedMission;
	typedef TimedData< aa::modules::nav::obstacles::BlindSpot > TimedBlindSpot;
	typedef aa::modules::nav::obstacles::SideObstacleSpot SideObstacleSpotData;

	typedef std::vector< SampledPoint > SampledPointList;
	typedef std::vector< SampledAction > SampledActionList;
	typedef std::vector< flt > DriftList;

	typedef std::vector< std::vector< MacroPlan > > MacroPlanVec;					//vector of lanes of vertex_descr
	typedef std::vector< std::vector< std::vector< SampledPointList > > > SampledPointPlanVec;
	typedef std::vector< std::vector< std::vector< SampledActionList > > > SampledActionPlanVec;
	typedef std::vector< std::vector< std::vector< aa::modules::nav::controller::Plan_ptr > > > MicroPlanVec;		//vector of lanes of shifts of plans

	typedef boost::function<flt(Behaviour * , aa::modules::nav::controller::Plan_ptr) > EvalFunction;				//evaluation function
	typedef std::pair< EvalFunction, flt > WeightedEvalFunction;					//evaluation function and associated weight

	typedef std::vector< std::vector< std::vector<flt > > > EvaluationVec;			//vector of lanes of shifts of evaluation

    typedef math::PolySpline<Vec3, flt, 4u> SplineType;
	typedef aa::modules::nav::obstacles::ObstaclesOnSpline<SplineType, BaseObstacleBundle> ObstacleMath;

	typedef std::map<aa::modules::nav::controller::Plan_ptr, TimedBlindSpot> BlindSpotData;


private:
	std::vector<geometry_msgs::PoseStamped> mMicroPlanVec						/** preferred shift index (from -mNumShifts to +mNumShifts) for debugging purposes */
private:
	QMutex mMutex;

	TimedEgoState mCurEgoState;
	TimedMission mCurMission;
	TimedBaseObstacleBundle_ptr mCurObstacles;
	ShortestPath mShortestPath;

	aa::modules::nav::controller::Plan_ptr mPlan;
	aa::modules::nav::controller::Plan_ptr mLastPlan;

	int mLastShift;
	aa::modules::nav::controller::Plan::action_descr mLastLaneChange;
	aa::modules::nav::controller::Plan::action_descr mLastTurn;
	DriftList mLastDrift;

	std::vector<geometry_msgs::PoseStamped> mMacroPlanVec;
	SampledPointPlanVec mSampledPointPlanVec;
	SampledActionPlanVec mSampledActionPlanVec;
	std::vector<geometry_msgs::PoseStamped> mMicroPlanVec;
	std::vector<geometry_msgs::PoseStamped> mTemplatePlanVec;

	std::vector<geometry_msgs::PoseStamped> mEvaluationVec;
	BlindSpotData mBlindSpotData;


    ::aa::modules::models::arnd::ARNDGraph & mARNDGraph;
	::aa::modules::nav::controller::ComfortSettings & mComfortSettings;

	bool mInPreviousLaneChange;
	flt mLastSwerveTemplateParam;
	flt mLastLaneChangeTemplateParam;
	flt mLastSampleStep;
	flt mLastPlanDist;

	int mCyclesOvertakingConditionMet;
	int mCyclesReplanNeeded;


	//helper functions
	flt getPlanDist(flt curSpeed);

    bool findRightNeighbour(flt param, ::aa::modules::models::arnd::edge_descr const & edge, ::aa::modules::models::arnd::edge_descr & rightEdge, Vec3 & rightPos, flt & rightParam, bool extCheck = true) const;
    bool findLeftNeighbour(flt param, ::aa::modules::models::arnd::edge_descr const & edge, ::aa::modules::models::arnd::edge_descr & leftEdge, Vec3 & leftPos, flt & leftParam, bool extCheck = true) const;

    std::pair<TemplatePlan, flt> buildSwerveTemplate(::aa::modules::models::arnd::edge_descr const & edge, flt param, int s, Vec3 const & curPos, Vec3 const & curDir, flt swerveLength, flt curSpeed, swerve_type type);
//	std::pair<TemplatePlan, flt> buildShiftTemplate(MacroPlan macroPlan, int vertexIndex, edge_descr const & edge, flt param, int s, Vec2 const & curPos, Vec2 const & curDir, flt shiftLength);
    bool verifyHook(aa::modules::nav::controller::Plan_ptr plan, flt param, Vec3 pos, Vec3 dir);

	int numMacroPlans() const;
	int numMicroPlans() const;

    std::vector< ::aa::modules::models::arnd::edge_descr> outEdges(::aa::modules::models::arnd::vertex_descr v, bool allowMirrored, bool allowUTurn, bool allowInZone) const;
    std::vector< ::aa::modules::models::arnd::edge_descr> inEdges(::aa::modules::models::arnd::vertex_descr v, bool allowMirrored, bool allowUTurn, bool allowInZone) const;

    bool connectedEdges(::aa::modules::models::arnd::edge_descr a, ::aa::modules::models::arnd::edge_descr b) const;

    flt distOnSpline(SplineType spline, flt param, Vec3 pos) const;
    flt distOnSpline(SplineType spline, Vec3 posA, Vec3 posB) const;

    bool isInDir(Vec3 baseDir, Vec3 dirToCheck, flt angleTolerance) const;
};

}
}

