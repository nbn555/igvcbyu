/**
 * @file YclopsnavigationSystem.cpp
 * @date Dec 9, 2010
 * @author igvcbyu
 * @brief AI
 */

#include "YclopsNavigationSystem.h"  // Precomp header
#include "YClopsReactiveNavInterface.h"

#include <mrpt/reactivenav.h>

#include <mrpt/utils.h>
#include <mrpt/system/filesystem.h>

#include "logging.h"

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::reactivenav;
using namespace std;

#define		PREVIOUS_VALUES_IN_LOG		50

/*---------------------------------------------------------------
					Constructor
  ---------------------------------------------------------------*/
YclopsNavigationSystem::YclopsNavigationSystem(
	CReactiveInterfaceImplementation   &react_iterf_impl,
    bool					enableConsoleOutput,
    bool					enableLogToFile)
	:
	CAbstractReactiveNavigationSystem(react_iterf_impl)
{
	// Initialize some members:
	logFile				= NULL;
	holonomicMethod		= NULL;
	enableConsoleOutput = enableConsoleOutput;
	CerrandoHilo		= false;
	nIteration			= 0;
	meanExecutionPeriod	= 0.1f;
	last_cmd_v			= 0;
	last_cmd_w			= 0;

	PTGs.resize(0);
	enableLogFile( enableLogToFile );

	// Reset behavior:
	navigatorBehavior = beNormalNavigation;
}

/*---------------------------------------------------------------
						initialize
  ---------------------------------------------------------------*/
void YclopsNavigationSystem::initialize()
{
	// Compute collision grids:
	STEP1_CollisionGridsBuilder();
}


/*---------------------------------------------------------------
						changeRobotShape
  ---------------------------------------------------------------*/
void YclopsNavigationSystem::changeRobotShape( math::CPolygon &shape )
{
	collisionGridsMustBeUpdated = true;

	if ( shape.verticesCount()<3 )
		THROW_EXCEPTION("The robot shape has less than 3 vertices!!")

	robotShape = shape;
}

/*---------------------------------------------------------------
						getLastLogRecord
	Provides a copy of the last log record with information
		about execution.
  ---------------------------------------------------------------*/
void YclopsNavigationSystem::getLastLogRecord( CLogFileRecord &o )
{
	mrpt::synch::CCriticalSectionLocker lock(&m_critZoneLastLog);
	try {
		o = lastLogRecord;
	}
	catch (...) { }
}


/*---------------------------------------------------------------
						loadConfigFile
  ---------------------------------------------------------------*/
void YclopsNavigationSystem::loadConfigFile(const mrpt::utils::CConfigFileBase &ini, const mrpt::utils::CConfigFileBase &robotIni )
{
	MRPT_START;
	collisionGridsMustBeUpdated = true;

	// Load config from INI file:
	// ------------------------------------------------------------
	robotName = robotIni.read_string("ROBOT_NAME","Name", "", true );

	unsigned int PTG_COUNT = ini.read_int(robotName,"PTG_COUNT",0, true );

	refDistance = ini.read_float(robotName,"MAX_REFERENCE_DISTANCE",5 );
	colGridRes_x = ini.read_float(robotName,"RESOLUCION_REJILLA_X",0.02f );
	colGridRes_y = ini.read_float(robotName,"RESOLUCION_REJILLA_Y",0.02f);
	targetAllowedDistance = ini.read_float(robotName, "TARGET_REACH_DISTANCE", 1.00f);

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(robotMax_V_mps,float,  ini,robotName);
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(robotMax_W_degps,float,  ini,robotName);
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(ROBOTMODEL_TAU,float,  ini,robotName);
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(ROBOTMODEL_DELAY,float,  ini,robotName);


	ini.read_vector( robotName, "weights", vector_float(0), weights, true );
	ASSERT_(weights.size()==6);

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(minObstaclesHeight,float,  ini,robotName);
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(maxObstaclesHeight,float,  ini,robotName);

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(DIST_TO_TARGET_FOR_SENDING_EVENT,float,  ini,robotName);


	int HoloMethod = ini.read_int("GLOBAL_CONFIG","HOLONOMIC_METHOD",1, true);

	setHolonomicMethod( (THolonomicMethod) HoloMethod );

	holonomicMethod->initialize( ini );


	badNavAlarm_AlarmTimeout = ini.read_float("GLOBAL_CONFIG","ALARM_SEEMS_NOT_APPROACHING_TARGET_TIMEOUT", 10, true);

	// -----
	DOOR_CROSSING_SEARCH_TARGET_DISTANCEx2 = ini.read_float("DOOR_CROSSING","DOOR_CROSSING_SEARCH_TARGET_DISTANCEx2", 12.0f );
	VORONOI_MINIMUM_CLEARANCE		   	   = ini.read_float("DOOR_CROSSING","VORONOI_MINIMUM_CLEARANCE", 0.2f );
	DISABLE_PERIOD_AFTER_FAIL		   	   = ini.read_float("DOOR_CROSSING","DISABLE_PERIOD_AFTER_FAIL",3.0f );
	VORONOI_PATH_DIST_FROM_DOORWAY		   = ini.read_float("DOOR_CROSSING","VORONOI_PATH_DIST_FROM_DOORWAY",1.0f );
	DOORCROSSING_HEADING_ACCURACY_DEG	   = ini.read_float("DOOR_CROSSING","DOORCROSSING_HEADING_ACCURACY_DEG",10.0f );
	DOORCROSSING_ROTATION_CTE_DEG		   = ini.read_float("DOOR_CROSSING","DOORCROSSING_ROTATION_CTE_DEG", 50.0f );
	DOOR_CROSSING_DIST_TO_AUX_TARGETS	   = ini.read_float("DOOR_CROSSING","DOOR_CROSSING_DIST_TO_AUX_TARGETS", 0.15f );
	DOOR_CROOSING_BEH3_TIMEOUT			   = ini.read_float("DOOR_CROSSING","DOOR_CROOSING_BEH3_TIMEOUT", 4.0f );
	DOOR_CROSSING_MAXIMUM_DOORWAY_SIZE	   = ini.read_float("DOOR_CROSSING","DOOR_CROSSING_MAXIMUM_DOORWAY_SIZE", 1.0f );

	// Load robot shape:
	// ---------------------------------------------
	math::CPolygon		shape;
	vector_float        xs,ys;

	ini.read_vector(robotName,"RobotModel_shape2D_xs",vector_float(0), xs, true );
	ini.read_vector(robotName,"RobotModel_shape2D_ys",vector_float(0), ys, true );
	ASSERT_(xs.size()==ys.size());

	// Add to polygon
	for ( size_t i=0;((unsigned)(i))<xs.size();i++)
		shape.AddVertex(xs[i],ys[i]);

	changeRobotShape( shape );

	// Load PTGs from file:
	// ---------------------------------------------
	// Free previous PTGs:
	for (size_t i=0;i<PTGs.size();i++)	delete PTGs[i];
	PTGs.assign(PTG_COUNT,NULL);

	printf_debug("\n");

	for ( unsigned int n=0;n<PTG_COUNT;n++ )
	{
		// load params of this PTG:

		TParameters<double> params;
		params["ref_distance"] = refDistance;
		params["resolution"]   = colGridRes_x;

		params["PTG_type"]	= ini.read_int(robotName,format("PTG%u_Type", n ),1, true );
		params["v_max"]		= ini.read_float(robotName,format("PTG%u_v_max_mps", n ), 5, true);
		params["w_max"]		= DEG2RAD(ini.read_float(robotName,format("PTG%u_w_max_gps", n ), 0, true));
		params["K"]			= ini.read_int(robotName,format("PTG%u_K", n ), 1, false);
		params["cte_a0v"]	= DEG2RAD( ini.read_float(robotName,format("PTG%u_cte_a0v_deg", n ), 0, false) );
		params["cte_a0w"]	= DEG2RAD( ini.read_float(robotName,format("PTG%u_cte_a0w_deg", n ), 0, false) );

		const int nAlfas = ini.read_int(robotName,format("PTG%u_nAlfas", n ),100, true );

		// Generate it:
		printf_debug("[loadConfigFile] Generating PTG#%u...",n);

		PTGs[n] = CParameterizedTrajectoryGenerator::CreatePTG(params);

		printf_debug(PTGs[n]->getDescription().c_str());

		PTGs[n]->simulateTrajectories(
		    nAlfas,					// alfas,
		    75,						// max.tim,
		    refDistance,			// max.dist,
		    600,					// max.n,
		    0.010f,					// diferencial_t
		    0.015f					// min_dist
		);

		// Solo para depurar, hacer graficas, etc...
		PTGs[n]->debugDumpInFiles(n);

		printf_debug("...OK!\n");
	}
	printf_debug("\n");

	// Show the loaded configuration in the console:
	// --------------------------------------------------------
	printf_debug("\tLOADED CONFIGURATION:\n");
	printf_debug("-------------------------------------------------------------\n");
	printf_debug("  Holonomic method \t\t= ");
	switch (  HoloMethod )
	{
	case hmVIRTUAL_FORCE_FIELDS:
		printf_debug("VFF (Virtual Force Fields)");
		break;
	case hmSEARCH_FOR_BEST_GAP:
		printf_debug("ND (Nearness Diagram)");
		break;
	default:
		printf_debug("Unknown!! (Selecting default one)");
		break;
	};

	printf_debug("\n");
	printf_debug("  Robot name \t\t\t= ");
	printf_debug(robotName.c_str());
	printf_debug("\n  GPT Count\t\t\t= %u\n", (int)PTG_COUNT );
	printf_debug("  Max. ref. distance\t\t= %f\n", refDistance );
	printf_debug("  Cells resolution (x,y) \t= (%.04f,%.04f)\n", colGridRes_x,colGridRes_y );
	printf_debug("  Max. speed (v,w)\t\t= (%.04f m/sec, %.04f deg/sec)\n", robotMax_V_mps, robotMax_W_degps );
	printf_debug("  Robot Shape Points Count \t= %u\n", robotShape.verticesCount() );
	printf_debug("  Obstacles 'z' axis range \t= [%.03f,%.03f]\n", minObstaclesHeight, maxObstaclesHeight );
	printf_debug("\n\n");

	MRPT_END;
}



/*---------------------------------------------------------------
						setHolonomicMethod
  ---------------------------------------------------------------*/
void YclopsNavigationSystem::setHolonomicMethod(
    mrpt::reactivenav::THolonomicMethod	method,
    const char							*config_INIfile)
{
	// Delete current method:
	if (holonomicMethod) delete holonomicMethod;

	switch (method)
	{
	default:
	case hmSEARCH_FOR_BEST_GAP:
		holonomicMethod = new CHolonomicND();
		break;

	case hmVIRTUAL_FORCE_FIELDS:
		holonomicMethod = new CHolonomicVFF();
		break;
	};


}


/*---------------------------------------------------------------
						enableLogFile
  ---------------------------------------------------------------*/
void YclopsNavigationSystem::enableLogFile(bool enable)
{
	try
	{
		// Disable:
		// -------------------------------
		if (!enable)
		{
			if (logFile)
			{
				printf_debug("[YclopsNavigationSystem::enableLogFile] Stopping logging.\n");
				// Close file:
				delete logFile;
				logFile = NULL;
			}
			else return;	// Already disabled.
		}
		else
		{	// Enable
			// -------------------------------
			if (logFile) return; // Already enabled:

			// Open file, find the first free file-name.
			char	aux[100];
			int     nFichero=0;
			bool    nombre_libre= false;

			system::createDirectory("./reactivenav.logs");

			while (!nombre_libre)
			{
				nFichero++;
				sprintf(aux, "./reactivenav.logs/log_%03u.reactivenavlog", nFichero );

				nombre_libre = !system::fileExists(aux);
			}

			// Open log file:
			logFile = new CFileOutputStream(aux);

			printf_debug("[YclopsNavigationSystem::enableLogFile] Logging to file:");
			printf_debug(aux);
			printf_debug("\n");

		}
	} catch (...) {
		printf_debug("[YclopsNavigationSystem::enableLogFile] Exception!!\n");
	}

}

/*---------------------------------------------------------------
	  				performNavigationStep

  Este metodo se ejecuta periodicamente solo si se está en el estado
   NAVIGATING. Aqui se implementa el algoritmo de navegacion reactiva
  ---------------------------------------------------------------*/
void  YclopsNavigationSystem::performNavigationStep()
{
	static utils::CTicTac						totalExecutionTime, executionTime, tictac;
	static mrpt::slam::CSimplePointsMap			WS_Obstacles;
	static CLogFileRecord						newLogRec;
	static std::vector<THolonomicMovement>		selectedHolonomicMovements;
	static std::vector<vector_double>			TP_Obstacles;
	float										targetDist;
	poses::CPoint2D								relTarget;		// The target point, relative to current robot pose.
	static std::vector<mrpt::poses::CPoint2D, Eigen::aligned_allocator<mrpt::poses::CPoint2D> >			TP_Targets;		// Target location (x,y) in TP-Space
	poses::CPose2D								curPose;
	float										curVL;			// en metros/seg
	float										curW;			// en rad/segundo
	static std::vector<THolonomicMovement>		holonomicMovements;
	THolonomicMovement							selectedHolonomicMovement;
	float										cmd_v=0,cmd_w=0;	// The non-holonomic command
	float 										desired_cmd_v, desired_cmd_w;
	std::vector<CHolonomicLogFileRecordPtr>		HLFRs;
	static std::vector<float>					times_TP_transformations, times_HoloNav;
	static std::vector<bool>					valid_TP;
	static float								meanExecutionTime = 0.1f;
	static float								meanTotalExecutionTime= 0.1f;
	int											nSelectedPTG;
	static vector_float							prevV,prevW,prevSelPTG;
	static int									nLastSelectedPTG = -1;
//	static CDynamicWindow						DW;
//	static TNavigatorBehavior					lastStepBehavior;
//	TNavigatorBehavior							saveLastBehavior;

	float cur_approx_heading_dir = 0;

	// Already closing??
	if (CerrandoHilo) return;

	// Lock
	mrpt::synch::CCriticalSectionLocker lock( &m_critZoneNavigating );

	try
	{
		// Iterations count:
		nIteration++;

		// Start timer
		totalExecutionTime.Tic();

		// ----------------------------------------------------------------
		//	  Request current robot pose and velocities
		// ----------------------------------------------------------------
		LOG(DEBUG4) << "Running CurrentPose and Speeds" << endl;
		if ( !m_robot.getCurrentPoseAndSpeeds(curPose, curVL, curW ) )
		{
			Error_ParadaDeEmergencia("ERROR calling m_robot.getCurrentPoseAndSpeeds, stopping robot and finishing navigation");
			return;
		}
		cout << "CurPose: (" << curPose.x() << "," << curPose.y() << "," << curPose.phi()*180./M_PI << ")" << endl;
		 
		cout << "Target: (" << m_navigationParams.target.x << "," << m_navigationParams.target.y << ")" << endl;
		 
		LOG(DEBUG) << "Cur Velocity: Linear " << curVL << " Angular " << curW << endl;
		// ----------------------------------------------------------------
		// 	  Have we reached the target location?
		// ----------------------------------------------------------------
		targetDist = curPose.distance2DTo( m_navigationParams.target.x, m_navigationParams.target.y );

		

		cout << "Range to target: " << targetDist << endl;
		

		if ( targetDist < m_navigationParams.targetAllowedDistance )
		{

			LOG_AI(INFO) << "Reached Way Point" << endl;
			points.erase(points.begin());
			if(points.size() != 0)//this->points)
			{
				gotoNextPoint();
			}
			else
			{
				m_robot.stop();
				m_navigationState = IDLE;
				LOG_AI(INFO) << "Navigation target was reached!" << endl;

				if (!navigationEndEventSent)
				{
					navigationEndEventSent = true;
					m_robot.sendNavigationEndEvent();
				}
				return;
			}

		}

		// Check the "no approaching the target"-alarm:
		// -----------------------------------------------------------
		if (targetDist < badNavAlarm_minDistTarget )
		{
			badNavAlarm_minDistTarget = targetDist;
			badNavAlarm_lastMinDistTime =  system::getCurrentTime();
		}
		else
		{
			// Too much time have passed?
			if ( badNavAlarm_AlarmTimeout && system::timeDifference( badNavAlarm_lastMinDistTime, system::getCurrentTime() ) > badNavAlarm_AlarmTimeout)
			{

				LOG_AI(ERROR) << "\n--------------------------------------------\n Timed out trying new Target \n---------------------------------" << endl;
				if(navigatorBehavior == goAround)
				{
					points.erase(points.begin());
				}

				points.insert(points.begin(), makeAuxTarget(curPose));

				gotoNextPoint();
				navigatorBehavior = goAround;
				return;
			}
		}


		// Compute target location relative to current robot pose:
		// ---------------------------------------------------------------------
		relTarget = CPoint2D(m_navigationParams.target) - curPose;
		LOG_AI(INFO) << "at: " << curPose.x() << "," << curPose.y() << " looking for " << m_navigationParams.target.x << "," << m_navigationParams.target.y << endl;

		// STEP1: Collision Grids Builder.
		// -----------------------------------------------------------------------------
		STEP1_CollisionGridsBuilder();

		// STEP2: Sense obstacles.
		// -----------------------------------------------------------------------------
		if (! STEP2_Sense( WS_Obstacles ) )
		{
			LOG_AI(WARNING) << "Warning: Error while sensing obstacles. Robot will be stopped." << endl;
			m_robot.stop();
			m_navigationState = NAV_ERROR;
			return;
		}


		// Start timer
		executionTime.Tic();

		// For some behaviors:
		//  If set to true, "cmd_v" & "cmd_w" must be set to the desired values:
		bool		skipNormalReactiveNavigation = false;


		if (! skipNormalReactiveNavigation )
		{
			// Clip obstacles by "z" axis coordinates:
			WS_Obstacles.clipOutOfRangeInZ( minObstaclesHeight, maxObstaclesHeight );
			// Clip obstacles out of the reactive method range:
			CPoint2D    dumm(0,0);
			WS_Obstacles.clipOutOfRange( dumm, refDistance+1.5f );


			//  STEP3: Build TP-Obstacles and transform target location into TP-Space
			// -----------------------------------------------------------------------------
			// Vectors for each PTG:
			if (PTGs.size() != TP_Targets.size() )   TP_Targets.resize( PTGs.size() );
			if (PTGs.size() != valid_TP.size() )	valid_TP.resize( PTGs.size() );

			// Vectors for each PTG & Security Distance:
			const size_t n = PTGs.size();
			if (n != TP_Obstacles.size() )				TP_Obstacles.resize( n );
			if (n != times_TP_transformations.size() )	times_TP_transformations.resize( n );
			if (n != holonomicMovements.size() )		holonomicMovements.resize( n );
			if (n != HLFRs.size() )						HLFRs.resize(n);
			if (n != times_HoloNav.size() )				times_HoloNav.resize( n );
			newLogRec.infoPerPTG.resize( n );

			// For each PTG:
			for (size_t indexPTG=0;indexPTG<PTGs.size();indexPTG++)
			{
				// Target location:
				float	alfa,dist;
				int		k;

				// Firstly, check if target falls into the PTG domain!!
				valid_TP[indexPTG] = true;
				//valid_TP[i] = PTGs[i]->PTG_IsIntoDomain( relTarget.x,relTarget.y );

				if (valid_TP[indexPTG])
				{
					PTGs[indexPTG]->lambdaFunction(
					    relTarget.x(),
					    relTarget.y(),
					    k,
					    dist );

					alfa = PTGs[indexPTG]->index2alfa(k);
					TP_Targets[indexPTG].x( cos(alfa) * dist );
					TP_Targets[indexPTG].y( sin(alfa) * dist );
				}

				// And for each security distance:
				tictac.Tic();

				// TP-Obstacles
				STEP3_SpaceTransformer(	WS_Obstacles,
				                        PTGs[indexPTG],
				                        TP_Obstacles[indexPTG] );

				times_TP_transformations[indexPTG] = tictac.Tac();

			} // indexPTG

			//  STEP4: Holonomic navigation method
			// -----------------------------------------------------------------------------
			// For each PTG:
			for (size_t indexPTG=0;indexPTG<PTGs.size();indexPTG++)
			{
				tictac.Tic();

				holonomicMovements[indexPTG].PTG = PTGs[indexPTG];

				if (valid_TP[indexPTG])
				{
					STEP4_HolonomicMethod(	TP_Obstacles[indexPTG],
					                       TP_Targets[indexPTG],
					                       PTGs[indexPTG]->getMax_V_inTPSpace(),
					                       holonomicMovements[indexPTG],
					                       HLFRs[indexPTG]);
				}
				else
				{
					holonomicMovements[indexPTG].direction = 0;
					holonomicMovements[indexPTG].evaluation = 0;
					holonomicMovements[indexPTG].speed = 0;
					HLFRs[indexPTG] = CLogFileRecord_VFF::Create();
				}
				times_HoloNav[indexPTG] = (float)tictac.Tac();
			} // indexPTG

			// STEP5: Evaluate each movement to assign them a "evaluation" value.
			// ---------------------------------------------------------------------

			// For each PTG:
			for (size_t indexPTG=0;indexPTG<PTGs.size();indexPTG++)
			{
				if (valid_TP[indexPTG])
					STEP5_Evaluator(	holonomicMovements[indexPTG],
					                 TP_Obstacles[indexPTG],
					                 relTarget,
					                 TP_Targets[indexPTG],
					                 nLastSelectedPTG == (int)indexPTG,
					                 newLogRec.infoPerPTG[indexPTG] );

				if ( HLFRs[indexPTG].present()  )
				{
					if (IS_CLASS(HLFRs[indexPTG], CLogFileRecord_ND))
					{
						if (CLogFileRecord_NDPtr(HLFRs[indexPTG])->situation == CHolonomicND::SITUATION_TARGET_DIRECTLY)
						{
							holonomicMovements[indexPTG].evaluation += 1.0f;
						}
					}
				}
			} // indexPTG

			// STEP6: Selects the best movement
			// ---------------------------------------------------------------------
			STEP6_Selector( holonomicMovements, selectedHolonomicMovement, nSelectedPTG );
			nLastSelectedPTG = nSelectedPTG;


			// Compute the approximate heading direction
			// ----------------------------------------------------------------
			{
				static int nCnt=0;
				if (++nCnt>10)
				{
					nCnt=0;

					float x=1;
					float y=0;
					float p,t;
					selectedHolonomicMovement.PTG->getCPointWhen_d_Is(2.0, selectedHolonomicMovement.PTG->alfa2index(selectedHolonomicMovement.direction),x,y,p,t);

					cur_approx_heading_dir = curPose.phi();
					//m_robot.notifyHeadingDirection(cur_approx_heading_dir);
				}
			}

			// STEP7: Get the non-holonomic movement command.
			// ---------------------------------------------------------------------
			STEP7_NonHolonomicMovement( selectedHolonomicMovement, cmd_v, cmd_w);

			last_cmd_v = cmd_v;
			last_cmd_w = cmd_w;

			// STEP8: Dynamics: Finds the best (cmd_v/w) closer to (desired_cmd_v/w);
			// ---------------------------------------------------------------------
			desired_cmd_v = cmd_v;
			desired_cmd_w = cmd_w;

			//		DW.v_max = min( robotMax_V_mps, curVL + robotMax_V_accel_mpss * deg);
			//		DW.v_min = max(-robotMax_V_mps, curVL - robotMax_V_accel_mpss * meanExecutionPeriod);
			//		DW.w_max = min( DEG2RAD(robotMax_W_degps), curW + DEG2RAD(robotMax_W_accel_degpss * meanExecutionPeriod) );
			//		DW.w_min = max(-DEG2RAD(robotMax_W_degps), curW - DEG2RAD(robotMax_W_accel_degpss * meanExecutionPeriod) );


		} // end of "!skipNormalReactiveNavigation"



		//double cmd_w = atan2(m_navigationParams.target.y - curPose.y(), m_navigationParams.target.x - curPose.x());
		//double cmd_v = targetDist;

		// ---------------------------------------------------------------------
		//				SEND MOVEMENT COMMAND TO THE ROBOT
		// ---------------------------------------------------------------------
		
		cout << "CMD: " << cmd_v << "\t" << cmd_w << endl; 
		LOG(DEBUG4) << "\n\n\n\n\n\n\nRunning change speeds to: Linear " << cmd_v << " Angular " << cmd_w << "\n\n\n\n\n\n\n\n" << endl;
		if ( cmd_v == 0.0 && cmd_w == 0.0 ) {
			m_robot.stop();
		} else {
			if ( !m_robot.changeSpeeds( cmd_v, cmd_w ) )
			{
				Error_ParadaDeEmergencia("\nERROR calling RobotMotionControl::changeSpeeds!! Stopping robot and finishing navigation\n");
				return;
			}
		}

		// Statistics:
		// ----------------------------------------------------
		float	executionTimeValue = (float) executionTime.Tac();
		meanExecutionTime=  0.3f * meanExecutionTime +
		                    0.7f * executionTimeValue;
		meanTotalExecutionTime=  0.3f * meanTotalExecutionTime +
		                         0.7f * ((float)totalExecutionTime.Tac() );
		meanExecutionPeriod = 0.3f * meanExecutionPeriod +
		                      0.7f * min(1.0f, (float)timerForExecutionPeriod.Tac());


		timerForExecutionPeriod.Tic();

		//printf("%c[%d;%dmCMD:%.02lfm/s,%.02lfd/s \t", 27, 1, 31,
		//           (double)cmd_v,
		//           (double)RAD2DEG( cmd_w ));
		//printf("%c[%dm", 27, 0);
		LOG_AI(INFO) << "CMD:" << (double)cmd_v << "m/s, " << (double)RAD2DEG( cmd_w ) << "d/s \t" << endl;

		printf_debug(" T=%.01lfms Exec:%.01lfms|%.01lfms \t",
		           1000.0*meanExecutionPeriod,
		           1000.0*meanExecutionTime,
		           1000.0*meanTotalExecutionTime );

		if (!skipNormalReactiveNavigation)
		{
			printf_debug("E=%.01lf ", (double)selectedHolonomicMovement.evaluation );
			printf_debug("PTG#%i ", nSelectedPTG);
		}
		else
		{
			nSelectedPTG = 0;
		}

		printf_debug("BEHAVIOR:%i\n", (int)navigatorBehavior );

		// ---------------------------------------
		// Generate log record
		// ---------------------------------------
		newLogRec.WS_Obstacles			= WS_Obstacles;
		newLogRec.robotOdometryPose		= curPose;
		newLogRec.WS_target_relative	= relTarget;
		newLogRec.v						= cmd_v;
		newLogRec.w						= cmd_w;
		newLogRec.nSelectedPTG			= nSelectedPTG;
		newLogRec.executionTime			= executionTimeValue;
		newLogRec.actual_v				= curVL;
		newLogRec.actual_w				= curW;
		newLogRec.estimatedExecutionPeriod = meanExecutionPeriod;
		newLogRec.nPTGs					= PTGs.size();
		newLogRec.navigatorBehavior		= navigatorBehavior;

		const size_t nVerts = robotShape.size();
		if (newLogRec.robotShape_x.size() != ((unsigned)(nVerts)))
		{
			newLogRec.robotShape_x.resize(nVerts);
			newLogRec.robotShape_y.resize(nVerts);
		}
		for (size_t i=0;i<nVerts;i++)
		{
			newLogRec.robotShape_x[i]= robotShape.GetVertex_x(i);
			newLogRec.robotShape_y[i]= robotShape.GetVertex_y(i);
		}

		// Previous values:
		if (prevV.size() != PREVIOUS_VALUES_IN_LOG) prevV.resize(PREVIOUS_VALUES_IN_LOG);
		if (prevW.size() != PREVIOUS_VALUES_IN_LOG) prevW.resize(PREVIOUS_VALUES_IN_LOG);
		if (prevSelPTG.size() != PREVIOUS_VALUES_IN_LOG) prevSelPTG.resize(PREVIOUS_VALUES_IN_LOG);

		// Shift:
		for (size_t i=0;i<PREVIOUS_VALUES_IN_LOG-1;i++)
		{
			prevV[i]=prevV[i+1];
			prevW[i]=prevW[i+1];
			prevSelPTG[i]=prevSelPTG[i+1];
		}
		// Last values:
		prevV[PREVIOUS_VALUES_IN_LOG-1] = cmd_v;
		prevW[PREVIOUS_VALUES_IN_LOG-1] = cmd_w;
		prevSelPTG[PREVIOUS_VALUES_IN_LOG-1] = (float)nSelectedPTG;

		newLogRec.prevV					= prevV;
		newLogRec.prevW					= prevW;
		newLogRec.prevSelPTG			= prevSelPTG;

		// For each PTG:
		if (!skipNormalReactiveNavigation)
		{
			for (size_t indexPTG=0;indexPTG<PTGs.size();indexPTG++)
			{
				newLogRec.infoPerPTG[indexPTG].PTG_desc					= PTGs[indexPTG]->getDescription();
				mrpt::utils::metaprogramming::copy_container_typecasting(TP_Obstacles[indexPTG], newLogRec.infoPerPTG[indexPTG].TP_Obstacles);
				newLogRec.infoPerPTG[indexPTG].TP_Target					= TP_Targets[indexPTG];
				newLogRec.infoPerPTG[indexPTG].timeForTPObsTransformation	= times_TP_transformations[indexPTG];
				newLogRec.infoPerPTG[indexPTG].timeForHolonomicMethod		= times_HoloNav[indexPTG];
				newLogRec.infoPerPTG[indexPTG].HLFR = HLFRs[indexPTG];
				newLogRec.infoPerPTG[indexPTG].desiredDirection = holonomicMovements[indexPTG].direction;
				newLogRec.infoPerPTG[indexPTG].desiredSpeed = holonomicMovements[indexPTG].speed;
				newLogRec.infoPerPTG[indexPTG].evaluation = holonomicMovements[indexPTG].evaluation;
			}
		}
		else
		{
			newLogRec.infoPerPTG.clear();
		}

		// --------------------------------------
		//  Save to log file:
		// --------------------------------------
		if (logFile) (*logFile) << newLogRec;

		// --------------------------------------
		// Set as last log record
		// --------------------------------------
		// Lock
		{
			mrpt::synch::CCriticalSectionLocker lock_log(&m_critZoneLastLog);
			// COPY
			lastLogRecord = newLogRec;
		}
	}
	catch (std::exception &e)
	{
		LOG_AI(ERROR) << e.what();
		LOG_AI(ERROR) << "[YclopsNavigationSystem::performNavigationStep] Exceptions!!" << endl;
	}
	catch (...)
	{
		LOG_AI(ERROR) << "[YclopsNavigationSystem::performNavigationStep] Unexpected exception!!:" << endl;
	}

}

/**
 * @brief C-Paths generation. Collision Check between each grid cell and
 * the contour of the robot
 */
void YclopsNavigationSystem::STEP1_CollisionGridsBuilder()
{
	try
	{
		if (collisionGridsMustBeUpdated)
		{
			collisionGridsMustBeUpdated = false;

			mrpt::reactivenav::build_PTG_collision_grids(
				PTGs,
				robotShape,
				format("ReacNavGrid_%s",robotName.c_str())
				);
		}
	}
	catch (std::exception &e)
	{
		printf_debug("[YclopsNavigationSystem::STEP1_CollisionGridsBuilder] Exception:");
		printf_debug(e.what());
	}
}

/**
 * @brief Sensors adquisition and obstacle points fusion
 */
bool YclopsNavigationSystem::STEP2_Sense(
	mrpt::slam::CSimplePointsMap				&out_obstacles)
{
	try
	{
		LOG(DEBUG4) << "Running senseObstacles" << endl;
		return m_robot.senseObstacles( out_obstacles );
	}
	catch (std::exception &e)
	{
		printf_debug("[YclopsNavigationSystem::STEP2_Sense] Exception:");
		printf_debug((char*)(e.what()));
		return false;
	}
	catch (...)
	{
		printf_debug("[YclopsNavigationSystem::STEP2_Sense] Unexpected exception!\n");
		return false;
	}

}

/**
 * @brief Space transformer of the obstacle space according to a determined PT
 */
void YclopsNavigationSystem::STEP3_SpaceTransformer(
    mrpt::poses::CPointsMap					&in_obstacles,
    CParameterizedTrajectoryGenerator	*in_PTG,
    vector_double						&out_TPObstacles
)
{
	try
	{
		const size_t Ki = in_PTG->getAlfaValuesCount();

		// See if there is currently reserved space in the TP-Obstacles:
		if ( out_TPObstacles.size() != ((unsigned)(Ki)) )
			out_TPObstacles.resize( Ki );

		// Choose "k"s and "Distances" with those that connect each obstacle point
		// of the grid of the PT data.
		const size_t nObs = in_obstacles.getPointsCount();

		for (size_t k=0;k<Ki;k++)
		{
			// Start the max distance until abs(phi)=pi.
			out_TPObstacles[k] = in_PTG->refDistance;

			// If it stops spinning 180 degrees, end there.
			float phi = in_PTG->GetCPathPoint_phi(k,in_PTG->getPointsCountInCPath_k(k)-1);

			if (fabs(phi) >= M_PI* 0.95 )
				out_TPObstacles[k]= in_PTG->GetCPathPoint_d(k,in_PTG->getPointsCountInCPath_k(k)-1);
		}

		for (size_t obs=0;obs<nObs;obs++)
		{
			float ox,oy;
			in_obstacles.getPoint(obs, ox,oy);

			const CParameterizedTrajectoryGenerator::TCollisionCell & cell = in_PTG->m_collisionGrid.getTPObstacle(ox,oy);

			// Keep the minimum distance:
			for (CParameterizedTrajectoryGenerator::TCollisionCell::const_iterator i=cell.begin();i!=cell.end();i++)
				if ( i->second < out_TPObstacles[ i->first ] )
					out_TPObstacles[i->first] = i->second;
		}

		// Distances in TP-Space are normalized to [0,1]:
		for (size_t i=0;i<Ki;i++)
			out_TPObstacles[i] /= in_PTG->refDistance;
	}
	catch (std::exception &e)
	{
		printf_debug("[YclopsNavigationSystem::STEP3_SpaceTransformer] Exception:");
		printf_debug((char*)(e.what()));
	}
	catch (...)
	{
		std::cout << "\n[YclopsNavigationSystem::STEP3_SpaceTransformer] Unexpected exception!:\n";
		std::cout << format("*in_PTG = %p\n", (void*)in_PTG );
		if (in_PTG)
			std::cout << format("PTG = %s\n",in_PTG->getDescription().c_str());
		std::cout << std::endl;
	}


}

/*************************************************************************

                             STEP4_HolonomicMethod

*************************************************************************/
void YclopsNavigationSystem::STEP4_HolonomicMethod(
    vector_double						&in_Obstacles,
    mrpt::poses::CPoint2D						&in_Target,
    float								in_maxRobotSpeed,
    THolonomicMovement					&out_selectedMovement,
    CHolonomicLogFileRecordPtr			&in_HLFR )
{
	try
	{
		holonomicMethod->navigate(	in_Target,
		                           in_Obstacles,
		                           in_maxRobotSpeed,
		                           out_selectedMovement.direction,
		                           out_selectedMovement.speed,
		                           in_HLFR );
	}
	catch (std::exception &e)
	{
		printf_debug("[YclopsNavigationSystem::STEP4_HolonomicMethod] Exception:");
		printf_debug((char*)(e.what()));
	}
	catch (...)
	{
		printf_debug("[YclopsNavigationSystem::STEP4_HolonomicMethod] Unexpected exception!\n");
	}
}

/*************************************************************************

						 	STEP5_Evaluator

*************************************************************************/
void YclopsNavigationSystem::STEP5_Evaluator(
    THolonomicMovement			&in_holonomicMovement,
    vector_double				&in_TPObstacles,
    mrpt::poses::CPoint2D		&WS_Target,
    mrpt::poses::CPoint2D		&TP_Target,
    bool						wasSelectedInLast,
    CLogFileRecord::TInfoPerPTG	&log)
{
	int		DEBUG_POINT = 0;

	try
	{
		float	a;
		float	factor1, factor2,factor3,factor4,factor5,factor6;

		if (TP_Target.x()!=0 || TP_Target.y()!=0)
			//a = atan2( TP_Target.y(), TP_Target.x());
			a = atan2( TP_Target.x(), TP_Target.y());
		else	a = 0;

		DEBUG_POINT = 1;

		const int		TargetSector = in_holonomicMovement.PTG->alfa2index( a );
		const double	TargetDist = TP_Target.norm();
		const int		kDirection = in_holonomicMovement.PTG->alfa2index( in_holonomicMovement.direction );
		const double	refDist	   = in_holonomicMovement.PTG->refDistance;

		DEBUG_POINT = 2;

		// Las coordenadas en C-Space representativas de la trajectoria seleccionada:
		float	x,y,phi,t,d;
		d = min( in_TPObstacles[ kDirection ], 0.90f*TargetDist);
		in_holonomicMovement.PTG->getCPointWhen_d_Is( d, kDirection,x,y,phi,t );

		DEBUG_POINT = 3;

		// Factor 1: Distancia hasta donde llego por esta GPT:
		// -----------------------------------------------------
		factor1 = in_TPObstacles[kDirection];

		//	if (kDirection == TargetSector )	// Si es TARGET_DIRECTLY:
		//			factor1 = 1 - TargetDist;				// Llego todo lo lejos q quiero ir!! :-)
		//	else	factor1 = in_TPObstacles[kDirection];

		DEBUG_POINT = 4;

		// Factor 2: Distancia en sectores:
		// -------------------------------------------
		float   dif = fabs(((float)( TargetSector - kDirection )));
		float	nSectors = (float)in_TPObstacles.size();
		if ( dif > (0.5f*nSectors)) dif = nSectors - dif;
		factor2= exp(-square( dif / (in_TPObstacles.size()/3.0f))) ;

		DEBUG_POINT = 5;

		// Factor 3: Angulo que hará el robot con target en (x,y):
		// -----------------------------------------------------
		float   t_ang = atan2( WS_Target.y() - y, WS_Target.x() - x );
		t_ang -= phi;

		while (t_ang> M_PI)  t_ang-=(float)M_2PI;
		while (t_ang<-M_PI)  t_ang+=(float)M_2PI;

		factor3 = exp(-square( t_ang / (float)(0.5f*M_PI)) );

		DEBUG_POINT = 5;

		// Factor4:		DECREMENTO de la distancia euclidea entre (x,y) y target:
		//  Se valora negativamente el alejarse del target
		// -----------------------------------------------------
		float dist_eucl_final = WS_Target.distance2DTo(x,y);
		float dist_eucl_now   = WS_Target.norm();

		//    float dist_eucl_final = sqrt( square( d*cos(in_holonomicMovement.direction)-TP_Target.x ) + square( d*sin(in_holonomicMovement.direction) -TP_Target.y ) );
		//	float dist_eucl_now   = sqrt( square( TP_Target.x ) + square( TP_Target.y ) );

		factor4 = min(2.0*refDist,max(0.0,((dist_eucl_now - dist_eucl_final)+refDist)))/(2*refDist);

		// ---------
		//	float decrementDistanc = dist_eucl_now - dist_eucl_final;
		//	if (dist_eucl_now>0)
		//			factor4 = min(1.0,min(refDist*2,max(0,decrementDistanc + refDist)) / dist_eucl_now);
		//	else	factor4 = 0;
		// ---------
		//	factor4 = min(2*refDist2,max(0,decrementDistanc + refDist2)) / (2*refDist2);
		//  factor4=  (refDist2 - min( refDist2, dist_eucl ) ) / refDist2;

		DEBUG_POINT = 6;

		// Factor5: Histeresis:
		// -----------------------------------------------------
		float	want_v,want_w;
		in_holonomicMovement.PTG->directionToMotionCommand( kDirection, want_v,want_w);

		float	likely_v = exp( -fabs(want_v-last_cmd_v)/0.10f );
		float	likely_w = exp( -fabs(want_w-last_cmd_w)/0.40f );

		factor5 = min( likely_v,likely_w );
		//factor5 = wasSelectedInLast ? 1:0;

		DEBUG_POINT = 7;

		// Factor6: Security distance !!
		// -----------------------------------------------------
		factor6 = 0;

		DEBUG_POINT = 8;

		// --------------------
		//  SAVE LOG
		// --------------------
		log.evalFactors.resize(6);
		log.evalFactors[0] = factor1;
		log.evalFactors[1] = factor2;
		log.evalFactors[2] = factor3;
		log.evalFactors[3] = factor4;
		log.evalFactors[4] = factor5;
		log.evalFactors[5] = factor6;

		DEBUG_POINT = 9;

		if (in_holonomicMovement.speed==0)
		{
			// If no movement has been found -> the worst evaluation:
			in_holonomicMovement.evaluation = 0;
		}
		else
		{
			// Sum: Dos casos:
			if (dif<2	&&										// Heading the target
			        in_TPObstacles[kDirection]*0.95f>TargetDist 	// and free space towards the target
			   )
			{
				// Caso de camino directo al target:
//				in_holonomicMovement.evaluation = 1.0f + (1 - TargetDist) + factor5 * weight5 + factor6*weight6;
				in_holonomicMovement.evaluation = 1.0f + (1 - t/15.0f) + factor5 * weights[4] + factor6*weights[5];
			}
			else
			{
				// Caso general:
				in_holonomicMovement.evaluation = (
				                                      factor1 * weights[0] +
				                                      factor2 * weights[1] +
				                                      factor3 * weights[2] +
				                                      factor4 * weights[3] +
				                                      factor5 * weights[4] +
				                                      factor6 * weights[5]
				                                  ) / ( math::sum(weights));
			}
		}

		DEBUG_POINT = 10;
	}
	catch (std::exception &e)
	{
		THROW_STACKED_EXCEPTION(e);
	}
	catch (...)
	{
		std::cout << "[YclopsNavigationSystem::STEP5_Evaluator] Unexpected exception!:\n";
		std::cout << format("DEBUG_POINT = %u\n",DEBUG_POINT );
	}

}

/*************************************************************************

							STEP6_Selector

*************************************************************************/
void YclopsNavigationSystem::STEP6_Selector(
    std::vector<THolonomicMovement>		&in_holonomicMovements,
    THolonomicMovement					&out_selected,
    int									&out_nSelectedPTG)
{
	// Si no encontramos nada mejor, es que no hay movimiento posible:
	out_selected.direction= 0;
	out_selected.speed = 0;
	out_selected.PTG = NULL;
	out_selected.evaluation= 0;		// Anotacion pa mi: Don't modify this 0 and the ">" comparison
	out_nSelectedPTG = 0;

	// Coger la trayectoria con mejor evaluacion, mientras no produzca
	//  colision:
	for (unsigned int i=0;i<in_holonomicMovements.size();i++)
	{
		float ev = in_holonomicMovements[i].evaluation;
		if ( ev > out_selected.evaluation )
		{
			out_selected = in_holonomicMovements[i];
			out_selected.evaluation = ev;
			out_nSelectedPTG = i;
		}
	}
}

/*************************************************************************

						STEP7_NonHolonomicMovement

*************************************************************************/
void YclopsNavigationSystem::STEP7_NonHolonomicMovement(
    THolonomicMovement					&in_movement,
    float								&out_v,
    float								&out_w)
{
	try
	{
		if (in_movement.speed==0)
		{
			// The robot will stop:
			out_v = out_w = 0;
		}
		else
		{
			// Take the normalized movement command:
			in_movement.PTG->directionToMotionCommand(
			    in_movement.PTG->alfa2index( in_movement.direction ),
			    out_v,
			    out_w );

			// Scale holonomic speeds to real-world one:
			const double reduction = min(1.0, in_movement.speed / in_movement.PTG->getMax_V_inTPSpace());

			// To scale:
			out_v*=reduction;
			out_w*=reduction;

			// Assure maximum speeds:
			if (fabs(out_v)>robotMax_V_mps)
			{
				// Scale:
				float F = fabs(robotMax_V_mps / out_v);
				out_v *= F;
				out_w *= F;
			}

			if (fabs(out_w)>DEG2RAD(robotMax_W_degps))
			{
				// Scale:
				float F = fabs((float)DEG2RAD(robotMax_W_degps) / out_w);
				out_v *= F;
				out_w *= F;
			}
		}
	}
	catch (std::exception &e)
	{
		printf_debug("[YclopsNavigationSystem::STEP7_NonHolonomicMovement] Exception:");
		printf_debug((char*)(e.what()));
	}
	catch (...)
	{
		printf_debug("[YclopsNavigationSystem::STEP7_NonHolonomicMovement] Unexpected exception!\n");
	}
}


/*************************************************************************
	 		Destructor
*************************************************************************/
YclopsNavigationSystem::~YclopsNavigationSystem()
{
	CerrandoHilo = true;

	// Esperar a que termine la ejecucion actual, por si esta en otro hilo:
	m_critZoneNavigating.enter();
	m_critZoneNavigating.leave();

	// Por si acaso...
	m_robot.stop();

	if (logFile)
	{
		delete logFile;
		logFile = NULL;
	}


	// Free PTGs:
	for (size_t i=0;i<PTGs.size();i++)	delete PTGs[i];
	PTGs.clear();

	// Free holonomic method:
	if (holonomicMethod)
	{
		delete holonomicMethod;
		holonomicMethod = NULL;
	}


}




/*************************************************************************
			 Evaluar navegacion:
*************************************************************************/
float  YclopsNavigationSystem::evaluate( TNavigationParams *params )
{
	return 0.5f;
}

/*************************************************************************
			 Iniciar navegacion:
*************************************************************************/
void  YclopsNavigationSystem::navigate(YclopsNavigationSystem::TNavigationParams *params )
{
	if(points.size() == 0)
		{
			mrpt::poses::CPoint2D* point = new CPoint2D();
			point->x(params->target.x);
			point->y(params->target.y);
			points.push_back(*point);
		}
	navigationEndEventSent = false;

	// Copiar datos:
	m_navigationParams = *params;

	// Reset behavior:
	navigatorBehavior = beNormalNavigation;

	// Si se piden coordenadas relativas, transformar a absolutas:
	if ( m_navigationParams.targetIsRelative )
	{
		std::cout << format("TARGET COORDS. ARE RELATIVE!! -> Translating them...\n");
		// Obtener posicion actual:
		poses::CPose2D		currentPose;
		float				velLineal_actual,velAngular_actual;

		if ( !m_robot.getCurrentPoseAndSpeeds(currentPose, velLineal_actual,velAngular_actual) )
		{
			Error_ParadaDeEmergencia("\n[YclopsNavigationSystem] Error querying current robot pose to resolve relative coordinates\n");
			return;
		}

		poses::CPoint2D	absTarget;
		absTarget = currentPose + m_navigationParams.target;
		m_navigationParams.target = absTarget;
		m_navigationParams.targetIsRelative=false;       // Ya no son relativas
	}

	// new state:
	m_navigationState = NAVIGATING;

	// Reset the bad navigation alarm:
	badNavAlarm_minDistTarget = 1e10f;
	badNavAlarm_lastMinDistTime = system::getCurrentTime();
}

/*************************************************************************
        Cambiar params. de la navegacion actual
*************************************************************************/
void  YclopsNavigationSystem::setParams( YclopsNavigationSystem::TNavigationParams  *params )
{

}

/*************************************************************************
                Para la silla y muestra un mensaje de error.
*************************************************************************/
void YclopsNavigationSystem::Error_ParadaDeEmergencia( const char *msg )
{
	// Mostrar mensaje y parar navegacion si estamos moviendonos:
	printf_debug( msg );
	printf_debug( "\n");

	m_robot.stop();

	m_navigationState = NAV_ERROR;
	return;
}


/* -------------------------------------------------------------
   ------------------------------------------------------------- */
void  YclopsNavigationSystem::CDynamicWindow::findMinMaxCurvatures(float &minCurv, float &maxCurv)
{
	if (fabs(v_min)<0.005f) v_min=0.005f*sign(v_min);
	if (fabs(v_max)<0.005f) v_max=0.005f*sign(v_max);

	// Compute the curvature for the 4 corners:
	c1 = w_max / v_min;
	c2 = w_min / v_min;
	c3 = w_max / v_max;
	c4 = w_min / v_max;

	minCurv = min( min(c1,c2),min(c3,c4) );
	maxCurv = max( max(c1,c2),max(c3,c4) );
}


/* -------------------------------------------------------------
   ------------------------------------------------------------- */
void  YclopsNavigationSystem::CDynamicWindow::findBestApproximation(float desV,float desW, float &outV,float &outW)
{
	// Try to find a "cut", if not, find just the closest corner.
	if (findClosestCut(desV,desW,outV,outW))
		return;

	float closestX,closestY;

	float	d[4];
	d[0] = math::minimumDistanceFromPointToSegment(v_min,w_max, 0,0,desV,desW,closestX,closestY);
	d[1] = math::minimumDistanceFromPointToSegment(v_min,w_min, 0,0,desV,desW,closestX,closestY);
	d[2] = math::minimumDistanceFromPointToSegment(v_max,w_max, 0,0,desV,desW,closestX,closestY);
	d[3] = math::minimumDistanceFromPointToSegment(v_max,w_min, 0,0,desV,desW,closestX,closestY);

	float	d_min=1e6;
	int		idx_min=-1,i;
	for (i=0;i<4;i++)
		if (d[i]<d_min)
		{
			d_min = d[i];
			idx_min = i;
		}

	switch (idx_min)
	{
	case 0:
		outV = v_min;
		outW = w_max;
		break;
	case 1:
		outV = v_min;
		outW = w_min;
		break;
	case 2:
		outV = v_max;
		outW = w_max;
		break;
	case 3:
		outV = v_max;
		outW = w_min;
		break;
	};
}


/* -------------------------------------------------------------
   ------------------------------------------------------------- */
bool  YclopsNavigationSystem::CDynamicWindow::findClosestCut( float cmd_v, float cmd_w,	// IN
        float &out_v,float &out_w)	// OUT
{
	if (fabs(cmd_v)<0.005f) cmd_v = 0.005f * sign(cmd_v);
	//float desiredCurv	= cmd_w / cmd_v;

	// Find the 1..4 cuts:
	vector_float	vs,ws;
	vs.reserve(4);
	ws.reserve(4);
	float			v,w;

	if ( math::SegmentsIntersection( v_min,w_min,v_min,w_max, 0,0,cmd_v,cmd_w , v,w) )
	{
		vs.push_back(v);
		ws.push_back(w);
	}
	if ( math::SegmentsIntersection( v_min,w_max,v_max,w_max, 0,0,cmd_v,cmd_w , v,w) )
	{
		vs.push_back(v);
		ws.push_back(w);
	}
	if ( math::SegmentsIntersection( v_max,w_max,v_max,w_min, 0,0,cmd_v,cmd_w , v,w) )
	{
		vs.push_back(v);
		ws.push_back(w);
	}
	if (math::SegmentsIntersection( v_min,w_min,v_max,w_min, 0,0,cmd_v,cmd_w , v,w) )
	{
		vs.push_back(v);
		ws.push_back(w);
	}

	// Any cut??
	if (!vs.size()) return false;

	// Find closest cut:
	float	d_min=1e6;
	int		idx_min=-1;
	for (unsigned int i=0;i<vs.size();i++)
	{
		float d = math::distanceBetweenPoints( cmd_v,cmd_w, vs[i],ws[i] );
		if (d<d_min)
		{
			d_min = d;
			idx_min = i;
		}
	}

	ASSERT_(idx_min>=0 && idx_min<(int)vs.size())

	// Returns the closes cut point:
	out_v = vs[idx_min];
	out_w = ws[idx_min];

	return true;
}

void YclopsNavigationSystem::gotoNextPoint()
{
	mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t::iterator iter = points.begin();

	//setting up the next waypoint
	mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams*   navParams = new mrpt::reactivenav::CAbstractReactiveNavigationSystem::TNavigationParams();
	navParams->target.x = iter->x();
	navParams->target.y = iter->y();
	//distance from the waypoint in meters before the robot considers itself at the point
	navParams->targetAllowedDistance = targetAllowedDistance;
	//navParams.targetIsRelative = !challange; is being ignored
	//gives the reactive nav the next goal
	navigate(navParams );
}
void YclopsNavigationSystem::setFileName(std::string & fileName, bool inMeters)
{
	LOG_AI(DEBUG3) << "Using " << fileName << " for points file."<< endl;
	this->fileName = fileName;
	this->inMeters = inMeters;
}
void YclopsNavigationSystem::stop(){
	points = mrpt::aligned_containers<mrpt::poses::CPoint2D>::vector_t();
	m_robot.stop();
	m_navigationState = IDLE;
	return;
}
void YclopsNavigationSystem::setup()
{

	//interface that will be used by the reactive nav to sense the environment and make the robot move
		//initial position
	mrpt::poses::CPose2D pose = CPoint2D();
	float v = 0;
	float w = 0;
	m_robot.getCurrentPoseAndSpeeds(pose, v,w);
	double lat = pose.x();
	double lon = pose.y();
	AbstractNavigationInterface *nav = NULL;

	if(NavChallenge)
	{
		// solves the tsp problem, in autonomous challenge we need to add some code here to make it work.
		nav = new TSPNavigation(lat,lon);
	}
	else
	{
		 nav =  new SequentialNavigation(lat, lon);
	}

	nav->loadPoints(fileName, !inMeters);

	//waypoints in the order we want to visit them
	points =  nav->solve(false);

	gotoNextPoint();

}

void YclopsNavigationSystem::setChallenge(bool c)
{
	NavChallenge = c;
}
CPoint2D YclopsNavigationSystem::makeAuxTarget(CPose2D& curPose)
{
	const double angle_off = M_PI/4; // angle away to go
	const double distance = 1.5;// distance to go down that path
	CPoint2D target;
	double angle = curPose.phi();
	target.x(curPose.x() + sin(angle + angle_off)*distance);
	target.y(curPose.y() - cos(angle + angle_off)*distance);
	return target;
}
void YclopsNavigationSystem::getTarget(mrpt::poses::TPoint2D *target)
{
	*target = m_navigationParams.target;
}

