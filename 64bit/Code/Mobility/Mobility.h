/************************************************************************************
* Copyright (C) 2013                                                               *
* TETCOS, Bangalore. India                                                         *
*                                                                                  *
* Tetcos owns the intellectual property rights in the Product and its content.     *
* The copying, redistribution, reselling or publication of any or all of the       *
* Product or its content without express prior written consent of Tetcos is        *
* prohibited. Ownership and / or any other right relating to the software and all *
* intellectual property rights therein shall remain at all times with Tetcos.      *
*                                                                                  *
* Author:    Shashi Kant Suman                                                       *
*                                                                                  *
* ---------------------------------------------------------------------------------*/
#pragma comment(lib,"libZigbee")
#ifndef _NETSIM_MOBILITY_H_
#define _NETSIM_MOBILITY_H_
#ifdef  __cplusplus
extern "C" {
#endif

#define Mobility_MODEL_DEFAULT					_strdup("NO_MOBILITY")
#define Mobility_GROUP_ID_DEFAULT				1
#define Mobility_FILE_NAME_DEFAULT				_strdup("")
#define Mobility_STEP_SIZE_DEFAULT				1
#define Mobility_CALCULATION_INTERVAL_DEFAULT	1

	int fn_NetSim_localisation();
	typedef enum enum_MobilityModel MOBILITY_MODEL;

	typedef struct stru_NetSim_Mobility NetSim_MOBILITY;
	typedef struct stru_NetSim_MobilityVar MOBILITY_VAR;
	/** Structure to store the mobility variables */

	typedef enum
	{
		MOVE_GROUP=PROTOCOL_MOBILITY*100+1,
		NODE_JOIN,
		NODE_LEAVE,
	}MOBILTY_SUBEVNET;
	/* Enumeration for mobility model*/
	enum enum_MobilityModel
	{
		MobilityModel_NOMOBILITY,
		MobilityModel_RANDOMWAYPOINT,
		MobilityModel_RANDOMWALK,
		MobilityModel_FILEBASEDMOBILITY,
		MobilityModel_GROUP,
		MobilityModel_SUMO,
	};

	double dSimulationArea_X; //Store the simulation area_x
	double dSimulationArea_Y;//Store the simulationar area_y

	/* Store the mobility variable for each device*/
	struct stru_NetSim_Mobility
	{
		double dAvgSpeed;			//Average speed of device
		MOBILITY_MODEL nMobilityType; //Mobility model
		void* pstruMobVar;			/*Pointer pointing to addition mobility model variables
								 *That may be used by developer. This can be accessed by
								 *DEVICE[]->Mobility->MobilityVar
								 */
		struct stru_NetSim_Coordinates* pstruCurrentPosition; //Current position for device
		struct stru_NetSim_Coordinates* pstruNextPosition;	  //Next position of device
	};

	struct stru_NetSim_MobilityVar
	{		
		double dPauseTime; ///< To store the pause time.		
		double dVelocity; ///< To store the velocity.		
		unsigned long ulSeed1; ///< Used to generate random point.		
		unsigned long ulSeed2; ///< Used to generate random point		
		double dLastTime; ///< Represent the devices last move time.
		NETSIM_ID nGroupId; ///< To store group id of device.
		double dCalculationInterval;
		//Sumo
		char* sumoFileName;
		double step_size;
	};

	void add_mobility_animation(NETSIM_ID d,
								double t,
								double x,
								double y,
								double z);
	int FileBasedMobilityPointersFree();
	int fn_NMo_RandomPoint(double* X, double* Y,double velocity,double interval,unsigned long *pulSeed1, unsigned long *pulSeed2);
	int fn_NetSim_MoveGroup();
	int fn_NetSim_Mobility_Group_init();
	int add_to_group(NETSIM_ID group_id, NETSIM_ID dev_id);
	
	// Sumo Interface
	double *corr(char* id);
	void pipes_init();
	void init_sumo();
	void sumo_run();

	//File based mobility
	int FileBasedMobilityReadingFile();
	void process_filebased_mobility_event();
	int FileBasedMobilityPointersFree();

#ifdef  __cplusplus
}
#endif
#endif

