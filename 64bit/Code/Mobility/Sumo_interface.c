/************************************************************************************
* Copyright (C) 2016                                                               *
* TETCOS, Bangalore. India                                                         *
*                                                                                  *
* Tetcos owns the intellectual property rights in the Product and its content.     *
* The copying, redistribution, reselling or publication of any or all of the       *
* Product or its content without express prior written consent of Tetcos is        *
* prohibited. Ownership and / or any other right relating to the software and all  *
* intellectual property rights therein shall remain at all times with Tetcos.      *
*                                                                                  *
* Author:    Kshitij Singh
* Date:		 7 July 2016	
*                                                                                  *
* ---------------------------------------------------------------------------------*/

#include "main.h"
#include "Mobility.h"
#include "Animation.h"

char* sumoname;
double step_size;

char gui;    //By Default, Set GUI to 0
HANDLE hPipe;  //Create a Handle to pipe




double *corr(char* id);
void pipes_init();

//This function is mainly used for calling python and passing sumo configuration files to it
void init_sumo()
{
	static int nosumo=1;		// Declared as static, since we want it to be declared and changed only once							
	if(nosumo)					// This will run only at the 1st time
	{
		char command_to_python[BUFSIZ];		// to be passed to vanet.exe
		sprintf(command_to_python,"start vanet.exe -main \"%s\" ", sumoname);		// store it in one variable		
		fprintf(stderr,"Executing command for opening Sumo - %s",command_to_python);	// Error statement in standard error
		system(command_to_python);				// Call vanet.exe and subsequently python
		fprintf(stderr,"....done.\n");			
		printf("Init sumo pipe\n");
		pipes_init();				// Initiate pipes for connection
		nosumo=0;
	}
}

//This functon will call Pipes to get coordinates from python. Refer to mobility_run in mobility.c to understand the work of other variables
void sumo_run()
{
	MOBILITY_VAR* pstruMobilityVar=(MOBILITY_VAR*)NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId-1]->pstruDeviceMobility->pstruMobVar;	//Define Mobility variable
	double dPresentTime = pstruMobilityVar->dLastTime;

	memcpy(NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId-1]->pstruDeviceMobility->pstruCurrentPosition,
		NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId-1]->pstruDeviceMobility->pstruNextPosition,
		sizeof* NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId-1]->pstruDeviceMobility->pstruCurrentPosition);

	if(pstruMobilityVar->dLastTime+pstruMobilityVar->dPauseTime*1000000<pstruEventDetails->dEventTime+1000000)	//Everytime Mobility being called
	{
		double* coordinates;		// Pointer for array of X and Y coordinates
		coordinates = corr(DEVICE_NAME(pstruEventDetails->nDeviceId));	//Get coordinates from python
		if (coordinates!=NULL)
		{
			NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId-1]->pstruDeviceMobility->pstruNextPosition->X = coordinates[0];	// Update the coordinates in Network stack
			NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId-1]->pstruDeviceMobility->pstruNextPosition->Y = coordinates[1];
			free(coordinates);			// Free memory of pointer
		}

		//store the last time
		pstruMobilityVar->dLastTime = pstruEventDetails->dEventTime+1000000*step_size;			// Update Last time since we want to match timings with SUMO
	}
	//update the device position
	memcpy(NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId-1]->pstruDevicePosition,		
		NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId-1]->pstruDeviceMobility->pstruCurrentPosition,
		sizeof* NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId-1]->pstruDevicePosition);

	if(NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId-1]->pstruDevicePosition->ismap)	//Convert to Longitudes and Latitudes for Earth coordinates 
	{
		convert_3D_to_lat_lon(NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId-1]->pstruDevicePosition);
		//Animate the nodes for initial positions
		add_mobility_animation(pstruEventDetails->nDeviceId,
							   pstruEventDetails->dEventTime,
							   NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDevicePosition->lat,
							   NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDevicePosition->lon,
							   0);
	}
	else
	{
		add_mobility_animation(pstruEventDetails->nDeviceId,
							   pstruEventDetails->dEventTime,
							   (int)NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDevicePosition->X,
							   (int)NETWORK->ppstruDeviceList[pstruEventDetails->nDeviceId - 1]->pstruDevicePosition->Y,
							   0);
	}
	//Add event for next point 
	pstruEventDetails->dEventTime+=1000000*step_size;
	fnpAddEvent(pstruEventDetails);
	pstruEventDetails->dEventTime-=1000000*step_size;
}


// This function Initiates a Pipes connection and sends GUI =1 or 0 based on Animation Status

void pipes_init()
{
	BOOL   fSuccess;								//If reading/writing successfil or not
	DWORD  cbToWrite, cbWritten;			// For Pipes
	DWORD dwMode;
	char message_to_be_sent[2];	
	
	LPCSTR lpszPipename = "\\\\.\\pipe\\netsim_sumo_pipe";		//Pipename
	gui='0';
	fSuccess=FALSE;
	//getch();
	fprintf(stderr,"Creating Sumo pipe\n");

	while (1) 
	{ 
		hPipe = CreateFileA
			( 
			lpszPipename,   // pipe name 
			GENERIC_READ |  // read and write access 
			GENERIC_WRITE, 
			0,              // no sharing 
			NULL,           // default security attributes
			OPEN_EXISTING,  // opens existing pipe 
			0,              // default attributes 
			NULL			// no template file
			);						   

		if (hPipe != INVALID_HANDLE_VALUE) 
			break; 

	}

	fprintf(stderr,"Connecting Sumo and NetSim in real time\n");
	dwMode = PIPE_READMODE_MESSAGE; // The pipe connected; change to message-read mode. 
	fSuccess = SetNamedPipeHandleState
		( 
		hPipe,    // pipe handle 
		&dwMode,  // new pipe mode 
		NULL,     // don't set maximum bytes 
		NULL	// don't set maximum time 
		);			   

	if ( ! fSuccess) 
		fnSystemError("Error in connection netsim sump pipe.\n");
	else
		fprintf(stderr,"Connection done\n");

	if(anim_get_anim_flag() == ANIMFLAG_ONLINE)
		gui = '1';
	else
		gui = '0';
	

	message_to_be_sent[0]=gui;	//Send GUI enable or disable
	message_to_be_sent[1]=0;

	cbToWrite=(DWORD)strlen(message_to_be_sent);

	fSuccess = WriteFile
		( 
		hPipe,                  // pipe handle 
		message_to_be_sent,             // message 
		cbToWrite,              // message length 
		&cbWritten,             // bytes written 
		NULL					// not overlapped 
		);                  
}
//////////////////////////////////////////////////////



double *corr(char* id)
{
	BOOL   fSuccess;								//If reading/writing successfil or not
	DWORD  cbRead, cbToWrite, cbWritten;			// For Pipes
	CHAR  chBuf[BUFSIZ]; //For reading messages from pipes

	double xcor1,ycor1;	 // x and y coordinates to be received from sumo
	double* coordinates;

	do 
	{ 
		// Read Garbage from the pipe. 

		fSuccess = ReadFile
			( 
			hPipe,    // pipe handle 
			chBuf,    // buffer to receive reply 
			BUFSIZ*sizeof(CHAR),  // size of buffer 
			&cbRead,  // number of bytes read 
			NULL		// not overlapped 
			);

		if ( ! fSuccess && GetLastError() != ERROR_MORE_DATA )
			break; 
		chBuf[cbRead]=0;
	} while (!fSuccess);  // repeat loop if ERROR_MORE_DATA 


	//Send Vehicle Name to Python 

	cbToWrite=(DWORD)strlen(id)+1;
	fSuccess = WriteFile
		( 
		hPipe,                  // pipe handle 
		id,             // message 
		cbToWrite,              // message length 
		&cbWritten,             // bytes written 
		NULL					// not overlapped 
		);                  


	// Read acknowledgment for vehicle ids
	do 
	{ 

		fSuccess = ReadFile
			( 
			hPipe,    // pipe handle 
			chBuf,    // buffer to receive reply 
			BUFSIZ*sizeof(CHAR),  // size of buffer 
			&cbRead,  // number of bytes read 
			NULL    // not overlapped 
			);

		if ( ! fSuccess && GetLastError() != ERROR_MORE_DATA )
			break; 
		chBuf[cbRead]=0;

	} while (!fSuccess);  // repeat loop if ERROR_MORE_DATA 



	if(chBuf[0]=='c')  // If the vehicle is present ('c' for confirmation)
	{
		xcor1=0;ycor1=0;
		do 
		{ 
			// Read X coordinate 
			fSuccess = ReadFile
				( 
				hPipe,    // pipe handle 
				chBuf,    // buffer to receive reply 
				BUFSIZ*sizeof(CHAR),  // size of buffer 
				&cbRead,  // number of bytes read 
				NULL	// not overlapped 
				);    

			if ( ! fSuccess && GetLastError() != ERROR_MORE_DATA )
				break; 

		} while (!fSuccess);  // repeat loop if ERROR_MORE_DATA 

		xcor1 = atof(chBuf);
		if(xcor1<0)
			xcor1=0;

		do 
		{ 
			// Read Y coordinate

			fSuccess = ReadFile
				( 
				hPipe,    // pipe handle 
				chBuf,    // buffer to receive reply 
				BUFSIZ*sizeof(CHAR),  // size of buffer 
				&cbRead,  // number of bytes read 
				NULL		// not overlapped 	
				);    

			if ( ! fSuccess && GetLastError() != ERROR_MORE_DATA )
				break; 
			chBuf[cbRead]=0;

		} while (!fSuccess);  // repeat loop if ERROR_MORE_DATA 

		ycor1 = atof(chBuf);
		if(ycor1<0)
			ycor1=0;

		coordinates = (double*)malloc(2*sizeof* coordinates);
		coordinates[0]=xcor1;
		coordinates[1]=ycor1;
		return (coordinates);				//Return X Y Coordinates  
	}

	else
	{
		return(NULL);		// If vehicle not found, return NULL
	}

}
