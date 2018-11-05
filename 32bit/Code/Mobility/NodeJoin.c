/************************************************************************************
* Copyright (C) 2015                                                               *
* TETCOS, Bangalore. India                                                         *
*                                                                                  *
* Tetcos owns the intellectual property rights in the Product and its content.     *
* The copying, redistribution, reselling or publication of any or all of the       *
* Product or its content without express prior written consent of Tetcos is        *
* prohibited. Ownership and / or any other right relating to the software and all  *
* intellectual property rights therein shall remain at all times with Tetcos.      *
*                                                                                  *
* Author:    Shashi Kant Suman                                                     *
*                                                                                  *
* ---------------------------------------------------------------------------------*/
#include "main.h"
#include "Mobility.h"
#include "Animation.h"
static unsigned int nCallBackCount=0;

#define MOBILITY_JOIN_AT_DEFAULT	_strdup("0,")
#define MOBILITY_LEAVE_AT_DEFAULT	_strdup("")

int fn_NetSim_Mobility_configure_NodeJoinProfile(void* xmlNode,NETSIM_ID nDeviceId)
{
	char* szValJoin;
	char* szValLeave;
	char* t;
	char* s;
	PNODE_JOIN_PROFILE* joinprofile=&NETWORK->ppstruDeviceList[nDeviceId-1]->node_join_profile;
	getXmlVar(&szValJoin,JOIN_AT,xmlNode,1,_STRING,MOBILITY);
	getXmlVar(&szValLeave,LEAVE_AT,xmlNode,0,_STRING,MOBILITY);
	s=szValJoin;
	t=strtok(szValJoin,",");
	while(t && *t)
	{
		PNODE_JOIN_PROFILE njp = JOIN_PROFILE_ALLOC();
		njp->action = JOIN;
		njp->time=atof(t)*SECOND;
		njp->NodeId=nDeviceId;
		JOIN_PROFILE_ADD(joinprofile,njp);
		if(atoi(t) == 0)
			NETWORK->ppstruDeviceList[nDeviceId-1]->node_status=CONNECTED;
		t=strtok(NULL,",");
	}
	free(s);
	s=szValLeave;
	t=strtok(szValLeave,",");
	while(t && *t)
	{
		PNODE_JOIN_PROFILE njp = JOIN_PROFILE_ALLOC();
		njp->action = LEAVE;
		njp->time=atof(t)*SECOND;
		njp->NodeId=nDeviceId;
		JOIN_PROFILE_ADD(joinprofile,njp);
		if(atoi(t) == 0)
			NETWORK->ppstruDeviceList[nDeviceId-1]->node_status=NOT_CONNECTED;
		t=strtok(NULL,",");
	}
	free(s);
	return 0;
}

int fn_NetSim_Mobility_NodeJoinInit(NETSIM_ID nDevIndex)
{
	PNODE_JOIN_PROFILE join=NETWORK->ppstruDeviceList[nDevIndex]->node_join_profile;
	NetSim_EVENTDETAILS pevent;
	memset(&pevent,0,sizeof pevent);
	pevent.nDeviceId=nDevIndex+1;
	pevent.nDeviceType=DEVICE_TYPE(nDevIndex+1);
	pevent.nEventType=TIMER_EVENT;
	pevent.nProtocolId=PROTOCOL_MOBILITY;
	while(join)
	{
		pevent.dEventTime = join->time;
		if(join->action==JOIN)
			pevent.nSubEventType=NODE_JOIN;
		else if(join->action==LEAVE)
			pevent.nSubEventType=NODE_LEAVE;
		fnpAddEvent(&pevent);
		JOIN_PROFILE_NEXT(join);
	}
	return 0;
}

int fn_NetSim_Mobility_NodeJoined()
{
	extern ANIM_HANDLE animHandle;
	unsigned int nLoop;
	NETSIM_ID devid = pstruEventDetails->nDeviceId;
	NETWORK->ppstruDeviceList[devid-1]->node_status = CONNECTED;

	//Call animation
	animation_add_new_entry(animHandle, ANIM_NODEJOIN, "%d,%lf",
							devid,
							pstruEventDetails->dEventTime);

	//call all the callback function
	for(nLoop=0;nLoop<nCallBackCount;nLoop++)
	{
		fnNodeJoinCallBack[nLoop](pstruEventDetails->nDeviceId,pstruEventDetails->dEventTime,JOIN);
	}
	return 0;
}

int fn_NetSim_Mobility_NodeLeaved()
{
	extern ANIM_HANDLE animHandle;
	unsigned int nLoop;
	NETSIM_ID devid = pstruEventDetails->nDeviceId;
	NETWORK->ppstruDeviceList[devid-1]->node_status = NOT_CONNECTED;

	//Call animation
	animation_add_new_entry(animHandle, ANIM_NODELEAVE, "%d,%lf",
							devid,
							pstruEventDetails->dEventTime);

	//call all the callback function
	for(nLoop=0;nLoop<nCallBackCount;nLoop++)
	{
		fnNodeJoinCallBack[nLoop](pstruEventDetails->nDeviceId,pstruEventDetails->dEventTime,LEAVE);
	}
	return 0;
}

_declspec(dllexport) int fnNodeJoinRegisterCallBackFunction(_fnNodeJoinCallBack fnCallBack)
{
	if(!nCallBackCount)
	{
		fnNodeJoinCallBack = (_fnNodeJoinCallBack*)calloc(1,sizeof* fnNodeJoinCallBack);
	}
	else
	{
		fnNodeJoinCallBack = (_fnNodeJoinCallBack*)realloc(fnNodeJoinCallBack,(nCallBackCount+1)*sizeof* fnNodeJoinCallBack);
	}
	fnNodeJoinCallBack[nCallBackCount] = fnCallBack;
	nCallBackCount++;
	return 0;
}
