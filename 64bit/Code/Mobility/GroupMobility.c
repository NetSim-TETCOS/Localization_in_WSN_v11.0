/************************************************************************************
* Copyright (C) 2014                                                               *
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
#include "List.h"
extern unsigned int nCallBackCount=0;
int nGroupCount=0;
typedef struct stru_Group_Mobility
{
	NETSIM_ID nGroupId;
	unsigned int nDeviceCount;
	NETSIM_ID* nDevIds;
	_ele* ele;
}GROUP_MOBILITY;
#define GROUP_ALLOC() (GROUP_MOBILITY*)list_alloc(sizeof(GROUP_MOBILITY),offsetof(GROUP_MOBILITY,ele))
#define GROUP_NEXT(var) var=(GROUP_MOBILITY*)LIST_NEXT(var)

GROUP_MOBILITY* group = NULL;

GROUP_MOBILITY* get_group_ptr(NETSIM_ID nGroupId)
{
	GROUP_MOBILITY* temp = group;
	while(temp)
	{
		if(temp->nGroupId == nGroupId)
			return temp;
		GROUP_NEXT(temp);
	}
	return NULL;
}

GROUP_MOBILITY* group_add_new(NETSIM_ID group_id)
{
	GROUP_MOBILITY* temp = GROUP_ALLOC();
	nGroupCount++;
	temp->nGroupId=group_id;
	LIST_ADD_LAST((void**)&group,temp);
	return temp;
}

int add_to_group(NETSIM_ID group_id, NETSIM_ID dev_id)
{
	GROUP_MOBILITY* temp=get_group_ptr(group_id);
	if(!temp)
		temp=group_add_new(group_id);
	temp->nDeviceCount++;
	temp->nDevIds = (NETSIM_ID*)realloc(temp->nDevIds,temp->nDeviceCount*sizeof(NETSIM_ID));
	temp->nDevIds[temp->nDeviceCount-1] = dev_id;
	return 0;
}

int fn_NetSim_Mobility_Group_init()
{
	GROUP_MOBILITY* temp=group;
	while(temp)
	{
		pstruEventDetails->dEventTime = 0;
		pstruEventDetails->dPacketSize = 0;
		pstruEventDetails->nApplicationId = 0;
		pstruEventDetails->nDeviceId = temp->nGroupId;
		pstruEventDetails->nDeviceType = Not_Device;
		pstruEventDetails->nEventType = TIMER_EVENT;
		pstruEventDetails->nInterfaceId = 0;
		pstruEventDetails->nPacketId = 0;
		pstruEventDetails->nProtocolId = PROTOCOL_MOBILITY;
		pstruEventDetails->nSubEventType = MOVE_GROUP;
		pstruEventDetails->pPacket = NULL;
		fnpAddEvent(pstruEventDetails);
		GROUP_NEXT(temp);
	}
	return 0;
}
int fnValidateposition(GROUP_MOBILITY* group,double diff_x,double diff_y,bool* flag)
{
	unsigned int i;
	if (DEVICE_MOBILITY(group->nDevIds[0])->pstruCurrentPosition->ismap)
	{
		*flag = false;
		return 0;
	}

	for(i=0;i<group->nDeviceCount;i++)
	{
		double x=DEVICE_MOBILITY(group->nDevIds[i])->pstruCurrentPosition->X;
		double y=DEVICE_MOBILITY(group->nDevIds[i])->pstruCurrentPosition->Y;
		x += diff_x;
		y += diff_y;
		if(x > dSimulationArea_X || x < 0 || y < 0 || y > dSimulationArea_Y)
			return -1;
	}
	*flag=false;
	return 0;
}

int fn_NetSim_MoveGroup()
{
	unsigned int nLoop;
	bool flag=true;
	double vel;
	double x,y,diff_x,diff_y;
	NETSIM_ID group_id = pstruEventDetails->nDeviceId;
	GROUP_MOBILITY* group = get_group_ptr(group_id);
	unsigned int i;
	MOBILITY_VAR* pstruMobilityVar;
	for(i=0;i<group->nDeviceCount;i++)
	{
		NETSIM_ID dev=group->nDevIds[i];
		memcpy(DEVICE_MOBILITY(dev)->pstruCurrentPosition,
			DEVICE_MOBILITY(dev)->pstruNextPosition,
			sizeof* DEVICE_MOBILITY(dev)->pstruCurrentPosition);
	}
	pstruMobilityVar=(MOBILITY_VAR*)DEVICE_MOBILITY(group->nDevIds[0])->pstruMobVar;
	vel=pstruMobilityVar->dVelocity;
	do
	{
		NetSim_COORDINATES* curr = DEVICE_MOBILITY(group->nDevIds[0])->pstruCurrentPosition;
		x=curr->X;
		y=curr->Y;
		fn_NMo_RandomPoint(&x,&y,vel,pstruMobilityVar->dCalculationInterval,&pstruMobilityVar->ulSeed1,&pstruMobilityVar->ulSeed2);
		diff_x=x-DEVICE_MOBILITY(group->nDevIds[0])->pstruCurrentPosition->X;
		diff_y=y-DEVICE_MOBILITY(group->nDevIds[0])->pstruCurrentPosition->Y;
		fnValidateposition(group,diff_x,diff_y,&flag);
	} while (flag);

	for(i=0;i<group->nDeviceCount;i++)
	{
		NETSIM_ID dev=group->nDevIds[i];
		DEVICE_MOBILITY(dev)->pstruNextPosition->X += diff_x;
		DEVICE_MOBILITY(dev)->pstruNextPosition->Y += diff_y;

		memcpy(DEVICE_POSITION(dev),
			   DEVICE_MOBILITY(dev)->pstruCurrentPosition,
			   sizeof* DEVICE_MOBILITY(dev)->pstruCurrentPosition);

		add_mobility_animation(dev,
							   pstruEventDetails->dEventTime,
							   DEVICE_POSITION(dev)->X,
							   DEVICE_POSITION(dev)->Y,
							   0);
	}
	pstruMobilityVar->dLastTime = pstruEventDetails->dEventTime+1000000;
	//Add event for next point 
	pstruEventDetails->dEventTime+=1000000;
	fnpAddEvent(pstruEventDetails);
	pstruEventDetails->dEventTime-=1000000;

	//call all the callback function
	for(nLoop=0;nLoop<nCallBackCount;nLoop++)
	{
		for(i=0;i<group->nDeviceCount;i++)
			fnMobilityCallBack[nLoop](group->nDevIds[i]);
	}
	return 0;
}



