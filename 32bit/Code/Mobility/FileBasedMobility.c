/************************************************************************************
File Based Mobility                                                  *

The name of the trace File generated should be kept as mobility.txt and it should
be in the NS2 trace File format. The user can genmerate the trace file by any traffic
simulator. One of the Traffic Generator is VanetMobSim.
A user wishing to simulate Vanet Scenario can place the nodes randomly at any position
and start the simulation. The device Initial and Intermediate device Postions will
be taken care by the following three functions

The function FileBasedMobilityInitializeNodePosition(int ); sets the initial positions
of the nodes.

The function FileBasedMobility(int , double ); is defined to change the position of
the current deiveceID and to read the next Position Change of the current deviceID
and add this Event to the List of events to be performed.

The function FileBasedMobilityPointersFree(int ); closes the file and frees all the
pointers.

#
#nodes: 5  max x = 1000.0, max y: 1000.0
#
$node_(0) set X_ 0.6642883828044
$node_(0) set Y_ 0.2309939067026
$node_(0) set Z_ 0.0
$node_(1) set X_ 50.6642883828044
$node_(1) set Y_ 50.2309939067026
$node_(1) set Z_ 0.0
$node_(2) set X_ 100.1527892303775
$node_(2) set Y_ 100.0017151661647
$node_(2) set Z_ 0.0
$node_(3) set X_ 150.3207048718017
$node_(3) set Y_ 150.7817679768309
$node_(3) set Z_ 0.0
$node_(4) set X_ 200.6792971281983
$node_(4) set Y_ 200.2182340231691
$node_(4) set Z_ 0.0
$time 0.0 "$node_(0) 0.00 0.00 0.00"
$time 0.0 "$node_(1) 50.0 50.0 0.0"
$time 0.0 "$node_(2) 100 100 0"
$time 0.0 "$node_(3) 150 150 0"
$time 0.0 "$node_(4) 200 200 0"
$time 0.05 "$node_(0) 50 0 0"
$time 0.05 "$node_(1) 100 50 0"
...
...
...
********************************************************************************
*******************************************************************************/

#include "main.h"
#include "Mobility.h"
#include "NetSim_utility.h"

typedef struct stru_filebasedmobilityinfo
{
	NETSIM_ID d;
	double time;
	double x;
	double y;
	double z;
	struct stru_filebasedmobilityinfo* next;
}INFO,*ptrINFO;
ptrINFO* mobilityInfo;
ptrINFO* lastInfo;

static void add_mobility_info(NETSIM_ID d,
	double time,
	double x,
	double y,
	double z)
{
	if (!mobilityInfo)
	{
		mobilityInfo = calloc(NETWORK->nDeviceCount, sizeof* mobilityInfo);
		lastInfo = calloc(NETWORK->nDeviceCount, sizeof* lastInfo);
	}
	ptrINFO info = mobilityInfo[d - 1];

	ptrINFO t = calloc(1, sizeof* t);
	t->d = d;
	t->time = time;
	t->x = x;
	t->y = y;
	t->z = z;
	if (info)
	{
		lastInfo[d - 1]->next = t;
		lastInfo[d - 1] = t;
	}
	else
	{
		mobilityInfo[d - 1] = t;
		lastInfo[d-1] = t;
	}
	fprintf(stderr, "%d,%lf,%lf,%lf%lf\n",
		d, time, x, y, z);
}

static void add_mobility_event()
{
	NETSIM_ID d;
	if (!mobilityInfo)
		return;
	for (d = 0; d < NETWORK->nDeviceCount; d++)
	{
		if (!mobilityInfo[d])
			continue;
		NetSim_MOBILITY* mob = DEVICE_MOBILITY(d + 1);
		mob->pstruCurrentPosition = calloc(1, sizeof* mob->pstruCurrentPosition);
		mob->pstruNextPosition = calloc(1, sizeof* mob->pstruNextPosition);
		ptrINFO i = mobilityInfo[d];
		pstruEventDetails->dEventTime = i->time;
		pstruEventDetails->dPacketSize = 0;
		pstruEventDetails->nApplicationId = 0;
		pstruEventDetails->nDeviceId = d+1;
		pstruEventDetails->nDeviceType = NETWORK->ppstruDeviceList[d]->nDeviceType;
		pstruEventDetails->nEventType = TIMER_EVENT;
		pstruEventDetails->nInterfaceId = 0;
		pstruEventDetails->nPacketId = 0;
		pstruEventDetails->nProtocolId = PROTOCOL_MOBILITY;
		pstruEventDetails->nSubEventType = 0;
		pstruEventDetails->pPacket = NULL;
		fnpAddEvent(pstruEventDetails);
	}
}

void process_filebased_mobility_event()
{
	if (!mobilityInfo)
		return;
	NETSIM_ID d = pstruEventDetails->nDeviceId;
	ptrINFO i = mobilityInfo[d - 1];
	if (!i)
		return;
	NetSim_MOBILITY* mob = DEVICE_MOBILITY(d);
	NetSim_COORDINATES* c = mob->pstruCurrentPosition;
	NetSim_COORDINATES* n = mob->pstruNextPosition;
	NetSim_COORDINATES* dc = DEVICE(d)->pstruDevicePosition;
	dc->X = i->x;
	dc->Y = i->y;
	dc->Z = i->z;
	c->X = i->x;
	c->Y = i->y;
	c->Z = i->z;
	if (i->next)
	{
		n->X = i->next->x;
		n->Y = i->next->y;
		n->Z = i->next->z;
	}
	mobilityInfo[d - 1] = i->next;
	free(i);
	if (mobilityInfo[d - 1])
	{
		pstruEventDetails->dEventTime = mobilityInfo[d - 1]->time;
		fnpAddEvent(pstruEventDetails);
	}
}

/** This function is to free the file pointers */
int FileBasedMobilityPointersFree()
{
	NETSIM_ID d;
	if (mobilityInfo)
	{
		for (d = 0; d < NETWORK->nDeviceCount; d++)
		{
			ptrINFO i = mobilityInfo[d];
			while (i)
			{
				mobilityInfo[d] = i->next;
				free(i);
				i = mobilityInfo[d];
			}
		}
		free(mobilityInfo);
	}
	return 0;
}

/** This function is to open the file, to define position pointers and to set the initial positions for all the nodes */
int FileBasedMobilityReadingFile()
{
	int lineno = 0;
	double time, x, y, z;
	NETSIM_ID d;
	FILE* fp;
	if (!mobilityInfo)
	{
		fp = fopen("mobility.txt", "r");
	}
	else
	{
		fnSystemError("Unable to open mobility.txt file.\n");
		perror("Mobility.txt");
		return -1;
	}

	char data[BUFSIZ];
	while (fgets(data, BUFSIZ, fp))
	{
		lineno++;
		lskip(data);
		if (*data == '#' ||
			*data == 0)
			continue;
		if (*data != '$')
		{
			fprintf(stderr, "In mobility.txt, invalid data at line no %d\n", lineno);
			continue;
		}
		if (*(data + 1) == 'n')
			continue;
		if (*(data + 1) != 't')
		{
			fprintf(stderr, "In mobility.txt, invalid data at line no %d\n", lineno);
			continue;
		}

		sscanf(data, "$time %lf \"$node_(%d) %lf %lf %lf\"",
			&time, &d, &x, &y, &z);
		add_mobility_info(d + 1, time*SECOND, x, y, z);
	}
	fclose(fp);

	add_mobility_event();
	return 0;
}
