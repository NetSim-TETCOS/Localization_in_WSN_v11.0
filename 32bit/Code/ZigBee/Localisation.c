
/************************************************************************************
* Copyright (C) 2013                                                                *
* TETCOS, Bangalore. India                                                          *
*                                                                                   *
* Tetcos owns the intellectual property rights in the Product and its content.      *
* The copying, redistribution, reselling or publication of any or all of the        *
* Product or its content without express prior written consent of Tetcos is         *
* prohibited. Ownership and / or any other right relating to the software and all   *
* intellectual property rights therein shall remain at all times with Tetcos.       *
*                                                                                   *
* Author:    Soniya Kandala					                                        *
* Key Terms                                                                         *
* a. Anchor node: It is a sensor node which has prior knowledge of its location     *
* co-ordinates                                                                      *
* when it is deployed in the network environment since it is equipped with GPS.     *
* b. Lateration: It is a technique, where the unknown sensor node location is       *
* determined through                                                                *
* distance measurement from unknown sensor node to anchor nodes                     *
* c. Unknown Node: It is the sensor whose position is determined by anchor nodes    *
* using trilateration                                                               *
*                                                                                   *
* ---------------------------------------------------------------------------------*/



#include "main.h"
#include <time.h>
#include "802_15_4.h"
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "main.h"

//Function prototype
_declspec (dllexport) int fn_NetSim_localisation();
int fn_NetSim_trilateration_method(int unknown_id, double x_p, double y_p);
int get_device_id(int an[]); // Will return the device ID per NetSim GUI

typedef struct stru_anchor_details *ANCHOR_DETAILS;
typedef struct stru_sensor_details *SENSOR_DETAILS;

int an[100] = { 0 }; // Max number of achor nodes can be set here. Default is 100

//Set up unknown node
unknown_node_count = 2;
unknown_node_IDs[10] = { 4 , 7 };

double power = 0;
int unknown_id = 0;
double recv_power[100][100] = { 0 };
int neighbour_node[100] = { 0 }, temp[100] = { 0 };
FILE *local = NULL;

char buf[100];
int status;
NETSIM_ID dim = 0, dcount = 0, dcounti, dcountj;
char sensor1[50];
char sinknode1[50];
bool result1, result2;
bool **array;
NETSIM_ID *Node_Id;
double *xc, *yc, *sxc, *syc;
NETSIM_ID *h;
unsigned short *nrank;
int anchor_clr = 1, unknown_clr = 2, tri_node_clr = 3;
int neighbour_node_count = 0;


//Unknown Node Checking
bool IsUnknownNode(NETSIM_ID devId)
{
	int i;
	for (i = 0; i < unknown_node_count; i++)
		if (unknown_node_IDs[i] == devId)
			return true;
	return false;
}

//Anchor Node Checking
bool determine_anchor_node(NETSIM_ID devId)
{
	int i;
	for (i = 0; i < NETWORK->nDeviceCount; i++)
		if (neighbour_node[i] == devId)
			return true;
	return false;
}

//Unknown node details
struct stru_sensor_details
{
	double x_pos;
	double y_pos;
	ANCHOR_DETAILS *anchor_node;
};
SENSOR_DETAILS *sen_det;

//Anchor node details
struct stru_anchor_details
{
	int anchor_id;
	double x_pos, y_pos;
	double dist;
};
ANCHOR_DETAILS *an_det;

//Function to implement localisation
_declspec (dllexport) int fn_NetSim_localisation()
{
	int i = 0, j = 0, k = 0, l = 0, x = 0, y = 0, z = 0;
	double x_pos = 0, y_pos = 0;
	int count = 0;

	sen_det = (SENSOR_DETAILS *)calloc(NETWORK->nDeviceCount + 1, sizeof *sen_det);
	an_det = (ANCHOR_DETAILS *)calloc(unknown_node_count * 3, sizeof *an_det);
	static int file = 0;
	for (i = 0; i < NETWORK->nDeviceCount; i++)
	{
		if (pstruEventDetails->nDeviceId == unknown_node_IDs[i])
		{
			count++;
		}
	}
	if (count == 0)
	{
		return 0;
	}

	if (file == 0)
	{
		local = remove("localisation.txt");
		file = 1;
	}
	//fdelete("localisation.txt");
	local = fopen("localisation.txt", "a");
	//Unknown nodes
	fprintf(local, "\nUnknown nodes\n");
	for (i = 0; i < unknown_node_count; i++)
	{
		fprintf(local, "%d\n", unknown_node_IDs[i]);
	}

	//Finding the anchor nodes
	for (i = 0; i < NETWORK->nDeviceCount; i++)
	{
		if (!strcmp(NETWORK->ppstruDeviceList[i]->type, "SENSOR") && !IsUnknownNode(NETWORK->ppstruDeviceList[i]->nDeviceId))
		{
			neighbour_node[neighbour_node_count] = NETWORK->ppstruDeviceList[i]->nDeviceId;
			neighbour_node_count++;
		}
	}

	
	//Calculating the received powers from all anchor nodes to unknown nodes using API GET_RX_POWER_mw
	fprintf(local, "Received powers\n");
	for (i = 0; i < unknown_node_count; i++)
	{
		sen_det[i] = (SENSOR_DETAILS *)calloc(1, sizeof *sen_det[i]);
		for (j = 0; j < NETWORK->nDeviceCount; j++)
		{
			if (determine_anchor_node(neighbour_node[j]) && neighbour_node[j] != 0 && !IsUnknownNode(neighbour_node[j]))
			{
				recv_power[neighbour_node[j]][unknown_node_IDs[i]] = GET_RX_POWER_dbm(neighbour_node[j], unknown_node_IDs[i], pstruEventDetails->dEventTime);
				fprintf(local, "From %d to %d is: %.10f dbm\n", neighbour_node[j], unknown_node_IDs[i], recv_power[neighbour_node[j]][unknown_node_IDs[i]]);
			}
		}
	}

	//Finding anchor nodes based on received power
	for (i = 0; i < unknown_node_count; ++i)
	{
		count = 0;
		for (l = 0; l < neighbour_node_count; l++)
		{
			temp[l] = neighbour_node[l];
		}
		for (j = 0; j < neighbour_node_count; ++j)
		{
			for (k = j + 1; k < neighbour_node_count; ++k)
			{
				if (determine_anchor_node(neighbour_node[j]) && determine_anchor_node(neighbour_node[k]) && neighbour_node[j] != 0 && neighbour_node[k] != 0)
				{
					{
						x = unknown_node_IDs[i];
						y = neighbour_node[j];
						z = neighbour_node[k];
						if (recv_power[y][x] < recv_power[z][x])
						{
							power = recv_power[y][x];
							recv_power[y][x] = recv_power[z][x];
							recv_power[z][x] = power;
							power = temp[j];
							temp[j] = temp[k];
							temp[k] = power;
						}
					}
				}
			}
		}
		//Anchor nodes based on highest received powers		
		for (j = 0; j < neighbour_node_count; j++)
		{
			an[j] = temp[j];
		}
		sen_det[i]->anchor_node = (SENSOR_DETAILS *)calloc(3, sizeof* sen_det[i]->anchor_node);

		//Anchor node details
		for (j = 0; j < 3; j++)
		{
			sen_det[i]->anchor_node[j] = (SENSOR_DETAILS *)calloc(1, sizeof *sen_det[i]->anchor_node[j]);
			an_det[j] = (ANCHOR_DETAILS *)calloc(1, sizeof *an_det[j]);
			an_det[j]->anchor_id = an[j];
			y = get_device_id(an[j]);
			an_det[j]->x_pos = NETWORK->ppstruDeviceList[y]->pstruDevicePosition->X;
			an_det[j]->y_pos = NETWORK->ppstruDeviceList[y]->pstruDevicePosition->Y;
			an_det[j]->dist = DEVICE_DISTANCE(unknown_node_IDs[i], an[j]);
			sen_det[i]->anchor_node[j]->anchor_id = an_det[j]->anchor_id;
			sen_det[i]->anchor_node[j]->x_pos = an_det[j]->x_pos;
			sen_det[i]->anchor_node[j]->y_pos = an_det[j]->y_pos;
			sen_det[i]->anchor_node[j]->dist = an_det[j]->dist;
			//Check whether the anchor nodes are in a straight line or not
			if (j == 2)
			{
				if (an_det[j]->x_pos == an_det[j - 1]->x_pos && an_det[j - 1]->x_pos == an_det[j - 2]->x_pos)
				{
					an[j] = an[j + 1];
					j = 0;
				}
				if (an_det[j]->y_pos == an_det[j - 1]->y_pos && an_det[j - 1]->y_pos == an_det[j - 2]->y_pos)
				{
					an[j] = an[j + 1];
					j = 0;
				}
			}
		}

		fprintf(local, "Unknown node = %d\nAnchor nodes = ", unknown_node_IDs[i]);
		for (j = 0; j < 3; j++)
		{
			fprintf(local, "%d, ", sen_det[i]->anchor_node[j]->anchor_id);
			
		}
		fprintf(local, "\n");
		free(an_det[j]);

		fn_NetSim_trilateration_method(i, x_pos, y_pos);
		fprintf(local, "The position of Unknown node %d at time %fµs = %.0f, %.0f\n", unknown_node_IDs[i], pstruEventDetails->dEventTime, sen_det[i]->x_pos, sen_det[i]->y_pos);

		for (j = 0; j < NETWORK->nDeviceCount; ++j)
		{
			an[j] = 0;
		}
	}
	free(sen_det[i]);
	count = 0;
	neighbour_node_count = 0;
}

int get_device_id(int z)
{
	int x = 0;
	for (x = 0; x < NETWORK->nDeviceCount; ++x)
	{
		if (z == NETWORK->ppstruDeviceList[x]->nDeviceId)
		{
			return x;
		}
	}
}

//Implement triangulation method to find the position of unknown sensor
int fn_NetSim_trilateration_method(int unknown_id, double x_pos, double y_pos)
{
	double A[2][2] = { 0 };
	double B[2][1] = { 0 };
	double x = 0, y = 0;

	A[0][0] = 2 * (sen_det[unknown_id]->anchor_node[1]->x_pos - sen_det[unknown_id]->anchor_node[0]->x_pos);
	A[0][1] = 2 * (sen_det[unknown_id]->anchor_node[1]->y_pos - sen_det[unknown_id]->anchor_node[0]->y_pos);
	A[1][0] = 2 * (sen_det[unknown_id]->anchor_node[2]->x_pos - sen_det[unknown_id]->anchor_node[1]->x_pos);
	A[1][1] = 2 * (sen_det[unknown_id]->anchor_node[2]->y_pos - sen_det[unknown_id]->anchor_node[1]->y_pos);

	B[0][0] = (sen_det[unknown_id]->anchor_node[1]->x_pos * sen_det[unknown_id]->anchor_node[1]->x_pos) - \
		(sen_det[unknown_id]->anchor_node[0]->x_pos * sen_det[unknown_id]->anchor_node[0]->x_pos) + \
		(sen_det[unknown_id]->anchor_node[1]->y_pos * sen_det[unknown_id]->anchor_node[1]->y_pos) - \
		(sen_det[unknown_id]->anchor_node[0]->y_pos * sen_det[unknown_id]->anchor_node[0]->y_pos) + \
		(sen_det[unknown_id]->anchor_node[0]->dist * sen_det[unknown_id]->anchor_node[0]->dist) - \
		(sen_det[unknown_id]->anchor_node[1]->dist * sen_det[unknown_id]->anchor_node[1]->dist);
	B[0][1] = (sen_det[unknown_id]->anchor_node[2]->x_pos * sen_det[unknown_id]->anchor_node[2]->x_pos) - \
		(sen_det[unknown_id]->anchor_node[1]->x_pos * sen_det[unknown_id]->anchor_node[1]->x_pos) + \
		(sen_det[unknown_id]->anchor_node[2]->y_pos * sen_det[unknown_id]->anchor_node[2]->y_pos) - \
		(sen_det[unknown_id]->anchor_node[1]->y_pos * sen_det[unknown_id]->anchor_node[1]->y_pos) + \
		(sen_det[unknown_id]->anchor_node[1]->dist * sen_det[unknown_id]->anchor_node[1]->dist) - \
		(sen_det[unknown_id]->anchor_node[2]->dist * sen_det[unknown_id]->anchor_node[2]->dist);

	//Unknown sensor coordinates	
	sen_det[unknown_id]->x_pos = (B[0][0] * A[1][1] - B[1][0] * A[0][1]) / (A[1][1] * A[0][0] - A[0][1] * A[1][0]);
	sen_det[unknown_id]->y_pos = (B[0][0] * A[1][0] - A[0][0] * B[1][0]) / (A[0][1] * A[1][0] - A[0][0] * A[1][1]);
}


