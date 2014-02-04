#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"

/***********
 * Globals *
 ***********/

// Serial Communication Globals
#define BufferSize 100
volatile char XBeeBuffer[BufferSize];
volatile char PiBuffer[BufferSize];
volatile char XBeeAddress = 0;
volatile char PiAddress = 0;
volatile char error = 0;

char command[256];
char ack[1] = {0x06};
char nak[1] = {0x15};

char XbeePosition = 0;
char PiPosition = 0;

// RobotID
char robotID = 0x01;

// Grid Globals
#define NORTH      1
#define EAST       2
#define SOUTH      4
#define WEST       8

int CardinalDir;
char curCol, curRow;
char curPosition[8] = {0x02, 0x01, 0x02, 0x06, 0x00, 0x00, 0x03, 0x00};
char objPosition[8] = {0x02, 0x01, 0x02, 0x08, 0x00, 0x00, 0x03, 0x00};
char turnCommand[6] = {0x02, 0x01, 0x02, 0x00, 0x03, 0x00};

// ADC Globals
/***********************
 * Distance[0] = left  *
 * Distance[1] = right *
 * Distance[2] = front *
 ***********************/
char Distance[3];

// Path Planner Globals and Defines

// These defines are used to specify which linked list we are cycling through or changing
#define OPEN 0
#define CLOSED 1
#define NEIGHBOR 2

// Map maximum dimensions
#define MAP_WIDTH  10
#define MAP_HEIGHT 10

// Actual data structure,  This structure contains 4 linked lists and data that corresponds
// to a specific position on the map given in Cartesian Coordinates xPos and yPos.  These
// nodes are all contained in a massive array structure as seen later
typedef struct Node node;
struct Node
{
    int f;
    int g;
    int h;
    int wall;
    int xPos;
    int yPos;
    node *parent;
    node *nextOpen;
    node *nextClosed;
    node *nextNeighbor;
};

// This is the array that stores all of the above nodes
node grid[MAP_WIDTH * MAP_HEIGHT];

// This struct stores a viable path given from the A* Algorithm
typedef struct Path path;
struct Path
{
    int xPos;
    int yPos;
    path *next;
};

// List define so we can keep count of the number of objects in a list and have a nice place
// for keeping the head pointer
typedef struct linked_list linked_list_t;
struct linked_list
{
    int count;
    node *head;
};

// Global for Random Things
char temp;

// Used for Sprintf
char msg[256];

/*****************************************************************************
 *The error routine that is called if the driver library encounters an error *
 *****************************************************************************/
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

void UART4IntHandler(void) //Xbee
{
    uint32_t ui32Status;

    // Get the interrupt status.
    ui32Status = UARTIntStatus(UART4_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART4_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART4_BASE))
    {
    	XBeeBuffer[XBeeAddress++] = UARTCharGetNonBlocking(UART4_BASE);
    	if(XBeeAddress == BufferSize)
    		XBeeAddress = 0;
    }
}

void UART3IntHandler(void) //3pi
{
    uint32_t ui32Status;

    // Get the interrupt status.
    ui32Status = UARTIntStatus(UART3_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART3_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART3_BASE))
    {
    	PiBuffer[PiAddress++] = UARTCharGetNonBlocking(UART3_BASE);
    	if(PiAddress == BufferSize)
    		PiAddress = 0;
    }
}

void UART4Send(char *pui8Buffer, uint32_t ui32Count)
{
    // Loop while there are more characters to send.
    while(ui32Count--)
    {
        // Write the next character to the UART.
    	UARTCharPutNonBlocking(UART4_BASE, *pui8Buffer++);
    	// Delay in between each sent char
    	SysCtlDelay(100000);
    }
}

void UART3Send(char *pui8Buffer, uint32_t ui32Count)
{
    // Loop while there are more characters to send.
    while(ui32Count--)
    {
        // Write the next character to the UART.
        UARTCharPutNonBlocking(UART3_BASE, *pui8Buffer++);
        // Delay in between each sent char
        SysCtlDelay(100000);
    }
}

char read_next_byte(uint32_t device)
{
	//Clear Previous Errors
	error = 0;

	//Start Time out
	TimerEnable(TIMER0_BASE, TIMER_A);

    //Wait till either we get a serial byte or we time out
	if(device)
		while((XBeeAddress == XbeePosition) && (error == 0));
	else
		while((PiAddress == PiPosition) && (error == 0));

	//Turn off timer we used for Time out
	TimerDisable(TIMER0_BASE, TIMER_A);

	//If we Time out return an ERROR, else return the received byte and increase our buffer position
	if(error)
		return 0xFF;
	else
	{
		if(device)
		{
			temp = XBeeBuffer[XbeePosition];

			XbeePosition++;

			//This is what makes a ring buffer a ring buffer, when it hits the end of the buffer it starts over
			if(XbeePosition == BufferSize)
				XbeePosition = 0;

			return temp;
		}
		else
		{
			char temp = PiBuffer[PiPosition];

			PiPosition++;

			//This is what makes a ring buffer a ring buffer, when it hits the end of the buffer it starts over
			if(PiPosition == BufferSize)
				PiPosition = 0;

			return temp;
		}
	}
}

//*****************************************************************************
// The interrupt handler for the first timer interrupt.
//*****************************************************************************
void Timer0IntHandler(void)
{
    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Set Error Flag
    error = 1;
}

void turn(char dir)
{
	char i, LRC;

	// Put the 'L', 'R', 'B' into the array to be sent to the Base
	turnCommand[3] = dir;

	// Calculate the Checksum with the data
	LRC = 0;
	for(i=0;i<5;i++)
		LRC ^= turnCommand[i];
	turnCommand[5] = LRC;

	// Send the Turn Command
	UART3Send((char *)turnCommand, 6);

	// Wait for the Ack Back from the Base
	i = 0;
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
	while((temp = read_next_byte(0)) != 0x06)
	{
		//UART3Send((char *)turnCommand, 6);
		if(error)
			i++;
		if(i == 5)
			break; // TODO Implement a more graceful way to handle the Base not responding.
	}
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
}

void follow_segment()
{
	// Send Follow Segment Command to Base
	char follow_segment[5] = {0x02, 0x00, 0x01, 0x03, 0x00};
	UART3Send((char *)follow_segment, 5);

	// Wait for the Ack Back from the Base
	char i = 0;
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
	while((temp = read_next_byte(0)) != 0x06)
	{
		//UART3Send((char *)follow_segment, 5);
		if(error)
			i++;
		if(i == 5)
			break; // TODO Implement a more graceful way to handle the Base not responding.
	}
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
}

void readSensorVoltage(uint32_t *voltages)
{
	uint32_t sData[3];

    // Get ADC data
	ADCProcessorTrigger(ADC0_BASE, 1);
	while(!ADCIntStatus(ADC0_BASE, 1, false));
	ADCIntClear(ADC0_BASE, 1);
	ADCSequenceDataGet(ADC0_BASE, 1, sData);

	// Go through the data you get from ADC's and convert it to milliVolts
	for(temp=0; temp<3; temp++)
		voltages[temp] = (sData[temp] * 3.3 * 1000) / 4096;
}

// Lookup table for turning voltages into distance in cm
char lookupDistance(uint32_t voltage)
{
	char distance;

	if(voltage < 400)
		distance = 80;
	else if(voltage < 450)
		distance = 70;
	else if(voltage < 550)
		distance = 60;
	else if(voltage < 600)
		distance = 50;
	else if(voltage < 750)
		distance = 40;
	else if(voltage < 950)
		distance = 30;
	else if(voltage < 1100)
		distance = 25;
	else if(voltage < 1300)
		distance = 20;
	else if(voltage < 1650)
		distance = 15;
	else if(voltage < 2350)
		distance = 10;
	else if(voltage < 2800)
		distance = 8;
	else if(voltage < 3300)
		distance = 6;
	else
		distance = 4;
	
	// Divide distance by 2, not quite sure why
	return distance/2;
}

// Reads ADC and grabs all three distances
void getDistances(char *Distances)
{
    uint32_t SensorVoltages[3];

	// Get the milliVolt levels for each sensor
    readSensorVoltage(SensorVoltages);

	// Change the milliVolt levels to actual distances using a LUT
    int i;
    for(i=0; i<3; i++)
    	Distances[i] = lookupDistance(SensorVoltages[i]);
}

void goto_point(char colGoal, char rowGoal)
{
	while(curCol != colGoal)
    {
		if(curCol < colGoal)
		{
			if(CardinalDir == NORTH)
			{
				turn('R');
				follow_segment();
			}
			else if(CardinalDir == EAST)
			{
				follow_segment();
			}
			else if(CardinalDir == SOUTH)
			{
				turn('L');
				follow_segment();
			}
			else if(CardinalDir == WEST)
			{
				turn('B');
				follow_segment();
			}
			CardinalDir = EAST;
            ++curCol;
        }
		else
        {
			if(CardinalDir == NORTH)
			{
				turn('L');
				follow_segment();
			}
			else if(CardinalDir == EAST)
			{
				turn('B');
				follow_segment();
			}
			else if(CardinalDir == SOUTH)
			{
				turn('R');
				follow_segment();
			}
			else if(CardinalDir == WEST)
			{
				follow_segment();
			}

			CardinalDir = WEST;
            --curCol;
		}

		// Put Current Position in the buffer
		curPosition[4] = curCol;
		curPosition[5] = curRow;

		// Calculate the Checksum with the new data inserted
		char i, LRC = 0;
		for(i=0;i<7;i++)
			LRC ^= curPosition[i];
		curPosition[7] = LRC;

		// Send that data out the serial port to the control center
		UART4Send((char *)curPosition, 8);
    	//sprintf(msg, "Col:%d, Row:%d \r\n", curCol, curRow);
        //UART4Send(msg, strlen(msg));
	}
	while (curRow != rowGoal)
    {

		if(curRow < rowGoal)
        {
			if(CardinalDir == NORTH)
			{
				// Straight
				follow_segment();
			}
			else if(CardinalDir == EAST)
			{
				turn('L');
				follow_segment();
			}
			else if(CardinalDir == SOUTH)
			{
				turn('B');
				follow_segment();
			}
			else if(CardinalDir == WEST)
			{
				turn('R');
				follow_segment();
			}

			CardinalDir = NORTH;
			++curRow;
		}
		else
        {
			if(CardinalDir == NORTH)
			{
				turn('B');
				follow_segment();
			}
			else if(CardinalDir == EAST)
			{
				turn('R');
				follow_segment();
			}
			else if(CardinalDir == SOUTH)
			{
				follow_segment();
			}
			else if(CardinalDir == WEST)
			{
				turn('L');
				follow_segment();
			}
			CardinalDir = SOUTH;
			--curRow;
		}

		// Put Current Position in the buffer
		curPosition[4] = curCol;
		curPosition[5] = curRow;

		// Calculate the Checksum with the new data inserted
		char i, LRC = 0;
		for(i=0;i<7;i++)
			LRC ^= curPosition[i];
		curPosition[7] = LRC;

		// Send that data out the serial port to the control center
		UART4Send((char *)curPosition, 8);
    	//sprintf(msg, "Col:%d, Row:%d \r\n", curCol, curRow);
        //UART4Send(msg, strlen(msg));
	}
}

// Functions for Path Following
// Sets a new list to default values
void initializeList(linked_list_t *list)
{
    list->count = 0;
    list->head = NULL;
}

// This initializes your array values based on min and max x and y values
void initializeGrid(node *gridArray)
{
    int x, y;
    for(y=0; y<MAP_HEIGHT; y++)
    {
        for(x=0;x<MAP_WIDTH;x++)
        {
            gridArray[(y*MAP_WIDTH)+x].f = 0;
            gridArray[(y*MAP_WIDTH)+x].g = 0;
            gridArray[(y*MAP_WIDTH)+x].h = 0;
            gridArray[(y*MAP_WIDTH)+x].wall = 0;
            gridArray[(y*MAP_WIDTH)+x].xPos = x;
            gridArray[(y*MAP_WIDTH)+x].yPos = y;
            gridArray[(y*MAP_WIDTH)+x].parent = NULL;
            gridArray[(y*MAP_WIDTH)+x].nextOpen = NULL;
            gridArray[(y*MAP_WIDTH)+x].nextClosed = NULL;
            gridArray[(y*MAP_WIDTH)+x].nextNeighbor = NULL;
        }
    }
}

// Resets the pointers so you don't randomly get infinite loops because of old paths
void resetGrid(node *gridArray)
{
    int x, y;
    for(y=0; y<MAP_HEIGHT; y++)
    {
        for(x=0;x<MAP_WIDTH;x++)
        {
            gridArray[(y*MAP_WIDTH)+x].f = 0;
            gridArray[(y*MAP_WIDTH)+x].g = 0;
            gridArray[(y*MAP_WIDTH)+x].h = 0;
            gridArray[(y*MAP_WIDTH)+x].parent = NULL;
            gridArray[(y*MAP_WIDTH)+x].nextOpen = NULL;
            gridArray[(y*MAP_WIDTH)+x].nextClosed = NULL;
            gridArray[(y*MAP_WIDTH)+x].nextNeighbor = NULL;
        }
    }
}

// This function can be used to add walls to the map
void initializeWalls(node *gridArray)
{
    int x, y;
    y = 4;
    for(x=2;x<MAP_WIDTH;x++)
        gridArray[(y*MAP_WIDTH)+x].wall = 1;
}

// Calculates the Manhattan Distance
int heuristic(int nodeX, int nodeY, int goalX, int goalY)
{
    int dx = abs(nodeX - goalX);
    int dy = abs(nodeY - goalY);
    return (dx + dy);
}

// Just prints out the nodes in a given list
void displayList(int choice, linked_list_t *list)
{
    node *curr;
    curr = list->head;

    while(curr != NULL)
    {
		sprintf(msg,"x:%d, y:%d to %d\r\n", curr->xPos, curr->yPos, choice);
		UART4Send(msg, strlen(msg));

        if(choice == OPEN)
            curr = curr->nextOpen;
        else if(choice == CLOSED)
            curr = curr->nextClosed;
        else if(choice == NEIGHBOR)
            curr = curr->nextNeighbor;
    }
}

// Returns 1 if a node is in the array checked by xPos and yPos, and a 0 if not
int isInList(int choice, linked_list_t *list, node *isInList)
{
    node *itr = list->head;
    int length;
    for(length=0; length<list->count; length++)
    {
        if((isInList->xPos == itr->xPos) && (isInList->yPos == itr->yPos))
            return 1;

        if(choice == OPEN)
            itr = itr->nextOpen;
        else if(choice == CLOSED)
            itr = itr->nextClosed;
        else if(choice == NEIGHBOR)
            itr = itr->nextNeighbor;
    }

    return 0;
}

// Adds a node to the list specified by the pointer and choice of pointer Next
void add_node(int choice, linked_list_t *list, node *new)
{
    if(list->head == NULL)// If Head is NULL, the list doesn't yet exist, so we start one
    {

        list->head = new;

        if(choice == OPEN)
            list->head->nextOpen = NULL;
        else if(choice == CLOSED)
            list->head->nextClosed = NULL;
        else if(choice == NEIGHBOR)
            list->head->nextNeighbor = NULL;

        // Debug Statements
        //printf("Added Node\r\n");
        //displayList(choice, list);
        list->count++;
        return;
    }
    else //List does exist so we append to the head of it
    {
        if(!isInList(choice, list, new))
        {
            if(choice == OPEN)
                new->nextOpen = list->head;
            else if(choice == CLOSED)
                new->nextClosed = list->head;
            else if(choice == NEIGHBOR)
                new->nextNeighbor = list->head;

            list->head = new;

            // Debug Statements
            //printf("Added Node\r\n");
            //displayList(choice, list);
            list->count++;
        }
        return;
    }
}

// Removes a node from the list
void remove_node(int choice, linked_list_t *list, node *remove)
{
    node *curr, *prev;

    curr = list->head;
    while(curr != NULL)
    {
        if((remove->xPos == curr->xPos) && (remove->yPos == curr->yPos))
        {
            if(curr == list->head)
            {
                if(choice == OPEN)
                {
                    list->head = curr->nextOpen;
                    curr->nextOpen = NULL;
                }
                else if(choice == CLOSED)
                {
                    list->head = curr->nextClosed;
                    curr->nextClosed = NULL;
                }
                else if(choice == NEIGHBOR)
                {
                    list->head = curr->nextNeighbor;
                    curr->nextNeighbor = NULL;
                }

                list->count--;

                // Debug Statements
                //printf("Removed Node\r\n");
                //displayList(choice, list);
                return;
            }
            else
            {

                if(choice == OPEN)
                {
                    prev->nextOpen = curr->nextOpen;
                    curr->nextOpen = NULL;
                }
                else if(choice == CLOSED)
                {
                    prev->nextClosed = curr->nextClosed;
                    curr->nextClosed = NULL;
                }
                else if(choice == NEIGHBOR)
                {
                    prev->nextNeighbor = curr->nextNeighbor;
                    curr->nextNeighbor = NULL;
                }

                list->count--;

                // Debug Statements
                //printf("Removed Node\r\n");
                //displayList(choice, list);
                return;
            }
        }
        else
        {
            prev = curr;

            if(choice == OPEN)
                curr = curr->nextOpen;
            else if(choice == CLOSED)
                curr = curr->nextClosed;
            else if(choice == NEIGHBOR)
                curr = curr->nextNeighbor;
        }
    }
    return;
}

// Prints out the wall variable for the whole grid
void displayGrid(node *gridArray)
{
    int i, j;
    for(i=0; i<MAP_HEIGHT; i++)
    {
        for(j=0;j<MAP_WIDTH;j++)
        {
        	sprintf(msg, "%d ", gridArray[(i*MAP_WIDTH)+j].wall);
            UART4Send(msg, strlen(msg));
        }
    	sprintf(msg, "\r\n");
        UART4Send(msg, strlen(msg));
    }
	sprintf(msg, "\r\n");
    UART4Send(msg, strlen(msg));
}

void displayCoordinates(path *head)
{
    path *curr;
    curr = head;
	sprintf(msg, "Current Path\r\n");
    UART4Send(msg, strlen(msg));

    while(curr != NULL)
    {
    	sprintf(msg, "x:%d, y:%d \r\n", curr->xPos, curr->yPos);
        UART4Send(msg, strlen(msg));
        curr = curr->next;
    }
}

void freePath(path *head)
{
    path *curr, *prev;

    curr = head;
    while(curr != NULL)
    {
    	prev = curr;
    	curr = curr->next;
    	free(prev);
    }
    head = NULL;
}

path *getPath(int startX, int startY, int goalX, int goalY)
{
    path *PathHead;
    PathHead = NULL;

    // Iterator
    int i;

    // OPEN = priority queue containing START
    linked_list_t *openSet = (linked_list_t *)malloc(sizeof(linked_list_t));
    initializeList(openSet);

    add_node(OPEN, openSet, &grid[(startY*MAP_WIDTH)+startX]);

    // CLOSED = empty set
    linked_list_t *closedSet = (linked_list_t *)malloc(sizeof(linked_list_t));
    initializeList(closedSet);

    // NEIGHBOR = empty set
    linked_list_t *neighborSet = (linked_list_t *)malloc(sizeof(linked_list_t));
    initializeList(neighborSet);

    // while lowest rank in OPEN is not the GOAL:
    while(openSet->count > 0)
    {
        // current = remove lowest rank item from OPEN
        node *currentNode = openSet->head;
        node *iterator = openSet->head;

        for(i=0; i<openSet->count; i++)
        {
            if(iterator->f < currentNode->f)
                currentNode = iterator;

            iterator = iterator->nextOpen;
        }

        // If we find the goal we are done, track down the optimal path using parent pointer
        if((currentNode->xPos == goalX) && (currentNode->yPos == goalY))
        {
            // Used the neighbor set as a holder for the final path.
            neighborSet->count = 0;
            neighborSet->head = NULL;

            node *curr = currentNode;

            // This code goes through the parent pointers and reverses the list such that the path is in order
            while(curr->parent != NULL)
            {
                if(PathHead == NULL)// If Head is NULL, the list doesn't yet exist, so we start one
                {
                    path *new = (path *)malloc(sizeof(path));
                    new->xPos = curr->xPos;
                    new->yPos = curr->yPos;
                    new->next = NULL;
                    PathHead = new;
                }
                else //List does exist so we append to the head of it
                {
                    path *new = (path *)malloc(sizeof(path));
                    new->xPos = curr->xPos;
                    new->yPos = curr->yPos;
                    new->next = PathHead;
                    PathHead = new;
                }

                // Change the wall variable just so we can see the path in the displayGrid()
                //curr->wall = 1;
                curr = curr->parent;
            }
            // We have finished, we return a pointer with the correct path
            //displayGrid(grid);

            resetGrid(grid);
            return PathHead;
        }

        // add current to CLOSED
        add_node(CLOSED, closedSet, currentNode);
        remove_node(OPEN, openSet, currentNode);

        // add Neighbors of currentNode to list
        if(currentNode->xPos-1 > 0)
            add_node(NEIGHBOR, neighborSet, &grid[(currentNode->yPos*MAP_WIDTH)+(currentNode->xPos-1)]);
        if(currentNode->xPos+1 < MAP_WIDTH)
            add_node(NEIGHBOR, neighborSet, &grid[(currentNode->yPos*MAP_WIDTH)+(currentNode->xPos+1)]);
        if(currentNode->yPos-1 > 0)
            add_node(NEIGHBOR, neighborSet, &grid[((currentNode->yPos-1)*MAP_WIDTH)+currentNode->xPos]);
        if(currentNode->yPos+1 < MAP_HEIGHT)
            add_node(NEIGHBOR, neighborSet, &grid[((currentNode->yPos+1)*MAP_WIDTH)+currentNode->xPos]);

        // for neighbors of current:
        node *neighbor = neighborSet->head;
        for(i=0; i<neighborSet->count; i++)
        {
            // If that neighbor is not a wall
            if(neighbor->wall != 1)
            {
                // cost = g(current) + movementcost(current, neighbor)
                int Cost = currentNode->g + 1;

                // if neighbor in OPEN and cost less than g(neighbor):
                // remove neighbor from OPEN, because new path is better
                if(isInList(OPEN, openSet, neighbor) && (Cost < neighbor->g))
                    remove_node(OPEN, openSet, neighbor);

                // if neighbor in CLOSED and cost less than g(neighbor):
                // This should never happen if you have an monotone admissible heuristic. However in games we often have inadmissible heuristics.
                // if(isInList(closedSet, neighbor) && (Cost < neighbor->g))

                // if neighbor not in OPEN and neighbor not in CLOSED:
                if(!isInList(OPEN, openSet, neighbor) && !isInList(CLOSED, closedSet, neighbor))
                {
                    add_node(OPEN, openSet, neighbor);
                    neighbor->g = Cost;
                    neighbor->h = heuristic(neighbor->xPos, neighbor->yPos, goalX, goalY);
                    neighbor->f = neighbor->g + neighbor->h;
                    neighbor->parent = currentNode;
                    neighbor->wall = 2;

                    // Display The iterations of the grid as it explores nodes
                    //displayGrid(grid);
                }
            }
            neighbor = neighbor->nextNeighbor;
        }

        // Clean out neighbor set
        neighbor = neighborSet->head;
        while(neighbor->nextNeighbor != NULL)
        {
            node *next = neighbor->nextNeighbor;
            remove_node(NEIGHBOR, neighborSet, neighbor);
            neighbor = next;
        }
    }

    // If it finishes looking at the open set we know a path is not possible so exit out and return NULL pointer
    resetGrid(grid);
    return PathHead;
}

int pathToAction(path *PathHead)
{
    path *curr;
    curr = PathHead;
    while(curr != NULL)
    {
    	getDistances(Distance);

    	if(Distance[0] <= 8)//left
    	{
			if(CardinalDir == NORTH)
				if(grid[((curRow)*MAP_WIDTH)+(curCol-1)].wall != 1)
				{
					grid[((curRow)*MAP_WIDTH)+(curCol-1)].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol-1;
					objPosition[5] = curRow;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
			else if(CardinalDir == EAST)
				if(grid[((curRow+1)*MAP_WIDTH)+curCol].wall != 1)
				{
					grid[((curRow+1)*MAP_WIDTH)+curCol].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol;
					objPosition[5] = curRow+1;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
			else if(CardinalDir == SOUTH)
				if(grid[((curRow)*MAP_WIDTH)+(curCol+1)].wall != 1)
				{
					grid[((curRow)*MAP_WIDTH)+(curCol+1)].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol+1;
					objPosition[5] = curRow;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
			else if(CardinalDir == WEST)
				if(grid[((curRow-1)*MAP_WIDTH)+(curCol)].wall != 1)
				{
					grid[((curRow-1)*MAP_WIDTH)+(curCol)].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol;
					objPosition[5] = curRow-1;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
    	}

    	if(Distance[1] <= 8)//right
    	{
			if(CardinalDir == NORTH)
				if(grid[((curRow)*MAP_WIDTH)+(curCol+1)].wall != 1)
				{
					grid[((curRow)*MAP_WIDTH)+(curCol+1)].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol+1;
					objPosition[5] = curRow;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
			else if(CardinalDir == EAST)
				if(grid[((curRow-1)*MAP_WIDTH)+curCol].wall != 1)
				{
					grid[((curRow-1)*MAP_WIDTH)+curCol].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol;
					objPosition[5] = curRow-1;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
			else if(CardinalDir == SOUTH)
				if(grid[((curRow)*MAP_WIDTH)+(curCol-1)].wall != 1)
				{
					grid[((curRow)*MAP_WIDTH)+(curCol-1)].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol-1;
					objPosition[5] = curRow;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
			else if(CardinalDir == WEST)
				if(grid[((curRow+1)*MAP_WIDTH)+(curCol)].wall != 1)
				{
					grid[((curRow+1)*MAP_WIDTH)+(curCol)].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol;
					objPosition[5] = curRow+1;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
    	}

    	if(Distance[2] <= 8)//front
    	{
			if(CardinalDir == NORTH)
				if(grid[((curRow+1)*MAP_WIDTH)+curCol].wall != 1)
				{
					grid[((curRow+1)*MAP_WIDTH)+curCol].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol;
					objPosition[5] = curRow+1;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
			else if(CardinalDir == EAST)
				if(grid[(curRow*MAP_WIDTH)+(curCol+1)].wall != 1)
				{
					grid[(curRow*MAP_WIDTH)+(curCol+1)].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol+1;
					objPosition[5] = curRow;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
			else if(CardinalDir == SOUTH)
				if(grid[((curRow-1)*MAP_WIDTH)+curCol].wall != 1)
				{
					grid[((curRow-1)*MAP_WIDTH)+curCol].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol;
					objPosition[5] = curRow-1;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
			else if(CardinalDir == WEST)
				if(grid[(curRow*MAP_WIDTH)+(curCol-1)].wall != 1)
				{
					grid[(curRow*MAP_WIDTH)+(curCol-1)].wall = 1;

					// Put Object Position in the buffer
					objPosition[4] = curCol-1;
					objPosition[5] = curRow;

					// Calculate the Checksum with the new data inserted
					char i, LRC = 0;
					for(i=0;i<7;i++)
						LRC ^= objPosition[i];
					objPosition[7] = LRC;

					// Send that data out the serial port to the control center
					UART4Send((char *)objPosition, 8);

					return 0;
				}
    	}

    	goto_point((char)curr->xPos, (char)curr->yPos);
        curr = curr->next;
    }
    return 1;
}

// MAIN LOOP
int main(void)
{
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPUEnable();
    FPULazyStackingEnable();

    // Set the clocking to run directly from the crystal
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    /***********************
     * UART Initialization *
     ***********************/

    // Enable the peripherals used by this UART4. XBee
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);  Don't need this line because Both UARTs use PortC

    // Enable the peripherals used by this UART3. 3pi
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Enable the peripherals used by ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Enable processor interrupts.
    IntMasterEnable();

    // Set GPIO C4 and C5 as UART4 pins.
    GPIOPinConfigure(GPIO_PC4_U4RX);
    GPIOPinConfigure(GPIO_PC5_U4TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Set GPIO C6 and C7 as UART3 pins.
    GPIOPinConfigure(GPIO_PC6_U3RX);
    GPIOPinConfigure(GPIO_PC7_U3TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    // Set E1, E2, and E3 as ADC pins.
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1 |GPIO_PIN_2 | GPIO_PIN_3);


    // Configure the UART2 and UART3 for 9600, 8-N-1 operation.
    UARTConfigSetExpClk(UART4_BASE, SysCtlClockGet(), 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 9600,
                           (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE));

    // Enable the UART2 and UART3 interrupt.
    IntEnable(INT_UART4);
    UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_RT);
    IntEnable(INT_UART3);
    UARTIntEnable(UART3_BASE, UART_INT_RX | UART_INT_RT);


    /**********************
     * ADC Initialization *
     **********************/

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);//AIN0 -- PE3
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);//AIN1 -- PE2
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH2|ADC_CTL_IE|ADC_CTL_END);//AIN2 -- PE1
    ADCSequenceEnable(ADC0_BASE, 1);
    ADCIntClear(ADC0_BASE, 1);

    /************************
     * Timer Initialization *
     ************************/

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Configure the two 32-bit periodic timers.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());

    // Setup the interrupts for the timer timeouts.
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /************************
     * Initialize Variables *
     ************************/
    initializeGrid(grid);
    //uint8_t ui8PinData = 2; //For LED color changes
    char i, j, p, LRC; //Counters and checksum
    CardinalDir = NORTH;
    curCol = 0;
    curRow = 0;

    /************************
     * Start Actual Program *
     ************************/

	//Loop Forever Loop, Loops Forever
	while(1)
	{
		//Error Detection and escape loop
		while(1)
		{
    		//Wait for STX
			while((command[0] = read_next_byte(1)) != 0x02)
            {

				getDistances(Distance);

				if(Distance[0] < 5)//red //left
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
				if(Distance[1] < 5)//blue //right
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
				if(Distance[2] < 5) //green //front
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);

				//GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, ui8PinData);
				//if(ui8PinData==8) {ui8PinData=2;} else {ui8PinData=ui8PinData*2;}
			}

			//RobotID
			command[1] = read_next_byte(1);
			if(error == 1)
				break;

			//Datalength
			command[2] = read_next_byte(1);
			if(error == 1)
				break;

			//Command
			command[3] = read_next_byte(1);
			if(error == 1)
				break;

            //If there is actual Data, else assume that command[5] is the ETX
			j = 4;
			if(command[2] > 0x00)
			{
				i = command[2];
				while(i--)
				{
					command[j++] = read_next_byte(1);
					if(error == 1)
						break;
				}
			}

			//ETX
			command[j++] = read_next_byte(1);
			if(error == 1)
				break;

			//Checksum
			command[j] = read_next_byte(1);
			if(error == 1)
				break;

			//calculate Tiva side checksum
			LRC = 0;
			for(p=0; p<j; p++)
				LRC ^= command[p];

            //If the Calculated checksum isn't the same as the checksum sent by the control center, there is an error in the data
			if(LRC != command[j])
				break;

			/****************************************************
             * Complete/(Most Likely Correct) Command Received! *
             ****************************************************/

    		//check if this is the correct robot
			if(command[1] == robotID)
			{
	            //Assuming this message was meant for this Robot, acknowledge that it was received.
				UART4Send((char *)ack, 1);

				//Now we can finally figure out what to do with this command since it has been tested, verified, and is indeed for this particular Robot
				if(command[3] == 0x00) //Initialize Sensors
				{
					CardinalDir = NORTH;
					curCol = 0;
					curRow = 0;
					char calibrate[5] = {0x02, 0x00, 0x00, 0x03, 0x01};
					UART3Send((char *)calibrate, 5);

					// Wait for the Ack Back from the Base
					i = 0;
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);
					while((temp = read_next_byte(0)) != 0x06)
					{
						//UART3Send((char *)calibrate, 5);
						if(error)
							i++;
						if(i == 5)
							break; //TODO Implement a more graceful way to handle the Base not responding.
					}
					GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
				}
				else if(command[3] == 0x01) //Straight
				{
					if(CardinalDir == NORTH)
						goto_point(curCol,curRow+1);
					else if(CardinalDir == EAST)
						goto_point(curCol+1,curRow);
					else if(CardinalDir == SOUTH)
						goto_point(curCol,curRow-1);
					else if(CardinalDir == WEST)
						goto_point(curCol-1,curRow);
				}
				else if(command[3] == 0x02) //Left
				{
					if(CardinalDir == NORTH)
						goto_point(curCol,curRow-1);
					else if(CardinalDir == EAST)
						goto_point(curCol-1,curRow);
					else if(CardinalDir == SOUTH)
						goto_point(curCol,curRow+1);
					else if(CardinalDir == WEST)
						goto_point(curCol+1,curRow);
				}
				else if(command[3] == 0x03) //Back
				{
					if(CardinalDir == NORTH)
						goto_point(curCol-1,curRow);
					else if(CardinalDir == EAST)
						goto_point(curCol,curRow+1);
					else if(CardinalDir == SOUTH)
						goto_point(curCol+1,curRow);
					else if(CardinalDir == WEST)
						goto_point(curCol,curRow-1);
				}
				else if(command[3] == 0x04) //Right
				{
					if(CardinalDir == NORTH)
						goto_point(curCol+1,curRow);
					else if(CardinalDir == EAST)
						goto_point(curCol,curRow-1);
					else if(CardinalDir == SOUTH)
						goto_point(curCol-1,curRow);
					else if(CardinalDir == WEST)
						goto_point(curCol,curRow+1);
				}
				else if(command[3] == 0x05) // Go To Point
				{
					// Create a variable to store the path in
				    path *Pathway;

					// Get the first path
				    Pathway = getPath((int)curCol,(int)curRow,command[4],command[5]);
				    //displayCoordinates(Pathway);

					// If your pathToAction functions finds a wall that isn't currently in the memory stop and grab a new path
				    while(!pathToAction(Pathway))
				    {
				    	freePath(Pathway);
				    	Pathway = getPath((int)curCol,(int)curRow,command[4],command[5]);
				    	//displayCoordinates(Pathway);
				    }
				}
				else if(command[3] == 0x06) // Send Back Current Position
				{
					//Put Current Position in the buffer
					curPosition[4] = curCol;
					curPosition[5] = curRow;

					//Calculate the Checksum with the new data inserted
					LRC = 0;
					char i;
					for(i=0;i<7;i++)
						LRC ^= curPosition[i];
					curPosition[7] = LRC;

					//Send that data out the serial port to the control center
					UART4Send((char *)curPosition, 8);
				}
				else if(command[3] == 0x07) // Update the RobotID
				{
					robotID = command[4];
					curPosition[1] = command[4];
					objPosition[1] = command[4];
				}
				else
				{
					//TODO, Add error condition reporting
				}
			}
		}

        /*******************************************************
         * GOT AN ERROR, SEND A NAK AND START THE DECODER OVER *
         *******************************************************/
		UART4Send((char *)nak, 1);
	}
}
