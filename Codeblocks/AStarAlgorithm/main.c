#include <stdio.h>
#include <stdlib.h>

// These defines are used to specify which linked list we are cycling through or changing
#define OPEN 0
#define CLOSED 1
#define NEIGHBOR 2

// Map maximum dimensions
#define MAP_WIDTH 14
#define MAP_HEIGHT 31

// Actual data structure,  This structure contains 4 linked lists and data that corresponds
// to a specific position on the map given in Cartesian Coordinates xPos and yPos.
typedef struct Node node;
struct Node
{
    int f;
    int g;
    int h;
    int disp;
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

// This structure is used for storing the finished path as a linked list that isn't related to the above structure
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
            gridArray[(y*MAP_WIDTH)+x].disp = 0;
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

// For use if recalculating the path, this way we can keep using the same grid array
// This is basically the same as the initilizeGrid function except that it keeps the walls
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
        gridArray[(y*MAP_WIDTH)+x].wall = 9;
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
        printf("x:%d, y:%d to %d\r\n", curr->xPos, curr->yPos, choice);

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

// Adds a node to the list specfied by the pointer and choice of pointer Next
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

// Prints out the disp variable for the whole grid
void displayGrid(node *gridArray)
{
    int i, j;
    for(i=0; i<MAP_HEIGHT; i++)
    {
        for(j=0;j<MAP_WIDTH;j++)
            if(gridArray[(i*MAP_WIDTH)+j].wall == 1)
                printf("W ");
            else
                printf("%d ", gridArray[(i*MAP_WIDTH)+j].disp);

        printf("\r\n");
    }
    printf("\r\n");
}

void displayCoordinates(path *head)
{
    path *curr;
    curr = head;
    printf("Current Path\r\n");
    while(curr != NULL)
    {
        printf("x:%d, y:%d \r\n", curr->xPos, curr->yPos);
        curr = curr->next;
    }
    printf("\r\n");
}

path *getPath(int startX, int startY, int goalX, int goalY)
{
    path *PathHead;
    PathHead = NULL;

    // Iterator
    int i;

    resetGrid(grid);

    // OPEN = priority queue containing START
    linked_list_t *openSet = (linked_list_t *)malloc(sizeof(linked_list_t));
    initializeList(openSet);

    grid[(startY*MAP_WIDTH)+startX].disp = 1;

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
                curr->disp = 1;
                curr = curr->parent;
            }
            // We have finished, we return a pointer with the correct path
            displayGrid(grid);
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
            if(!neighbor->wall)
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
                    neighbor->disp = 2;
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
    //printf("No Path Possible\r\n");
    return PathHead;
}

void freePath(path *PathHead)
{
    path *curr, *prev;

    curr = PathHead;
    while(curr != NULL)
    {
        prev = curr;
        curr = curr->next;
        free(prev);
    }

    PathHead = NULL;
}

int main()
{
    initializeGrid(grid);

    //Plan initial Path (supposedly to 13, 30)
    path *temp;
    temp = getPath(0, 0, 5, 2);
    displayCoordinates(temp);
    freePath(temp);

    //Simulate finding an obstacle
    grid[(4*MAP_WIDTH)+5].wall = 1;
    grid[(4*MAP_WIDTH)+5].wall = 1;
    grid[(4*MAP_WIDTH)+5].wall = 1;

    //Replan path and continue
    temp = getPath(5, 2, 5, 5);
    displayCoordinates(temp);
    freePath(temp);

    return 0;
}
