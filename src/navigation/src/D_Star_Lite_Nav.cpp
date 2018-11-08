#include <ros/ros.h>
#include <algorithm>
#include "geometry_msgs/Point32.h"
#include <limits>
#include <cmath>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "std_msgs/Float32.h"
#include <vector>
#include <iostream>
using namespace std;

//declare constants
int const gridSize = 203;
int const edgeNum = 8 * (gridSize) * (gridSize)-20 - 12 * (gridSize - 2);


/*Custom Edge Struct

in: index of incoming edge
out: index of outgoing edge
length: cost to go from incoming edge to outgoing edge

*/
struct edge
{
  public:
    int in;
    int out;
    float length;
};

/* Converts a position in a 2d array into the corresponding index value for a single dimension array

Inputs:
    row: vertical position of index, with 0 being the top
    col: horozontal position of index, with 0 being the left

Output:
    An int that will provide the correct index for a 1 dimensional array.
*/

int indexConv(int row, int col)
{
    return row * gridSize + col;
}

/* Returns a point containing the x and y positions corresponding to a node's position on a grid
Inputs:
    node: the index of a node
Output:
    A Point32 message with two used values
        x: horozontal position of node in grid
        y: vertical position of node in grid
*/
geometry_msgs::Point32 nodeConv(int node){

    int col=node%gridSize;
    int row=((node-col)/gridSize);

    geometry_msgs::Point32 posXY;

    posXY.x=col;
    posXY.y=row;

    return posXY;
}

/* Returns the cost of going from one edge to another, factoring in both distance and blocked edges

Inputs:
    out: outgoing node
    in: incoming node
    edgeList: the list of all edges between nodes

Output:
    The cost of going from one edge to another
    
*/

float cost(int out, int in, edge edgeList[edgeNum]){
    edge testEdge;
    for(int i=0; i<edgeNum;i++){
        testEdge=edgeList[i];
        if(testEdge.out==out&&testEdge.in==in){
            return testEdge.length;
        }
    }

    cout<<"\nCould not find edge for out "<<out<<"and in "<<in;
    return 0;
}


/* returns the euclidean distance between two nodes on a grid

Inputs:
    node1: the index of the first node
    node2: the index of the second node 

Output:
    Euclidean distance between nodes
*/

float dist(int node1, int node2){

    geometry_msgs::Point32 pos1;
    geometry_msgs::Point32 pos2;
    pos1=nodeConv(node1);
    pos2=nodeConv(node2);

    return sqrt((pos1.x-pos2.x)*(pos1.x-pos2.x)+(pos1.y-pos2.y)*(pos1.y-pos2.y));

}

class edgeCosts{
    
    int edgeIndex;
    int inIndex;
    int currentNode;

    public:
    edge edgeList[edgeNum];
    void edgeListInit(bool*);

};

void edgeCosts::edgeListInit(bool blockedNodes[gridSize*gridSize]){

edgeIndex=0;
inIndex=0;
currentNode=0;

    //initialize edges: diagonals cost sqrt(2), cardinal directions cost 1
    while (edgeIndex < edgeNum)
    {
        //check surrounding nodes
        for (int i = -1; i < 2; i = i + 1)
        {
            for (int j = -1; j < 2; j = j + 1)
            {
                //don't check self
                if (i != 0 || j != 0)
                {
                    inIndex = indexConv(i, j);
                    //cout << "\ni=" << i << " j=" << j;
                    
                    //if current node is in bounds of grid
                    if (currentNode + inIndex > -1 && currentNode + inIndex < gridSize * gridSize)
                    {
                        //check for wrapping around grid since array is 1 dimensional
                        if (currentNode % gridSize == 0 &&j == -1)
                        {
                        } //left col check
                        else if(currentNode % gridSize == gridSize-1 &&j == 1)
                        {
                        } //right col check
                        else
                        {
                            edgeList[edgeIndex].in = currentNode+ inIndex;
                            edgeList[edgeIndex].out = currentNode;
                            
                            if(blockedNodes[edgeList[edgeIndex].in]){
                                edgeList[edgeIndex].length = 10000;
                            }
                            
                            else if (i * j == 0)
                            {
                                edgeList[edgeIndex].length = 1;
                            }
                            else
                            {
                                edgeList[edgeIndex].length = sqrt(2);
                            }
                            //cout << "Added Edge #" << edgeIndex + 1 << ": " << edgeList[edgeIndex].out << "," << edgeList[edgeIndex].in <<" Lenght: "<<edgeList[edgeIndex].length<< "\n";
                            edgeIndex = edgeIndex + 1;
                        }
                    }
                }
            }
        }
        currentNode = currentNode + 1;
    }

}

class aStar{

    bool openList[gridSize*gridSize];
    bool closedList[gridSize*gridSize];
   
    float CTC[gridSize*gridSize];//cost to come to a node from the starting node

    float minCost=std::numeric_limits<float>::max();
    int xj;
    int xi;//testing neighbor of current node
    float Vnew;//cost to compare to node's old cost
    //int iterations;//# of nodes checked while finding shortest path
    


public:
void runAStar(int, int, edgeCosts);
 int backPointerList[gridSize*gridSize];//index of node that provides shortest path back to starting node

};

void aStar::runAStar(int startInd, int goalInd,edgeCosts edge_List){

    for(int i=0; i<gridSize*gridSize; i++){
    openList[i]=0;
    closedList[i]=0;
    backPointerList[gridSize*gridSize]=-1;
    CTC[i]=std::numeric_limits<float>::max();
    }

    openList[startInd]=1;
    CTC[startInd]=0;

    float minCost=std::numeric_limits<float>::max();
    int xj=0;//current node along shortest path
    int xi;//testing neighbor of current node
    float Vnew;//cost to compare to node's old cost
    //int iterations=0;//# of nodes checked while finding shortest path

    //run A*
while(!closedList[goalInd]){
minCost=std::numeric_limits<float>::max();
for(int i=0; i<gridSize*gridSize; i++){
    if(openList[i]&&CTC[i]+dist(i,goalInd)<minCost){
        openList[i]=0;
        closedList[i]=1;
        xj=i;
        minCost=CTC[i]+dist(i,goalInd);
        //cout<<xj<<" is new node\n";
    }
} 
//end if goal index is in closed list
if(closedList[goalInd]){
            break;
}   
for (int i = -1; i < 2; i = i + 1){
            for (int j = -1; j < 2; j = j + 1){
                xi=indexConv(i,j)+xj;//xj neighbor
                //cout<<xi<<" is new neighbor test\n";
                if ((i != 0 || j != 0)&&closedList[xi]==0){
                    if (xi > -1 && xi < gridSize * gridSize)
                    {
                        if (xj % gridSize == 0 &&j == -1)
                        {
                        } //left col check
                        else if(xj % gridSize == gridSize-1 &&j == 1)
                        {
                        } //right col check
                        else
                        {
                        openList[xi]=1;
                        //cout<<xi<<" is in open list\n";
                        Vnew=cost(xj,xi,edge_List.edgeList)+CTC[xj];
                        if(CTC[xi]>Vnew){
                        CTC[xi]=Vnew;
                        backPointerList[xi]=xj;
                        //cout<<xj<<"<-"<<xi<<"\n";
                        //iterations=iterations+1;
                        }
                        }
                    }
                }
            }
        }
}

}

void gridPrint(bool blockedNodes[gridSize*gridSize]){

    //print grid (will be removed in full implimentation)


    geometry_msgs::Point32 grid[gridSize * gridSize];

    for (int i = 0; i < gridSize; i = i + 1)
    {

        for (int j = 0; j < gridSize; j = j + 1)
        {
            grid[(indexConv(i, j))].x = j;
            grid[(indexConv(i, j))].y = gridSize - i - 1;
            grid[(indexConv(i, j))].z = 0;
            
            //if node is blocked, print an x
            if(blockedNodes[indexConv(i, j)]){
                cout << "  X";
            }

            else if(indexConv(i, j)<10){
                cout << "  " << indexConv(i, j);
            }
            else{
            cout << " " << indexConv(i, j);
            }
        }
        cout << "\n";
    }    


}

int main(int argc, char **agrv)
{
    bool blockedNodes[gridSize*gridSize];//nodes that can't be accessed (later from LiDAR data)

    for(int i=0; i<gridSize*gridSize; i++){
    blockedNodes[i]=0;
    }


//pick nodes to dissalow evolution of
blockedNodes[78]=1;
blockedNodes[11]=1;
blockedNodes[21]=1;
blockedNodes[31]=1;
blockedNodes[32]=1;
blockedNodes[42]=1;
blockedNodes[53]=1;
blockedNodes[1]=1;

edgeCosts edge_List;

edge_List.edgeListInit(blockedNodes);

//choose starting end ending points
int startInd=0;
int goalInd=40900;

aStar initPath;

initPath.runAStar(startInd,goalInd,edge_List);

//gridPrint(blockedNodes);

int pathSize=1;
//print final path from start to goal
int index=goalInd;
cout<<"\n"<<index;
while(!index==startInd){
index=initPath.backPointerList[index];
cout<<"<-"<<index;
pathSize++;
}

//geometry_msgs::Point32 newWaypoints[pathSize-2];

sensor_msgs::PointCloud newWaypointCloud;
newWaypointCloud.points.reserve(pathSize-2);
index=goalInd;
int backIndex=pathSize-1;

while(backIndex>1)
{
    index=initPath.backPointerList[index];
    cout<<"\nBackpointer:"<<index;
    newWaypointCloud.points.insert(newWaypointCloud.points.begin(),nodeConv(index));


    backIndex=backIndex-1;
}

cout<<"\nWaypoint Cloud:"<<newWaypointCloud;

cout<<"\nPath Size:"<<pathSize<<"\n";

    return 0;
}