#include "vector.h"
#include "testing/SimpleTest.h"
#include "console.h"
#include "simpio.h"
#include <iostream>
#include <fstream>
#include "error.h"
#include "filelib.h"
#include "grid.h"
#include "queue.h"
#include "set.h"
#include "stack.h"
#include "vector.h"
#include "PathFinder.h"

using namespace std;

Set<GridLocation> findValidMoves(Grid<string>& maze, GridLocation cur, string destination);
bool pathHelper(Grid<string>& area, Grid<string> originalArea, Stack<GridLocation>& path, GridLocation currentSpot, string destination);
Vector<string> pathDirectionsHelper(Stack<GridLocation>& path, Vector<string>& directions, GridLocation prevMove);
int calculateMinutes(Grid<string>& area, Stack<GridLocation> path, string transport, string destination);


//Breadth First Search Version:
/* TODO: Implement this function
 * The PathFinder funtion is the BFS implementation of the GPS. You are given a Grid of the area around the user, the starting
 * location, destination, and mode of transportation. You will need to return a Map of the time a given path takes to the
 * path itself.
 */
Map<int, Stack<GridLocation>> PathFinder(Grid<string>& area, GridLocation currentSpot, string destination, string transport){

    Queue<Stack<GridLocation>> exploringPaths;
    Set<GridLocation> alreadyVisited;//holds the GridLocations that have already been visited in the path being built so that they aren't revisted
    Stack<GridLocation> path = {currentSpot};
    Map<int, Stack<GridLocation>> foundPaths;
    exploringPaths.enqueue(path);

    while(!exploringPaths.isEmpty()){
        Stack<GridLocation> curPath = exploringPaths.dequeue();
        GridLocation next = curPath.peek();//this holds the next spot to travel to within the path

        if(!foundPaths.isEmpty()){//a valid path was just found and added, now looking for another path
            alreadyVisited.clear();//gets rid of the visited locations from the confirmed path
            Stack<GridLocation> copyCurPath = curPath;
            for(int i = 0; i<copyCurPath.size(); i++){//this makes alreadyVisited contain only the current path's GridLocations
                GridLocation temp = copyCurPath.pop();
                alreadyVisited.add(temp);
            }
        }
        Set<GridLocation> neighbors = findValidMoves(area, next, destination);
        for(GridLocation neighbor: neighbors){
            if(!alreadyVisited.contains(neighbor)){
                Stack<GridLocation> newPath = curPath;
                newPath.push(neighbor);
                alreadyVisited.add(neighbor);
                if(area.get(neighbor) != destination){
                    exploringPaths.enqueue(newPath);
                }
                else{
                    int time = calculateMinutes(area, newPath, transport, destination);
                    foundPaths.put(time, newPath);
                    break;
                }
            }
        }
    }
    return foundPaths;
}

//Backtracking Recursion Version:
/* TODO: Implement this function
 * The pathFinder function is the DFS Backtracking version of the GPS. You are given a Grid of the area around the user, the
 * starting location, destination, and mode of transportation. You will need to return a Map of the time a given path takes to
 * the path itself.
 */
Map<int, Stack<GridLocation>> pathFinder(Grid<string>& area, GridLocation currentSpot, string destination, string transport){
    Map<int, Stack<GridLocation>> foundPaths;
    Set<Stack<GridLocation>> recordOfPaths;//stores only the found paths to make sure the same path does not repeat twice
    Grid<string> initialArea = area;//area before some roads are marked as roadsUsed
    Stack<GridLocation> path;
    while(pathHelper(area,initialArea, path,currentSpot,destination) && !recordOfPaths.contains(path)){
        int time = calculateMinutes(initialArea, path, transport, destination);
        foundPaths.put(time, path);
        recordOfPaths.add(path);
        path.clear();
    }
    return foundPaths;
}

/*TODO: Implement this function
 * pathHelper is the backtracking recursion portion of the DFS implementation. This function takes in the original area,
 * area that was modified wihtin this function, the current path, current spot, and destination. A boolean representing
 * whether or not the path is valid (leads to the destination) is returned. Be sure to implement the "choose-explore-unchoose"
 * steps within your solution.
 */
bool pathHelper(Grid<string>& area, Grid<string> originalArea, Stack<GridLocation>& path, GridLocation currentSpot, string destination){
    //base case: the current spot is at the exit
    if(area.get(currentSpot) == destination){
        path.push(currentSpot);
        return true;
    }
    //recursive case: choose
    path.push(currentSpot);
    area[currentSpot] = "roadUsed";

    //explore
    Set<GridLocation> initialNeighbors = findValidMoves(originalArea,currentSpot, destination);
    Set<GridLocation> neighbors;//finalized neighbors
    if(initialNeighbors.size()>1){//if there more than 1 possible path to take from the current path
        for(GridLocation neighbor: initialNeighbors){
            if(area.get(neighbor) == "road" || area.get(neighbor) == destination){//since there is more than 1 road ahead, the road that was chosen in the previous path needs to not be chosen again
                neighbors.add(neighbor);
            }
        }
    }
    else{
        neighbors = initialNeighbors;//there is only one path from here, must take the same path as a previous path took
    }
    for(GridLocation neighbor: neighbors){
        if(pathHelper(area,originalArea,path,neighbor,destination)){
            return true;
        }
    }

    //unchoose
    path.pop();
    area[currentSpot] = "road";
    return false;
}

Set<GridLocation> findValidMoves(Grid<string>& area, GridLocation cur, string destination){

    Set<GridLocation> neighbors;
    Vector<GridLocation> possibleLocations = {GridLocation(cur.row + 1, cur.col), GridLocation(cur.row - 1,cur.col), GridLocation(cur.row,cur.col - 1), GridLocation(cur.row,cur.col + 1)};
    for(GridLocation current: possibleLocations){
        if((area.inBounds(current) && area.get(current) == "road") || (area.inBounds(current) && area.get(current) == destination)){
            neighbors.add(current);
        }
    }
    return neighbors;
}

int calculateMinutes(Grid<string>& area, Stack<GridLocation> path, string transport, string destination){
    path.pop(); //so that the destination is not counted (there would be one extra travel interval unless one GridLocation was removed

    int timePerGl;
    int totalTime = 0;

    if(transport == "walking"){
        timePerGl = 5;
    }
    else if(transport == "biking"){
        timePerGl = 2;
    }
    else{
        timePerGl = 1;//driving
    }

    while(!path.isEmpty()){
        GridLocation cur = path.pop();
        Set<GridLocation> neighbors = findValidMoves(area,cur,destination);//all neighbors are roads
        bool containsDestination = false;//represents whether or not the destination is in the list of neighbors
        for(GridLocation neighbor: neighbors){
            if(area.get(neighbor) == destination){
                containsDestination = true;//so that the destination is not included when calculating intersections
            }
        }
            totalTime += timePerGl;
            if(path.size()!=1){
                if(neighbors.size()==4){//4 way intersection
                    if(containsDestination){
                        totalTime += 2;
                    }
                    else{
                        totalTime += 3;
                    }
                }
                else if(neighbors.size()==3 && !containsDestination){//3 way intersection
                    totalTime += 2;
                }
            }
    }
    return totalTime;
}

string getDirections(Stack<GridLocation> path, GridLocation startLocation){
    Vector<string> finalDirections;
    Vector<string> cardinalDirections;
    Vector<int> numFeet = {500};//500 feet per GridLocation
    cardinalDirections = pathDirectionsHelper(path,cardinalDirections,startLocation);

    string prev = cardinalDirections[0];
    finalDirections.add(prev);

    for(int i = 1; i<cardinalDirections.size(); i++){
        string cur = cardinalDirections[i];

        if(prev == cur){//travelling in same direction
            numFeet[numFeet.size()-1] += 500;//adds 1 more GridLocation of feet to the last GridLocation direction in the vector
        }
        else{//not travelling in same direction
            finalDirections.add(cardinalDirections[i]);
            numFeet.add(500);
        }
    }

    string directions = "Start by heading down the road to your " + finalDirections[0] + " for " + integerToString(numFeet[0]) + " feet.";
    for(int i = 1; i<finalDirections.size(); i++){
        directions += "Then turn " + finalDirections[i] + " and continue straight for " + integerToString(numFeet[i]) + " feet.";
    }

    directions += " You have reached your destination.";
    return directions;
}

Vector<string> pathDirectionsHelper(Stack<GridLocation>& path, Vector<string>& directions, GridLocation prevMove){
    if(path.isEmpty()){
        return directions;
    }

    GridLocation move = path.pop();//removes the spot where the user starts their path
    string curDirection;

    if(move == GridLocation(prevMove.row, prevMove.col+1)){//moving right (east)
        curDirection = "east";
    }
    else if(move == GridLocation(prevMove.row, prevMove.col-1)){//moving left (west)
        curDirection = "west";
    }
    else if(move == GridLocation(prevMove.row+1, prevMove.col)){//moving up (north)
        curDirection = "north";
    }
    else{//moving down (south)
        curDirection = "south";
    }
    directions.add(curDirection);

    pathDirectionsHelper(path, directions, move);

    return directions;
}

Vector<string> directionsForTesting(Map<int,Stack<GridLocation>> returnedPaths, GridLocation start){
    Vector<string> allDirections;
    for(int key: returnedPaths){
        allDirections.add(getDirections(returnedPaths[key], start));
    }
    return allDirections;
}

PROVIDED_TEST("Medium complexity input: PathFinder and pathFinder"){
    Grid<string> area = {{"home", "house", "park", "pharmacy"},
                         {"road", "road", "road", "road"},
                         {"parking lot", "road", "grocery store", "plant store"},
                         {"school", "road", "road", "road"}};

    Map<int,Stack<GridLocation>> returnedPathsBFS = PathFinder(area, {0,0}, "plant store", "walking");
    Map<int,Stack<GridLocation>> returnedPathsDFS = pathFinder(area, {0,0}, "plant store", "walking");

    Vector<string> directionsBFS = directionsForTesting(returnedPathsBFS, {0,0});
    Vector<string> directionsDFS = directionsForTesting(returnedPathsDFS, {0,0});

    Map<int,Stack<GridLocation>> expected;

    Stack<GridLocation> path1 = {{0,0},{1,0},{1,1},{1,2},{1,3},{2,3}};
    Stack<GridLocation> path2 = {{0,0},{1,0},{1,1},{2,1},{3,1},{3,2},{3,3},{2,3}};

    expected.put(27, path1);
    expected.put(37, path2);

    Vector<string> expectedDirections = directionsForTesting(expected,{0,0});

    EXPECT(returnedPathsBFS == expected);
    EXPECT(returnedPathsDFS == expected);
    EXPECT(directionsBFS == expectedDirections);
    EXPECT(directionsDFS == expectedDirections);
}

PROVIDED_TEST("Complex input: PathFinder and pathFinder"){
    Grid<string> area = {{"home", "house", "park", "pharmacy"},
                         {"road", "road", "road", "road"},
                         {"parking lot", "road", "road", "plant store"},
                         {"school", "road", "road", "road"}};

    Map<int,Stack<GridLocation>> returnedPathsDFS = pathFinder(area, {0,0}, "plant store", "walking");
    Map<int,Stack<GridLocation>> returnedPathsBFS = PathFinder(area, {0,0}, "plant store", "walking");

    Vector<string> directionsBFS = directionsForTesting(returnedPathsBFS, {0,0});
    Vector<string> directionsDFS = directionsForTesting(returnedPathsDFS, {0,0});

    Map<int,Stack<GridLocation>> expected;

    Stack<GridLocation> path1 = {{0,0},{1,0},{1,1},{1,2},{1,3},{2,3}};
    Stack<GridLocation> path2 = {{0,0},{1,0},{1,1},{2,1},{3,1},{3,2},{3,3},{2,3}};
    Stack<GridLocation> path3 = {{0,0},{0,1},{1,1},{1,2},{2,2},{3,2}};

    expected.put(29, path1);
    expected.put(32, path2);
    expected.put(32, path3);//only 2 paths returned

    Map<int,Stack<GridLocation>> resultFinalDFS = expected - returnedPathsDFS;//random 2 of the 3 paths in expected should be returned, so one of the expected paths should be left over
    Map<int,Stack<GridLocation>> resultFinalBFS = expected - returnedPathsDFS;//random 2 of the 3 paths in expected should be returned, so one of the expected paths should be left over

    EXPECT(resultFinalDFS.size() == 1);
    EXPECT(resultFinalBFS.size() == 1);
}

PROVIDED_TEST("Simple Input: One path to destination. PathFinder and pathFinder"){
    Grid<string> area = {{"road", "home", "road", "school"}};

    Map<int,Stack<GridLocation>> returnedPathsBFS = PathFinder(area, {0,1}, "school", "walking");
    Map<int,Stack<GridLocation>> returnedPathsDFS = pathFinder(area, {0,1}, "school", "walking");

    Map<int,Stack<GridLocation>> expected;

    Stack<GridLocation> path1 = {{0,1},{0,2},{0,3}};

    expected.put(10, path1);

    EXPECT(expected == returnedPathsBFS);
    EXPECT(expected == returnedPathsDFS);
}

PROVIDED_TEST("No valid path to destination"){
    Grid<string> area = {{"flower shop", "road", "coffee shop", "home"},
                         {"road", "climbing gym", "road", "road"},
                         {"road", "bike shop", "road", "road"}};

    Map<int,Stack<GridLocation>> returnedPathsBFS = PathFinder(area, {0,3}, "flower shop", "walking");
    Map<int,Stack<GridLocation>> returnedPathsDFS = pathFinder(area, {0,3}, "flower shop", "walking");

    Map<int,Stack<GridLocation>> expected;

    EXPECT(expected == returnedPathsBFS);
    EXPECT(expected == returnedPathsDFS);
}



