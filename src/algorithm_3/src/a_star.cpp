// A C++ Program to implement A* Search Algorithm
#include <bits/stdc++.h>
#include "mixed_reality_library/mixed_reality_library.h"
using namespace std;

double lat_start = 30.632670;
double long_start = -96.475155;
double lat_goal = 30.632589;
double long_goal = -96.475236;
/*
double lat_goal = 30.632391;
double long_goal = -96.475462;
*/
double lat_floor;
double long_floor;
double lat_increment;
double long_increment;

ros::Publisher ros_path_publisher;

int ROW;
int COL;

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int> > pPair;

// A structure to hold the necessary parameters
struct cell {
// Row and Column index of its parent
// Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
int parent_i, parent_j;
// f = g + h
double f, g, h;
};


// A utility function to round a number either up or down, 
// depending on if it's greater than or less than zero
int roundup(double num)
{
  int answer = abs(round(num));
  if (answer == 0)
  {
    answer = 1;
  }
  return answer;
}

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(int row, int col)
{
  // Returns true if row number and column number
  // is in range
  return (row >= 0) && (row < ROW) && (col >= 0)
  && (col < COL);
}

// A Utility Function to check whether the given cell is
// blocked or not
bool isUnBlocked(vector<vector<int>> grid, int row, int col)
{
  // Returns true if the cell is not blocked else false
  if (grid[row][col] == 1)
  return (true);
  else
  return (false);
}

// A Utility Function to check whether destination cell has
// been reached or not
bool isDestination(int row, int col, Pair dest)
{
  if (row == dest.first && col == dest.second)
  return (true);
  else
  return (false);
}

// A Utility Function to calculate the 'h' heuristics.
double calculateHValue(int row, int col, Pair dest)
{
  // Return using the distance formula
  return ((double)sqrt(
  (row - dest.first) * (row - dest.first)
  + (col - dest.second) * (col - dest.second)));
}

// A Utility Function to trace the path from the source
// to destination
void tracePath(vector<vector<cell>> cellDetails, Pair dest)
{
  printf("\nThe Path is ");
  int row = dest.first;
  int col = dest.second;

  stack<Pair> Path;

  while (!(cellDetails[row][col].parent_i == row
  && cellDetails[row][col].parent_j == col)) {//100
    Path.push(make_pair(row, col));
    int temp_row = cellDetails[row][col].parent_i;
    int temp_col = cellDetails[row][col].parent_j;
    row = temp_row;
    col = temp_col;
  }
  // Find the basline for lat and long
  vector<double> lat_vector {lat_start};
  vector<double> long_vector {long_start};
  Path.push(make_pair(row, col));
  int counter = 1;
  while (!Path.empty()) {
    pair<int, int> p = Path.top();
    Path.pop();
    //printf("-> (%d,%d) ", p.first, p.second);
    double lat_meters = p.first;
    double long_meters = p.second;
    lat_vector.push_back(lat_floor - lat_meters/111320);
    long_vector.push_back(long_floor + long_meters*360/ 
      (40075000*cos(lat_vector.at(counter)*M_PI/180)));
    counter = counter + 1;
  }
  int L = lat_vector.size();
  ROS_INFO("length = %d", L);
  lat_vector.at(L-1) = lat_goal;
  long_vector.at(L-1) = long_goal;
  //display GPS path coordinates and pack into a nav_msgs/Path message
  nav_msgs::Path gps_path;
  geometry_msgs::PoseStamped pose;
  for (int i = 0; i < L; i++)
  {
    //ROS_INFO("%f  %f", lat_vector.at(i), long_vector.at(i));
    pose.pose.position.x = lat_vector.at(i);
    pose.pose.position.y = long_vector.at(i);
    gps_path.poses.push_back(pose);
  }
  ros_path_publisher.publish(gps_path);
  return;
}

// A Function to find the shortest path between
// a given source cell to a destination cell according
// to A* Search Algorithm
void aStarSearch(vector<vector<int>> grid, Pair src, Pair dest)
{
// If the source is out of range
if (isValid(src.first, src.second) == false) {
printf("Source is invalid\n");
return;
}

// If the destination is out of range
if (isValid(dest.first, dest.second) == false) {
printf("Destination is invalid\n");
return;
}

// Either the source or the destination is blocked
if (isUnBlocked(grid, src.first, src.second) == false
|| isUnBlocked(grid, dest.first, dest.second)
== false) {
printf("Source or the destination is blocked\n");
return;
}

// If the destination cell is the same as source cell
if (isDestination(src.first, src.second, dest)
== true) {
printf("We are already at the destination\n");
return;
}

// Create a closed list and initialise it to false which
// means that no cell has been included yet This closed
// list is implemented as a boolean 2D array
bool closedList[ROW][COL];
memset(closedList, false, sizeof(closedList));

// Declare a 2D array of structure to hold the details
// of that cell
//cell cellDetails[ROW][COL];
vector<vector<cell>> cellDetails( ROW, vector<cell> (COL));

int i, j;

for (i = 0; i < ROW; i++) {
  for (j = 0; j < COL; j++) {
    cellDetails[i][j].f = FLT_MAX;
    cellDetails[i][j].g = FLT_MAX;
    cellDetails[i][j].h = FLT_MAX;
    cellDetails[i][j].parent_i = -1;
    cellDetails[i][j].parent_j = -1;
  }
}

// Initialising the parameters of the starting node
i = src.first, j = src.second;
cellDetails[i][j].f = 0.0;
cellDetails[i][j].g = 0.0;
cellDetails[i][j].h = 0.0;
cellDetails[i][j].parent_i = i;
cellDetails[i][j].parent_j = j;

/*
Create an open list having information as-
<f, <i, j>>
where f = g + h,
and i, j are the row and column index of that cell
Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
This open list is implemented as a set of pair of
pair.*/
set<pPair> openList;

// Put the starting cell on the open list and set its
// 'f' as 0
openList.insert(make_pair(0.0, make_pair(i, j)));

// We set this boolean value as false as initially
// the destination is not reached.
bool foundDest = false;

while (!openList.empty()) {
pPair p = *openList.begin();

// Remove this vertex from the open list
openList.erase(openList.begin());

// Add this vertex to the closed list
i = p.second.first;
j = p.second.second;
closedList[i][j] = true;

/*
Generating all the 8 successor of this cell

N.W N N.E
\ | /
\ | /
W----Cell----E
/ | \
/ | \
S.W S S.E

Cell-->Popped Cell (i, j)
N --> North (i-1, j)
S --> South (i+1, j)
E --> East (i, j+1)
W --> West (i, j-1)
N.E--> North-East (i-1, j+1)
N.W--> North-West (i-1, j-1)
S.E--> South-East (i+1, j+1)
S.W--> South-West (i+1, j-1)*/

// To store the 'g', 'h' and 'f' of the 8 successors
double gNew, hNew, fNew;

//----------- 1st Successor (North) ------------

// Only process this cell if this is a valid one
if (isValid(i - 1, j) == true) {
// If the destination cell is the same as the
// current successor
if (isDestination(i - 1, j, dest) == true) {
// Set the Parent of the destination cell
cellDetails[i - 1][j].parent_i = i;
cellDetails[i - 1][j].parent_j = j;
printf("The destination cell is found\n");
tracePath(cellDetails, dest);
foundDest = true;
return;
}
// If the successor is already on the closed
// list or if it is blocked, then ignore it.
// Else do the following
else if (closedList[i - 1][j] == false
&& isUnBlocked(grid, i - 1, j)
== true) {
gNew = cellDetails[i][j].g + 1.0;
hNew = calculateHValue(i - 1, j, dest);
fNew = gNew + hNew;

// If it isn’t on the open list, add it to
// the open list. Make the current square
// the parent of this square. Record the
// f, g, and h costs of the square cell
// OR
// If it is on the open list already, check
// to see if this path to that square is
// better, using 'f' cost as the measure.
if (cellDetails[i - 1][j].f == FLT_MAX
|| cellDetails[i - 1][j].f > fNew) {
openList.insert(make_pair(
fNew, make_pair(i - 1, j)));

// Update the details of this cell
cellDetails[i - 1][j].f = fNew;
cellDetails[i - 1][j].g = gNew;
cellDetails[i - 1][j].h = hNew;
cellDetails[i - 1][j].parent_i = i;
cellDetails[i - 1][j].parent_j = j;
}
}
}

//----------- 2nd Successor (South) ------------

// Only process this cell if this is a valid one
if (isValid(i + 1, j) == true) {
// If the destination cell is the same as the
// current successor
if (isDestination(i + 1, j, dest) == true) {
// Set the Parent of the destination cell
cellDetails[i + 1][j].parent_i = i;
cellDetails[i + 1][j].parent_j = j;
printf("The destination cell is found\n");
tracePath(cellDetails, dest);
foundDest = true;
return;
}
// If the successor is already on the closed
// list or if it is blocked, then ignore it.
// Else do the following
else if (closedList[i + 1][j] == false
&& isUnBlocked(grid, i + 1, j)
== true) {
gNew = cellDetails[i][j].g + 1.0;
hNew = calculateHValue(i + 1, j, dest);
fNew = gNew + hNew;

// If it isn’t on the open list, add it to
// the open list. Make the current square
// the parent of this square. Record the
// f, g, and h costs of the square cell
// OR
// If it is on the open list already, check
// to see if this path to that square is
// better, using 'f' cost as the measure.
if (cellDetails[i + 1][j].f == FLT_MAX
|| cellDetails[i + 1][j].f > fNew) {
openList.insert(make_pair(
fNew, make_pair(i + 1, j)));
// Update the details of this cell
cellDetails[i + 1][j].f = fNew;
cellDetails[i + 1][j].g = gNew;
cellDetails[i + 1][j].h = hNew;
cellDetails[i + 1][j].parent_i = i;
cellDetails[i + 1][j].parent_j = j;
}
}
}

//----------- 3rd Successor (East) ------------

// Only process this cell if this is a valid one
if (isValid(i, j + 1) == true) {
// If the destination cell is the same as the
// current successor
if (isDestination(i, j + 1, dest) == true) {
// Set the Parent of the destination cell
cellDetails[i][j + 1].parent_i = i;
cellDetails[i][j + 1].parent_j = j;
printf("The destination cell is found\n");
tracePath(cellDetails, dest);
foundDest = true;
return;
}

// If the successor is already on the closed
// list or if it is blocked, then ignore it.
// Else do the following
else if (closedList[i][j + 1] == false
&& isUnBlocked(grid, i, j + 1)
== true) {
gNew = cellDetails[i][j].g + 1.0;
hNew = calculateHValue(i, j + 1, dest);
fNew = gNew + hNew;

// If it isn’t on the open list, add it to
// the open list. Make the current square
// the parent of this square. Record the
// f, g, and h costs of the square cell
// OR
// If it is on the open list already, check
// to see if this path to that square is
// better, using 'f' cost as the measure.
if (cellDetails[i][j + 1].f == FLT_MAX
|| cellDetails[i][j + 1].f > fNew) {
openList.insert(make_pair(
fNew, make_pair(i, j + 1)));

// Update the details of this cell
cellDetails[i][j + 1].f = fNew;
cellDetails[i][j + 1].g = gNew;
cellDetails[i][j + 1].h = hNew;
cellDetails[i][j + 1].parent_i = i;
cellDetails[i][j + 1].parent_j = j;
}
}
}

//----------- 4th Successor (West) ------------

// Only process this cell if this is a valid one
if (isValid(i, j - 1) == true) {
// If the destination cell is the same as the
// current successor
if (isDestination(i, j - 1, dest) == true) {
// Set the Parent of the destination cell
cellDetails[i][j - 1].parent_i = i;
cellDetails[i][j - 1].parent_j = j;
printf("The destination cell is found\n");
tracePath(cellDetails, dest);
foundDest = true;
return;
}

// If the successor is already on the closed
// list or if it is blocked, then ignore it.
// Else do the following
else if (closedList[i][j - 1] == false
&& isUnBlocked(grid, i, j - 1)
== true) {
gNew = cellDetails[i][j].g + 1.0;
hNew = calculateHValue(i, j - 1, dest);
fNew = gNew + hNew;

// If it isn’t on the open list, add it to
// the open list. Make the current square
// the parent of this square. Record the
// f, g, and h costs of the square cell
// OR
// If it is on the open list already, check
// to see if this path to that square is
// better, using 'f' cost as the measure.
if (cellDetails[i][j - 1].f == FLT_MAX
|| cellDetails[i][j - 1].f > fNew) {
openList.insert(make_pair(
fNew, make_pair(i, j - 1)));

// Update the details of this cell
cellDetails[i][j - 1].f = fNew;
cellDetails[i][j - 1].g = gNew;
cellDetails[i][j - 1].h = hNew;
cellDetails[i][j - 1].parent_i = i;
cellDetails[i][j - 1].parent_j = j;
}
}
}

//----------- 5th Successor (North-East)
//------------

// Only process this cell if this is a valid one
if (isValid(i - 1, j + 1) == true) {
// If the destination cell is the same as the
// current successor
if (isDestination(i - 1, j + 1, dest) == true) {
// Set the Parent of the destination cell
cellDetails[i - 1][j + 1].parent_i = i;
cellDetails[i - 1][j + 1].parent_j = j;
printf("The destination cell is found\n");
tracePath(cellDetails, dest);
foundDest = true;
return;
}

// If the successor is already on the closed
// list or if it is blocked, then ignore it.
// Else do the following
else if (closedList[i - 1][j + 1] == false
&& isUnBlocked(grid, i - 1, j + 1)
== true) {
gNew = cellDetails[i][j].g + 1.414;
hNew = calculateHValue(i - 1, j + 1, dest);
fNew = gNew + hNew;

// If it isn’t on the open list, add it to
// the open list. Make the current square
// the parent of this square. Record the
// f, g, and h costs of the square cell
// OR
// If it is on the open list already, check
// to see if this path to that square is
// better, using 'f' cost as the measure.
if (cellDetails[i - 1][j + 1].f == FLT_MAX
|| cellDetails[i - 1][j + 1].f > fNew) {
openList.insert(make_pair(
fNew, make_pair(i - 1, j + 1)));

// Update the details of this cell
cellDetails[i - 1][j + 1].f = fNew;
cellDetails[i - 1][j + 1].g = gNew;
cellDetails[i - 1][j + 1].h = hNew;
cellDetails[i - 1][j + 1].parent_i = i;
cellDetails[i - 1][j + 1].parent_j = j;
}
}
}

//----------- 6th Successor (North-West)
//------------

// Only process this cell if this is a valid one
if (isValid(i - 1, j - 1) == true) {
// If the destination cell is the same as the
// current successor
if (isDestination(i - 1, j - 1, dest) == true) {
// Set the Parent of the destination cell
cellDetails[i - 1][j - 1].parent_i = i;
cellDetails[i - 1][j - 1].parent_j = j;
printf("The destination cell is found\n");
tracePath(cellDetails, dest);
foundDest = true;
return;
}

// If the successor is already on the closed
// list or if it is blocked, then ignore it.
// Else do the following
else if (closedList[i - 1][j - 1] == false
&& isUnBlocked(grid, i - 1, j - 1)
== true) {
gNew = cellDetails[i][j].g + 1.414;
hNew = calculateHValue(i - 1, j - 1, dest);
fNew = gNew + hNew;

// If it isn’t on the open list, add it to
// the open list. Make the current square
// the parent of this square. Record the
// f, g, and h costs of the square cell
// OR
// If it is on the open list already, check
// to see if this path to that square is
// better, using 'f' cost as the measure.
if (cellDetails[i - 1][j - 1].f == FLT_MAX
|| cellDetails[i - 1][j - 1].f > fNew) {
openList.insert(make_pair(
fNew, make_pair(i - 1, j - 1)));
// Update the details of this cell
cellDetails[i - 1][j - 1].f = fNew;
cellDetails[i - 1][j - 1].g = gNew;
cellDetails[i - 1][j - 1].h = hNew;
cellDetails[i - 1][j - 1].parent_i = i;
cellDetails[i - 1][j - 1].parent_j = j;
}
}
}

//----------- 7th Successor (South-East)
//------------

// Only process this cell if this is a valid one
if (isValid(i + 1, j + 1) == true) {
// If the destination cell is the same as the
// current successor
if (isDestination(i + 1, j + 1, dest) == true) {
// Set the Parent of the destination cell
cellDetails[i + 1][j + 1].parent_i = i;
cellDetails[i + 1][j + 1].parent_j = j;
printf("The destination cell is found\n");
tracePath(cellDetails, dest);
foundDest = true;
return;
}

// If the successor is already on the closed
// list or if it is blocked, then ignore it.
// Else do the following
else if (closedList[i + 1][j + 1] == false
&& isUnBlocked(grid, i + 1, j + 1)
== true) {
gNew = cellDetails[i][j].g + 1.414;
hNew = calculateHValue(i + 1, j + 1, dest);
fNew = gNew + hNew;

// If it isn’t on the open list, add it to
// the open list. Make the current square
// the parent of this square. Record the
// f, g, and h costs of the square cell
// OR
// If it is on the open list already, check
// to see if this path to that square is
// better, using 'f' cost as the measure.
if (cellDetails[i + 1][j + 1].f == FLT_MAX
|| cellDetails[i + 1][j + 1].f > fNew) {
openList.insert(make_pair(
fNew, make_pair(i + 1, j + 1)));

// Update the details of this cell
cellDetails[i + 1][j + 1].f = fNew;
cellDetails[i + 1][j + 1].g = gNew;
cellDetails[i + 1][j + 1].h = hNew;
cellDetails[i + 1][j + 1].parent_i = i;
cellDetails[i + 1][j + 1].parent_j = j;
}
}
}

//----------- 8th Successor (South-West)
//------------

// Only process this cell if this is a valid one
if (isValid(i + 1, j - 1) == true) {
// If the destination cell is the same as the
// current successor
if (isDestination(i + 1, j - 1, dest) == true) {
// Set the Parent of the destination cell
cellDetails[i + 1][j - 1].parent_i = i;
cellDetails[i + 1][j - 1].parent_j = j;
printf("The destination cell is found\n");
tracePath(cellDetails, dest);
foundDest = true;
return;
}

// If the successor is already on the closed
// list or if it is blocked, then ignore it.
// Else do the following
else if (closedList[i + 1][j - 1] == false
&& isUnBlocked(grid, i + 1, j - 1)
== true) {
gNew = cellDetails[i][j].g + 1.414;
hNew = calculateHValue(i + 1, j - 1, dest);
fNew = gNew + hNew;

// If it isn’t on the open list, add it to
// the open list. Make the current square
// the parent of this square. Record the
// f, g, and h costs of the square cell
// OR
// If it is on the open list already, check
// to see if this path to that square is
// better, using 'f' cost as the measure.
if (cellDetails[i + 1][j - 1].f == FLT_MAX
|| cellDetails[i + 1][j - 1].f > fNew) {
openList.insert(make_pair(
fNew, make_pair(i + 1, j - 1)));

// Update the details of this cell
cellDetails[i + 1][j - 1].f = fNew;
cellDetails[i + 1][j - 1].g = gNew;
cellDetails[i + 1][j - 1].h = hNew;
cellDetails[i + 1][j - 1].parent_i = i;
cellDetails[i + 1][j - 1].parent_j = j;
}
}
}
}

// When the destination cell is not found and the open
// list is empty, then we conclude that we failed to
// reach the destination cell. This may happen when the
// there is no way to destination cell (due to
// blockages)
if (foundDest == false)
printf("Failed to find the Destination Cell\n");

return;
}



// Driver program to test above function
int main(int argc, char** argv)
{

  // Publish gps_path message for the pure pursuit code to pick up
  ros::init(argc, argv, "a_star");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
  double ygn = (lat_goal - lat_start)*111320; // meters north of robot
  double xgn = (long_goal - long_start)*40075000*
    cos(lat_start*M_PI/180)/360; //meters east of robot
  ROS_INFO("meters north = %f, meters east = %f", ygn, xgn);
  //define the grid; each row and col represents 1m
  ROW = roundup(ygn);
  COL = roundup(xgn);
  ROS_INFO("Rows = %d, Columns = %d", ROW, COL);
  vector<vector<int>> grid( ROW, vector<int> (COL, 1));
  cout << "Grid\n" << endl;

  //Add obstacles
  for (int j = 1; j < COL; j++)
  {
    grid[2][j] = 0;
    grid[3][j] = 0;
  }
  for (int j = 0; j < COL - 1; j++)
  {
    grid[6][j] = 0;
    grid[7][j] = 0;
  }
  /*
  // Print array as grid
  for (int i = 0; i < ROW; ++i)
  {
    for(int j = 0; j < COL; ++j)
    {
      cout << grid[i][j];
    }
    cout << '\n';
  }
  cout << endl;
  */
  Pair src;
  Pair dest;
  // Define start and end points in grid for search algorithm
  // Also define lat and long baseline for future calculations
  if (xgn >= 0 && ygn >= 0)
  {
  src = make_pair(ROW-1, 0);
  dest = make_pair(0, COL-1);
  lat_floor = lat_goal;
  long_floor = long_start;
  }
  else if (xgn < 0 && ygn < 0)
  {
  src = make_pair(0, COL-1);
  dest = make_pair(ROW-1, 0);
  lat_floor = lat_start;
  long_floor = long_goal;
  }
  else if (xgn >= 0 && ygn <= 0)
  {
  src = make_pair(0, 0);
  dest = make_pair(ROW-1, COL-1);
  lat_floor = lat_start;
  long_floor = long_start;
  }
  else if (xgn <= 0 && ygn >= 0)
  {
  src = make_pair(ROW-1, COL-1);
  dest = make_pair(0, 0);
  lat_floor = lat_goal;
  long_floor = long_goal;
  }
  //ROS_INFO("source.first %d, ROW = %d, source.second %d, COL = %d", 
  //  src.first, ROW, src.second, COL);
  ros_path_publisher = n.advertise<nav_msgs::Path>("/gps_path", 1000);
  aStarSearch(grid, src, dest);

  ros::spinOnce();
  loop_rate.sleep();
  }
  return (0);
}
