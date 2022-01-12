// Demos basics of making a 2D vector array where each entry holds
// multiple data points
// Also demos how to write data to a file
#include "mixed_reality_library/mixed_reality_library.h"

int ROW;
int COL;

using namespace std;

struct cell {
int parent_i, parent_j;
double f, g, h;
};

int print_array(vector<vector<cell>> grid)
{
  // Print out the array
  for (int i = 0; i < ROW; i++)
  {
    for (int j = 0; j < COL; j++)
    {
      grid[i][j].f = 1.02;
      cout << grid[i][j].f << " ";
    }
    cout << endl;
  }
}

int main()
{
  ROW = 3;
  COL = 4;
  vector<vector<cell>> grid( ROW, vector<cell> (COL));
 
  fstream my_file;
  my_file.open("my_file.txt", ios::out);
  cout <<"File created successfully!";
  for (int i = 0; i < 10; i++)
  {
    my_file << "data = " << i << endl;
  }
  my_file.close();
  //print_array(grid);
  return 0;
}
