#include "Wall.h"



Wall::Wall():leftWall(10,0),rightWall(10,0),frontWall(10,0), placeInLoopForWalls(0){}

double average(const std::vector<int>& v)
{
    if (v.empty()) return 0.0;

    long long sum = 0;
    for (int x : v) sum += x;
    return static_cast<double>(sum) / v.size();
}

void Wall::checkForWall() {
  if(average(leftWall) < 5)
  {
    Serial.println("hitting left wall");
  }
  if(average(rightWall) < 5)
  {
    Serial.println("hitting right wall");
  }
  if(average(frontWall) < 20)
  {
    Serial.println("hitting front wall!!!");
  }
}

void Wall::updateWalls(int left, int right , int front)
{
  leftWall[placeInLoopForWalls] = left;
  rightWall[placeInLoopForWalls] = right;
  frontWall[placeInLoopForWalls] = front;
  placeInLoopForWalls++ ; 
  if(placeInLoopForWalls>=10)
  {
    placeInLoopForWalls=0;
  }
}
