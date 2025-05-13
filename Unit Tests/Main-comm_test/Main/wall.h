#ifndef WALL_H
#define WALL_H

#include <vector>
#include <Arduino.h> //for prints can rempve later

class Wall {
  private:
    std::vector<int> leftWall;
    std::vector<int> rightWall;
    std::vector<int> frontWall;
    int placeInLoopForWalls;

  public:
    Wall();
    void checkForWall();
    void updateWalls(int left, int right , int front);

};


#endif