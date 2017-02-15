#ifndef _AI_H_
#define _AI_H_

#define BREADTH 1
#define DEPTH 2
#define BEST 3
#define ASTAR 4
#define DIJKSTRA 5

#include "Node.h"
#include <allegro.h>
#include <vector>

//agent class to allow for later multiagent pathing and agent behaviours
class Agent 
{
private:
	std::vector<Node*> m_path;
	Node* m_finalPoint;
	Node* m_currentNode;
	int ManhattanHeuristic(int _currentX, int _currentY, int _goalX, int _goalY);
	void BestFirstSearch(Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos);
	void BreadthFirstSearch(Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos);
	void DepthFirstSearch(Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos);
	void AStarSearch(Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos);
	void DijkstraSearch(Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos);
public:
	Agent();
	//GetPath() runs a pathfinding algorithm of choice and returns a vector of nodes which make up the path
	std::vector<Node*> GetPath(int _search, Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos); 
	Node* GetCurrentNode();
	bool Move();
	void Draw(BITMAP* _buffer);
};

#endif