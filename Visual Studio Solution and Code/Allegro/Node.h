#ifndef _NODE_H_
#define _NODE_H_

#include<allegro.h>

//definitions set up to allow easy changing of grid size (before compiling)
#define GRID_SIZE 100				
#define GRID_SCALE 500/GRID_SIZE

class Node
{
private:

	bool m_onList;
	bool m_isStart;
	bool m_isGoal;
	bool m_isWall;

	int m_posX;
	int m_posY;	
	int m_heuristicWeight;
	int m_actualWeight; //g() weight
	int m_fWeight;

	Node* m_parent;

public:

	Node();
	Node(int posX, int posY);

	int GetPosX(){return m_posX;}
	int GetPosY(){return m_posY;}
	bool GetOnList(){return m_onList;}
	bool GetIsGoal(){return m_isGoal;}
	bool GetIsStart(){return m_isStart;}
	bool GetIsWall(){return m_isWall;}
	int GetHeuristicWeight(){return m_heuristicWeight;}
	int GetActualWeight(){return m_actualWeight;}
	int GetFWeight(){return m_fWeight;}
	Node* GetParent(){return m_parent;}

	void SetOnList();
	void SetParent(Node* _parent);
	void SetIsGoal();
	void SetIsStart();
	void SetIsNotStart();
	void SetIsWall();
	void SetIsNotWall();
	void SetHeuristicWeight(int _weight);
	void SetActualWeight(int _weight);
	void SetFWeight(int _weight);

	//Reset returns node values to default except whether the node is an obstacle and it's position
	void Reset(); 

	void Draw(BITMAP* _buffer); 
	//DrawLink draws a line from the current node to it's parent
	void DrawLink(BITMAP* _buffer); 
};


#endif