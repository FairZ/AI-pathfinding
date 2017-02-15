#include"Node.h"

Node::Node(int posX, int posY){ //run by a nested for loop which sets up the node's position values in the grid
	m_posX = posX;
	m_posY = posY;
	m_onList = false;
	m_parent = nullptr;
	m_isGoal = false;
	m_isStart = false;
	m_isWall = false;
	m_heuristicWeight = 0;
	m_actualWeight = 0;
}


void Node::SetOnList()
{
	m_onList = true;
}

void Node::SetParent(Node* _parent)
{
	m_parent = _parent;
}

void Node::SetIsGoal()
{
	m_isGoal = true;
}

void Node::SetIsStart()
{
	m_isStart = true;
}

void Node::SetIsNotStart()
{
	m_isStart=false;
}


void Node::SetIsWall()
{
	m_isWall = true;
}

void Node::SetIsNotWall()
{
	m_isWall = false;
}

void Node::SetHeuristicWeight(int _weight)
{
	m_heuristicWeight = _weight;
}

void Node::SetActualWeight(int _weight)
{
	m_actualWeight = _weight;
}

void Node::SetFWeight(int _weight)
{
	m_fWeight = _weight;
}


void Node::Reset()
{
	m_onList = false;
	m_parent = nullptr;
	m_isGoal = false;
	m_isStart = false;
	m_heuristicWeight = 0;
	m_actualWeight = 0;
}

void Node::Draw(BITMAP* _buffer)
{
	if(m_isWall)//allegro draw calls based on the type of node being drawn
	{
		rectfill(_buffer,(m_posX*GRID_SCALE),(m_posY*GRID_SCALE),(m_posX*GRID_SCALE)+(GRID_SCALE),(m_posY*GRID_SCALE)+(GRID_SCALE), 0);
	}
	else if (m_isGoal)
	{
		rectfill(_buffer,(m_posX*GRID_SCALE),(m_posY*GRID_SCALE),(m_posX*GRID_SCALE)+(GRID_SCALE),(m_posY*GRID_SCALE)+(GRID_SCALE), 0x00ff00);
	}
	else
	{
		rectfill(_buffer,(m_posX*GRID_SCALE),(m_posY*GRID_SCALE),(m_posX*GRID_SCALE)+(GRID_SCALE),(m_posY*GRID_SCALE)+(GRID_SCALE), 0xff0000);
	}

}

void Node::DrawLink(BITMAP* _buffer)
{
	if (m_parent != nullptr)
	{
		line(_buffer,((m_parent->m_posX)*GRID_SCALE)+(GRID_SCALE/2),((m_parent->m_posY)*GRID_SCALE)+(GRID_SCALE/2),(m_posX*GRID_SCALE)+(GRID_SCALE/2),(m_posY*GRID_SCALE)+(GRID_SCALE/2),0xFF0000);
	}
}