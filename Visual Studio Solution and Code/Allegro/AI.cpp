#include "AI.h"

Agent::Agent()
{
	m_finalPoint = nullptr;
};

int Agent::ManhattanHeuristic(int _currentX, int _currentY, int _goalX, int _goalY)
{
	//return the absolute difference in x values plus the difference in y values between the current position and the goal position to give a distance regardless of direction
	return (abs(_goalX - _currentX)+abs(_goalY - _currentY));
}

void Agent::BreadthFirstSearch(Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos)
{
	std::vector<Node*> open;

	Node* current;
	Node* neighbour;

	//give goal node appropriate values
	_grid[_goalXPos][_goalYPos]->SetIsGoal();

	//give start node appropriate values and push it to the open list
	_grid[_startXPos][_startYPos]->SetIsStart();
	_grid[_startXPos][_startYPos]->SetOnList();	
	open.push_back(_grid[_startXPos][_startYPos]);

	while(open.size() > 0)
	{
		//assign current to first position of open
		current = _grid[open[0]->GetPosX()][open[0]->GetPosY()];

		//if the current position is the goal position stop searching
		if(current->GetIsGoal())
		{
			break;
		}
		//else expand to the neighbours
		else
		{	
			//these 2 for loops give the position offsets of the neighbours
			for(int y= -1; y <= 1;y++)
			{
				for(int x= -1; x <= 1; x++)
				{
					//if the neighbour would have no offset (would be the current position) do not continue to evaluate it and therefore do not push to open
					if(!((x==0)&&(y==0)))
					{
						//if the neighbour's position would be outside of the bounds do not continue to evaluate it and therefore do not push to open
						if((current->GetPosX()+x >=0) &&(current->GetPosX()+x <GRID_SIZE) &&(current->GetPosY()+y >=0) &&(current->GetPosY()+y <GRID_SIZE))
						{
							//if the nodes inbetween a diagonol expansion are walls do not continue to expand the node so that the path cannot go through 1 thick diagonol lines
							if((_grid[current->GetPosX()+x][current->GetPosY()]->GetIsWall())&&(_grid[current->GetPosX()][current->GetPosY()+y]->GetIsWall()))
							{
								continue;
							}
							//assign neighbour to the neighbour currently being evaluated
							neighbour = _grid[current->GetPosX()+x][current->GetPosY()+y];

							//if the neighbour is already on a list do not continue to evaluate it and therefore do not push to open
							if(!(neighbour->GetOnList()))
							{
								//if the neighbour is a wall do not continue to evaluate it and therefore do not push to open
								if(!(neighbour->GetIsWall()))
								{
									//if all checks have been passed give appropriate values to the neighbour and push it to open
									neighbour->SetOnList();
									neighbour->SetParent(current);
									open.push_back(neighbour);
								}
							}
						}
					}
				}
			}
		}
		//remove the expanded node from open
		open.erase(open.begin());
	}
	//set the final point of the path to the last node that was expanded
	m_finalPoint = current;
}

//the below algorithms will only have new or changed sections of code commented
//all other code remains the same with the same functions

void Agent::DepthFirstSearch(Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos)
{
	std::vector<Node*> open;

	Node* current;
	Node* neighbour;

	_grid[_goalXPos][_goalYPos]->SetIsGoal();

	_grid[_startXPos][_startYPos]->SetIsStart();
	_grid[_startXPos][_startYPos]->SetOnList();	
	open.push_back(_grid[_startXPos][_startYPos]);


	while(open.size() > 0)
	{
		//assign current to final position of open (all expanded nodes are pushed to the back of open therefore this is depth first)
		current = _grid[open.back()->GetPosX()][open.back()->GetPosY()];

		open.pop_back();

		if((current->GetPosX() == _goalXPos)&&(current->GetPosY() == _goalYPos))
		{
			break;
		}
		else
		{	
			for(int y= -1; y <= 1;y++)
			{
				for(int x= -1; x <= 1; x++)
				{
					if(!((x==0)&&(y==0)))
					{
						if((current->GetPosX()+x >=0) &&(current->GetPosX()+x <GRID_SIZE) &&(current->GetPosY()+y >=0) &&(current->GetPosY()+y <GRID_SIZE))
						{
							if((_grid[current->GetPosX()+x][current->GetPosY()]->GetIsWall())&&(_grid[current->GetPosX()][current->GetPosY()+y]->GetIsWall()))
							{
								continue;
							}
							neighbour = _grid[current->GetPosX()+x][current->GetPosY()+y];
							if(!(neighbour->GetOnList()))
							{
								if(!(neighbour->GetIsWall()))
								{
									neighbour->SetOnList();
									neighbour->SetParent(current);
									open.push_back(neighbour);
								}
							}
						}
					}
				}
			}
		}
	}
	m_finalPoint = current;
}

void Agent::BestFirstSearch(Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos)
{
	std::vector<Node*> open;

	Node* current;
	Node* neighbour;

	_grid[_goalXPos][_goalYPos]->SetIsGoal();

	_grid[_startXPos][_startYPos]->SetIsStart();
	_grid[_startXPos][_startYPos]->SetHeuristicWeight(ManhattanHeuristic(_startXPos,_startYPos,_goalXPos,_goalYPos)*10);
	_grid[_startXPos][_startYPos]->SetOnList();	
	open.push_back(_grid[_startXPos][_startYPos]);


	while(open.size() > 0)
	{
		current = _grid[open[0]->GetPosX()][open[0]->GetPosY()];

		open.erase(open.begin());

		if((current->GetPosX() == _goalXPos)&&(current->GetPosY() == _goalYPos))
		{
			break;
		}
		else
		{	
			for(int y= -1; y <= 1;y++)
			{
				for(int x= -1; x <= 1; x++)
				{	
					if(!((x==0)&&(y==0)))
					{
						if((current->GetPosX()+x >=0) &&(current->GetPosX()+x <GRID_SIZE) &&(current->GetPosY()+y >=0) &&(current->GetPosY()+y <GRID_SIZE))
						{
							if((_grid[current->GetPosX()+x][current->GetPosY()]->GetIsWall())&&(_grid[current->GetPosX()][current->GetPosY()+y]->GetIsWall()))
							{
								continue;
							}
							neighbour = _grid[current->GetPosX()+x][current->GetPosY()+y];
							if(!(neighbour->GetOnList()))
							{
								if(!(neighbour->GetIsWall()))
								{
									neighbour->SetOnList();
									neighbour->SetParent(current);
									//manhattan distance from this point to end point is set as the heuristic weight
									neighbour->SetHeuristicWeight(ManhattanHeuristic(neighbour->GetPosX(),neighbour->GetPosY(),_goalXPos,_goalYPos));
									//this next if/else statement places the expanded neighbour into the correct position in the open list based on it's heuristic value
									//this causes the open list to be correctly ordered
									if (open.size() > 0)
									{
										bool inserted = false;
										for (int i = 0; i < open.size(); i++)
										{
											if (neighbour->GetHeuristicWeight() < open[i]->GetHeuristicWeight())
											{
												open.insert(open.begin()+i,neighbour);
												inserted = true;
												break;
											}
										}
										if (!inserted)
										{
											open.push_back(neighbour);	
										}
									}
									else
									{
										open.push_back(neighbour);
									}
								}
							}
						}
					}
				}
			}
		}
	}

	m_finalPoint = current;

}

void Agent::AStarSearch(Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos)
{
	std::vector<Node*> open;
	std::vector<Node*> closed;

	Node* current;
	Node* neighbour;

	//variable which changes based on the direction of expansion
	int weight = 0;

	_grid[_goalXPos][_goalYPos]->SetIsGoal();

	_grid[_startXPos][_startYPos]->SetIsStart();
	_grid[_startXPos][_startYPos]->SetHeuristicWeight(ManhattanHeuristic(_startXPos,_startYPos,_goalXPos,_goalYPos)*10);
	_grid[_startXPos][_startYPos]->SetActualWeight(0);
	_grid[_startXPos][_startYPos]->SetFWeight(_grid[_startXPos][_startYPos]->GetHeuristicWeight());
	_grid[_startXPos][_startYPos]->SetOnList();
	open.push_back(_grid[_startXPos][_startYPos]);

	while(open.size() > 0)
	{
		current = _grid[open[0]->GetPosX()][open[0]->GetPosY()];
		closed.push_back(current);
		open.erase(open.begin());
		
		if((current->GetPosX() == _goalXPos)&&(current->GetPosY() == _goalYPos))
		{
			break;
		}
		else
		{	
			for(int y= -1; y <= 1;y++)
			{
				for(int x= -1; x <= 1; x++)
				{	
					if(!((x==0)&&(y==0)))
					{
						if((current->GetPosX()+x >=0) &&(current->GetPosX()+x <GRID_SIZE) &&(current->GetPosY()+y >=0) &&(current->GetPosY()+y <GRID_SIZE))
						{
							if((_grid[current->GetPosX()+x][current->GetPosY()]->GetIsWall())&&(_grid[current->GetPosX()][current->GetPosY()+y]->GetIsWall()))
							{
								continue;
							}
							neighbour = _grid[current->GetPosX()+x][current->GetPosY()+y];
							//if the expansion is diagonol the weight is 14 otherwise it is 10
							if((x!=0)&&(y!=0))
							{
								weight = 14;
							}
							else
							{
								weight = 10;
							}
							if(!(neighbour->GetIsWall()))
							{
								if(!(neighbour->GetOnList()))
								{
									neighbour->SetOnList();
									neighbour->SetParent(current);
									//set the heuristic, actual (g) weight, and f weight based on the current expansion
									neighbour->SetHeuristicWeight(ManhattanHeuristic(neighbour->GetPosX(),neighbour->GetPosY(),_goalXPos,_goalYPos)*10);
									neighbour->SetActualWeight(current->GetActualWeight() + weight);
									neighbour->SetFWeight(neighbour->GetActualWeight()+(neighbour->GetHeuristicWeight()));

									if (open.size() > 0)
									{
										bool inserted = false;
										for (int i = 0; i < open.size(); i++)
										{
											if (neighbour->GetFWeight() < open[i]->GetFWeight())
											{
												open.insert(open.begin()+i,neighbour);
												inserted = true;
												break;
											}

										}
										if (!inserted)
										{
												open.push_back(neighbour);
												
										}
									}
									else
									{
										open.push_back(neighbour);
									}
								}
								else
								{
									//if the expanded neighbour is already on the list then attempt to improve it's parent/weight					
									
									if((current->GetActualWeight()+weight) < neighbour->GetActualWeight())
									{
										
										bool inOpen = false;
										neighbour->SetActualWeight(current->GetActualWeight() + weight);
										neighbour->SetFWeight(neighbour->GetActualWeight() + neighbour->GetHeuristicWeight());
										neighbour->SetParent(current);
										//check if the expanded node is in open or closed and relocate them to the correct position in open
										for(unsigned int i = 0; i<open.size(); i++)
										{
											if(open[i] ==  neighbour)
											{
												inOpen = true;
												open.erase(open.begin()+i);
												bool inserted = false;
												for (int j = 0; j < open.size(); j++)
												{
													if (neighbour->GetFWeight() < open[j]->GetFWeight())
													{
														open.insert(open.begin()+j,neighbour);
														inserted = true;
														break;
													}
												}
												if (!inserted)
												{
													open.push_back(neighbour);
													
												}
											}
										}
										if(!inOpen)
										{
											for(unsigned int i = 0; i<closed.size(); i++)
											{
												if(closed[i] ==  neighbour)
												{
													closed.erase(closed.begin()+i);
													bool inserted = false;
													for (int j = 0; j < open.size(); j++)
													{
														if (neighbour->GetFWeight() < open[j]->GetFWeight())
														{
															open.insert(open.begin()+j,neighbour);
															inserted = true;
															break;
														}
													}
													if (!inserted)
													{
														open.push_back(neighbour);
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	m_finalPoint = current;

}

void Agent::DijkstraSearch(Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos)
{
	std::vector<Node*> open;
	std::vector<Node*> closed;

	Node* current;
	Node* neighbour;

	int weight = 0;

	_grid[_goalXPos][_goalYPos]->SetIsGoal();

	_grid[_startXPos][_startYPos]->SetIsStart();
	_grid[_startXPos][_startYPos]->SetActualWeight(0);
	_grid[_startXPos][_startYPos]->SetOnList();
	open.push_back(_grid[_startXPos][_startYPos]);


	while(open.size() > 0)
	{

		current = _grid[open[0]->GetPosX()][open[0]->GetPosY()];
		closed.push_back(current);

		open.erase(open.begin());
		

		if((current->GetPosX() == _goalXPos)&&(current->GetPosY() == _goalYPos))
		{
			break;
		}
		else
		{	
			for(int y= -1; y <= 1;y++)
			{
				for(int x= -1; x <= 1; x++)
				{
					
					if(!((x==0)&&(y==0)))
					{
						if((current->GetPosX()+x >=0) &&(current->GetPosX()+x <GRID_SIZE) &&(current->GetPosY()+y >=0) &&(current->GetPosY()+y <GRID_SIZE))
						{
							if((_grid[current->GetPosX()+x][current->GetPosY()]->GetIsWall())&&(_grid[current->GetPosX()][current->GetPosY()+y]->GetIsWall()))
							{
								continue;
							}
							neighbour = _grid[current->GetPosX()+x][current->GetPosY()+y];
							if((x!=0)&&(y!=0))
							{
								weight = 14;
							}
							else
							{
								weight = 10;
							}
							if(!(neighbour->GetIsWall()))
							{
								if(!(neighbour->GetOnList()))
								{
									neighbour->SetOnList();
									neighbour->SetParent(current);
									//only set actual (g) weight and place it into the open based on it's actual weight
									neighbour->SetActualWeight(current->GetActualWeight() + weight);

									if (open.size() > 0)
									{
										bool inserted = false;
										for (int i = 0; i < open.size(); i++)
										{
											if (neighbour->GetActualWeight() < open[i]->GetActualWeight())
											{
												open.insert(open.begin()+i,neighbour);
												inserted = true;
												break;
											}

										}
										if (!inserted)
										{
												open.push_back(neighbour);
												
										}
									}
									else
									{
										open.push_back(neighbour);
									}
								}
								else
								{
									//if the expanded neighbour is already on the list then attempt to improve it's parent/weight				
									
									if((current->GetActualWeight()+weight) < neighbour->GetActualWeight())
									{
										
										bool inOpen = false;
										neighbour->SetActualWeight(current->GetActualWeight() + weight);
										neighbour->SetParent(current);
										for(unsigned int i = 0; i<open.size(); i++)
										{
											if(open[i] ==  neighbour)
											{
												inOpen = true;
												open.erase(open.begin()+i);
												bool inserted = false;
												for (int j = 0; j < open.size(); j++)
												{
													if (neighbour->GetActualWeight() < open[j]->GetActualWeight())
													{
														open.insert(open.begin()+j,neighbour);
														inserted = true;
														break;
													}
												}
												if (!inserted)
												{
													open.push_back(neighbour);
													
												}
											}
										}
										if(!inOpen)
										{
											for(unsigned int i = 0; i<closed.size(); i++)
											{
												if(closed[i] ==  neighbour)
												{
													closed.erase(closed.begin()+i);
													bool inserted = false;
													for (int j = 0; j < open.size(); j++)
													{
														if (neighbour->GetActualWeight() < open[j]->GetActualWeight())
														{
															open.insert(open.begin()+j,neighbour);
															inserted = true;
															break;
														}
													}
													if (!inserted)
													{
														open.push_back(neighbour);
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

	m_finalPoint = current;

}

std::vector<Node*> Agent::GetPath(int _search, Node* _grid[GRID_SIZE][GRID_SIZE], int _startXPos, int _startYPos, int _goalXPos, int _goalYPos)
{
	//clear the previously stored path
	m_path.clear();
	//perform a search algorithm based on input
	switch(_search)
	{
	case 1:
		BreadthFirstSearch(_grid, _startXPos, _startYPos, _goalXPos, _goalYPos);
		break;
	case 2:
		DepthFirstSearch(_grid, _startXPos, _startYPos, _goalXPos, _goalYPos);
		break;
	case 3:
		BestFirstSearch(_grid, _startXPos, _startYPos, _goalXPos, _goalYPos);
		break;
	case 4:
		AStarSearch(_grid, _startXPos, _startYPos, _goalXPos, _goalYPos);
		break;
	case 5:
		DijkstraSearch(_grid, _startXPos, _startYPos, _goalXPos, _goalYPos);
		break;
	default:
		BestFirstSearch(_grid, _startXPos, _startYPos, _goalXPos, _goalYPos);
		break;
	}

	//if the final point of the path is not the goal node (the path could not reach the goal) set the path to the start position
	//this will cause the program to change goal location and repath
	if((m_finalPoint->GetPosX() != _goalXPos)&&(m_finalPoint->GetPosY() != _goalYPos))
	{
		m_path.push_back(_grid[_startXPos][_startYPos]);
		return m_path;
	}

	//if the final point of the path is the start node cause the progrma to repath (in order to stop the agent from getting irreversibly stuck)
	if((m_finalPoint->GetPosX() == _startXPos)&&(m_finalPoint->GetPosY() == _startYPos))
	{
		m_path.push_back(m_finalPoint);
		return m_path;
	}

	//if neither of the previous errors occured, create a path of nodes and create it
	while(m_finalPoint->GetParent() != nullptr)
	{			
		m_path.push_back(m_finalPoint);
		m_finalPoint = m_finalPoint->GetParent();
	}

	return m_path;
}

bool Agent::Move()
{
	bool finish = false;
	//if the agent hasn't reached the end of the path move it to the next node on said path
	if (m_path.size() > 1)
	{	
		m_currentNode = m_path.back();
		m_path.pop_back();
	}	
	//otherwise move it to the final position and cause the program to repath
	else if (m_path.size() == 1)
	{
		m_currentNode = m_path.back();
		finish = true;
	}
	return finish;
}

void Agent::Draw(BITMAP* _buffer)
{
	m_currentNode->Draw(_buffer);
}

Node* Agent::GetCurrentNode()
{
	return m_currentNode;
}

