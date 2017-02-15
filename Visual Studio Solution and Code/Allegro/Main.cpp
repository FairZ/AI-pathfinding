#include "AI.h"
#include <random>
#include <fstream>
#include <sstream>

//allegro initialisations
int init();

//frame rate limiter
void Ticker(void);

//allegro text drawing fro the RHS of the screen
void DrawText(BITMAP* _buffer, int _pathingType);

void SaveMap(Node* _grid[GRID_SIZE][GRID_SIZE]);
void LoadMap(Node* _grid[GRID_SIZE][GRID_SIZE], int& _mapNumber);

volatile int TICK = 0;

int main(void)
{
	//if the allegro initialisation worked run the program
	if (init() == 0)
	{

		std::vector<Node*> path;
		Node* grid[GRID_SIZE][GRID_SIZE];
		Agent agent;
		bool drawGridlines = true;
		bool drawPathLine = true;
		bool reset = false;
		bool wallDrawing = false;
		int pathingType = ASTAR;
		int frameTime = 32;
		int startX,startY,goalX,goalY;
		int mapNumber = 1;
		BITMAP* buffer = create_bitmap(1000,1000);

		//begin the frame limiter
		install_int(Ticker,1);

		show_mouse(screen);

		srand(time(NULL));
		
		//clear screen and buffer
		clear_to_color(screen, 0xffffff);
		clear_to_color(buffer, 0xffffff);

		//create the grid of nodes
		for (int y = 0; y < GRID_SIZE; y++)
		{
			for (int x = 0; x < GRID_SIZE; x++)
			{
				grid[x][y] = new Node(x,y);
			}
		}

		startX = rand()%GRID_SIZE;
		startY = rand()%GRID_SIZE;
		goalX = rand()%GRID_SIZE;
		goalY = rand()%GRID_SIZE;

		//find an initial path
		path = agent.GetPath(pathingType,grid,startX,startY,goalX,goalY);

		//enter the game loop
		while (!key[KEY_ESC])
		{

			clear_to_color(buffer, 0xffffff);

			#pragma region Input Handling Region
			//this region contains all input handling including keyboard and mouse

			if (key[KEY_G])
			{
				drawGridlines = !drawGridlines;
				//the while loops are used to pause the game in order to stop the toggle from repeatedly switching
				while(key[KEY_G])
				{
				}
			}

			if (key[KEY_P])
			{
				drawPathLine = !drawPathLine;
				while(key[KEY_P])
				{
				}
			}

			if (key[KEY_W])
			{
				wallDrawing = !wallDrawing;
				while(key[KEY_W])
				{
				}
				//reset the pathing start position when switching between dynamic update and pathing once the end of a path is reached
				startX=agent.GetCurrentNode()->GetPosX();
				startY=agent.GetCurrentNode()->GetPosY();
			}

			if (key[KEY_R])
			{
				reset = true;
				while(key[KEY_R])
				{
				}
			}

			if (key[KEY_S])
			{
				SaveMap(grid);
				while(key[KEY_S])
				{
				}
			}

			if(key[KEY_L])
			{
				mapNumber += 1;
				LoadMap(grid, mapNumber);
				while(key[KEY_L])
				{
				}

			}

			if(key[KEY_LEFT])
			{
				//if the frame timer is at 64ms do not increase it further in order to not make the program too slow
				if (frameTime != 64)
				{
					frameTime = frameTime*2;
				}
				while(key[KEY_LEFT])
				{
				}
			}

			if(key[KEY_RIGHT])
			{
				//if the frame timer is at 8ms do not decreae it further in order to not make the program too fast
				if (frameTime != 8)
				{
					frameTime = frameTime/2;
				}
				while(key[KEY_RIGHT])
				{
				}
			}

			if (key[KEY_1])
			{
				pathingType = BREADTH;
				//create a new path when the path type changes
				startX=agent.GetCurrentNode()->GetPosX();
				startY=agent.GetCurrentNode()->GetPosY();
				for (int y = 0; y < GRID_SIZE; y++)
				{
					for (int x = 0; x < GRID_SIZE; x++)
					{
						grid[x][y]->Reset();
					}
				}
				path = agent.GetPath(pathingType,grid,startX,startY,goalX,goalY);
			}
			if (key[KEY_2])
			{
				pathingType = DEPTH;
				startX=agent.GetCurrentNode()->GetPosX();
				startY=agent.GetCurrentNode()->GetPosY();
				for (int y = 0; y < GRID_SIZE; y++)
				{
					for (int x = 0; x < GRID_SIZE; x++)
					{
						grid[x][y]->Reset();
					}
				}
				path = agent.GetPath(pathingType,grid,startX,startY,goalX,goalY);
			}
			if (key[KEY_3])
			{
				pathingType = BEST;
				startX=agent.GetCurrentNode()->GetPosX();
				startY=agent.GetCurrentNode()->GetPosY();
				for (int y = 0; y < GRID_SIZE; y++)
				{
					for (int x = 0; x < GRID_SIZE; x++)
					{
						grid[x][y]->Reset();
					}
				}
				path = agent.GetPath(pathingType,grid,startX,startY,goalX,goalY);
			}
			if (key[KEY_4])
			{
				pathingType = ASTAR;
				startX=agent.GetCurrentNode()->GetPosX();
				startY=agent.GetCurrentNode()->GetPosY();
				for (int y = 0; y < GRID_SIZE; y++)
				{
					for (int x = 0; x < GRID_SIZE; x++)
					{
						grid[x][y]->Reset();
					}
				}
				path = agent.GetPath(pathingType,grid,startX,startY,goalX,goalY);
			}
			if (key[KEY_5])
			{
				pathingType = DIJKSTRA;

				startX=agent.GetCurrentNode()->GetPosX();
				startY=agent.GetCurrentNode()->GetPosY();
				for (int y = 0; y < GRID_SIZE; y++)
				{
					for (int x = 0; x < GRID_SIZE; x++)
					{
						grid[x][y]->Reset();
					}
				}
				path = agent.GetPath(pathingType,grid,startX,startY,goalX,goalY);
			}

			//if the mouse is clicked while in drawing mode 
			if((mouse_b & 1)&&(wallDrawing == true))
			{
				if (mouse_x<500)
				{
					grid[mouse_x/5][mouse_y/5]->SetIsWall();
				}
			}
			if((mouse_b & 2)&&(wallDrawing == true))
			{
				if (mouse_x<500)
				{
					grid[mouse_x/5][mouse_y/5]->SetIsNotWall();
				}
			}
			#pragma endregion

			//while in wall drawing mode, update the path each frame, therefore making it dynamic
			if (wallDrawing)
			{
				for (int y = 0; y < GRID_SIZE; y++)
				{
					for (int x = 0; x < GRID_SIZE; x++)
					{
						grid[x][y]->Reset();
					}
				}
				path = agent.GetPath(pathingType,grid,startX,startY,goalX,goalY);
			}

			//move the agent and if it has reached the end of its path create a new path
			if (agent.Move())
			{
				for (int y = 0; y < GRID_SIZE; y++)
				{
					for (int x = 0; x < GRID_SIZE; x++)
					{
						grid[x][y]->Reset();
					}
				}
				startX=agent.GetCurrentNode()->GetPosX();
				startY=agent.GetCurrentNode()->GetPosY();		

				do
				{
					goalX= rand()%GRID_SIZE;
					goalY= rand()%GRID_SIZE;
					
				}while(grid[goalX][goalY]->GetIsWall());
				
				path = agent.GetPath(pathingType,grid,startX,startY,goalX,goalY);

			}

			if (wallDrawing)
			{
				startX=agent.GetCurrentNode()->GetPosX();
				startY=agent.GetCurrentNode()->GetPosY();
			}

			//draw the agent
			agent.Draw(buffer);

			//draw other neccessary nodes
			for (int y = 0; y < GRID_SIZE; y++)
			{
				for (int x = 0; x < GRID_SIZE; x++)
				{
					if ((grid[x][y]->GetIsWall())||(grid[x][y]->GetIsGoal()))
					{
						grid[x][y]->Draw(buffer);
					}
				}
			}

			//draw gridlines if they are enabled
			if(drawGridlines)
			{
				for(int i = 0; i < GRID_SIZE+1; i++)
				{
					line(buffer,0,i*GRID_SCALE,500,i*GRID_SCALE,0);
					line(buffer,i*GRID_SCALE,0,i*GRID_SCALE,500,0);
				} 
			}

			//draw the path line if it is enabled
			if(drawPathLine)
			{
				for(unsigned int i = 0; i < path.size(); i++)
				{
					path[i]->DrawLink(buffer);
				}
			}

			DrawText(buffer, pathingType);

			//push the buffer frame to the screen
			blit(buffer, screen, 0, 0, 0, 0, 1000, 1000);
			
			//if the grid needs to be reset, reset it
			if (reset)
			{
				for (int y = 0; y < GRID_SIZE; y++)
				{
					for (int x = 0; x < GRID_SIZE; x++)
					{
						grid[x][y]->Reset();
						grid[x][y]->SetIsNotWall();
						reset = false;
					}
				}
			}
			
			//if the time taken fro the previous frame was less than the cap, sleep for the difference in time
			if(TICK < frameTime)
			{
				_sleep(frameTime-TICK);
			}

			TICK = 0;
			
		}

	}

	return 0;
}
END_OF_MAIN()


void DrawText(BITMAP* _buffer, int _pathingType)
{
	textout_centre_ex(_buffer,font,"Controls",750,50,0,0xFFFFFF);
	textout_centre_ex(_buffer,font,"Press G to toggle gridlines",750,100,0,0xFFFFFF);
	textout_centre_ex(_buffer,font,"Press P to toggle path line",750,125,0,0xFFFFFF);
	textout_centre_ex(_buffer,font,"Press W to toggle wall drawing and pathing per frame",750,150,0,0xFFFFFF);
	textout_centre_ex(_buffer,font,"(left click to draw, right click to erase)",750,160,0,0xFFFFFF);
	textout_centre_ex(_buffer,font,"Press R to clear walls",750,185,0,0xFFFFFF);
	textout_centre_ex(_buffer,font,"Press 1-5 to change pathing algorithm",750,205,0,0xFFFFFF);
	switch (_pathingType)
	{
	case 1:
		textout_centre_ex(_buffer,font,"The current algorithm is Breadth First",750,215,0,0xFFFFFF);
		break;
	case 2:
		textout_centre_ex(_buffer,font,"The current algorithm is Depth First",750,215,0,0xFFFFFF);
		break;
	case 3:
		textout_centre_ex(_buffer,font,"The current algorithm is Best First",750,215,0,0xFFFFFF);
		break;
	case 4:
		textout_centre_ex(_buffer,font,"The current algorithm is A*",750,215,0,0xFFFFFF);
		break;
	case 5:
		textout_centre_ex(_buffer,font,"The current algorithm is Dijkstra's",750,215,0,0xFFFFFF);
		break;
	}

	textout_centre_ex(_buffer,font,"Press S to save current map",750,240,0,0xFFFFFF);
	textout_centre_ex(_buffer,font,"Press L to cycle through maps",750,265,0,0xFFFFFF);
	textout_centre_ex(_buffer,font,"Press Left/Right arrow keys to increase/decrease framerate cap",750,290,0,0xFFFFFF);
	textout_centre_ex(_buffer,font,"Press Esc to exit",750,400,0,0xFFFFFF);
}


void SaveMap(Node* _grid[GRID_SIZE][GRID_SIZE])
{
	std::fstream map;
	std::stringstream string;
	int fileIterator = 1;
	bool fileOpen = true;

	//check if a file of the desired name already exists, if not change the name
	while(fileOpen)
	{
		string << "Map #" << fileIterator << ".txt";
		map.open(string.str().c_str(),std::ios_base::in | std::ios_base::out);
		if (map.is_open())
		{
			map.close();
			fileIterator++;
			string.str("");
		}
		else
		{
			fileOpen = false;
		}
	}

	//create a file of the new name
	map.open(string.str().c_str(),std::ios_base::out);

	//clear the string
	string.str("");

	//step through the grid, if the node is a wall send a 1 to the string, otherwise send a 0 and at the end of each line send an endline
	for(int y=0; y<GRID_SIZE; y++)
	{
		for (int x=0; x<GRID_SIZE; x++)
		{
			if (_grid[x][y]->GetIsWall())
			{
				string << "1";
			}
			else
			{
				string << "0";
			}
		}
		string << std::endl;
	}

	//write the string to the file
	map << string.str().c_str();

	//close the stream
	map.close();
	//clear the string
	string.str("");
}


void LoadMap(Node* _grid[GRID_SIZE][GRID_SIZE], int& _mapNumber)
{
	std::fstream map;
	std::stringstream mapName;
	//open the file of the correct name
	mapName << "Map #" << _mapNumber << ".txt";
	map.open(mapName.str().c_str(),std::ios_base::in);
	//if the file doesn't exist return to the first map
	if (!map.is_open())
	{
		map.close();
		_mapNumber = 1;
		mapName.str("");
		mapName << "Map #" << _mapNumber << ".txt";
		map.open(mapName.str().c_str(),std::ios_base::in);
	}
	//step through the file and grid and assign the correct values to m_isWall based on the data in the file
	for (int y = 0; y < GRID_SIZE; y++)
	{
		for (int x = 0; x < GRID_SIZE; x++)
		{
			switch(map.get())
			{
			case '1':
				_grid[x][y]->SetIsWall();
				break;
			case '0':
				_grid[x][y]->SetIsNotWall();
				break;
			default:
				x--;
				break;
			}

		}
	}
}


int init()
{
	int retval = 0;
	int ret;
		if (allegro_init() != 0)
		{
			retval = -1;
		}
		else
		{
			install_keyboard();
			install_timer();
			install_mouse();
			set_color_depth(32);
			ret = set_gfx_mode(GFX_AUTODETECT_WINDOWED, 1000, 500, 0, 0);
			if (ret != 0)
			{
				allegro_message(allegro_error);
				retval = -2;
			}
		}
	return retval;
}


void Ticker(void)
{
	//increase the TICK value every millisecond
	TICK++;
}
END_OF_FUNCTION(Ticker);
