#include "GridCell.h"

 ////////////////////////////
 ////////////////////////////
 /////////CELL///////////////
 ////////////////////////////
 ////////////////////////////

GridCell::GridCell(int i, int j)
{
	m_gridproperty = FREE;
	m_cellX = i;
	m_cellY = j;
	c_height = heightCell;
	c_width = widthCell;
	c_size = c_width + c_height;
	cellUnchecked = true;
	numberOfPatchesTried = 0;
}

bool GridCell::isFree()
{ 
	if (m_gridproperty == FREE)
		return true;
	return false;
}

 void GridCell::updateCell(Patch* p)	//Update cell when a patch is in that place
 {
	 m_gridproperty = OCCUPIED;
	 m_patch = p;
	 p->available = false;
	 p->becomesTarget();

 }

 void GridCell::updateSideVertices(Patch* p)								//Should i keep this?
 {
	p->m_rightside_vertices[0] = p->m_vertices[0];
	p->m_rightside_vertices[1] = p->m_vertices[1];
	p->m_topside_vertices[0] = p->m_vertices[1];
	p->m_topside_vertices[1] = p->m_vertices[2];
	p->m_leftside_vertices[0] = p->m_vertices[2];
	p->m_leftside_vertices[1] = p->m_vertices[3];
	p->m_bottomside_vertices[0] = p->m_vertices[3];
	p->m_bottomside_vertices[1] = p->m_vertices[0];
			
 }

 ////////////////////////////
 ////////////////////////////
 /////////GRID///////////////
 ////////////////////////////
 ////////////////////////////

 Grid::Grid(int width, int height)
 {
	g_width = width;													//width of the grid
	g_height = height;													//height of the grid
	centerCellX = floor(width/2);										//cell in the center of the grid
	centerCellY = floor(height/2);										//cell in the center of the grid
//	std::tie(centerOfGridX, centerOfGridY) = retrieveXY(centerCellX, centerCellY);	//Retrieve coordinates of X and Y 
	m_grid.resize(g_width);			
	for (int i = 0; i < g_width; i++)	{
		m_grid[i].resize(g_height);
    }

	//Fill the grid with empty cells
	for (int i = 0; i < g_width; i++){
		for (int j = 0; j < g_height; j++)
		{
			GridCell* cell = new GridCell(i, j);
			m_grid[i][j] = cell;
		}
	}
	retrieveXY();			//Look coordinates of the center in x and Y of each cell
 }

void Grid::updateGrid(Patch* p)
{
	m_grid[centerCellX][centerCellY]->updateCell(p);			//Update the cell of the grid with the new patch
//	p->getSideVertices(p->m_vertices, centerOfGridX, centerOfGridY);
}

void Grid::updatePossibleCell(int i_grid, int j_grid, /*int centerX, int centerY, */Patch* patch)
{
	m_grid[i_grid][j_grid]->m_patch = patch;														//Set the patch to a particular cell
	/*m_grid[i_grid][j_grid]->c_centerX = centerX;
	m_grid[i_grid][j_grid]->c_centerY = centerY;	*/
}

bool Grid::isTargetUnchecked(Patch* patch, std::vector<std::tuple<PatchSide, PatchSide, Patch*>> m_neighbours, int i_grid, int j_grid)
{
	bool flag = false;
	bool cellChecked = false;
	//bool targetChecked = false;
	int targetsChecked = 0;
	//If this side of the target hasn´t been checked for that patch
	if( patch->targetsChecked.size() == 0)
		return true;
	
	for (std::size_t i = 0; i < patch->cellsWherePatchHasBeen.size(); i++)  //if this patch has been in this cell
	{
		if (patch->cellsWherePatchHasBeen[i] == m_grid[i_grid][j_grid])
			cellChecked = true;
	}
	
	if (cellChecked == false) //if this cell hasnt been checked, return true for "targetUnchecked"
		return true;
	else //if this cell has been checked, check if has also being checked for potential new targets
	{
		for (std::size_t i = 0; i < m_neighbours.size(); i++)
		{
			Patch* target = std::get<2>(m_neighbours[i]);
			for (std::size_t j = 0; j < patch->targetsChecked.size(); j++)
			{
				if (patch->targetsChecked[j].first == target && patch->targetsChecked[j].second == target->patch_side) //target and targetSide already checked before
				{
					//targetChecked = true;
					targetsChecked++;
					j = patch->targetsChecked.size();
				}
			}
		}
	}
	if (targetsChecked < m_neighbours.size()) //if there are some targets unchecked
		return true;
	else 
		return false;
	//return !targetChecked;
}
 
void Grid::retrieveXY()			//Retrieve the centerx and centery coordinates from the i and j identifier of the grid
{
	for (int i = 0; i < g_width; i++){
		for (int j = 0; j < g_height; j++)
		{
			m_grid[i][j]->c_centerX = (widthCell / 2) + (sizeCell * i);
			m_grid[i][j]->c_centerY = (heightCell / 2) + (sizeCell * j);
		}
	}

}

void Grid::rotate4Times(Patch* target, Patch* patch, int i_grid, int j_grid, OgreBites::ParamsPanel* mDetailsPanel, Ogre::SceneManager* mSceneMgr, int patchId, Ogre::Root* mRoot, int centerX, int centerY)
{
	double error = 0.0;
	int sideOfPatch;
	PatchSide _patchSide, _targetSide;
	for (int rotationNumber = 0; rotationNumber <1; rotationNumber++)								//Rotate 3 times
	{
		for (std::size_t neighbor = 0; neighbor < m_neighbours.size(); neighbor++)					//For every neighbour
		{
			target = std::get<2>(m_neighbours[neighbor]);											//Get the target from the neighbours vector
			_patchSide = patch->patch_side = std::get<0>(m_neighbours[neighbor]);					//Get the current side of the patch
			_targetSide = target->patch_side = std::get<1>(m_neighbours[neighbor]);					//Get the current side of the target

			if (rotationNumber == 0)
			{																					//If there is no rotation yet, calculate error in first position
				sideOfPatch = patch->patch_side;
				patch->computeError(target, _patchSide, _targetSide, mDetailsPanel, patch, m_grid[i_grid][j_grid], patchId, mSceneMgr,mRoot );	  //patchSide and targetSide for each neighbor
			}
			else
			{																									
				if(neighbor == 0)																    //Rotate the patch only once for all the neighbours, if this value is bigger than 0, it was already rotated for the previous neighbour and the patch is on the right position now
				{	
					patch->rotatePatch(mSceneMgr, centerX, centerY, mRoot);								//Rotate patch (Create new vertices after 90 degres rotation)
					mRoot->renderOneFrame();
				}
				patch->computeError(target,  _patchSide, _targetSide, mDetailsPanel,  patch, m_grid[i_grid][j_grid], patchId, mSceneMgr, mRoot);			

			}
		}

		//Compute average of the error for more than 1 target
		if (m_neighbours.size() > 1)	
		{
			bestErrorOfPatch current_error;
			double currError = 0;
			for (std::size_t i = 0; i < patch->temporalError.size(); i++)
			{
				currError +=  patch->temporalError[i].error;
			}
			currError = currError/m_neighbours.size();
			current_error.error = currError;
			current_error.vertices = patch->m_vertices;
			current_error.cell = m_grid[i_grid][j_grid];
			current_error.patchId = patchId;
			current_error.orientation = patch->m_orientation;
			current_error.zPos = patch->currentZposition;
			patch->m_curError.push_back(current_error);
			patch->temporalError.clear();
		}
		
	}		
	for (std::size_t numberOfNeighbour = 0; numberOfNeighbour < m_neighbours.size(); numberOfNeighbour++)
	{
		target = std::get<2>(m_neighbours[numberOfNeighbour]);	
		patch->targetsChecked.push_back(std::make_pair(target, target->patch_side));
	}
	patch->cellsWherePatchHasBeen.push_back(m_grid[i_grid][j_grid]);									//save this cell as visited cell by the patch

	//patch->rotatePatch(mSceneMgr, centerX, centerY, mRoot);												//Return to original position
}

void Grid::transverseGrid(Patch* patch, Patch* target, Ogre::SceneManager* mSceneMgr, Ogre::Root* mRoot, OgreBites::ParamsPanel* mDetailsPanel, int patchId, int numberOfCells)
{
	int centerXofCell; //Center of the cell in X
	int centerYofCell; //Center of the cell in y
	double z_position = patch->OriginalZPos;
	int patchCounter = patchId;
	
	bool positionCorrect = false;						//Patch has not yet been translated to correct posittion
	std::vector<PatchSide> sideToMove;
	std::pair<double, double> newCoordinates;

	updateGrid(target);																//Set the target into the cell and mark it as occupied																
	for (int i_grid = 0; i_grid < g_width; i_grid++){																//Go through all the grid
		for (int j_grid = 0; j_grid< g_height; j_grid++){
			if (stitchingIsPossible(i_grid,j_grid))																	//If that cell is free and its neighbours are occupied is a possible position
			{		
				m_neighbours =  getNeighbours(i_grid, j_grid);	
				if (isTargetUnchecked(patch, m_neighbours, i_grid, j_grid))														//If that target for that side hasnt been checked already
				{						
					updatePossibleCell(i_grid, j_grid, patch);
					centerXofCell = m_grid[i_grid][j_grid]->c_centerX;										//Get the coordinates of the selected cell
					centerYofCell = m_grid[i_grid][j_grid]->c_centerY;
				//	for (std::size_t translation_Number = 0; translation_Number < 3; translation_Number++)				//Translate in "Z" another 2 times in that position
				//	{
				//		z_position = getPositionInZ(translation_Number, mSceneMgr, patch);
						patch->translatePatch(centerXofCell, centerYofCell, 0, mSceneMgr, mRoot);									//Translates the patch to the correct cell			
						rotate4Times(target, patch, i_grid, j_grid, mDetailsPanel, mSceneMgr, patchId, mRoot, centerXofCell, centerYofCell);		//Rotate the patch 4 times and calculate the error
				//	}					
				}
			}														
		}
	}

	_bestFitOfPatch.push_back(bestFitOfPatch(patch));					
	patch->translatePatchToOrigin(mSceneMgr, patchId);
	mRoot->renderOneFrame();													
}

bestErrorOfPatch Grid::bestFitOfPatch(Patch* p)
{
	int i = 0;
	while (p->m_curError[i].cell->m_gridproperty == OCCUPIED){i++;}

	double error = p->m_curError[i].error;
	GridCell* c = p->m_curError[i].cell;
	std::vector<Ogre::Vector3> bestError_vertices = p->m_curError[i].vertices;
	int patchId = p->m_curError[i].patchId;
	Ogre::Quaternion orientation = p->m_curError[i].orientation;
	Ogre::Real z_pos = p->m_curError[i].zPos;
	
	for (std::size_t i = 0; i < p->m_curError.size(); i++){
		double minTmp = p->m_curError[i].error;
		if ( minTmp < error && p->m_curError[i].cell->m_gridproperty != OCCUPIED && p->available == TRUE)									//If new error is lower than the last
		{
			error = minTmp;
			c = p->m_curError[i].cell;
			bestError_vertices = p->m_curError[i].vertices;
			patchId = p->m_curError[i].patchId;
			orientation = p->m_curError[i].orientation;
			z_pos = p->m_curError[i].zPos;
		}
	}
	bestErrorOfPatch bestFitOfPatch;
	bestFitOfPatch.cell = c;
	bestFitOfPatch.error = error;
	bestFitOfPatch.vertices = bestError_vertices;
	bestFitOfPatch.patchId = patchId;										
	bestFitOfPatch.orientation = orientation;
	bestFitOfPatch.zPos = z_pos;
	return bestFitOfPatch;
}

bestErrorOfPatch Grid::bestFitInGrid(std::vector<bestErrorOfPatch> bfit)
{
	double error; 
	std::vector<Ogre::Vector3> bestError_vertices;
	GridCell* c;
	int patchId;
	Ogre::Quaternion orientation;
	int x;
	int y; 
	
	int i = 0;
	while (bfit[i].cell->m_gridproperty == OCCUPIED){i++;}

	c = bfit[i].cell;
	error = bfit[i].error;
	bestError_vertices = bfit[i].vertices;
	patchId = bfit[i].patchId;
	orientation = bfit[i].orientation;
	Ogre::Real z_pos = bfit[i].zPos;
	
	for (std::size_t i = 0; i < bfit.size(); i++){
		double minTmp = bfit[i].error;
		x = bfit[i].cell->m_cellX;
		y = bfit[i].cell->m_cellY;
		if ( minTmp < error && m_grid[x][y]->m_gridproperty != OCCUPIED && bfit[i].cell->m_patch->available == true) //If new error is lower than the last
		{
			c = bfit[i].cell;
			error = bfit[i].error;
			bestError_vertices = bfit[i].vertices;
			patchId = bfit[i].patchId;
			orientation = bfit[i].orientation;		
			z_pos = bfit[i].zPos;
		}
	}

	bestErrorOfPatch bestFit;
	bestFit.error = error;
	bestFit.cell = c;
	bestFit.vertices = bestError_vertices;
	bestFit.patchId = patchId;
	bestFit.orientation = orientation;
	bestErrors.clear();														//Delete the error for that iteration, and leave it clean for the next one
	bestFit.zPos = z_pos;
	return bestFit;
}

std::vector<std::tuple<PatchSide, PatchSide, Patch*>> Grid::getNeighbours(int i, int j)		
{
	std::vector<std::tuple<PatchSide, PatchSide, Patch*>> neighbours;
	m_neighbours.clear();
	pSide.clear();
	tSide.clear();
	 if (i+1 < g_width  && !m_grid[i+1][j]->isFree()){
		 pSide.push_back(RIGHT);
		 tSide.push_back(LEFT);
		 m_grid[i+1][j]->m_patch->patch_side = LEFT;
		 neighbours.push_back(std::make_tuple(RIGHT, LEFT,m_grid[i+1][j]->m_patch));			//Save in the neighbrous vector the side of the patch, the side of the target and the target itself
	 }
	 if (j+1 < g_height && !m_grid[i][j+1]->isFree()){
		 pSide.push_back(TOP);
		 tSide.push_back(BOTTOM);
		 m_grid[i][j+1]->m_patch->patch_side = BOTTOM;
		 neighbours.push_back(std::make_tuple(TOP, BOTTOM,m_grid[i][j+1]->m_patch));
	 }
	 if (i-1 >= 0 && !m_grid[i-1][j]->isFree()){
		 pSide.push_back(LEFT);
		 tSide.push_back(RIGHT);
		 m_grid[i-1][j]->m_patch->patch_side = RIGHT;
		 neighbours.push_back(std::make_tuple(LEFT, RIGHT ,m_grid[i-1][j]->m_patch));
	 }
	 if (j-1 >= 0 && !m_grid[i][j-1]->isFree()){
		 pSide.push_back(BOTTOM);
		 tSide.push_back(TOP);
		 m_grid[i][j-1]->m_patch->patch_side = TOP;
		 neighbours.push_back(std::make_tuple(BOTTOM, TOP,m_grid[i][j-1]->m_patch));
	 }

	 return neighbours;

	
}

double Grid::getPositionInZ(std::size_t  translationNumber, Ogre::SceneManager* mSceneMgr, Patch *patch)
{
	Ogre::Real z_position;
	if (translationNumber == 1)																	//modify the "z" position 
		z_position = -0.3;
	else if (translationNumber == 2)
		z_position = 0.3;
	else z_position = patch->OriginalZPos;

	return z_position;
}

bool Grid::stitchingIsPossible(int i, int j)
{	
	if(m_grid[i][j]->isFree())
	{
		if( i+1 < g_width  && !m_grid[i+1][j]->isFree()) //Check if its neighbours are occupied and if it is not outised of the grid
			return true;
		if (j+1 < g_height && !m_grid[i][j+1]->isFree())
			return true;
		if (i-1 >= 0 && !m_grid[i-1][j]->isFree())
			return true;
		if (j-1 >= 0 && !m_grid[i][j-1]->isFree()) 
			return true;
	}
	return false;
}

