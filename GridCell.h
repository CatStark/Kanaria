#pragma once
#ifndef __GRIDCELL_H
#define __GRIDCELL_H

#include "Patch.h"
#include <iostream>
#include <cmath>
#include <iomanip>  // setprecision()
#include <cassert>  // assert()

typedef enum GridProperty
{
	FREE,
	OCCUPIED,
	POSSIBLE
}GridProperty;

//struct BestFit
//{
//	GridCell *cell;
//	std::vector<Ogre::Vector3> vertices;
//	double error;
//};

class GridCell
{
public:

	int m_cellX, m_cellY;
	GridCell(void);
	GridCell(int, int);
	~GridCell(void) {};
	bool isFree();												//Is this cell free? 
	void updateCell(Patch* p);									//Update cell when a patch is in that place
	void getSideVertices(Patch* p);
	void updateSideVertices(Patch* p);							//Update de sideVertices after rotating

	int c_centerX, c_centerY;								//center of the cell in X and Y
	Patch *m_patch;
	PatchSide bestSide;
	double bestError;
	std::vector<Ogre::Vector3> patch_vertices;
	bool cellUnchecked;
	int numberOfPatchesTried;							//How many patches have been tried in that cell
	GridProperty m_gridproperty;
						//Save a list of all patches that checked that cell

private:
	int c_height, c_width;									//width and height of each cell
	int c_size;												//Size of the cell
	
	
};

class Grid
{
public:
private:
    int g_width, g_height;														//widht and height of the grid
	int centerOfGridX, centerOfGridY, centerCellX, centerCellY;

public:
	Grid(int numberOfTotalCells);
	Grid(int, int);
	void transverseGrid(Patch* patch, Patch* target, Ogre::SceneManager*, Ogre::Root* mRoot, OgreBites::ParamsPanel* mDetailsPanel, int patchId, int);
	bool stitchingIsPossible(int, int);
	std::vector<bestErrorOfPatch> bestErrors;
	bestErrorOfPatch bestFitInGrid(std::vector<bestErrorOfPatch> bfit);							//The best fit in all the grid
	void Grid::updatePossibleCell(int, int, /* int, int,*/ Patch* p);
	void updateGrid(Patch* p);
	std::vector<std::tuple<PatchSide, PatchSide, Patch*>> getNeighbours(int, int);
	void Grid::rotate4Times(Patch* target, Patch* patch, int i_grid, int j_grid, OgreBites::ParamsPanel* mDetailsPanel, Ogre::SceneManager* mSceneMgr, int patchId, Ogre::Root* mRoot, int, int);
	double Grid::getPositionInZ(std::size_t translationNumber);
	bestErrorOfPatch bestFitOfPatch(Patch* p);
	bool isTargetUnchecked(Patch* patch, std::vector<std::tuple<PatchSide, PatchSide, Patch*>> m_neighbours, int, int);

	void retrieveXY(); //Retrieve the x and ycoordinates from the i and j identifier of the grid
	std::vector<std::vector<GridCell*>> m_grid;
	std::vector<std::tuple<PatchSide, PatchSide, Patch*>> m_neighbours;
	std::vector<int> pSide;
	std::vector<int> tSide;
	/*std::vector<PatchSide> pSide;
	std::vector<PatchSide> tSide;*/
	std::vector<Patch*> targets;
	std::vector<bestErrorOfPatch> _bestFitOfPatch;
	std::vector<int> patchesUsed;	
};


#endif

