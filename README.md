# NewStitching

#Initialize Ogre code

NewStitching.cpp has all the Ogre code by default (initialize the camera, the buffers, the key controllers, etc)

In the `createCamera` method, two cameras are created; one for the front view and one for the side view. 
There is also two viewports in `createViewports` one for each camera.

The `createTemplate` method creates the entity and the node for the first template, but not the template object itself.
The `createPatches` method create "n" number of patches automatically according with the size of the grid
The `startAnimation" method starts the algorithm that won't stop until all the grid is filled.

#Start animation

The first step is to create the grid `Grid* grid = new Grid(widhtGrid, heightGrid);`
This will call the constructor of the grid in GridCell.cpp that will create single cells for all the grid.
Every cell will be initialized as free with the the appropiate size defined in `Patch.h`

```
#define widhtGrid 3 	//this would be a 3x3 grid
#define heightGrid 3
#define heightCell 10	// every cell will be 10x10 
#define widthCell 10
#define sizeCell 10
```

After that, the target itself will be created. To do this, will call the constructor for the `Patch.cpp` class, which has an overload method for the patches that are also targets
The target will be created and translated to the center of the grid. 
As the pivot of the mesh can be not exactly on the center of the mesh, it is necessary to move the mesh until it fits perfectly to the grid
This is done with the method `checkPositionsAreCorrect`

```
std::vector<PatchSide> sidesToMove;
		sidesToMove = incorrectSides();
		if (!sidesToMove.empty())
		{
			correctPositions = false;
			//Move to correct position
			for (std::size_t i = 0; i < sidesToMove.size(); i++) //For every side with incorrect positions 
			{
				switch (sidesToMove[i])
				{
				case(RIGHT) :
					mSceneMgr->getSceneNode("nodemm")->translate(move, 0, 0, Ogre::Node::TS_WORLD);
					break;
				case(LEFT) :
					mSceneMgr->getSceneNode("nodemm")->translate(-move, 0, 0, Ogre::Node::TS_WORLD);
					break;
				case(TOP) :
					mSceneMgr->getSceneNode("nodemm")->translate(0, move, 0, Ogre::Node::TS_WORLD);
					break;
				case(BOTTOM) :
					mSceneMgr->getSceneNode("nodemm")->translate(0, -move, 0, Ogre::Node::TS_WORLD);
					break;
				default:
					break;
				}
				mRoot->renderOneFrame();
			}
		}
		else
		{
			correctPositions = true;
			m_orientation = orientation;
		}
	} while (correctPositions == false);
```

After the creation of the template, it will start a loop for every possible cell, creating one patch for every place.
The next step will be done in `Gridcell.cpp` and it will use the created patch and will start visiting all the cell, looking for a possible cell to place the patch. 

```
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
```

If possible, will update the cell as possible, translate the patch to that cell, and will enter the `rotate4Times` method
In this method, a loop will be created to rotate 4 times the patch.
It will calculate the error. Rotate, and calculate again until the loop is finished.
It will check if there is more than 1 neigbhour on that cell, and if so, will calculate the average of the error with each one.

The error will be calculated in the `computeError` method in Patch.cpp

Here will create a list of all vertices for the template and the current patch, and save the distance between all of them on a list.
It will then sort the list from lowest error to the biggest one, and will then save only the first half.
It will get the average square error and will be saved in the patch with the `bestErrorOfPatch` structure

```
struct bestErrorOfPatch
{
	double error;
	GridCell* cell;
	std::vector<Ogre::Vector3> vertices;
	int patchId;
	Ogre::Quaternion orientation; 
	Ogre::Real zPos;
};
```

It will repeat the process for all possible targets in all the grid.
After the loop is finished, the thread will go back to `StartAnimation` where a new patch will be created and the process will be repeated
When all patches are checked, the `bestFitInGrid` method will be called, and will retrieve the patch with the smallest error.
The chosen patch will be translated to its corresponding cell and the process will be repeated until all cells are filled.

#Output
Right now is not working perfectly for all patches. In a grid of 3x3 it can produces the original mesh back, but in a bigger grid, (5x5 for example) is not perfect. Although is stitching back correctly in some places, in other places is placing the patches in a wrong cell.



