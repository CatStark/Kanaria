#pragma once
#ifndef __PATCH_H
#define __PATCH_H

#define widhtGrid 3
#define heightGrid 3
#define heightCell 20
#define widthCell 20
#define sizeCell 20

#include <vector>
#include <OGRE\Ogre.h>
#include <OgreLogManager.h>
#include <SdkTrays.h>

typedef enum PatchSide 
{
	RIGHT = 0,
	TOP = 1,
	LEFT = 2,
	BOTTOM = 3
} PatchSide;

class GridCell;
class Grid;

struct bestErrorOfPatch
{
	double error;
	GridCell* cell;
	std::vector<Ogre::Vector3> vertices;
	int patchId;
	Ogre::Quaternion orientation; 
	Ogre::Real zPos;
};

//struct targetsChecked
//{
//	Patch _target;
//	PatchSide _side;
//
//};
class Patch 
{

public:
	//Constructors
	Patch(bool target, int numberOfPatch, Ogre::SceneManager*);
	Patch(bool, Ogre::Entity*, Ogre::SceneManager* mSceneMgr, Ogre::Root* mRoot); //isTarget, vertices of the patch

	
	//Destructor
	~Patch(void);

	//Others
	bool isTargetUnchecked(Patch* patch, std::vector<std::tuple<PatchSide, PatchSide, Patch*>> m_neighbours);
	std::pair<std::vector<Ogre::Vector3>, std::vector<int>> getMeshInformation(const Ogre::MeshPtr* meshPtr,
									  size_t &vertex_count,
									  size_t &index_count, 
									  unsigned* &indices,
									  const Ogre::Vector3 &position,
									  const Ogre::Quaternion &orient,
									  const Ogre::Vector3 &scale);
	void getSideVertices(std::vector<Ogre::Vector3>, int, int); //sortVertices
	void computeError(Patch* target, PatchSide _patch, PatchSide _target, OgreBites::ParamsPanel* mDetailsPanel, Patch* patch, GridCell* cell, int, Ogre::SceneManager* mSceneMgr, Ogre::Root* mRoot );									//s from p
	void becomesTarget();																		//Change when the patch becomes a target
	void rotatePatch(Ogre::SceneManager* mSceneMgr, int, int, Ogre::Root* mRoot);
	void translatePatch(int, int, int, Ogre::SceneManager* mSceneMgr, Ogre::Root* mRoot);
	void translatePatchToOrigin(Ogre::SceneManager* mSceneMgr, int);
	void translatePatchDeffinitve(Ogre::SceneManager* mSceneMgr, bestErrorOfPatch, int, int, Ogre::Real);
	PatchSide getSideFromInt(int);
	std::pair<std::vector<Ogre::Vector3>,std::vector<Ogre::Vector3>> Patch::choseSide(Patch* target, PatchSide pSide, PatchSide tSide);						//picks patch and template side 
	void removeFromErrorList(GridCell* cell);
	void deleteRepeatedVertices();
	double getOffset(PatchSide);
	void getSelectedVertices(std::vector<Ogre::Vector3> sideP, std::vector<Ogre::Vector3> sideT, PatchSide offsetOrient);
	void updateVertices(int centerX, int centerY, Ogre::SceneManager* mSceneMgr);
	void DestroyAllAttachedMovableObjects( Ogre::SceneNode* i_pSceneNode );
	std::vector<PatchSide> incorrectSides();
	void checkPositionAreCorrect(int centerX, int centerY, Ogre::SceneManager* mSceneMgr);
	//std::pair<double, double> correctPositionOfPatch(/*std::vector<PatchSide> sidesToMove,*/ std::vector<Ogre::Vector3> side, int centerX, int centerY, int z_pos, Ogre::SceneManager* mSceneMgr);

	std::vector<Ogre::Vector3> m_vertices;														//Four vertices per patch
	std::vector<int> m_indices;
	std::vector<Ogre::Vector3> m_leftside_vertices;												
	std::vector<Ogre::Vector3> m_rightside_vertices;											
	std::vector<Ogre::Vector3> m_bottomside_vertices;											
	std::vector<Ogre::Vector3> m_topside_vertices;	
	std::vector<std::pair<Patch*, PatchSide>> targetsChecked;
	//std::vector<targetsChecked> _targetsChecked;
	//std::vector<Patch*> targetsChecked;
	bool isTarget;
	bool available;
	Ogre::Real p_centerX, p_centerY, p_centerZ;
	Ogre::Real p_width, p_height;
	std::vector<bestErrorOfPatch> _bestFitOfPatch;
	std::vector<bestErrorOfPatch> m_curError;
	bestErrorOfPatch bestFit;
	std::vector<GridCell*> cellsWherePatchHasBeen;
	Ogre::Real deviationInX, deviationInY;
	


	Ogre::Real currentZposition;

	
  PatchSide patch_side;
  Ogre::Quaternion m_orientation;

  Ogre::String nodeName;
  Ogre::String entName;
  Ogre::String mesh;
  Ogre::Entity* ent;
  Ogre::SceneNode* node;
  Ogre::SceneNode* _baseSceneNode;
  Ogre::String _baseSceneNodeName;
};




#endif
