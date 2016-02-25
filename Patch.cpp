#include "Patch.h"


Patch::Patch(bool target, int numberOfPatch, Ogre::SceneManager* mSceneMgr)
{
	isTarget = false;
	Ogre::Vector3 bbsize = Ogre::Vector3::ZERO; //Boundingbox size
	Ogre::Vector3 scale(0.5, 0.5, 0.5);
	Ogre::Quaternion rotation(Ogre::Degree(180), Ogre::Vector3::UNIT_Y); 
	Ogre::Quaternion rotation2(Ogre::Degree(180), Ogre::Vector3::UNIT_Z); 
	available = true;

	nodeName = "node" + Ogre::StringConverter::toString(numberOfPatch);
	entName = Ogre::StringConverter::toString(numberOfPatch);
	mesh = entName + ".mesh";
	ent = mSceneMgr->createEntity(entName,mesh);										//Create Entity

	Ogre::AxisAlignedBox bb = ent->getBoundingBox();
	bbsize = bb.getSize();
	p_centerX = bbsize.x/2;
	p_centerY = bbsize.y/2;
	p_width = bbsize.x;
	p_height = bbsize.y;

	deviationInX = std::abs( (widthCell/2) - p_centerX); //The patch has not uniform mass, therefore the center of the patch might be not exactly in the center, this value shows how much the error is
	deviationInY = std::abs((heightCell/2) - p_centerY);

	_baseSceneNodeName = "patch" + Ogre::StringConverter::toString(numberOfPatch);
	_baseSceneNode = mSceneMgr->getSceneNode("grid")->createChildSceneNode(_baseSceneNodeName);
	//_baseSceneNode->translate(deviationInX, deviationInY, 0, Ogre::Node::TS_PARENT);
	
	
	node = mSceneMgr->getSceneNode(_baseSceneNodeName)->createChildSceneNode(nodeName);				//Create Node
	node->attachObject(ent);
	node->rotate(rotation, Ogre::Node::TransformSpace::TS_LOCAL);
	node->rotate(rotation2, Ogre::Node::TransformSpace::TS_LOCAL);
	

	
	
	
	
	/*nodeBaseName = "base" + Ogre::StringConverter::toString(numberOfPatch);
	nodeBaseScene = mSceneMgr->getSceneNode("grid")->createChildSceneNode(nodeName);				//Create SceneNode
	nodeBaseScene->translate(deviationInX, deviationInY, 0, Ogre::Node::TS_PARENT);
	*/
	//node = mSceneMgr->getSceneNode(nodeBaseName)->createChildSceneNode(nodeName);				//Create Node

	//translatePatchToOrigin(mSceneMgr, numberOfPatch);

//	

	
}

Patch::Patch(bool target, Ogre::Entity* targetPatch, Ogre::SceneManager* mSceneMgr, Ogre::Root* mRoot)						//Target
{
	bool correctPositions = true;
	double move = 0.2;


	int centerOfGrid = (widhtGrid * widthCell) / 2;
	isTarget = true;
	Ogre::Vector3 bbsize = Ogre::Vector3::ZERO; //Boundingbox size
	std::vector<Ogre::Vector3> temporalList;
	
	std::vector<Ogre::Vector3> verticesList;
	size_t vertex_count,index_count;
	unsigned* indices;
	std::vector<Ogre::Vector3> vertices;														//Four vertices per patch
	std::vector<int> indice;
	
	do
	{
		Ogre::Vector3 pos = targetPatch->getParentSceneNode()->getPosition();
		Ogre::Vector3 scale = targetPatch->getParentSceneNode()->getScale();
		Ogre::Quaternion orientation = targetPatch->getParentSceneNode()->getOrientation();

		std::tie(m_vertices, m_indices) = getMeshInformation(&targetPatch->getMesh(), vertex_count, index_count, indices, pos, orientation, scale);
		deleteRepeatedVertices();
		getSideVertices(m_vertices, centerOfGrid, centerOfGrid);

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
					mSceneMgr->getSceneNode("nodemm")->translate(0, -move, 0, Ogre::Node::TS_WORLD);
					break;
				case(BOTTOM) :
					mSceneMgr->getSceneNode("nodemm")->translate(0, move, 0, Ogre::Node::TS_WORLD);
					break;
				default:
					break;
				}
				mRoot->renderOneFrame();
			}
		}
		else
			correctPositions = true;
		move += 0.1;
		m_orientation = orientation;
	} while (correctPositions == false);


	Ogre::AxisAlignedBox bb = targetPatch->getBoundingBox();
	bbsize = bb.getSize();
	p_centerX = bbsize.x/2;
	p_centerY = bbsize.y/2;
	p_width = bbsize.x;
	p_height = bbsize.y;
}

void Patch::getSideVertices(std::vector<Ogre::Vector3> verticesTemplate, int centerX, int centerY)
{
	Ogre::Vector3 firstVertex;
	m_rightside_vertices.clear();
	m_topside_vertices.clear();
	m_leftside_vertices.clear();
	m_bottomside_vertices.clear();
	

	//Offset is the center poing plus an arbitrary value 
	double offsetX1 = centerX + ((widthCell / 2) - 0.2); //To the right from the center point
	double offsetX2 = centerX - ((widthCell / 2) - 0.2);	//To the left
	double offsetY1 = centerY + ((heightCell / 2) - 0.2);  //To the top
	double offsetY2 = centerY - ((heightCell / 2) - 0.2);  //To the bottom

	//Apply offset here
	

	for (std::size_t i = 0; i < verticesTemplate.size(); i++)
	{
		//	if (verticesTemplate[i].z > 1)
		//	{
		if (verticesTemplate[i].x > offsetX1)													//Bigger than originX, will be right side
			m_rightside_vertices.push_back(verticesTemplate[i]);
		else if (verticesTemplate[i].x < offsetX2)
			m_leftside_vertices.push_back(verticesTemplate[i]);									//Smaller than OriginX, will be left side
		if (verticesTemplate[i].y > offsetY1)
			m_topside_vertices.push_back(verticesTemplate[i]);									//Bigger than originY, will be top side
		else if (verticesTemplate[i].y < offsetY2)
			m_bottomside_vertices.push_back(verticesTemplate[i]);								//Smaller than originY, will be bottom side
	}
}

void Patch::checkPositionAreCorrect(int centerX, int centerY, Ogre::SceneManager* mSceneMgr)
{
	bool correctPositions = true;
	double move = 0.2;
	
	do
	{
		updateVertices(centerX, centerY, mSceneMgr);
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
						mSceneMgr->getSceneNode(nodeName)->translate(move, 0, 0, Ogre::Node::TS_WORLD);
					break;
				case(LEFT) :
					mSceneMgr->getSceneNode(nodeName)->translate(-move, 0, 0, Ogre::Node::TS_WORLD);
					break;
				case(TOP) :
					mSceneMgr->getSceneNode(nodeName)->translate(0, move, 0, Ogre::Node::TS_WORLD);
					break;
				case(BOTTOM) :
					mSceneMgr->getSceneNode(nodeName)->translate(0, -move, 0, Ogre::Node::TS_WORLD);
					break;
				default:
					break;
				}
				//mRoot->renderOneFrame();
			}
		}
		else
			correctPositions = true;
	} while (!correctPositions);
}

void Patch::removeFromErrorList(GridCell* cell)
{
	double error = m_curError[0].error;
	GridCell* c = m_curError[0].cell;
	for (std::size_t i = 0; i < m_curError.size(); i++){
		double minTmp = m_curError[i].error;
		if ( minTmp < error)									//If new error is lower than the last
		{
			if (m_curError[i].cell == cell)
				m_curError.erase(m_curError.begin() + 1);
		}
	}
}

void Patch::computeError(Patch* target, PatchSide _patchSide, PatchSide _targetSide, OgreBites::ParamsPanel* mDetailsPanel, Patch* patch, GridCell* cell, int patchId, Ogre::SceneManager* mSceneMgr, Ogre::Root* mRoot)
{
	double error = 0.0;
	std::vector<Ogre::Real> temperror;
	std::vector<Ogre::Vector3> sideP;
	std::vector<Ogre::Vector3> sideT;
	int counter = 0;

	std::tie(sideP, sideT) = choseSide(target, _patchSide, _targetSide);
	
	for(std::size_t vertexPatch = 0; vertexPatch < sideP.size(); vertexPatch++)
	{
		for (std::size_t vertexTemplate = 0; vertexTemplate < sideT.size(); vertexTemplate++ )
		{
			Ogre::Real distance = sideP[vertexPatch].distance(sideT[vertexTemplate]);
			temperror.push_back(Ogre::Math::Sqrt(distance * distance)); 
		}
	}
	for (std::size_t i = 0; i < temperror.size(); i++)
	{
		if ( std::abs((double)temperror[i]) <= widthCell/6	)
		{
			error += temperror[i];	
			counter++;
		}
	}

	error = error/counter; //RMSE
	bestErrorOfPatch current_error;
	current_error.error = error;
	current_error.vertices = m_vertices;
	current_error.cell = cell;
	current_error.patchId = patchId;
	current_error.orientation = m_orientation;
	current_error.zPos = currentZposition;
	m_curError.push_back(current_error);

	mDetailsPanel->setParamValue(0, Ogre::StringConverter::toString(error).c_str());	
}

PatchSide Patch::getSideFromInt(int s)
{
	PatchSide side;

	switch(s)
	{
	case(0):
		side = RIGHT;
		break;
	case(1):
		side = TOP;
		break;
	case(2):
		side = LEFT;
		break;
	case(3):
		side = BOTTOM;
		break;
	}
	return side;
}

std::pair<std::vector<Ogre::Vector3>,std::vector<Ogre::Vector3>> Patch::choseSide(Patch* target, PatchSide pSide, PatchSide tSide)			//Retrieve the x and y coordinates from the i and j identifier of the grid
{
	std::vector<Ogre::Vector3> sideP;
	std::vector<Ogre::Vector3> sideT;
	
	switch (pSide)	{	
	case(RIGHT):sideP = m_rightside_vertices;	break;
	case(TOP):sideP = m_topside_vertices;		break;
	case(LEFT):sideP = m_leftside_vertices;		break;
	case(BOTTOM):sideP = m_bottomside_vertices;	break;
	}

	switch (tSide)	{
	case(RIGHT):sideT = target->m_rightside_vertices;	break;
	case(TOP):sideT = target->m_topside_vertices;		break;
	case(LEFT):sideT = target->m_leftside_vertices;	break;
	case(BOTTOM):sideT = target->m_bottomside_vertices;	break;
	}


	return std::make_pair(sideP, sideT);
}

void Patch::translatePatchToOrigin(Ogre::SceneManager* mSceneMgr, int patchId)
{
	int newPosX = patchId * widthCell;
	int newPosY = -10;
	if (patchId >= 6 && patchId < 12)
		newPosY -= 20;
	else if (patchId >= 12 && patchId < 18)
		newPosY -= 40;
	else if (patchId >= 18)
		newPosY -= 60;

	mSceneMgr->getSceneNode(nodeName)->setPosition(newPosX, newPosY, 0);
	
	
}

void Patch::translatePatch(int centerX, int centerY, int z_position, Ogre::SceneManager* mSceneMgr,  Ogre::Root* mRoot)
{
	isTarget = false;													//Targets never are moved. If the patch is moving, is not a target
	bool correctPositions = true;
	double move = 0.2;
	mSceneMgr->getSceneNode(nodeName)->setPosition(centerX, centerY, z_position);
	currentZposition = z_position;
	//updateVertices(centerX, centerY, mSceneMgr);
	checkPositionAreCorrect(centerX, centerY, mSceneMgr);
	mRoot->renderOneFrame();
	
	//do
	//{
	//	updateVertices(centerX, centerY, mSceneMgr);
	//	std::vector<PatchSide> sidesToMove;
	//	sidesToMove = incorrectSides();
	//	if (!sidesToMove.empty())
	//	{
	//		correctPositions = false;
	//		//Move to correct position
	//		for (std::size_t i = 0; i < sidesToMove.size(); i++) //For every side with incorrect positions 
	//		{
	//			switch (sidesToMove[i])
	//			{
	//			case(RIGHT) :
	//				mSceneMgr->getSceneNode(nodeName)->translate(move, 0, 0, Ogre::Node::TS_LOCAL);
	//				break;
	//			case(LEFT) :
	//				mSceneMgr->getSceneNode(nodeName)->translate(-move, 0, 0, Ogre::Node::TS_LOCAL);
	//				break;
	//			case(TOP) :
	//				mSceneMgr->getSceneNode(nodeName)->translate(0, move, 0, Ogre::Node::TS_LOCAL);
	//				break;
	//			case(BOTTOM) :
	//				mSceneMgr->getSceneNode(nodeName)->translate(0, -move, 0, Ogre::Node::TS_LOCAL);
	//				break;
	//			default:
	//				break;
	//			}
	//		}
	//		mRoot->renderOneFrame();
	//	}
	//	else
	//		correctPositions = true;
	//	move += 0.1;
	//} while (!correctPositions);

}

std::vector<PatchSide> Patch::incorrectSides()
{
	std::vector<PatchSide> sidesToMove;

	if (m_rightside_vertices.size() == 0)
	{
		sidesToMove.push_back(RIGHT);
	}
	if (m_leftside_vertices.size() == 0)
	{
		sidesToMove.push_back(LEFT);
	}
	if (m_topside_vertices.size() == 0)
	{
		sidesToMove.push_back(TOP);
	}
	if (m_bottomside_vertices.size() == 0)
	{
		sidesToMove.push_back(BOTTOM);
	}

	return sidesToMove;
}

void Patch::updateVertices(int centerX, int centerY, Ogre::SceneManager* mSceneMgr)
{
	Ogre::Vector3 originVertex;
	Ogre::Vector3 pos; 
	Ogre::Vector3 scale; 
	Ogre::Quaternion orientation; 
	size_t vertex_count,index_count;
	unsigned* indices;
	std::vector<Ogre::Vector3> vertices;														//Four vertices per patch
	std::vector<int> indice;
	m_vertices.clear();

	pos  = mSceneMgr->getSceneNode(nodeName)->getPosition();
	scale =  mSceneMgr->getSceneNode(nodeName)->getScale();
	orientation =  mSceneMgr->getSceneNode(nodeName)->getOrientation();
	std::tie(m_vertices, m_indices) = getMeshInformation(&mSceneMgr->getEntity(entName)->getMesh(),vertex_count,index_count,indices, pos, orientation, scale);
	deleteRepeatedVertices();
	getSideVertices(m_vertices, centerX, centerY);
	m_orientation = orientation;

}

void Patch::translatePatchDeffinitve(Ogre::SceneManager* mSceneMgr, bestErrorOfPatch bestFitOverall, int centerX, int centerY, Ogre::Real z_position)
{
	isTarget = true;
	
	mSceneMgr->getSceneNode(nodeName)->setPosition(centerX, centerY, z_position);
	mSceneMgr->getSceneNode(nodeName)->setOrientation(bestFitOverall.orientation);
	//checkPositionAreCorrect(centerX, centerY, mSceneMgr);

	updateVertices(centerX, centerY, mSceneMgr);
}

void Patch::rotatePatch(Ogre::SceneManager* mSceneMgr, int centerX, int centerY, Ogre::Root* mRoot)
{
	Ogre::Quaternion rotation(Ogre::Degree(90), Ogre::Vector3::UNIT_Z); 
	Ogre::Matrix3 rotationM;
	rotation.ToRotationMatrix(rotationM);	
	mSceneMgr->getSceneNode(nodeName)->rotate(rotation, Ogre::Node::TransformSpace::TS_LOCAL);
	updateVertices(centerX, centerY, mSceneMgr);
	checkPositionAreCorrect(centerX, centerY, mSceneMgr);
	mRoot->renderOneFrame();
}

void Patch::becomesTarget()
{
	isTarget = true;
}

void Patch::deleteRepeatedVertices()
{
	/////////////////////////////////////////
	//*eliminate duplicated vertices 
	//* returns 8 vertices, 4 per for the front face and 4 for the back face
	/////////////////////////////////////////

	std::vector<Ogre::Vector3> temporalList;
	std::vector<Ogre::Vector3> verticesList;
	bool flag = false;
	
	verticesList = m_vertices;
	temporalList.push_back(verticesList[0]);
	for (std::size_t i = 1; i < verticesList.size(); i++)
	{
		Ogre::Vector3 temp;
		temp = verticesList[i];													//start comparing from the first element
		for (int j = i-1; j >= 0; j--)
		{
			if (temp == verticesList[j])
			{
				flag = true;
			}
		}
		
		if (flag == false)
		{
			temporalList.push_back(temp);
		}
		flag = false;
	}

	m_vertices.clear();
	m_vertices = temporalList;
}

std::pair<std::vector<Ogre::Vector3>,std::vector<int>> Patch::getMeshInformation(const Ogre::MeshPtr* meshPtr,
									  size_t &vertex_count,
									  size_t &index_count, 
									  unsigned* &indices,
									  const Ogre::Vector3 &position = Ogre::Vector3::ZERO,
									  const Ogre::Quaternion &orient = Ogre::Quaternion::IDENTITY,
									  const Ogre::Vector3 &scale = Ogre::Vector3::UNIT_SCALE)
{
	std::vector<Ogre::Vector3> verticesList;
	std::vector<int> indicesList;
  
	vertex_count = index_count = 0;
    bool added_shared = false;
    size_t current_offset = vertex_count;
    size_t shared_offset = vertex_count;
    size_t next_offset = vertex_count;
    size_t index_offset = index_count;
    size_t prev_vert = vertex_count;
    size_t prev_ind = index_count;
	Ogre::Mesh *mesh = meshPtr->getPointer();

	
    // Calculate how many vertices and indices we're going to need
    for(int i = 0;i < mesh->getNumSubMeshes();i++)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);
 
        // We only need to add the shared vertices once
        if(submesh->useSharedVertices)
        {
            if(!added_shared)
            {
                Ogre::VertexData* vertex_data = mesh->sharedVertexData;
                vertex_count += vertex_data->vertexCount;
                added_shared = true;
            }
        }
        else
        {
            Ogre::VertexData* vertex_data = submesh->vertexData;
            vertex_count += vertex_data->vertexCount;
        }
 
        // Add the indices
        Ogre::IndexData* index_data = submesh->indexData;
        index_count += index_data->indexCount;
    }
 
    // Allocate space for the vertices and indices
   // vertices = new Ogre::Vector3[vertex_count];
    indices = new unsigned[index_count];
 
    added_shared = false;
 
    // Run through the submeshes again, adding the data into the arrays
    for(int i = 0;i < mesh->getNumSubMeshes();i++)
    {
        Ogre::SubMesh* submesh = mesh->getSubMesh(i);
 
        Ogre::VertexData* vertex_data = submesh->useSharedVertices ? mesh->sharedVertexData : submesh->vertexData;
        if((!submesh->useSharedVertices)||(submesh->useSharedVertices && !added_shared))
        {
            if(submesh->useSharedVertices)
            {
                added_shared = true;
                shared_offset = current_offset;
            }
 
            const Ogre::VertexElement* posElem = vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
            Ogre::HardwareVertexBufferSharedPtr vbuf = vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());
            unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
            Ogre::Real* pReal;
 
            for(size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
            {
                posElem->baseVertexPointerToElement(vertex, &pReal);
 
                Ogre::Vector3 pt;
 
                pt.x = (*pReal++);
                pt.y = (*pReal++);
                pt.z = (*pReal++);
 
                pt = (orient * (pt * scale)) + position;
				verticesList.push_back(Ogre::Vector3(pt.x, pt.y, pt.z));
            }
            vbuf->unlock();
            next_offset += vertex_data->vertexCount;
        }
        Ogre::IndexData* index_data = submesh->indexData;
        size_t numTris = index_data->indexCount / 3;
        unsigned short* pShort;
        unsigned int* pInt;
        Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;
        bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);
        if (use32bitindexes) pInt = static_cast<unsigned int*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        else pShort = static_cast<unsigned short*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
 
        for(size_t k = 0; k < numTris; ++k)
        {
            size_t offset = (submesh->useSharedVertices)?shared_offset:current_offset;
 
            unsigned int vindex = use32bitindexes? *pInt++ : *pShort++;
            indices[index_offset + 0] = vindex + offset;
            vindex = use32bitindexes? *pInt++ : *pShort++;
            indices[index_offset + 1] = vindex + offset;
            vindex = use32bitindexes? *pInt++ : *pShort++;
            indices[index_offset + 2] = vindex + offset;
 
            index_offset += 3;
        }
        ibuf->unlock();
        current_offset = next_offset;
	}

	//Create a list with all indices
	for (std::size_t i = 0; i < index_count; i++)
	{
		indicesList.push_back(indices[i]);
	}
	return std::make_pair(verticesList, indicesList);

}



void Patch::DestroyAllAttachedMovableObjects( Ogre::SceneNode* i_pSceneNode )
{

   // Destroy all the attached objects
   Ogre::SceneNode::ObjectIterator itObject = i_pSceneNode->getAttachedObjectIterator();

   while ( itObject.hasMoreElements() )
   {
      Ogre::MovableObject* pObject = static_cast<Ogre::MovableObject*>(itObject.getNext());
      i_pSceneNode->getCreator()->destroyMovableObject( pObject );
   }

   // Recurse to child SceneNodes
   Ogre::SceneNode::ChildNodeIterator itChild = i_pSceneNode->getChildIterator();

   while ( itChild.hasMoreElements() )
   {
      Ogre::SceneNode* pChildNode = static_cast<Ogre::SceneNode*>(itChild.getNext());
      DestroyAllAttachedMovableObjects( pChildNode );
   }
}


