#include "MyOctant.h"
using namespace Simplex;
uint MyOctant::m_uOctantCount = 0;
uint MyOctant::m_uMaxLevel = 3;
uint MyOctant::m_uIdealEntityCount = 5;

//Initialize all of the variables, setting them to their default values and getting an instance of the mesh/entity managers.
void MyOctant::Init(void)
{
	m_uChildren = 0;
	m_fSize = 0.0f;
	m_uID = m_uOctantCount;
	m_uLevel = 0;
	m_v3Center = vector3(0.0f);
	m_v3Min = vector3(0.0f);
	m_v3Max = vector3(0.0f);
	m_pMeshMngr = MeshManager::GetInstance();
	m_pEntityMngr = MyEntityManager::GetInstance();
	m_pRoot = nullptr;
	m_pParent = nullptr;
	for (uint i = 0; i < 0; i++)
	{
		m_pChild[i] = nullptr;
	}
}
//this constructor is for the root only.
MyOctant::MyOctant(uint a_nMaxLevel, uint a_nIdealEntityCount)
{
	//init variables
	Init();
	m_uOctantCount = 0;
	m_uMaxLevel = a_nMaxLevel;
	m_uIdealEntityCount = a_nIdealEntityCount;
	m_uID = m_uOctantCount;

	//this is the root
	m_pRoot = this;
	m_lChild.clear();

	//we need to find the overall size of the world to encompass it
	std::vector<vector3> minMax;

	//makes a vector of all of the mins and maxes of each entity to draw the octree bounding box.
	uint numObj = m_pEntityMngr->GetEntityCount();
	for (uint i = 0; i < numObj; i++)
	{
		MyEntity* entity = m_pEntityMngr->GetEntity(i);
		MyRigidBody* rigidBody = entity->GetRigidBody();
		minMax.push_back(rigidBody->GetMinGlobal());
		minMax.push_back(rigidBody->GetMaxGlobal());
	}

	//makes a rigidbody the size of the entire world
	MyRigidBody* rigidBody = new MyRigidBody(minMax);

	//gets the halfwidth of the bounding box.
	vector3 halfWidth = rigidBody->GetHalfWidth();
	float max = halfWidth.x;
	for (int i = 1; i < 3; i++)
	{
		if (max < halfWidth[i])
		{
			max = halfWidth[i];
		}
	}

	//we dont need the rigidbody anymore.
	vector3 rbCenter = rigidBody->GetCenterLocal();
	minMax.clear();
	SafeDelete(rigidBody);

	//this sets up the octree's current sizing, mins and maxes, and the center point.
	m_fSize = max * 2.0f;
	m_v3Center = rbCenter;
	m_v3Min = m_v3Center - (vector3(max));
	m_v3Max = m_v3Center + (vector3(max));
	m_uOctantCount++;

	//at the end, we create the octree and store all of the children in there.
	ConstructTree(m_uMaxLevel);
}

MyOctant::~MyOctant(void)
{
	Release();
}

//this is the constructor for non-root octants, as all of the other required information will have already been known.
MyOctant::MyOctant(vector3 a_v3Center, float a_fSize)
{
	Init();
	m_v3Center = a_v3Center;
	m_fSize = a_fSize;
	m_v3Min = m_v3Center - (vector3(m_fSize) / 2.0f);
	m_v3Max = m_v3Center + (vector3(m_fSize) / 2.0f);

	m_uOctantCount++;
}


MyOctant::MyOctant(MyOctant const& other)
{
	m_uChildren = other.m_uChildren;
	m_v3Center = other.m_v3Center;
	m_v3Min = other.m_v3Min;
	m_v3Max = other.m_v3Max;
	m_fSize = other.m_fSize;
	m_uID = other.m_uID;
	m_uLevel = other.m_uLevel;
	m_pParent = other.m_pParent;
	m_pRoot = other.m_pRoot;
	m_lChild = other.m_lChild;
	m_pMeshMngr = MeshManager::GetInstance();
	m_pEntityMngr = MyEntityManager::GetInstance();
	for (uint i = 0; i < 8; i++)
	{
		m_pChild[i] = other.m_pChild[i];
	}
}

MyOctant& MyOctant::operator=(MyOctant const& other) 
{ 
	if (this != &other)
	{
		Release();
		Init();
		MyOctant tempOct(other);
		Swap(tempOct);
	}
	return *this; 
}

void MyOctant::Swap(MyOctant& other)
{
	std::swap(m_uChildren, other.m_uChildren);
	std::swap(m_fSize, other.m_fSize);
	std::swap(m_uID, other.m_uID);
	std::swap(m_pRoot, other.m_pRoot);
	std::swap(m_lChild, other.m_lChild);
	std::swap(m_v3Center, other.m_v3Center);
	std::swap(m_v3Min, other.m_v3Min);
	std::swap(m_v3Max, other.m_v3Max);
	m_pMeshMngr = MeshManager::GetInstance();
	m_pEntityMngr = MyEntityManager::GetInstance();
	std::swap(m_uLevel, other.m_uLevel);
	std::swap(m_pParent, other.m_pParent);
	for (uint i = 0; i < 0; i++)
	{
		std::swap(m_pChild[i], other.m_pChild[i]);
	}
}

float MyOctant::GetSize(void)
{
	return m_fSize;
}

vector3 MyOctant::GetCenterGlobal(void)
{
	return m_v3Center;
}

vector3 MyOctant::GetMinGlobal(void)
{
	return m_v3Min;
}

vector3 MyOctant::GetMaxGlobal(void)
{
	return m_v3Max;
}

//checks for AABB collision between objects. since the objects are all non-rotated/scaled, we assume they're static for collision detection.
bool MyOctant::IsColliding(uint a_uRBIndex)
{
	uint numObjects = m_pEntityMngr->GetEntityCount();
	if (a_uRBIndex >= numObjects)
		return false;
	//only need to do AABB for the octree since we dont have it coded to scale or rotate at all. plus, all of the minecraft blocks use AABB.
	MyEntity* entity = m_pEntityMngr->GetEntity(a_uRBIndex);
	MyRigidBody* rigidBody = entity->GetRigidBody();
	
	vector3 min = rigidBody->GetMinGlobal();
	vector3 max = rigidBody->GetMaxGlobal();

	if (m_v3Min.x > max.x)
		return false;
	if (m_v3Max.x < min.x)
		return false;
	if (m_v3Min.y > max.y)
		return false;
	if (m_v3Max.y < min.y)
		return false;
	if (m_v3Min.z > max.z)
		return false;
	if (m_v3Max.x < min.z)
		return false;

	//it passed all of those tests, so it must be colliding.
	return true;
}
//display method for higher levels of the octant. first checks for subdivision level.
void MyOctant::Display(uint a_nIndex, vector3 a_v3Color)
{
	if (m_uID == a_nIndex)
	{
		m_pMeshMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) * 
			glm::scale(vector3(m_fSize)), a_v3Color, RENDER_WIRE);
		return;
	}
	for (uint i = 0; i < m_uChildren; i++)
	{
		m_pChild[i]->Display(a_nIndex);
	}
}

//display method that first recursively checks for children before drawing.
void MyOctant::Display(vector3 a_v3Color)
{
	for (uint i = 0; i < m_uChildren; i++)
	{
		m_pChild[i]->Display(a_v3Color);
	}
	m_pMeshMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) 
		* glm::scale(vector3(m_fSize)), a_v3Color, RENDER_WIRE);
}

//displays all octants that dont have children.
void MyOctant::DisplayLeafs(vector3 a_v3Color)
{
	int numLeafs = m_lChild.size();
	for (uint i = 0; i < numLeafs; i++)
	{
		m_lChild[i]->DisplayLeafs(a_v3Color);
	}
	m_pMeshMngr->AddWireCubeToRenderList(glm::translate(IDENTITY_M4, m_v3Center) 
		* glm::scale(vector3(m_fSize)), a_v3Color, RENDER_WIRE);
}

//clears the list of entities inside of this octant.
void MyOctant::ClearEntityList(void)
{
	for (uint i = 0; i < m_uChildren; i++)
	{
		m_pChild[i]->ClearEntityList();
	}
	m_EntityList.clear();
}

//if the number of entities within the bounds of this octant are more than 5, we create a sub division.
//the subdivision's size is calculated based on the parent octant.
//the new subdivision also gets a reference to the root, its parent, and the current level of subdivision.
void MyOctant::Subdivide(void)
{
	if (m_uLevel >= m_uMaxLevel)
		return;
	if (m_uChildren != 0)
		return;

	m_uChildren = 8;
	float size = m_fSize / 4.0f;
	float size1 = size * 2.0f;
	vector3 center;
	
	center = m_v3Center;
	center.x -= size;
	center.y -= size;
	center.z -= size;
	m_pChild[0] = new MyOctant(center, size1);
	
	center.x += size1;
	m_pChild[1] = new MyOctant(center, size1);

	center.z += size1;
	m_pChild[2] = new MyOctant(center, size1);

	center.x -= size1;
	m_pChild[3] = new MyOctant(center, size1);

	center.y += size1;
	m_pChild[4] = new MyOctant(center, size1);

	center.z -= size1;
	m_pChild[5] = new MyOctant(center, size1);

	center.x += size1;
	m_pChild[6] = new MyOctant(center, size1);

	center.z += size1;
	m_pChild[7] = new MyOctant(center, size1);

	for (uint i = 0; i < 8; i++)
	{
		m_pChild[i]->m_pRoot = m_pRoot;
		m_pChild[i]->m_pParent = this;
		m_pChild[i]->m_uLevel = m_uLevel + 1;
		if (m_pChild[i]->ContainsMoreThan(m_uIdealEntityCount))
		{
			m_pChild[i]->Subdivide();
		}
	}
}

//returns a child if its valid.
MyOctant* MyOctant::GetChild(uint a_nChild)
{
	if(a_nChild > 7) return nullptr;
	return m_pChild[a_nChild];
}

//gets the parent of this octant, returns nullptr if the node is root.
MyOctant* MyOctant::GetParent(void)
{
	return m_pParent;
}

//returns whether or not this octant has children.
bool MyOctant::IsLeaf(void)
{
	if (m_uChildren == 0)
		return true;
	else
		return false;
}

//checks if there's more than a specific number of collisions inside of the current octant's bounds.
bool MyOctant::ContainsMoreThan(uint a_nEntities)
{
	uint numBounds = 0;
	uint numObjects = m_pEntityMngr->GetEntityCount();
	for (uint i = 0; i < numObjects; i++)
	{
		if (IsColliding(i))
			numBounds++;
		if (numBounds > a_nEntities)
			return true;
	}
	return false;
}

//deletes all of the children of this octant
void MyOctant::KillBranches(void)
{
	for (uint i = 0; i < m_uChildren; i++)
	{
		m_pChild[i]->KillBranches();
		delete m_pChild[i];
		m_pChild[i] = nullptr;
	}
	
	m_uChildren = 0;
}

//creates the tree of octants and subdivides if needed.
void MyOctant::ConstructTree(uint a_nMaxLevel)
{
	if (m_uLevel != 0)
		return;

	m_uMaxLevel = a_nMaxLevel;
	m_uOctantCount = 1;
	m_EntityList.clear();

	KillBranches();
	m_lChild.clear();

	if (ContainsMoreThan(m_uIdealEntityCount))
	{
		Subdivide();
	}

	AssignIDtoEntity();
	ConstructList();
}

//asigns a unique identifier to each entity in this octant's bounds
void MyOctant::AssignIDtoEntity(void)
{
	for (uint i = 0; i < m_uChildren; i++)
	{
		m_pChild[i]->AssignIDtoEntity();
	}
	if (m_uChildren == 0)
	{
		uint numEntities = m_pEntityMngr->GetEntityCount();
		for (uint i = 0; i < numEntities; i++)
		{
			if (IsColliding(i))
			{
				m_EntityList.push_back(i);
				m_pEntityMngr->AddDimension(i, m_uID);
			}
		}
	}
}

//gets the number of octants that currently exist.
uint MyOctant::GetOctantCount(void)
{
	return m_uOctantCount;
}

//deletes and resets all of the data.
void MyOctant::Release(void)
{
	if (m_uLevel == 0)
	{
		KillBranches();
	}
	m_uChildren = 0;
	m_fSize = 0.0f;
	m_EntityList.clear();
	m_lChild.clear();
}

//creates a list of nodes of the children of each node.
void MyOctant::ConstructList(void)
{
	for (uint i = 0; i < m_uChildren; i++)
	{
		m_pChild[i]->ConstructList();
	}
	if (m_EntityList.size() > 0)
	{
		m_pRoot->m_lChild.push_back(this);
	}
}
