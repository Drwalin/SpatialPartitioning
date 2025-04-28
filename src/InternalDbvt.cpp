/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///btDbvt implementation by Nathanael Presson
/// 2025 modified and rewritten by Marek Zalewski aka Drwalin

#include <limits>

#include "../include/spatial_partitioning/InternalDbvt.hpp"

namespace spp
{
	
	
typedef std::vector<btDbvtNode*> tNodeArray;
typedef std::vector<const btDbvtNode*> tConstNodeArray;

//
struct btDbvtNodeEnumerator : btDbvt::ICollide
{
	tConstNodeArray nodes;
	void Process(const btDbvtNode* n) { nodes.push_back(n); }
};

//
static inline int indexof(const btDbvtNode* node)
{
	return (node->parent->childs[1] == node);
}

//
static inline Aabb merge(const Aabb& a,
									  const Aabb& b)
{
#ifdef BT_USE_SSE
	ATTRIBUTE_ALIGNED16(char locals[sizeof(btDbvtAabbMm)]);
	Aabb* ptr = (Aabb*)locals;
	Aabb& res = *ptr;
#else
	Aabb res;
#endif
	Merge(a, b, res);
	return (res);
}

// volume+edge lengths
static inline float size(const Aabb& a)
{
	const glm::vec3 edges = a.GetSizes();
	return (edges.x * edges.y * edges.z +
			edges.x + edges.y + edges.z);
}

//
static inline void deletenode(btDbvt* pdbvt,
								   btDbvtNode* node)
{
	delete pdbvt->m_free;
	pdbvt->m_free = node;
}

//
static void recursedeletenode(btDbvt* pdbvt,
							  btDbvtNode* node)
{
	if (node == 0) return;
	if (!node->isleaf())
	{
		recursedeletenode(pdbvt, node->childs[0]);
		recursedeletenode(pdbvt, node->childs[1]);
	}
	if (node == pdbvt->m_root) pdbvt->m_root = 0;
	deletenode(pdbvt, node);
}

//
static inline btDbvtNode* createnode(btDbvt* pdbvt,
										  btDbvtNode* parent,
										  void* data)
{
	btDbvtNode* node;
	if (pdbvt->m_free)
	{
		node = pdbvt->m_free;
		pdbvt->m_free = 0;
	}
	else
	{
		node = new btDbvtNode();
	}
	node->parent = parent;
	node->data = data;
	node->childs[1] = 0;
	return (node);
}

//
static inline btDbvtNode* createnode(btDbvt* pdbvt,
										  btDbvtNode* parent,
										  const Aabb& volume,
										  void* data)
{
	btDbvtNode* node = createnode(pdbvt, parent, data);
	node->volume = volume;
	return (node);
}

//
static inline btDbvtNode* createnode(btDbvt* pdbvt,
										  btDbvtNode* parent,
										  const Aabb& volume0,
										  const Aabb& volume1,
										  void* data)
{
	btDbvtNode* node = createnode(pdbvt, parent, data);
	Merge(volume0, volume1, node->volume);
	return (node);
}

//
static void insertleaf(btDbvt* pdbvt,
					   btDbvtNode* root,
					   btDbvtNode* leaf)
{
	if (!pdbvt->m_root)
	{
		pdbvt->m_root = leaf;
		leaf->parent = 0;
	}
	else
	{
		if (!root->isleaf())
		{
			do
			{
				root = root->childs[Select(leaf->volume,
										   root->childs[0]->volume,
										   root->childs[1]->volume)];
			} while (!root->isleaf());
		}
		btDbvtNode* prev = root->parent;
		btDbvtNode* node = createnode(pdbvt, prev, leaf->volume, root->volume, 0);
		if (prev)
		{
			prev->childs[indexof(root)] = node;
			node->childs[0] = root;
			root->parent = node;
			node->childs[1] = leaf;
			leaf->parent = node;
			do
			{
				if (!Contain(prev->volume, node->volume))
					Merge(prev->childs[0]->volume, prev->childs[1]->volume, prev->volume);
				else
					break;
				node = prev;
			} while (0 != (prev = node->parent));
		}
		else
		{
			node->childs[0] = root;
			root->parent = node;
			node->childs[1] = leaf;
			leaf->parent = node;
			pdbvt->m_root = node;
		}
	}
}

//
static btDbvtNode* removeleaf(btDbvt* pdbvt,
							  btDbvtNode* leaf)
{
	if (leaf == pdbvt->m_root)
	{
		pdbvt->m_root = 0;
		return (0);
	}
	else
	{
		btDbvtNode* parent = leaf->parent;
		btDbvtNode* prev = parent->parent;
		btDbvtNode* sibling = parent->childs[1 - indexof(leaf)];
		if (prev)
		{
			prev->childs[indexof(parent)] = sibling;
			sibling->parent = prev;
			deletenode(pdbvt, parent);
			while (prev)
			{
				const Aabb pb = prev->volume;
				Merge(prev->childs[0]->volume, prev->childs[1]->volume, prev->volume);
				if (NotEqual(pb, prev->volume))
				{
					prev = prev->parent;
				}
				else
					break;
			}
			return (prev ? prev : pdbvt->m_root);
		}
		else
		{
			pdbvt->m_root = sibling;
			sibling->parent = 0;
			deletenode(pdbvt, parent);
			return (pdbvt->m_root);
		}
	}
}

//
static void fetchleaves(btDbvt* pdbvt,
						btDbvtNode* root,
						tNodeArray& leaves,
						int depth = -1)
{
	if (root->isinternal() && depth)
	{
		fetchleaves(pdbvt, root->childs[0], leaves, depth - 1);
		fetchleaves(pdbvt, root->childs[1], leaves, depth - 1);
		deletenode(pdbvt, root);
	}
	else
	{
		leaves.push_back(root);
	}
}

//
static bool leftOfAxis(const btDbvtNode* node,
					   const glm::vec3& org,
					   const glm::vec3& axis)
{
	return glm::dot(axis, node->volume.GetCenter() - org) <= 0;
}

// Partitions leaves such that leaves[0, n) are on the
// left of axis, and leaves[n, count) are on the right
// of axis. returns N.
static int split(btDbvtNode** leaves,
				 int count,
				 const glm::vec3& org,
				 const glm::vec3& axis)
{
	int begin = 0;
	int end = count;
	for (;;)
	{
		while (begin != end && leftOfAxis(leaves[begin], org, axis))
		{
			++begin;
		}

		if (begin == end)
		{
			break;
		}

		while (begin != end && !leftOfAxis(leaves[end - 1], org, axis))
		{
			--end;
		}

		if (begin == end)
		{
			break;
		}

		// swap out of place nodes
		--end;
		btDbvtNode* temp = leaves[begin];
		leaves[begin] = leaves[end];
		leaves[end] = temp;
		++begin;
	}

	return begin;
}

//
static Aabb bounds(btDbvtNode** leaves,
						   int count)
{
#ifdef BT_USE_SSE
	ATTRIBUTE_ALIGNED16(char locals[sizeof(Aabb)]);
	Aabb* ptr = (Aabb*)locals;
	Aabb& volume = *ptr;
	volume = leaves[0]->volume;
#else
	Aabb volume = leaves[0]->volume;
#endif
	for (int i = 1, ni = count; i < ni; ++i)
	{
		Merge(volume, leaves[i]->volume, volume);
	}
	return (volume);
}

//
static void bottomup(btDbvt* pdbvt,
					 btDbvtNode** leaves,
					 int count)
{
	while (count > 1)
	{
		float minsize = std::numeric_limits<float>::infinity();
		int minidx[2] = {-1, -1};
		for (int i = 0; i < count; ++i)
		{
			for (int j = i + 1; j < count; ++j)
			{
				const float sz = size(merge(leaves[i]->volume, leaves[j]->volume));
				if (sz < minsize)
				{
					minsize = sz;
					minidx[0] = i;
					minidx[1] = j;
				}
			}
		}
		btDbvtNode* n[] = {leaves[minidx[0]], leaves[minidx[1]]};
		btDbvtNode* p = createnode(pdbvt, 0, n[0]->volume, n[1]->volume, 0);
		p->childs[0] = n[0];
		p->childs[1] = n[1];
		n[0]->parent = p;
		n[1]->parent = p;
		leaves[minidx[0]] = p;
		leaves[minidx[1]] = leaves[count - 1];
		--count;
	}
}

//
static btDbvtNode* topdown(btDbvt* pdbvt,
						   btDbvtNode** leaves,
						   int count,
						   int bu_treshold)
{
	static const glm::vec3 axis[] = {glm::vec3(1, 0, 0),
									 glm::vec3(0, 1, 0),
									 glm::vec3(0, 0, 1)};
	assert(bu_treshold > 2);
	if (count > 1)
	{
		if (count > bu_treshold)
		{
			const Aabb vol = bounds(leaves, count);
			const glm::vec3 org = vol.GetCenter();
			int partition;
			int bestaxis = -1;
			int bestmidp = count;
			int splitcount[3][2] = {{0, 0}, {0, 0}, {0, 0}};
			int i;
			for (i = 0; i < count; ++i)
			{
				const glm::vec3 x = leaves[i]->volume.GetCenter() - org;
				for (int j = 0; j < 3; ++j)
				{
					++splitcount[j][glm::dot(x, axis[j]) > 0 ? 1 : 0];
				}
			}
			for (i = 0; i < 3; ++i)
			{
				if ((splitcount[i][0] > 0) && (splitcount[i][1] > 0))
				{
					const int midp = (int)glm::abs(float(splitcount[i][0] - splitcount[i][1]));
					if (midp < bestmidp)
					{
						bestaxis = i;
						bestmidp = midp;
					}
				}
			}
			if (bestaxis >= 0)
			{
				partition = split(leaves, count, org, axis[bestaxis]);
				assert(partition != 0 && partition != count);
			}
			else
			{
				partition = count / 2 + 1;
			}
			btDbvtNode* node = createnode(pdbvt, 0, vol, 0);
			node->childs[0] = topdown(pdbvt, &leaves[0], partition, bu_treshold);
			node->childs[1] = topdown(pdbvt, &leaves[partition], count - partition, bu_treshold);
			node->childs[0]->parent = node;
			node->childs[1]->parent = node;
			return (node);
		}
		else
		{
			bottomup(pdbvt, leaves, count);
			return (leaves[0]);
		}
	}
	return (leaves[0]);
}

//
static inline btDbvtNode* sort(btDbvtNode* n, btDbvtNode*& r)
{
	btDbvtNode* p = n->parent;
	assert(n->isinternal());
	if (p > n)
	{
		const int i = indexof(n);
		const int j = 1 - i;
		btDbvtNode* s = p->childs[j];
		btDbvtNode* q = p->parent;
		assert(n == p->childs[i]);
		if (q)
			q->childs[indexof(p)] = n;
		else
			r = n;
		s->parent = n;
		p->parent = n;
		n->parent = q;
		p->childs[0] = n->childs[0];
		p->childs[1] = n->childs[1];
		n->childs[0]->parent = p;
		n->childs[1]->parent = p;
		n->childs[i] = p;
		n->childs[j] = s;
		std::swap(p->volume, n->volume);
		return (p);
	}
	return (n);
}

#if 0
static inline btDbvtNode*	walkup(btDbvtNode* n,int count)
{
	while(n&&(count--)) n=n->parent;
	return(n);
}
#endif

//
// Api
//

//
btDbvt::btDbvt()
{
	m_root = 0;
	m_free = 0;
	m_lkhd = -1;
	m_leaves = 0;
	m_opath = 0;
}

//
btDbvt::~btDbvt()
{
	clear();
}

//
void btDbvt::clear()
{
	if (m_root)
		recursedeletenode(this, m_root);
	delete m_free;
	m_free = 0;
	m_lkhd = -1;
	m_opath = 0;
}

//
void btDbvt::optimizeBottomUp()
{
	if (m_root)
	{
		tNodeArray leaves;
		leaves.reserve(m_leaves);
		fetchleaves(this, m_root, leaves);
		bottomup(this, &leaves[0], leaves.size());
		m_root = leaves[0];
	}
}

//
void btDbvt::optimizeTopDown(int bu_treshold)
{
	if (m_root)
	{
		tNodeArray leaves;
		leaves.reserve(m_leaves);
		fetchleaves(this, m_root, leaves);
		m_root = topdown(this, &leaves[0], leaves.size(), bu_treshold);
	}
}

//
void btDbvt::optimizeIncremental(int passes)
{
	if (passes < 0) passes = m_leaves;
	if (m_root && (passes > 0))
	{
		do
		{
			btDbvtNode* node = m_root;
			unsigned bit = 0;
			while (node->isinternal())
			{
				node = sort(node, m_root)->childs[(m_opath >> bit) & 1];
				bit = (bit + 1) & (sizeof(unsigned) * 8 - 1);
			}
			update(node);
			++m_opath;
		} while (--passes);
	}
}

//
btDbvtNode* btDbvt::insert(const Aabb& volume, void* data)
{
	btDbvtNode* leaf = createnode(this, 0, volume, data);
	insertleaf(this, m_root, leaf);
	++m_leaves;
	return (leaf);
}

//
void btDbvt::update(btDbvtNode* leaf, int lookahead)
{
	btDbvtNode* root = removeleaf(this, leaf);
	if (root)
	{
		if (lookahead >= 0)
		{
			for (int i = 0; (i < lookahead) && root->parent; ++i)
			{
				root = root->parent;
			}
		}
		else
			root = m_root;
	}
	insertleaf(this, root, leaf);
}

//
void btDbvt::update(btDbvtNode* leaf, Aabb& volume)
{
	btDbvtNode* root = removeleaf(this, leaf);
	if (root)
	{
		if (m_lkhd >= 0)
		{
			for (int i = 0; (i < m_lkhd) && root->parent; ++i)
			{
				root = root->parent;
			}
		}
		else
			root = m_root;
	}
	leaf->volume = volume;
	insertleaf(this, root, leaf);
}

//
bool btDbvt::update(btDbvtNode* leaf, Aabb& volume, float margin)
{
	if (Contain(leaf->volume, volume)) return (false);
	volume = volume.Expanded(margin);
	update(leaf, volume);
	return (true);
}

//
void btDbvt::remove(btDbvtNode* leaf)
{
	removeleaf(this, leaf);
	deletenode(this, leaf);
	--m_leaves;
}
	
}
