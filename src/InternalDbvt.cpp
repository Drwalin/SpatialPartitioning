/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use
of this software. Permission is granted to anyone to use this software for any
purpose, including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim
that you wrote the original software. If you use this software in a product, an
acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
/// btDbvt implementation by Nathanael Presson
///  2025 modified and rewritten by Marek Zalewski aka Drwalin

#include <cstdio>

#include "../include/spatial_partitioning/InternalDbvt.hpp"

#include "../include/spatial_partitioning/Dbvt.hpp"

namespace spp
{
typedef std::vector<uint32_t> tNodeArray;

inline void SignedExpand(Aabb &aabb, const glm::vec3 &e)
{
	if (e.x > 0)
		aabb.max.x = aabb.max.x + e.x;
	else
		aabb.min.x = aabb.min.x + e.x;
	if (e.y > 0)
		aabb.max.y = aabb.max.y + e.y;
	else
		aabb.min.y = aabb.min.y + e.y;
	if (e.z > 0)
		aabb.max.z = aabb.max.z + e.z;
	else
		aabb.min.z = aabb.min.z + e.z;
}

inline bool Contain(const Aabb &aabb, const Aabb &a)
{
	return aabb.ContainsAll(a);
	/*
	return ((aabb.min.x <= a.min.x) && (aabb.min.y <= a.min.y) &&
			(aabb.min.z <= a.min.z) && (aabb.max.x >= a.max.x) &&
			(aabb.max.y >= a.max.y) && (aabb.max.z >= a.max.z));
			*/
}

inline float Proximity(const Aabb &a, const Aabb &b)
{
	const glm::vec3 d = (a.min + a.max) - (b.min + b.max);
	return (glm::abs(d.x) + glm::abs(d.y) + glm::abs(d.z));
}

inline int Select(const Aabb &o, const Aabb &a, const Aabb &b)
{
	return (Proximity(o, a) < Proximity(o, b) ? 0 : 1);
}

inline void Merge(const Aabb &a, const Aabb &b, Aabb &r) { r = a + b; }

inline bool NotEqual(const Aabb &a, const Aabb &b)
{
	return ((a.min.x != b.min.x) || (a.min.y != b.min.y) ||
			(a.min.z != b.min.z) || (a.max.x != b.max.x) ||
			(a.max.y != b.max.y) || (a.max.z != b.max.z));
}

int btDbvt::indexof(uint32_t node) const
{
	if (isLeaf(node)) {
		return indexofLeaf(node);
	} else {
		return indexofNode(node);
	}
}

int btDbvt::indexofLeaf(uint32_t leaf) const
{
	assert(leaf && isLeaf(leaf));
	return nodes[getLeafParent(leaf)].childs[1] == leaf ? 1 : 0;
}

int btDbvt::indexofNode(uint32_t node) const
{
	assert(node && !isLeaf(node));
	return nodes[getNodeParent(node)].childs[1] == node;
}

bool btDbvt::isLeaf(uint32_t node) { return node & OFFSET; }
bool btDbvt::isInternal(uint32_t node) { return !isLeaf(node); }

Aabb btDbvt::getAabb(uint32_t node) const
{
	if (isLeaf(node)) {
		return getLeafAabb(node);
	} else {
		return getNodeAabb(node);
	}
}

uint32_t btDbvt::getParent(uint32_t node) const
{
	if (isLeaf(node)) {
		return getLeafParent(node);
	} else {
		return getNodeParent(node);
	}
}

Aabb btDbvt::getLeafAabb(uint32_t leaf) const
{
	assert(leaf && isLeaf(leaf));
	return (*ents)[leaf - OFFSET].aabb;
}

uint32_t btDbvt::getLeafParent(uint32_t leaf) const
{
	assert(leaf && isLeaf(leaf));
	return (*ents)[leaf - OFFSET].parent;
}

Aabb btDbvt::getNodeAabb(uint32_t node) const
{
	assert(node && !isLeaf(node));
	return nodes[node].aabb;
}

uint32_t btDbvt::getNodeParent(uint32_t node) const
{
	assert(node && !isLeaf(node));
	return nodes[node].parent;
}

void btDbvt::setParent(uint32_t node, uint32_t parent)
{
	if (isLeaf(node)) {
		setLeafParent(node, parent);
	} else {
		setNodeParent(node, parent);
	}
}

void btDbvt::setNodeParent(uint32_t node, uint32_t parent)
{
	nodes[node].parent = parent;
}

void btDbvt::setLeafParent(uint32_t leaf, uint32_t parent)
{
	(*ents)[leaf - OFFSET].parent = parent;
}

uint32_t btDbvt::getLeafId(uint32_t entityOffset)
{
	assert(entityOffset && !isLeaf(entityOffset));
	return entityOffset + OFFSET;
}

EntityType btDbvt::getLeafEntity(uint32_t leaf) const
{
	assert(leaf && isLeaf(leaf));
	return (*ents)[leaf - OFFSET].entity;
}

MaskType btDbvt::getLeafMask(uint32_t leaf) const
{
	assert(leaf && isLeaf(leaf));
	return (*ents)[leaf - OFFSET].mask;
}

void btDbvt::deletenode(const uint32_t node)
{
	assert(node && !isLeaf(node));
	if (node + 1 < nodes.size()) {
		const uint32_t movingId = nodes.size() - 1;
		const uint32_t parent = getNodeParent(movingId);
		for (int i = 0; i < 2; ++i) {
			setParent(nodes[movingId].childs[i], node);
		}
		nodes[node] = nodes[movingId];
		if (parent == 0) {
			assert(movingId == rootId);
			rootId = node;
		} else {
			assert(movingId != rootId);
			int i = indexofNode(movingId);
			nodes[parent].childs[i] = node;
		}
	}
	nodes.pop_back();
}

uint32_t btDbvt::createnode(uint32_t parent, const Aabb &aabb)
{
	uint32_t node = nodes.size();
	nodes.push_back({aabb, parent});
	return node;
}

uint32_t btDbvt::createnode(uint32_t parent) { return createnode(parent, {}); }

uint32_t btDbvt::createnode(uint32_t parent, const Aabb &aabb0,
							const Aabb &aabb1)
{
	return createnode(parent, aabb0 + aabb1);
}

void btDbvt::insertleaf(uint32_t root, const uint32_t leaf, const Aabb &aabb)
{
	assert(leaf && isLeaf(leaf));
	if (!rootId) {
		rootId = leaf;
		setParent(leaf, 0);
	} else {
		while (!isLeaf(root)) {
			int select = Select(aabb, getAabb(nodes[root].childs[0]),
								getAabb(nodes[root].childs[1]));
			root = nodes[root].childs[select];
		}
		uint32_t sibling = root;
		uint32_t parent = getLeafParent(sibling);
		assert(parent < nodes.size());
		uint32_t node = createnode(parent, aabb, getLeafAabb(sibling));
		if (parent) {
			nodes[parent].childs[indexofLeaf(sibling)] = node;
			nodes[node].childs[0] = sibling;
			nodes[node].childs[1] = leaf;
			setLeafParent(sibling, node);
			setLeafParent(leaf, node);
			Aabb ab = getNodeAabb(node);
			do {
				int i = indexof(node);
				nodes[parent].aabb = ab =
					ab + getAabb(nodes[parent].childs[1 - i]);
				node = parent;
				parent = getNodeParent(node);
			} while (parent);
		} else {
			nodes[node].childs[0] = sibling;
			nodes[node].childs[1] = leaf;
			setLeafParent(sibling, node);
			setLeafParent(leaf, node);
			rootId = node;
		}
	}
}

uint32_t btDbvt::removeleaf(const uint32_t leaf)
{
	if (leaf == rootId) {
		rootId = 0;
		setLeafParent(leaf, OFFSET);
		return 0;
	} else {
		uint32_t parent = getLeafParent(leaf);
		uint32_t prev = getNodeParent(parent);
		uint32_t sibling = nodes[parent].childs[1 - indexofLeaf(leaf)];
		if (prev) {
			nodes[prev].childs[indexofNode(parent)] = sibling;
			setParent(sibling, prev);
			while (prev) {
				const Aabb pb = getNodeAabb(prev);
				nodes[prev].aabb = getAabb(nodes[prev].childs[0]) +
								   getAabb(nodes[prev].childs[1]);
				if (NotEqual(pb, nodes[prev].aabb)) {
					prev = getNodeParent(prev);
				} else
					break;
			}
			deletenode(parent);
			if (prev == nodes.size()) {
				prev = parent;
			}
			setLeafParent(leaf, OFFSET + 1);
			return prev ? prev : rootId;
		} else {
			rootId = sibling;
			setParent(sibling, 0);
			deletenode(parent);
			setLeafParent(leaf, OFFSET + 2);
			return rootId;
		}
	}
}

uint32_t btDbvt::sort(uint32_t n, uint32_t &r)
{
	uint32_t p = getParent(n);
	assert(n && isInternal(n) && !isLeaf(n));
	if (p > n) {
		const int i = indexof(n);
		const int j = 1 - i;
		uint32_t s = nodes[p].childs[j];
		uint32_t q = getParent(p);
		assert(n == nodes[p].childs[i]);
		if (q)
			nodes[q].childs[indexof(p)] = n;
		else
			r = n;
		setParent(s, n);
		setNodeParent(p, n);
		setNodeParent(n, q);
		nodes[p].childs[0] = nodes[n].childs[0];
		nodes[p].childs[1] = nodes[n].childs[1];
		setParent(nodes[n].childs[0], p);
		setParent(nodes[n].childs[1], p);
		nodes[n].childs[i] = p;
		nodes[n].childs[j] = s;
		std::swap(nodes[p].aabb, nodes[n].aabb);
		return (p);
	}
	return (n);
}

btDbvt::btDbvt(spp::Dbvt *dbvt)
{
	ents = &(dbvt->ents);
	nodes.resize(1);
	rootId = 0;
	m_opath = 0;
}

btDbvt::~btDbvt() { clear(); }

void btDbvt::clear()
{
	nodes.resize(1);
	m_opath = 0;
}

void btDbvt::optimizeIncremental(int passes)
{
	if (passes < 0)
		passes = ents->Size();
	if (rootId && (passes > 0)) {
		do {
			uint32_t node = rootId;
			unsigned bit = 0;
			while (isInternal(node)) {
				node = nodes[sort(node, rootId)].childs[(m_opath >> bit) & 1];
				bit = (bit + 1) & (sizeof(unsigned) * 8 - 1);
			}
			updateLeaf(node);
			++m_opath;
		} while (--passes);
	}
}

void btDbvt::insert(const Aabb &aabb, uint32_t entityOffset)
{
	insertleaf(rootId, getLeafId(entityOffset), aabb);
}

void btDbvt::updateLeaf(uint32_t leaf, int lookahead)
{
	uint32_t root = removeleaf(leaf);
	if (root) {
		if (lookahead >= 0) {
			if ((lookahead > 0) && getParent(root)) {
				root = getParent(root);
				for (int i = 1; (i < lookahead) && getNodeParent(root); ++i) {
					root = getNodeParent(root);
				}
			}
		} else
			root = rootId;
	}
	insertleaf(root, leaf, getLeafAabb(leaf));
	IsTreeValid();
}

void btDbvt::updateEntityOffset(uint32_t entityOffset, const Aabb &aabb)
{
	uint32_t leaf = getLeafId(entityOffset);
	uint32_t root = removeleaf(leaf);
	if (root) {
		root = rootId;
	}

	assert(!NotEqual((*ents)[entityOffset].aabb, aabb));
	{
		// THIS SHOULD BE NOT NECESSARY
		(*ents)[entityOffset].aabb = aabb;
	}

	insertleaf(root, leaf, aabb);
	IsTreeValid();
}

bool btDbvt::updateEntityOffset(uint32_t entityOffset, const Aabb &aabb,
								float margin)
{
	uint32_t leaf = getLeafId(entityOffset);
	if (Contain(getLeafAabb(leaf), aabb))
		return (false);
	updateEntityOffset(leaf, aabb.Expanded(margin));
	return (true);
}

void btDbvt::remove(uint32_t entityOffset)
{
	uint32_t leaf = getLeafId(entityOffset);
	removeleaf(leaf);
}

void btDbvt::updateOffsetOfEntity(uint32_t oldEntityOffset,
								  uint32_t newEntityOffset)
{
	uint32_t oldLeaf = getLeafId(oldEntityOffset);
	uint32_t newLeaf = getLeafId(newEntityOffset);
	if (rootId == oldLeaf) {
		rootId = newLeaf;
	} else {
		uint32_t parent = getLeafParent(oldLeaf);
		int i = indexof(oldLeaf);
		nodes[parent].childs[i] = newLeaf;
	}
}

void btDbvt::collideTV(IntersectionCallback &cb)
{
	if (rootId) {
		stack.clear();
		stack.push_back(rootId);
		int i = 0;
		do {
			++i;
			uint32_t node = stack.back();
			printf("aabb it[%i]: %u\n", i, node);
			stack.pop_back();
			if (isLeaf(node)) {
				if (getLeafMask(node) & cb.mask) {
					cb.ExecuteIfRelevant(getLeafAabb(node),
										 getLeafEntity(node));
				}
			} else {
				cb.nodesTestedCount++;
				if (cb.IsRelevant(getNodeAabb(node))) {
					stack.push_back(nodes[node].childs[0]);
					stack.push_back(nodes[node].childs[1]);
				}
			}
		} while (!stack.empty());
	}
	printf("end\n");
}

void btDbvt::rayTestInternal(RayCallback &cb)
{
	if (rootId) {
		stack.clear();
		stack.push_back(rootId);
		do {
			uint32_t node = stack.back();
			stack.pop_back();
			if (isLeaf(node)) {
				if (getLeafMask(node) & cb.mask) {
					cb.ExecuteIfRelevant(getLeafAabb(node),
										 getLeafEntity(node));
				}
			} else {
				cb.nodesTestedCount++;
				if (cb.IsRelevant(getNodeAabb(node))) {
					stack.push_back(nodes[node].childs[0]);
					stack.push_back(nodes[node].childs[1]);
				}
			}
		} while (!stack.empty());
	}
}

size_t btDbvt::GetMemoryUsage() const
{
	return stack.capacity() * sizeof(uint32_t) +
		   nodes.capacity() * sizeof(NodeData);
}

static void BreakPoint() {}

int btDbvt::IsTreeValid(uint32_t node) const
{
	if (rootId == 0) {
		return 0;
	}
	if (node == 0) {
		node = rootId;
	}
	if (isLeaf(node)) {
		return 0;
	}
	Aabb a = getAabb(node);
	Aabb b = getAabb(nodes[node].childs[0]);
	Aabb c = getAabb(nodes[node].childs[1]);

	Aabb sum = b + c;

	int er = 0;

	if (a != sum) {
		++er;
		BreakPoint();
	}

	return er + IsTreeValid(nodes[node].childs[0]) +
		   IsTreeValid(nodes[node].childs[1]);
}

} // namespace spp
