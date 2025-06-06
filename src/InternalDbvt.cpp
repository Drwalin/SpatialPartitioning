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

#include "../glm/glm/common.hpp"

#include "../include/spatial_partitioning/Dbvt.hpp"

#include "../include/spatial_partitioning/InternalDbvt.hpp"

namespace spp
{
typedef std::vector<uint32_t> tNodeArray;

template <typename Aabb>
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

template <typename Aabb> inline bool Contain(const Aabb &aabb, const Aabb &a)
{
	return aabb.ContainsAll(a);
}

template <typename Aabb> inline float Proximity(const Aabb &a, const Aabb &b)
{
	const glm::vec3 d = (a.min + a.max) - (b.min + b.max);
	return (glm::abs(d.x) + glm::abs(d.y) + glm::abs(d.z));
}

inline int Select(const Aabb &o, const Aabb &a, const Aabb &b)
{
	return (Proximity(o, a) < Proximity(o, b) ? 0 : 1);
}

template <typename Aabb>
inline void Merge(const Aabb &a, const Aabb &b, Aabb &r)
{
	r = a + b;
}

template <typename Aabb> inline bool NotEqual(const Aabb &a, const Aabb &b)
{
	return ((a.min.x != b.min.x) || (a.min.y != b.min.y) ||
			(a.min.z != b.min.z) || (a.max.x != b.max.x) ||
			(a.max.y != b.max.y) || (a.max.z != b.max.z));
}

SPP_TEMPLATE_DECL_OFFSET
int btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::indexof(uint32_t node) const
{
	if (isLeaf(node)) {
		return indexofLeaf(node);
	} else {
		return indexofNode(node);
	}
}

SPP_TEMPLATE_DECL_OFFSET
int btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::indexofLeaf(uint32_t leaf) const
{
	assert(leaf && isLeaf(leaf));
	return nodes[getLeafParent(leaf)].childs[1] == leaf ? 1 : 0;
}

SPP_TEMPLATE_DECL_OFFSET
int btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::indexofNode(uint32_t node) const
{
	assert(node && !isLeaf(node));
	return nodes[getNodeParent(node)].childs[1] == node;
}

SPP_TEMPLATE_DECL_OFFSET
bool btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::isLeaf(uint32_t node)
{
	return node & OFFSET;
}

SPP_TEMPLATE_DECL_OFFSET
bool btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::isInternal(uint32_t node)
{
	return !isLeaf(node);
}

SPP_TEMPLATE_DECL_OFFSET
Aabb btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::getAabb(uint32_t node) const
{
	if (isLeaf(node)) {
		return getLeafAabb(node);
	} else {
		return getNodeAabb(node);
	}
}

SPP_TEMPLATE_DECL_OFFSET
uint32_t btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::getParent(uint32_t node) const
{
	if (isLeaf(node)) {
		return getLeafParent(node);
	} else {
		return getNodeParent(node);
	}
}

SPP_TEMPLATE_DECL_OFFSET
Aabb btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::getLeafAabb(uint32_t leaf) const
{
	assert(leaf && isLeaf(leaf));
	return (*ents)[leaf - OFFSET].aabb;
}

SPP_TEMPLATE_DECL_OFFSET
uint32_t btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::getLeafParent(uint32_t leaf) const
{
	assert(leaf && isLeaf(leaf));
	return (*ents)[leaf - OFFSET].parent;
}

SPP_TEMPLATE_DECL_OFFSET
Aabb btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::getNodeAabb(uint32_t node) const
{
	assert(node && !isLeaf(node));
	return nodes[node].aabb;
}

SPP_TEMPLATE_DECL_OFFSET
uint32_t btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::getNodeParent(uint32_t node) const
{
	assert(node && !isLeaf(node));
	return nodes[node].parent;
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::setParent(uint32_t node, uint32_t parent)
{
	if (isLeaf(node)) {
		setLeafParent(node, parent);
	} else {
		setNodeParent(node, parent);
	}
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::setNodeParent(uint32_t node,
													 uint32_t parent)
{
	nodes[node].parent = parent;
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::setLeafParent(uint32_t leaf,
													 uint32_t parent)
{
	(*ents)[leaf - OFFSET].parent = parent;
}

SPP_TEMPLATE_DECL_OFFSET
uint32_t btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::getLeafId(uint32_t entityOffset)
{
	assert(entityOffset && !isLeaf(entityOffset));
	return entityOffset + OFFSET;
}

SPP_TEMPLATE_DECL_OFFSET
EntityType btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::getLeafEntity(uint32_t leaf) const
{
	assert(leaf && isLeaf(leaf));
	return (*ents)[leaf - OFFSET].entity;
}

SPP_TEMPLATE_DECL_OFFSET
MaskType btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::getLeafMask(uint32_t leaf) const
{
	assert(leaf && isLeaf(leaf));
	return (*ents)[leaf - OFFSET].mask;
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::deletenode(const uint32_t node)
{
	assert(node);
	assert(!isLeaf(node));
	assert(node < nodes.size());
	assert(nodes[node].childs[1] != 0);
	if (node + 1 == nodes.size()) {
		nodes.back() = {{}, 0, {0, 0}};
		nodes.pop_back();
		return;
	} else {
		const uint32_t next = nodes[0].childs[0];
		nodes[node].childs[1] = 0;
		setNodeParent(node, 0);
		nodes[node].childs[0] = next;
		if (next) {
			assert(!isLeaf(next));
			assert(getNodeParent(next) == 0);
			assert(nodes[next].childs[1] == 0);
			setNodeParent(next, node);
		}
		nodes[0].childs[0] = node;
	}
}

SPP_TEMPLATE_DECL_OFFSET
uint32_t btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::createnode(uint32_t parent,
													  const Aabb &aabb)
{
	if (nodes[0].childs[0] != 0) {
		const uint32_t node = nodes[0].childs[0];
		assert(!isLeaf(node));
		assert(getNodeParent(node) == 0);
		assert(nodes[node].childs[1] == 0);
		const uint32_t next = nodes[node].childs[0];
		nodes[0].childs[0] = next;
		if (next) {
			assert(!isLeaf(next));
			assert(getNodeParent(next) == node);
			assert(nodes[next].childs[1] == 0);
			setNodeParent(next, 0);
		}
		nodes[node] = {aabb, parent, {0, 0}};
		return node;
	}
	const uint32_t node = nodes.size();
	nodes.push_back({aabb, parent, {0, 0}});
	return node;
}

SPP_TEMPLATE_DECL_OFFSET
uint32_t btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::createnode(uint32_t parent)
{
	return createnode(parent, {});
}

SPP_TEMPLATE_DECL_OFFSET
uint32_t btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::createnode(uint32_t parent,
													  const Aabb &aabb0,
													  const Aabb &aabb1)
{
	return createnode(parent, aabb0 + aabb1);
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::insertleaf(uint32_t root,
												  const uint32_t leaf,
												  const Aabb &aabb)
{
	assert(!ContainsRecurence(leaf));
	assert(leaf);
	assert(isLeaf(leaf));
	assert(getParent(leaf) == 0);
	if (!rootId) {
		rootId = leaf;
		setLeafParent(leaf, 0);
	} else {
		while (!isLeaf(root)) {
			int select = Select(aabb, getAabb(nodes[root].childs[0]),
								getAabb(nodes[root].childs[1]));
			root = nodes[root].childs[select];
		}
		const uint32_t sibling = root;
		assert(isLeaf(sibling));
		const uint32_t parent = getLeafParent(sibling);
		assert(parent < nodes.size());
		const uint32_t node = createnode(parent, aabb, getLeafAabb(sibling));
		if (parent) {
			assert(parent);
			assert(!isLeaf(parent));
			{
				const int i = indexofLeaf(sibling);
				assert(nodes[parent].childs[i] == sibling);
				assert(nodes[parent].childs[1 - i] != sibling);
				assert(getParent(nodes[parent].childs[i]) == parent);
			}
			nodes[parent].childs[indexofLeaf(sibling)] = node;
			nodes[node].childs[0] = sibling;
			nodes[node].childs[1] = leaf;
			setLeafParent(sibling, node);
			assert(!isLeaf(getParent(sibling)));
			setLeafParent(leaf, node);
			assert(!isLeaf(getParent(leaf)));
			Aabb ab = getNodeAabb(node);
			uint32_t node1 = node;
			uint32_t parent1 = parent;
			do {
				int i = indexof(node1);
				nodes[parent1].aabb = ab =
					ab + getAabb(nodes[parent1].childs[1 - i]);
				node1 = parent1;
				parent1 = getNodeParent(node1);
			} while (parent1);
		} else {
			nodes[node].childs[0] = sibling;
			nodes[node].childs[1] = leaf;
			setLeafParent(sibling, node);
			assert(!isLeaf(getParent(sibling)));
			setLeafParent(leaf, node);
			assert(!isLeaf(getParent(leaf)));
			rootId = node;
		}
	}
	assert(!isLeaf(getLeafParent(leaf)));
	if (getLeafParent(leaf)) {
		const uint32_t p = getLeafParent(leaf);
		const int i = indexofLeaf(leaf);
		assert(nodes[p].childs[i] == leaf);
		assert(nodes[p].childs[1 - i] != leaf);
	} else {
		assert(rootId == leaf);
	}
}

SPP_TEMPLATE_DECL_OFFSET
uint32_t btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::removeleaf(const uint32_t leaf)
{
	assert(ContainsRecurence(leaf));
	assert(isLeaf(leaf));
	if (leaf == rootId) {
		assert(getLeafParent(leaf) == 0);
		rootId = 0;
		setLeafParent(leaf, 0);
		return 0;
	} else {
		const uint32_t parent = getLeafParent(leaf);
		const uint32_t prev = getNodeParent(parent);
		const int i = indexofLeaf(leaf);
		uint32_t sibling = nodes[parent].childs[1 - i];
		assert(sibling != leaf);
		assert(nodes[parent].childs[i] == leaf);
		if (prev) {
			nodes[prev].childs[indexofNode(parent)] = sibling;
			setParent(sibling, prev);
			uint32_t _prev = prev;
			while (_prev) {
				const Aabb pb = getNodeAabb(_prev);
				nodes[_prev].aabb = getAabb(nodes[_prev].childs[0]) +
									getAabb(nodes[_prev].childs[1]);
				if (NotEqual(pb, nodes[_prev].aabb)) {
					_prev = getNodeParent(_prev);
				} else
					break;
			}
			deletenode(parent);
			assert(getParent(sibling) == prev);
			setLeafParent(leaf, 0);
			return _prev ? _prev : rootId;
		} else {
			rootId = sibling;
			setParent(sibling, 0);
			deletenode(parent);
			setLeafParent(leaf, 0);
			return rootId;
		}
	}
}

SPP_TEMPLATE_DECL_OFFSET
uint32_t btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::sort(const uint32_t n, uint32_t &r)
{
	const uint32_t p = getParent(n);
	assert(n);
	assert(isInternal(n));
	assert(!isLeaf(n));
	if (p > n) {
		const int i = indexof(n);
		const int j = 1 - i;
		uint32_t s = nodes[p].childs[j];
		uint32_t q = getParent(p);
		assert(n == nodes[p].childs[i]);
		if (q) {
			assert(getParent(nodes[q].childs[0]) == q);
			assert(getParent(nodes[q].childs[1]) == q);
			assert(nodes[q].childs[1] == p || nodes[q].childs[0] == p);
			nodes[q].childs[indexof(p)] = n;
		} else {
			r = n;
		}
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

		assert(getParent(nodes[n].childs[0]) == n);
		assert(getParent(nodes[n].childs[1]) == n);

		return (p);
	}
	return (n);
}

SPP_TEMPLATE_DECL_OFFSET
btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::btDbvt(
	spp::Dbvt<SPP_TEMPLATE_ARGS_OFFSET> *dbvt)
{
	ents = &(dbvt->ents);
	clear();
}

SPP_TEMPLATE_DECL_OFFSET
btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::~btDbvt() { clear(); }

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::clear()
{
	nodes.resize(1);
	rootId = 0;
	m_opath = 0;
	nodes[0] = {{{1, 1, 1}, {-1, -1, -1}}, 0, {0, 0}};
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::optimizeIncremental(int passes)
{
	if (passes < 0)
		passes = ents->Size();
	if (rootId && (passes > 0)) {
		do {
			uint32_t node = rootId;
			unsigned bit = 0;
			while (isInternal(node)) {
				assert(getParent(nodes[node].childs[0]) == node);
				assert(getParent(nodes[node].childs[1]) == node);
				node = nodes[sort(node, rootId)].childs[(m_opath >> bit) & 1];
				bit = (bit + 1) & (sizeof(unsigned) * 8 - 1);
			}
			updateLeaf(node);
			++m_opath;
		} while (--passes);
	}
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::insert(const Aabb &aabb,
											  uint32_t entityOffset)
{
	uint32_t leaf = getLeafId(entityOffset);
	assert(!ContainsRecurence(leaf));
	insertleaf(rootId, leaf, aabb);
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::updateLeaf(uint32_t leaf, int lookahead)
{
	assert(ContainsRecurence(leaf));
	assert(isLeaf(leaf) == true);
	assert(!isLeaf(getLeafParent(leaf)));
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
	assert(!isLeaf(getLeafParent(leaf)));
	IsTreeValid();
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::updateEntityOffset(uint32_t entityOffset,
														  const Aabb &aabb)
{
	uint32_t leaf = getLeafId(entityOffset);
	assert(ContainsRecurence(leaf));
	assert(!isLeaf(getLeafParent(leaf)));
	uint32_t root = removeleaf(leaf);
	if (root) {
		root = rootId;
	}

	assert(!NotEqual((*ents)[entityOffset].aabb, aabb));

	insertleaf(root, leaf, aabb);
	assert(!isLeaf(getLeafParent(leaf)));
	IsTreeValid();
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::remove(uint32_t entityOffset)
{
	uint32_t leaf = getLeafId(entityOffset);
	assert(ContainsRecurence(leaf));
	removeleaf(leaf);
	setLeafParent(leaf, 0);
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::updateOffsetOfEntity(
	uint32_t oldEntityOffset, uint32_t newEntityOffset)
{
	uint32_t oldLeaf = getLeafId(oldEntityOffset);
	assert(ContainsRecurence(oldLeaf));
	uint32_t newLeaf = getLeafId(newEntityOffset);
	assert(!ContainsRecurence(newLeaf));
	if (rootId == oldLeaf) {
		rootId = newLeaf;
	} else {
		uint32_t parent = getLeafParent(oldLeaf);
		int i = indexof(oldLeaf);
		nodes[parent].childs[i] = newLeaf;
	}
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::collideTV(AabbCallback &cb)
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
					assert(getParent(nodes[node].childs[0]) == node);
					assert(getParent(nodes[node].childs[1]) == node);
					stack.push_back(nodes[node].childs[0]);
					stack.push_back(nodes[node].childs[1]);
				}
			}
		} while (!stack.empty());
	}
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::rayTestInternal(RayCallback &cb)
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
					assert(getParent(nodes[node].childs[0]) == node);
					assert(getParent(nodes[node].childs[1]) == node);
					stack.push_back(nodes[node].childs[0]);
					stack.push_back(nodes[node].childs[1]);
				}
			}
		} while (!stack.empty());
	}
}

SPP_TEMPLATE_DECL_OFFSET
size_t btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::GetMemoryUsage() const
{
	return stack.capacity() * sizeof(uint32_t) +
		   nodes.capacity() * sizeof(NodeData);
}

SPP_TEMPLATE_DECL_OFFSET
void btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::IsTreeValid(uint32_t node) const
{
	return;
	if (node == 0) {
		node = rootId;
	}
	if (node == 0) {
		return;
	}
	if (isLeaf(node)) {
		assert(!isLeaf(getLeafParent(node)));
		return;
	}

	assert(getParent(nodes[node].childs[0]) == node);
	assert(getParent(nodes[node].childs[1]) == node);

	IsTreeValid(nodes[node].childs[0]);
	IsTreeValid(nodes[node].childs[1]);
}

SPP_TEMPLATE_DECL_OFFSET
bool btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::ContainsRecurence(uint32_t node,
														 uint32_t rel) const
{
	if (rel == 0) {
		rel = rootId;
	}
	if (rel == 0) {
		return false;
	}
	if (node == rel) {
		return true;
	}
	if (isLeaf(rel)) {
		assert(!isLeaf(getLeafParent(rel)));
		return false;
	}

	assert(getParent(nodes[rel].childs[0]) == rel);
	assert(getParent(nodes[rel].childs[1]) == rel);

	return ContainsRecurence(node, nodes[rel].childs[0]) ||
		   ContainsRecurence(node, nodes[rel].childs[1]);
}

SPP_DEFINE_VARIANTS_OFFSET(btDbvt)

} // namespace spp
