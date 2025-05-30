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

#pragma once

#include "IntersectionCallbacks.hpp"
#include "AssociativeArray.hpp"

namespace spp
{
SPP_TEMPLATE_DECL_OFFSET
class Dbvt;

SPP_TEMPLATE_DECL_OFFSET
class btDbvt
{
public:
	using AabbCallback = spp::AabbCallback<SPP_TEMPLATE_ARGS>;
	using RayCallback = spp::RayCallback<SPP_TEMPLATE_ARGS>;

	btDbvt(spp::Dbvt<SPP_TEMPLATE_ARGS_OFFSET> *dbvt);

	~btDbvt();
	void clear();
	bool empty() const { return (0 == rootId); }
	void optimizeIncremental(int passes);

	void insert(const Aabb &aabb, uint32_t entityOffset);

	void updateLeaf(uint32_t leaf, int lookahead = -1);
	void updateEntityOffset(uint32_t entityOffset, const Aabb &aabb);
	void remove(uint32_t entityOffset);

	void updateOffsetOfEntity(uint32_t oldEntityOffset,
							  uint32_t newEntityOffset);

	void collideTV(AabbCallback &cb);
	void rayTestInternal(RayCallback &cb);

	size_t GetMemoryUsage() const;

	void IsTreeValid(uint32_t node = 0) const;
	bool ContainsRecurence(uint32_t node, uint32_t rel = 0) const;

public:
	struct Data {
		Aabb aabb;
		uint32_t parent = 0;
		EntityType entity = 0;
		MaskType mask = 0;
	};

protected:
	inline const static uint32_t OFFSET = 0x80000000;

	struct NodeData {
		Aabb aabb;
		uint32_t parent = 0;
		uint32_t childs[2] = {0, 0};
	};

protected:
	static bool isLeaf(uint32_t node);
	static bool isInternal(uint32_t node);
	static uint32_t getLeafId(uint32_t entityOffset);

	int indexof(uint32_t node) const;
	int indexofLeaf(uint32_t leaf) const;
	int indexofNode(uint32_t node) const;
	Aabb getAabb(uint32_t node) const;
	uint32_t getParent(uint32_t node) const;
	Aabb getLeafAabb(uint32_t leaf) const;
	uint32_t getLeafParent(uint32_t leaf) const;
	Aabb getNodeAabb(uint32_t node) const;
	uint32_t getNodeParent(uint32_t node) const;
	void setParent(uint32_t node, uint32_t parent);
	void setNodeParent(uint32_t node, uint32_t parent);
	void setLeafParent(uint32_t leaf, uint32_t parent);

	EntityType getLeafEntity(uint32_t leaf) const;
	MaskType getLeafMask(uint32_t leaf) const;

	void deletenode(uint32_t node);
	uint32_t createnode(uint32_t parent);
	uint32_t createnode(uint32_t parent, const Aabb &aabb);
	uint32_t createnode(uint32_t parent, const Aabb &aabb0, const Aabb &aabb1);
	void insertleaf(uint32_t root, uint32_t leaf, const Aabb &aabb);
	uint32_t removeleaf(uint32_t leaf);
	uint32_t sort(uint32_t n, uint32_t &r);

protected:
	uint32_t rootId = 0;
	unsigned m_opath = 0;

	AssociativeArray<EntityType, uint32_t,
					 spp::btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::Data, false> *ents;
	std::vector<NodeData> nodes;

	std::vector<uint32_t> stack;

	/*
	 * nodes[0] - first emtpty node id holder
	 * nodes[free].parent - previous free node
	 * nodes[free].childs[0] - next free node
	 * nodes[free].childs[1] === 0 -> this is a free node
	 */
};

SPP_EXTERN_VARIANTS_OFFSET(btDbvt)

} // namespace spp
