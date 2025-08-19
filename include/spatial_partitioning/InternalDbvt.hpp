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

	void insert(const Aabb &aabb, OffsetType entityOffset);

	void updateLeaf(OffsetType leaf, int lookahead = -1);
	void updateEntityOffset(OffsetType entityOffset, const Aabb &aabb);
	void remove(OffsetType entityOffset);

	void updateOffsetOfEntity(OffsetType oldEntityOffset,
							  OffsetType newEntityOffset);

	void collideTV(AabbCallback &cb);
	void rayTestInternal(RayCallback &cb);

	size_t GetMemoryUsage() const;

	void IsTreeValid(OffsetType node = 0) const;
	bool ContainsRecurence(OffsetType node, OffsetType rel = 0) const;

public:
	struct Data {
		Aabb aabb;
		OffsetType parent = 0;
		EntityType entity = 0;
		MaskType mask = 0;
	};

protected:
	inline const static OffsetType OFFSET = 1 << (8 * sizeof(OffsetType) - 1);

	struct NodeData {
		Aabb aabb;
		OffsetType parent = 0;
		OffsetType childs[2] = {0, 0};
	};

protected:
	static bool isLeaf(OffsetType node);
	static bool isInternal(OffsetType node);
	static OffsetType getLeafId(OffsetType entityOffset);

	int indexof(OffsetType node) const;
	int indexofLeaf(OffsetType leaf) const;
	int indexofNode(OffsetType node) const;
	Aabb getAabb(OffsetType node) const;
	OffsetType getParent(OffsetType node) const;
	Aabb getLeafAabb(OffsetType leaf) const;
	OffsetType getLeafParent(OffsetType leaf) const;
	Aabb getNodeAabb(OffsetType node) const;
	OffsetType getNodeParent(OffsetType node) const;
	void setParent(OffsetType node, OffsetType parent);
	void setNodeParent(OffsetType node, OffsetType parent);
	void setLeafParent(OffsetType leaf, OffsetType parent);

	EntityType getLeafEntity(OffsetType leaf) const;
	MaskType getLeafMask(OffsetType leaf) const;

	void deletenode(OffsetType node);
	OffsetType createnode(OffsetType parent);
	OffsetType createnode(OffsetType parent, const Aabb &aabb);
	OffsetType createnode(OffsetType parent, const Aabb &aabb0, const Aabb &aabb1);
	void insertleaf(OffsetType root, OffsetType leaf, const Aabb &aabb);
	OffsetType removeleaf(OffsetType leaf);
	OffsetType sort(OffsetType n, OffsetType &r);

protected:
	OffsetType rootId = 0;
	unsigned m_opath = 0;

	AssociativeArray<EntityType, OffsetType,
					 spp::btDbvt<SPP_TEMPLATE_ARGS_OFFSET>::Data, false> *ents;
	std::vector<NodeData> nodes;

	std::vector<OffsetType> stack;

	/*
	 * nodes[0] - first emtpty node id holder
	 * nodes[free].parent - previous free node
	 * nodes[free].childs[0] - next free node
	 * nodes[free].childs[1] === 0 -> this is a free node
	 */
};

SPP_EXTERN_VARIANTS_OFFSET(btDbvt)

} // namespace spp
