// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>
#include <cstdio>

#include "../../thirdparty/glm/glm/ext/vector_int3.hpp"

#include "AssociativeArray.hpp"
#include "BroadPhaseBase.hpp"

namespace spp
{
/*
 * Limit of entities count is 268435456 (2^28-1)
 */
class LooseOctree final : public BroadphaseBase
{
public:
	// example usage LooseOctree(glm::ivec3(1,1,1)<<(levels-1), 1.0, levels)
	LooseOctree(glm::vec3 offset, int32_t levels,
				float loosnessFactor = 1.6);
	virtual ~LooseOctree();

	virtual const char *GetName() const override;

	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;

	virtual void Add(EntityType entity, Aabb aabb, MaskType mask) override;
	virtual void Update(EntityType entity, Aabb aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;

	virtual Aabb GetAabb(EntityType entity) const override;
	virtual MaskType GetMask(EntityType entity) const override;

	virtual void IntersectAabb(IntersectionCallback &callback) override;
	virtual void IntersectRay(RayCallback &callback) override;

	virtual void Rebuild() override;

private:
	struct IPosLevel {
		glm::ivec3 ipos;
		int32_t level;
	};

	void PruneEmptyEntitiesAtEnd();
	void UpdateAabb(int32_t entityId);

	void _Internal_IntersectAabb(IntersectionCallback &cb, const int32_t nodeId);
	void _Internal_IntersectRay(RayCallback &cb, const int32_t nodeId, int32_t level);

// 	int32_t GetChildIdFromCenter(glm::vec3 p) const;
// 	glm::vec3 GetCenterOffset(int32_t depth);
// 	// offset from parent node
// 	glm::vec3 GetPosOffsetOfNodeAtDepth(int32_t depth, int32_t childId) const;

	void RemoveStructureFor(int32_t offset);

	AabbCentered GetAabbOfNode(int32_t nodeId) const;
	IPosLevel CalcIPosLevel(Aabb aabb) const;
	
	static int32_t CalcChildId(glm::ivec3 parentPos, glm::ivec3 childPos, int32_t childLevel);

private:
	int32_t GetNodeIdAt(Aabb aabb);

private:
	struct alignas(64) Data {
		AabbCentered aabb;
		EntityType entity = 0;
		MaskType mask = 0;
		int32_t prev = 0;
		int32_t next = 0;
		int32_t parent = 0;
	};

	struct alignas(64) NodeData {
		int32_t children[8] = {0, 0, 0, 0, 0, 0, 0, 0};
		glm::ivec3 center;
		int32_t level;
		// MaskType mask = 0;
		int32_t firstEntity = 0;
		int32_t parentId = 0;
		
		bool HasData() const;
	};

	AssociativeArray<EntityType, int32_t, Data> data;
	NodesArray<int32_t, NodeData> nodes;

	const glm::vec3 centerOffset;

	const int32_t levels;
	const float loosnessFactor;
	const float invLoosenessFactor;
	const int32_t maxExtent;
	const float margin;
	int32_t rootNode = 0;
};
} // namespace spp
