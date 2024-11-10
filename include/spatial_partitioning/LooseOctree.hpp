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
	LooseOctree(glm::vec3 offset, float sizeScale, int32_t levels,
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
	void PruneEmptyEntitiesAtEnd();
	void UpdateAabb(int32_t entityId);

	void _Internal_IntersectAabb(IntersectionCallback &cb,
								 const int32_t nodeId, const Aabb &cbaabb);
	void _Internal_IntersectRay(RayCallback &cb, const int32_t nodeId, int32_t level);

	int32_t GetChildIdFromCenter(glm::vec3 p) const;
	glm::vec3 GetCenterOffset(int32_t depth);
	// offset from parent node
	glm::vec3 GetPosOffsetOfNodeAtDepth(int32_t depth, int32_t childId) const;

	void RemoveStructureFor(int32_t offset);

	Aabb GetAabbOfNode(int32_t nodeId) const;
	
	static int32_t CalcChildId(glm::ivec3 parentPos, glm::ivec3 childPos, int32_t childLevel);

private:
	int32_t GetNodeIdAt(glm::ivec3 pos, int32_t level);

private:
	struct IPosLevel {
		glm::vec3 ipos;
		int32_t level;
	};

	struct alignas(64) Data {
		Aabb aabb;
		EntityType entity = 0;
		MaskType mask = 0;
		int32_t prev = -1;
		int32_t next = -1;
		int32_t parent = -1;
	};

	struct alignas(64) NodeData {
		int32_t children[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
		glm::ivec3 pos;
		int32_t level;
		// MaskType mask = 0;
		int32_t firstEntity = -1;
		int32_t parentId = -1;
		bool HasData() const;
	};

	AssociativeArray<EntityType, int32_t, Data> data;
	NodesArray<int32_t, NodeData> nodes;

	const glm::vec3 offset;
	const float scale;

	const int32_t levels;
	const float loosnessFactor;
	const float invLoosenessFactor;
	const int32_t maxExtent;
	const float margin;
	int32_t rootNode = -1;

private:
	IPosLevel CalcIPosLevel(Aabb aabb) const;
};
} // namespace spp
