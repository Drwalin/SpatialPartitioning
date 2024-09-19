// This file is part of SpatialPartitioning.
// Copyright (c) 2024 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

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
	LooseOctree(glm::vec3 centerOffset, glm::vec3 sizeScale, int32_t depth,
				float cellExtensionFactor);
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
								 const int32_t nodeId);
	void _Internal_IntersectRay(RayCallback &cb, const int32_t nodeId);

	glm::vec3 GetSizeOnDepth(int32_t depth) const;
	int32_t GetChildIdFromCenter(glm::vec3 p) const;
	glm::vec3 GetCenterOffset(int32_t depth);
	// offset from parent node
	glm::vec3 GetPosOffsetOfNodeAtDepth(int32_t depth, int32_t childId) const;

private:
	struct NodePath {
		int8_t id[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
						 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		int32_t nodes[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
							 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
		int depth = 0;
	};

	NodePath CalculatePathTo(Aabb aabb);

private:
	struct alignas(64) Data {
		Aabb aabb;
		EntityType entity = 0;
		MaskType mask = 0;
		int32_t nextOffset = -1;
	};

	struct alignas(32) NodePartialData {
		Aabb aabb;
		MaskType mask = 0;
		int32_t offset = 0;
	};

	struct NodeData {
		NodePartialData children[8];
		Aabb aabbOfLeafs;
		MaskType maskOfLeafs = 0;
		int32_t firstChildOffset = 0;
	};

	AssociativeArray<EntityType, int32_t, Data> data;
	NodesArray<int32_t, NodeData> nodes;

	const glm::vec3 centerOffset;
	const glm::vec3 sizeScale;
	const int32_t depth;
	const float cellExtensionFactor;
	const int32_t halfTotalSize;
	int32_t rootNode = 0;
};
} // namespace spp
