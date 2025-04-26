// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include "AssociativeArray.hpp"
#include "BroadPhaseBase.hpp"

namespace spp
{
/*
 * Dynamic Bounding Volume Hierarchy
 * Split policy:
 *  Trying to do up to tripple middle of bounding AABB split at longest axis
 *    to find approximation of median split
 *
 * Limit of entities count is 268435456 (2^28-1)
 *
 * Supports dynamically adding and removing entities
 */
class Dbvh final : public BroadphaseBase
{
public:
	Dbvh();
	virtual ~Dbvh();

	virtual const char *GetName() const override;

	virtual void Clear() override;
	virtual size_t GetMemoryUsage() const override;
	virtual void ShrinkToFit() override;

	virtual void Add(EntityType entity, Aabb aabb, MaskType mask) override;
	virtual void Update(EntityType entity, Aabb aabb) override;
	virtual void Remove(EntityType entity) override;
	virtual void SetMask(EntityType entity, MaskType mask) override;

	virtual int32_t GetCount() const override;
	virtual bool Exists(EntityType entity) const override;

	virtual Aabb GetAabb(EntityType entity) const override;
	virtual MaskType GetMask(EntityType entity) const override;

	virtual void IntersectAabb(IntersectionCallback &callback) override;
	virtual void IntersectRay(RayCallback &callback) override;

	virtual void Rebuild() override;

	virtual BroadphaseBaseIterator *RestartIterator() override;

private:
	void UpdateAabb(const int32_t nodeId);
	void UpdateAabbAndMask(const int32_t nodeId);
	void UpdateAabbSimple(const int32_t nodeId);
	void UpdateMask(const int32_t nodeId);
	void RebuildNode(int32_t nodeId);

	void FastRebalance();
	void RebalanceNodesRecursively(int32_t nodeId, int32_t depth);
	void DoBestNodeRotation(int32_t nodeId);
	void RebalanceUpToRoot(int32_t nodeId, int32_t rebalancingDepth);

	bool GetRotationIntersectionVolume(int32_t parentNode, int32_t lId,
									   int32_t rId, float *resultValue) const;
	void DoRotation(int32_t parentNode, int32_t lId, int32_t rId);
	bool GetNodeOffsetsAndInfo(int32_t rootNodeId, int32_t id, int32_t *nodeId,
							   int32_t *parentNodeId,
							   int32_t *childIdOfParent) const;
	int32_t GetDirectMask(int32_t nodeId) const;
	int32_t GetIndirectMask(int32_t nodeId) const;
	Aabb GetDirectAabb(int32_t nodeId) const;
	Aabb GetIndirectAabb(int32_t nodeId) const;
	void SetParent(int32_t node, int32_t parent);

	void _Internal_IntersectAabb(IntersectionCallback &cb,
								 const int32_t nodeId);
	void _Internal_IntersectRay(RayCallback &cb, const int32_t nodeId);

	int32_t CountDepth() const;
	int32_t CountNodes() const;
	int32_t CountEntities() const;

private:
	inline const static int32_t OFFSET = 0x10000000;

	struct Data {
		Aabb aabb;
		EntityType entity = 0;
		MaskType mask = 0;
		int32_t parent = 0;
	};

	struct NodeData {
		AabbCentered aabb[2];
		MaskType mask = 0;
		int32_t parent = 0;
		// If (children[i] > 0x10000000) than it is a leaf
		int32_t children[2] = {0, 0};
	};

	AssociativeArray<EntityType, int32_t, Data> data;
	NodesArray<int32_t, NodeData> nodes;

	int32_t rootNode = 0;
	bool fastRebalance = false;

	class Iterator final : public BroadphaseBaseIterator
	{
	public:
		Iterator(Dbvh &bp);
		virtual ~Iterator();

		Iterator &operator=(Iterator &&other) = default;

		virtual bool Next() override;
		virtual bool Valid() override;
		bool FetchData();

		std::vector<Data> *data;
		int it;
	} iterator;
};
} // namespace spp
