// This file is part of SpatialPartitioning.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include <cstdint>

#include <memory>

#include "../../thirdparty/glm/glm/ext/vector_int3.hpp"
#include "../../thirdparty/glm/glm/ext/vector_int4.hpp"

#include "AssociativeArray.hpp"
#include "BvhMedianSplitHeap.hpp"
#include "BroadPhaseBase.hpp"

namespace spp
{
namespace experimental
{
class ChunkedLooseOctree final : public BroadphaseBase
{
public:
	ChunkedLooseOctree(int32_t chunkSize = 32, int32_t worldSize = 1024,
					   float loosness = 1.3f);
	virtual ~ChunkedLooseOctree();

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

public:
	int32_t GetCreateChunkId(glm::vec3 pos);
	int32_t GetChunkId(glm::vec3 pos) const;
	int32_t GetChunkId(EntityType entity) const;
	int32_t GetCreateNodeId(Aabb aabb);

	void AddToChunk(int32_t chunkId, int32_t entityOffset);
	void RemoveFromChunk(int32_t entityOffset);

	void UnlinkFromChunk(int32_t entityOffset);
	void CleanIfEmptyNodes(int32_t nodeId);

	std::shared_ptr<void> GetChunkData(int32_t chunkId);
	void SetChunkData(int32_t chunkId, std::shared_ptr<void> chunksData);

	void GetChunks(Aabb aabb, std::vector<int32_t> chunks);
	void GetChunks(bool (*isIn)(Aabb test, void *testData), void *testData,
				   std::vector<int32_t> chunksData);

	void ForEachChunkData(bool (*isIn)(Aabb test, void *testData),
						  void *testData,
						  void (*func)(ChunkedLooseOctree &self, int32_t chunk,
									   void *userData),
						  void *userData);

private:
	glm::ivec3 GetMaxChunk(glm::vec3 point) const;
	glm::ivec3 GetMinChunk(glm::vec3 point) const;
	glm::ivec4 GetNodePosSize(Aabb aabb) const;
	bool FitsInChunk(Aabb aabb) const;

	void _Internal_IntersectAabb(IntersectionCallback &cb,
								 const int32_t nodeId);
	void _Internal_IntersectRay(RayCallback &cb, const int32_t nodeId);

private:
	struct Data {
		Aabb aabb;
		EntityType entity = 0;
		MaskType mask = 0;
		int32_t chunkId = 0;
		int32_t nodeId = 0;
		int32_t nextDataId = 0;
		int32_t prevDataId = 0;
		bool big = false;
	};

	struct ChunkData {
		std::shared_ptr<void> userData;
		int32_t cx, cy, cz;

		int32_t smallNodeId = 0;
		int32_t parentId = 0;
	};

	struct NodeData {
		union {
			int32_t childrenId[2][2][2];
			int32_t childrenIdLinear[8] = {0, 0, 0, 0, 0, 0, 0, 0};
			uint64_t bigInts[4];
		};
		int32_t parentId = 0;
		int32_t chunkId = 0;
		int32_t firstEntity = 0;
		int32_t halfSize = 0;
		int32_t cx, cy, cz;

		AabbCentered GetAabb() const;
	};

	// half extents of sizes
	int32_t chunkSizeHalf;
	int32_t worldSizeHalf;
	int32_t chunkSize;
	int32_t worldSize;
	float loosness;
	float loosnessInv;
	float chunkCheckOffset;

	AssociativeArray<EntityType, int32_t, Data> data;
	NodesArray<int32_t, NodeData> nodes;
	NodesArray<int32_t, ChunkData> chunks;

	BvhMedianSplitHeap bigObjects;

	int32_t rootNode;

	class Iterator final : public BroadphaseBaseIterator
	{
	public:
		Iterator(ChunkedLooseOctree &bp);
		virtual ~Iterator();

		Iterator &operator=(Iterator &&other) = default;

		virtual bool Next() override;
		virtual bool Valid() override;
		bool FetchData();

		std::vector<Data> *data;
		int it;
	} iterator;
};
} // namespace experimental
} // namespace spp
