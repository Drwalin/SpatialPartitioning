// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <atomic>

#include "../include/spatial_partitioning/BulletDbvh.hpp"

static btVector3 bt(glm::vec3 v) { return {v.x, v.y, v.z}; }
static glm::vec3 gl(btVector3 v) { return {v.x(), v.y(), v.z()}; }

namespace spp
{
std::atomic<size_t> allocatedBytes = 0;
std::atomic<size_t> allocatedBlocks = 0;
std::atomic<size_t> totalMallocs = 0;

void *Allocate(size_t size)
{
	if (size != 0) {
		totalMallocs++;
		allocatedBlocks++;
		allocatedBytes += size;
		void *raw = malloc(size + 16);
		((size_t *)raw)[0] = size;
		char *ptr = (char *)raw;
		return ptr + 16;
	}
	exit(13);
	return nullptr;
}

void Free(void *ptr)
{
	if (ptr) {
		allocatedBlocks--;
		size_t *g = (size_t *)(((char *)ptr) - 16);
		size_t size = g[0];
		allocatedBytes -= size;
		free(g);
	}
}
	
int ___STATIC_INITIALIZATOR_HOLDER =
	(btAlignedAllocSetCustom(Allocate, Free), 0);

BulletDbvh::BulletDbvh() : broadphase(&cache), iterator(*this) {
	broadphase.m_deferedcollide = true;
}
BulletDbvh::~BulletDbvh() {
	printf("Total allocations count: %lu, sums up to %.2f MiB\n", totalMallocs.load(),
			allocatedBytes / (1024.0f*1024.0));
	Clear();
}

const char *BulletDbvh::GetName() const { return "BulletDbvh"; }

void BulletDbvh::Clear()
{
	auto &of = ents._Offsets();

	for (auto it : of) {
		if (ents[it.second].proxy) {
			broadphase.destroyProxy(ents[it.second].proxy, nullptr);
		}
	}
	ents.Clear();
}

size_t BulletDbvh::GetMemoryUsage() const
{
	return ents.GetMemoryUsage() + allocatedBytes;
}

void BulletDbvh::ShrinkToFit() {}

void BulletDbvh::SmallRebuildIfNeeded()
{
	if (requiresRebuild > 0) {
		IncrementalOptimize(requiresRebuild / 100 + 1);
		requiresRebuild = 0;
	}
}

void BulletDbvh::IncrementalOptimize(int iterations)
{
	broadphase.m_sets[0].optimizeIncremental(iterations);
	broadphase.m_sets[1].optimizeIncremental(iterations);
}

void BulletDbvh::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	int32_t offset = ents.Add(entity, Data{entity, mask});
	btBroadphaseProxy *proxy =
		broadphase.createProxy(bt(aabb.min), bt(aabb.max), 0,
							   (void *)(uint64_t)offset, mask, mask, nullptr);
	ents[offset].proxy = proxy;
	requiresRebuild++;
}

void BulletDbvh::Update(EntityType entity, Aabb aabb)
{
	int32_t offset = ents.GetOffset(entity);
	broadphase.setAabb(ents[offset].proxy, bt(aabb.min), bt(aabb.max), nullptr);
	requiresRebuild++;
}

void BulletDbvh::Remove(EntityType entity)
{
	int32_t offset = ents.GetOffset(entity);
	broadphase.destroyProxy(ents[offset].proxy, nullptr);
	ents.RemoveByKey(entity);
	requiresRebuild++;
}

void BulletDbvh::SetMask(EntityType entity, MaskType mask)
{
	int32_t offset = ents.GetOffset(entity);
	ents[offset].proxy->m_collisionFilterMask = mask;
	ents[offset].proxy->m_collisionFilterGroup = mask;
	ents[offset].mask = mask;
}

int32_t BulletDbvh::GetCount() const
{
	return ents.Size();
}

bool BulletDbvh::Exists(EntityType entity) const
{
	return ents.GetOffset(entity) > 0;
}

Aabb BulletDbvh::GetAabb(EntityType entity) const
{
	const Data &data = ents[ents.GetOffset(entity)];
	return {gl(data.proxy->m_aabbMin), gl(data.proxy->m_aabbMax)};
}

MaskType BulletDbvh::GetMask(EntityType entity) const
{
	return ents[ents.GetOffset(entity)].mask;
}

void BulletDbvh::Rebuild()
{
// 	broadphase.optimize();
	requiresRebuild = false;
	IncrementalOptimize(100);
}

class btAabbCb final : public btBroadphaseAabbCallback
{
public:
	btAabbCb(BulletDbvh *bp, IntersectionCallback *cb) : bp(bp), cb(cb) {}
	virtual ~btAabbCb() {}
	virtual bool process(const btBroadphaseProxy *p) override
	{
		int32_t offset = (int32_t)(uint64_t)(p->m_clientObject);
		BulletDbvh::Data &data = bp->ents[offset];
		if (cb->mask & data.mask) {
			cb->callback(cb, data.entity);
			cb->testedCount++;
		}
		// TODO: test return false
		return true;
	}

	BulletDbvh *bp;
	IntersectionCallback *cb;
};

void BulletDbvh::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;
	btAabbCb btCb{this, &cb};
	broadphase.aabbTest(bt(cb.aabb.min), bt(cb.aabb.max), btCb);
}

class btRayCb final : public btBroadphaseRayCallback
{
public:
	btRayCb(BulletDbvh *bp, RayCallback *cb) : bp(bp), cb(cb)
	{
		btVector3 rayDir = bt(cb->end - cb->start);

		rayDir.normalize();
		m_rayDirectionInverse[0] = rayDir[0] == btScalar(0.0)
									   ? btScalar(BT_LARGE_FLOAT)
									   : btScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == btScalar(0.0)
									   ? btScalar(BT_LARGE_FLOAT)
									   : btScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == btScalar(0.0)
									   ? btScalar(BT_LARGE_FLOAT)
									   : btScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot(bt(cb->end - cb->start));
	}
	virtual ~btRayCb() {}
	virtual bool process(const btBroadphaseProxy *p) override
	{
		if (cb->length == btScalar(0.f))
			return false;

		bool hasHit = false;
		int32_t offset = (int32_t)(uint64_t)(p->m_clientObject);
		BulletDbvh::Data &data = bp->ents[offset];
		if (cb->mask & data.mask) {
			auto res = cb->callback(cb, data.entity);
			if (res.intersection) {
				hasHit = true;
				if (res.dist + 0.00000001f < 1.0f) {
					if (res.dist < 0.0f)
						res.dist = 0.0f;
					else
						res.dist += 0.00000001f;
					cb->length *= res.dist;
					cb->dir *= res.dist;
					cb->end = cb->start + cb->dir;
					m_lambda_max *= res.dist;
				}
				++cb->hitCount;
			}
			++cb->testedCount;
		}
		// TODO: test return false
		return !hasHit;
	}

	BulletDbvh *bp;
	RayCallback *cb;
};

void BulletDbvh::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;
	cb.InitVariables();

	btRayCb btCb{this, &cb};
	broadphase.rayTest(bt(cb.start), bt(cb.end), btCb);
}

BroadphaseBaseIterator *BulletDbvh::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

BulletDbvh::Iterator::Iterator(BulletDbvh &bp)
{
	data = &(bp.ents._Data()._Data());
	it = 1;
	FetchData();
}

BulletDbvh::Iterator::~Iterator() {}

bool BulletDbvh::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);
	return FetchData();
}

bool BulletDbvh::Iterator::FetchData()
{
	if (Valid()) {
		entity = (*data)[it].entity;
		Data d = (*data)[it];
		aabb = {gl(d.proxy->m_aabbMin), gl(d.proxy->m_aabbMax)};
		mask = (*data)[it].mask;
		return true;
	}
	return false;
}

bool BulletDbvh::Iterator::Valid() { return it < data->size(); }
} // namespace spp
