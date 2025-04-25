// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/BulletDbvt.hpp"

static bullet::btVector3 bt(glm::vec3 v) { return {v.x, v.y, v.z}; }
static bullet::btDbvtAabbMm bt(spp::Aabb v)
{
	return bullet::btDbvtVolume::FromMM(bt(v.min), bt(v.max));
}

namespace spp
{
BulletDbvt::BulletDbvt() : iterator(*this) {}
BulletDbvt::~BulletDbvt() { Clear(); }

const char *BulletDbvt::GetName() const { return "BulletDbvt"; }

void BulletDbvt::Clear()
{
	ents.Clear();
	dbvt.clear();
}

size_t BulletDbvt::GetMemoryUsage() const {
	return ents.GetMemoryUsage() +
		(GetCount() * 2 - 1) * sizeof(bullet::btDbvtNode) +
		dbvt.m_stkStack.capacity() * sizeof(bullet::btDbvt::sStkNN) +
		stack.capacity() * sizeof(const bullet::btDbvtNode*);
}

void BulletDbvt::ShrinkToFit() {
	ents.ShrinkToFit();
}

void BulletDbvt::SmallRebuildIfNeeded()
{
	if (requiresRebuild > 1000) {
		IncrementalOptimize(requiresRebuild / 133 + 1);
		requiresRebuild = 0;
	}
}

void BulletDbvt::IncrementalOptimize(int iterations)
{
	dbvt.optimizeIncremental(iterations);
}

void BulletDbvt::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	if (Exists(entity)) {
		assert(false && "Entity already exists");
		return;
	}
	int32_t offset = ents.Add(entity, Data{entity, mask, aabb, nullptr});

	bullet::btDbvtAabbMm volume = bt(aabb);
	bullet::btDbvtNode *node = dbvt.insert(volume, (void *)(int64_t)offset);
	ents[offset].node = node;
	requiresRebuild++;
}

void BulletDbvt::Update(EntityType entity, Aabb aabb)
{
	int32_t offset = ents.GetOffset(entity);
	ents[offset].aabb = aabb;
	bullet::btDbvtAabbMm volume = bt(aabb);
	dbvt.update(ents[offset].node, volume);
	requiresRebuild++;
}

void BulletDbvt::Remove(EntityType entity)
{
	int32_t offset = ents.GetOffset(entity);
	dbvt.remove(ents[offset].node);
	ents.RemoveByKey(entity);
	requiresRebuild++;
}

void BulletDbvt::SetMask(EntityType entity, MaskType mask)
{
	int32_t offset = ents.GetOffset(entity);
	ents[offset].mask = mask;
}

int32_t BulletDbvt::GetCount() const { return ents.Size(); }

bool BulletDbvt::Exists(EntityType entity) const
{
	return ents.GetOffset(entity) > 0;
}

Aabb BulletDbvt::GetAabb(EntityType entity) const
{
	return ents[ents.GetOffset(entity)].aabb;
}

MaskType BulletDbvt::GetMask(EntityType entity) const
{
	return ents[ents.GetOffset(entity)].mask;
}

void BulletDbvt::Rebuild()
{
	requiresRebuild += 3000;
	SmallRebuildIfNeeded();
}

class btDbvtAabbCb final : public bullet::btDbvt::ICollide
{
public:
	btDbvtAabbCb(BulletDbvt *bp, IntersectionCallback *cb) : bp(bp), cb(cb) {}
	virtual ~btDbvtAabbCb() {}
	virtual void Process(const bullet::btDbvtNode *leaf) override
	{
		int32_t offset = (int32_t)(int64_t)(leaf->data);
		if (bp->ents[offset].mask & cb->mask) {
			cb->callback(cb, bp->ents[offset].entity);
			cb->testedCount++;
		}
	}

	BulletDbvt *bp;
	IntersectionCallback *cb;
};

void BulletDbvt::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;
	btDbvtAabbCb btCb{this, &cb};

	const ATTRIBUTE_ALIGNED16(bullet::btDbvtVolume) bounds = bt(cb.aabb);
// 	dbvt.collideTV(dbvt.m_root, bounds, btCb);
	dbvt.collideTVNoStackAlloc(dbvt.m_root, bounds, stack, btCb);
}

class btDbvtRayCb final : public bullet::btDbvt::ICollide
{
public:
	btDbvtRayCb(BulletDbvt *bp, RayCallback *cb) : bp(bp), cb(cb)
	{
		bullet::btVector3 rayDir = bt(cb->end - cb->start);

		rayDir.normalize();
		m_rayDirectionInverse[0] = rayDir[0] == bullet::btScalar(0.0)
									   ? bullet::btScalar(BT_LARGE_FLOAT)
									   : bullet::btScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == bullet::btScalar(0.0)
									   ? bullet::btScalar(BT_LARGE_FLOAT)
									   : bullet::btScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == bullet::btScalar(0.0)
									   ? bullet::btScalar(BT_LARGE_FLOAT)
									   : bullet::btScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = lambdaOrig = rayDir.dot(bt(cb->end - cb->start));
	}
	virtual ~btDbvtRayCb() {}
	virtual void Process(const bullet::btDbvtNode *leaf) override
	{
		if (cb->cutFactor == bullet::btScalar(0.f))
			return;

		int32_t offset = (int32_t)(uint64_t)(leaf->data);
		BulletDbvt::Data &data = bp->ents[offset];
		if (cb->mask & data.mask) {
			auto res = cb->ExecuteCallback(data.entity);
			if (res.intersection) {
				assert(res.dist >= 0);
				assert(res.dist <= 1);
				m_lambda_max = lambdaOrig * res.dist;
			}
		}
	}

	BulletDbvt *bp;
	RayCallback *cb;

	bullet::btVector3 m_rayDirectionInverse;
	unsigned int m_signs[3];
	float m_lambda_max;
	float lambdaOrig;
};

void BulletDbvt::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	SmallRebuildIfNeeded();

	cb.broadphase = this;
	cb.InitVariables();

	btDbvtRayCb btCb{this, &cb};

	dbvt.rayTestInternal(
			dbvt.m_root,
			bt(cb.start),
			bt(cb.end),
			btCb.m_rayDirectionInverse,
			btCb.m_signs,
			btCb.m_lambda_max,
			bullet::btVector3(0, 0, 0),
			bullet::btVector3(0, 0, 0),
			stack,
			btCb);
}

BroadphaseBaseIterator *BulletDbvt::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

BulletDbvt::Iterator::Iterator(BulletDbvt &bp)
{
	data = &(bp.ents._Data()._Data());
	it = 0;
	Next();
}

BulletDbvt::Iterator::~Iterator() {}

bool BulletDbvt::Iterator::Next()
{
	do {
		++it;
	} while (Valid() && (*data)[it].entity == EMPTY_ENTITY);
	return FetchData();
}

bool BulletDbvt::Iterator::FetchData()
{
	if (Valid()) {
		Data d = (*data)[it];
		entity = d.entity;
		aabb = d.aabb;
		mask = d.mask;
		return true;
	}
	return false;
}

bool BulletDbvt::Iterator::Valid() { return it < data->size(); }
} // namespace spp
