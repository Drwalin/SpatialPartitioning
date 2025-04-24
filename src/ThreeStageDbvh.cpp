// This file is part of SpatialPartitioning.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/ThreeStageDbvh.hpp"

void BreakPoint() {}

bool Assert(bool condition, const char *text, const char *function,
			const char *file, int line)
{
	if (!condition) {
		printf("Assert failed: `%s` %s %s:%i\n", text, function, file, line);
		fflush(stdout);
	}
	return !condition;
}

#define ASSERT(COND)                                                           \
	Assert(COND, #COND, __PRETTY_FUNCTION__, __FILE__, __LINE__)

#undef assert
#define assert(COND)                                                           \
	Assert(COND, #COND, __PRETTY_FUNCTION__, __FILE__, __LINE__)

namespace spp
{

ThreeStageDbvh::ThreeStageDbvh(std::shared_ptr<BroadphaseBase> optimised,
							   std::shared_ptr<BroadphaseBase> rebuilding,
							   std::unique_ptr<BroadphaseBase> &&dynamic,
							   std::unique_ptr<BroadphaseBase> &&dynamic2)
	: iterator(*this)
{
	this->_finishedRebuilding = std::make_shared<std::atomic<bool>>();
	this->finishedRebuilding = this->_finishedRebuilding.get();

	this->dbvhs[0] = optimised;
	this->dbvhs[1] = rebuilding;

	this->_rebuild = rebuilding;
	this->_optimised = optimised;
	this->rebuild = nullptr;
	this->optimised = _optimised.get();

	this->_dynamic = std::move(dynamic);
	this->_dynamic2 = std::move(dynamic2);
	this->dynamic = _dynamic.get();
	this->dynamic2 = _dynamic2.get();
}

ThreeStageDbvh::~ThreeStageDbvh() {}

const char *ThreeStageDbvh::GetName() const { return "ThreeStageDbvh"; }
void ThreeStageDbvh::Clear()
{
	if (rebuild) {
		clear = true;
		TryIntegrateOptimised();
	}
	entitiesData.clear();
	dynamic->Clear();
	dynamic2->Clear();
	optimised->Clear();
}

size_t ThreeStageDbvh::GetMemoryUsage() const
{
	return dbvhs[0]->GetMemoryUsage() + dbvhs[1]->GetMemoryUsage() +
		   dynamic->GetMemoryUsage() + dynamic2->GetMemoryUsage() +

		   6 * 32 +

		   toRemoveAfterRebuild.bucket_count() * sizeof(void *) +
		   toRemoveAfterRebuild.size() *
			   (sizeof(void *) * 2lu + sizeof(EntityType)) +

		   setMaskAfterRebuild.bucket_count() * sizeof(void *) +
		   setMaskAfterRebuild.size() *
			   (sizeof(void *) * 2lu + sizeof(EntityType) + sizeof(MaskType));
}

void ThreeStageDbvh::ShrinkToFit()
{
	dbvhs[0]->ShrinkToFit();
	dbvhs[1]->ShrinkToFit();
	dynamic->ShrinkToFit();
	toRemoveAfterRebuild.rehash((toRemoveAfterRebuild.size() * 3 + 7) / 2);
}

void ThreeStageDbvh::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	assert((optimised->Exists(entity) || dynamic->Exists(entity) ||
			dynamic2->Exists(entity)) == false &&
		   entitiesData.contains(entity) == false);

	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));

	entitiesData.insert({entity, aabb});

	dynamicUpdates++;
	dynamic->Add(entity, aabb, mask);

	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));
}

void ThreeStageDbvh::Update(EntityType entity, Aabb aabb)
{
	assert((optimised->Exists(entity) || dynamic->Exists(entity) ||
			dynamic2->Exists(entity)) == true &&
		   entitiesData.contains(entity) == true);

	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));

	if (assert(entitiesData.contains(entity))) {
		BreakPoint();
	}

	TryIntegrateOptimised();

	entitiesData[entity] = aabb;

	if (dynamic->Exists(entity)) {
		dynamicUpdates++;
		dynamic->Update(entity, aabb);
	} else {
		MaskType mask = 0;
		if (dynamic2->Exists(entity)) {
			mask = dynamic2->GetMask(entity);
			dynamic2->Remove(entity);
		} else {
			assert(optimised->Exists(entity) &&
				   "This is an alternative scenario");
			mask = optimised->GetMask(entity);
			optimised->Remove(entity);
		}

		optimisedUpdates++;
		dynamicUpdates++;
		dynamic->Add(entity, aabb, mask);

		if (rebuild) {
			toRemoveAfterRebuild.insert(entity);
		}
	}

	if (dynamicUpdates > 1000 || optimisedUpdates > 100) {
		TryScheduleRebuild();
	}

	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));
}

void ThreeStageDbvh::Remove(EntityType entity)
{
	assert((optimised->Exists(entity) || dynamic->Exists(entity) ||
			dynamic2->Exists(entity)) == true &&
		   entitiesData.contains(entity) == true);

	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));

	TryIntegrateOptimised();

	entitiesData.erase(entity);

	if (dynamic->Exists(entity)) {
		dynamic->Remove(entity);
		if (assert(dynamic2->Exists(entity) == false)) {
			BreakPoint();
		}
		if (assert(optimised->Exists(entity) == false)) {
			BreakPoint();
		}
	} else if (dynamic2->Exists(entity)) {
		dynamic2->Remove(entity);
		if (rebuild) {
			toRemoveAfterRebuild.insert(entity);
		}
		if (assert(optimised->Exists(entity) == false)) {
			BreakPoint();
		}
	} else if (optimised->Exists(entity)) {
		optimised->Remove(entity);
		if (rebuild) {
			toRemoveAfterRebuild.insert(entity);
		}
	} else {
		ASSERT(false);
	}

	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));
}

void ThreeStageDbvh::SetMask(EntityType entity, MaskType mask)
{
	if (dynamic->Exists(entity)) {
		dynamic->SetMask(entity, mask);
	} else if (dynamic2->Exists(entity)) {
		dynamic2->SetMask(entity, mask);
		if (rebuild) {
			setMaskAfterRebuild[entity] = mask;
		}
	} else if (optimised->Exists(entity)) {
		optimised->SetMask(entity, mask);
		if (rebuild) {
			setMaskAfterRebuild[entity] = mask;
		}
	} else {
		ASSERT(false);
	}
}

void ThreeStageDbvh::TryIntegrateOptimised()
{
	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));

	if (rebuild) {
		if (finishedRebuilding->load()) {
			if (clear) {
				clear = false;
				rebuild->Clear();
				rebuild = nullptr;
			} else {
				optimised->Clear();
				std::swap(_optimised, _rebuild);
				optimised = _optimised.get();
				rebuild = nullptr;

				for (auto it : setMaskAfterRebuild) {
					if (optimised->Exists(it.first)) {
						optimised->SetMask(it.first, it.second);
					}
				}
				for (auto entity : toRemoveAfterRebuild) {
					if (optimised->Exists(entity)) {
						optimised->Remove(entity);
					}
				}

				toRemoveAfterRebuild.clear();
				setMaskAfterRebuild.clear();

				for (auto it = dynamic2->RestartIterator(); it->Valid();
					 it->Next()) {
					if (optimised->Exists(it->entity)) {
						auto a = it->aabb;
						auto b = optimised->GetAabb(it->entity);
						glm::vec3 c = a.min - b.min;
						glm::vec3 d = a.max - b.max;
						float len = glm::length(c) + glm::length(d);
						assert(len < 0.001);
						toRemoveAfterRebuild.insert(it->entity);
					}
				}
				for (auto entity : toRemoveAfterRebuild) {
					dynamic2->Remove(entity);
				}
				toRemoveAfterRebuild.clear();

				for (auto it = dynamic2->RestartIterator(); it->Valid();
					 it->Next()) {
					if (assert(optimised->Exists(it->entity))) {
						BreakPoint();
					}
				}

				dynamic2->Clear();
			}
			finishedRebuilding->store(false);
		}
	}

	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));
}

void ThreeStageDbvh::TryScheduleRebuild()
{
	if (rebuild) {
		if (finishedRebuilding->load() == false) {
			return;
		}
	}

	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));

	if (scheduleRebuildFunc) {
		rebuild = _rebuild.get();
		rebuild->Clear();
		for (auto it = optimised->RestartIterator(); it->Valid(); it->Next()) {
			rebuild->Add(it->entity, it->aabb, it->mask);
		}

		for (auto it = dynamic->RestartIterator(); it->Valid(); it->Next()) {
			rebuild->Add(it->entity, it->aabb, it->mask);
		}
		for (auto it = dynamic2->RestartIterator(); it->Valid(); it->Next()) {
			rebuild->Add(it->entity, it->aabb, it->mask);
		}

		if (dynamic2->GetCount() < dynamic->GetCount()) {
			std::swap(_dynamic, _dynamic2);
			std::swap(dynamic, dynamic2);
		}

		for (auto it = dynamic->RestartIterator(); it->Valid(); it->Next()) {
			dynamic2->Add(it->entity, it->aabb, it->mask);
		}
		dynamic->Clear();

		tests = 0;
		dynamicUpdates = 0;
		optimisedUpdates = 0;
		finishedRebuilding->store(false);

		toRemoveAfterRebuild.clear();
		setMaskAfterRebuild.clear();

		scheduleRebuildFunc(_finishedRebuilding, _rebuild,
							scheduleUpdateUserData);
	} else {
		Rebuild();
		
		toRemoveAfterRebuild.clear();
		setMaskAfterRebuild.clear();
	}

	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));
}

int32_t ThreeStageDbvh::GetCount() const
{
	return dynamic->GetCount() + optimised->GetCount() + dynamic2->GetCount();
}

bool ThreeStageDbvh::Exists(EntityType entity) const
{
	return optimised->Exists(entity) || dynamic->Exists(entity) ||
		   dynamic2->Exists(entity);
}

Aabb ThreeStageDbvh::GetAabb(EntityType entity) const
{
	if (optimised->Exists(entity)) {
		return optimised->GetAabb(entity);
	} else if (dynamic->Exists(entity)) {
		return dynamic->GetAabb(entity);
	} else if (dynamic2->Exists(entity)) {
		return dynamic2->GetAabb(entity);
	} else {
		ASSERT(false && "DUPA DUPA DUPA DUP DUP DUP DUP DUP DUPAAAAAA DUPA");
		return {};
	}
}

MaskType ThreeStageDbvh::GetMask(EntityType entity) const
{
	if (optimised->Exists(entity)) {
		return optimised->GetMask(entity);
	} else if (dynamic->Exists(entity)) {
		return dynamic->GetMask(entity);
	} else if (dynamic2->Exists(entity)) {
		return dynamic2->GetMask(entity);
	} else {
		ASSERT(false && "DUPA DUPA DUPA DUP DUP DUP DUP DUP DUPAAAAAA DUPA");
		return 0;
	}
}

void ThreeStageDbvh::IntersectAabb(IntersectionCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	TryIntegrateOptimised();

	++tests;
	if (tests > 100000) {
		TryScheduleRebuild();
	}

	dynamic2->IntersectAabb(cb);
	dynamic->IntersectAabb(cb);
	optimised->IntersectAabb(cb);
}

void ThreeStageDbvh::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	TryIntegrateOptimised();

	++tests;
	if (tests > 100000) {
		TryScheduleRebuild();
	}

	dynamic2->IntersectRay(cb);
	dynamic->IntersectRay(cb);
	optimised->IntersectRay(cb);
}

void ThreeStageDbvh::Rebuild()
{
	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));

	if (rebuild) {
		clear = true;
	}

	for (auto it = dynamic->RestartIterator(); it->Valid(); it->Next()) {
		optimised->Add(it->entity, it->aabb, it->mask);
	}
	for (auto it = dynamic2->RestartIterator(); it->Valid(); it->Next()) {
		optimised->Add(it->entity, it->aabb, it->mask);
	}
	optimised->Rebuild();

	dynamic->Clear();
	dynamic2->Clear();

	tests = 0;
	dynamicUpdates = 0;
	optimisedUpdates = 0;
	finishedRebuilding->store(false);
	tests = 0;

	assert((optimised->GetCount() + dynamic->GetCount() +
			dynamic2->GetCount()) == entitiesData.size() ||
		   (BreakPoint(), false));
}

BroadphaseBaseIterator *ThreeStageDbvh::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

ThreeStageDbvh::Iterator::Iterator(ThreeStageDbvh &bp)
{
	this->bp = &bp;
	stage = 0;
	if (bp.optimised) {
		it = bp.optimised->RestartIterator();
	} else {
		it = nullptr;
	}
	if (it && it->Valid()) {
		FetchData();
	}
	Next();
}

ThreeStageDbvh::Iterator::~Iterator() {}

bool ThreeStageDbvh::Iterator::Next()
{
	if (it == nullptr) {
		return false;
	}

	if (it->Next() == false) {
		switch (stage) {
		case 0:
			stage = 1;
			it = bp->dynamic2->RestartIterator();
			if (it->Valid()) {
				break;
			}
		case 1:
			stage = 2;
			it = bp->dynamic->RestartIterator();
			if (it->Valid()) {
				break;
			}
		default:
			it = nullptr;
			return false;
		}
	}
	return FetchData();
}

bool ThreeStageDbvh::Iterator::FetchData()
{
	if (Valid()) {
		entity = it->entity;
		aabb = it->aabb;
		mask = it->mask;
		return true;
	}
	return false;
}

bool ThreeStageDbvh::Iterator::Valid() { return it; }

void ThreeStageDbvh::SetRebuildSchedulerFunction(
	void (*_ScheduleRebuildFunc)(
		std::shared_ptr<std::atomic<bool>> finishedRebuilding,
		std::shared_ptr<BroadphaseBase> dbvh, std::shared_ptr<void> data),
	std::shared_ptr<void> scheduleUpdateUserData)
{
	scheduleRebuildFunc = _ScheduleRebuildFunc;
	this->scheduleUpdateUserData = scheduleUpdateUserData;
}
} // namespace spp
#undef assert
