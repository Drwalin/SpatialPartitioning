// This file is part of SpatialPartitioning.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include <cstring>

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

// #undef assert
// #define assert(COND) ASSERT(COND)

namespace spp
{

SPP_TEMPLATE_DECL
ThreeStageDbvh<SPP_TEMPLATE_ARGS>::ThreeStageDbvh(std::shared_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> optimised,
							   std::shared_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> rebuilding,
							   std::unique_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> &&dynamic)
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
	this->dynamic = _dynamic.get();
}

SPP_TEMPLATE_DECL
ThreeStageDbvh<SPP_TEMPLATE_ARGS>::~ThreeStageDbvh() {}

SPP_TEMPLATE_DECL
const char *ThreeStageDbvh<SPP_TEMPLATE_ARGS>::GetName() const {
	thread_local char n[1024];
	snprintf(n, 1023, "ThreeStageDbvh %s %s [%i]", optimised->GetName(), dynamic->GetName(), dynamic->GetCount());
	return n;
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::Clear()
{
	if (rebuild) {
		clear = true;
		TryIntegrateOptimised();
	}
	dynamic->Clear();
	optimised->Clear();
}

SPP_TEMPLATE_DECL
size_t ThreeStageDbvh<SPP_TEMPLATE_ARGS>::GetMemoryUsage() const
{
	return dbvhs[0]->GetMemoryUsage() + (dbvhs[1] ? dbvhs[1]->GetMemoryUsage() : 0lu) +
		   dynamic->GetMemoryUsage() +

		   6 * 32 +

		   toRemoveAfterRebuild.capacity() * sizeof(EntityType) +

		   setMaskAfterRebuild.GetMemoryUsage();
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::ShrinkToFit()
{
	dbvhs[0]->ShrinkToFit();
	if (dbvhs[1]) {
		dbvhs[1]->ShrinkToFit();
	}
	dynamic->ShrinkToFit();
	toRemoveAfterRebuild.shrink_to_fit();
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::StartFastAdding()
{
	fastAdding = true;

	if (rebuild) {
		clear = true;
	}

	for (auto it = dynamic->RestartIterator(); it->Valid(); it->Next()) {
		optimised->Add(it->entity, it->aabb, it->mask);
	}
	dynamic->Clear();

	tests = 0;
	dynamicUpdates = 0;
	optimisedUpdates = 0;
	finishedRebuilding->store(false);
	tests = 0;
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::StopFastAdding() { fastAdding = false; }

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::Add(EntityType entity, Aabb aabb, MaskType mask)
{
	assert(Exists(entity) == false);

	if (fastAdding) {
		optimised->Add(entity, aabb, mask);
	} else {
		dynamicUpdates++;
		dynamic->Add(entity, aabb, mask);
	}
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::Update(EntityType entity, Aabb aabb)
{
	assert(Exists(entity) == true);

	if (fastAdding) {
		optimised->Update(entity, aabb);
		return;
	}

	TryIntegrateOptimised();

	if (dynamic->Exists(entity)) {
		dynamicUpdates++;
		dynamic->Update(entity, aabb);
	} else {
		if (rebuild) {
			toRemoveAfterRebuild.push_back(entity);
		}

		assert(optimised->Exists(entity) && "This is an alternative scenario");
		MaskType mask = optimised->GetMask(entity);
		optimised->Remove(entity);

		optimisedUpdates++;
		dynamicUpdates++;
		dynamic->Add(entity, aabb, mask);
	}

	if (dynamicUpdates + optimisedUpdates > 100000) {
		TryScheduleRebuild();
	}
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::Remove(EntityType entity)
{
	assert(Exists(entity) == true);

	if (fastAdding) {
		optimised->Remove(entity);
		return;
	}

	TryIntegrateOptimised();

	if (rebuild) {
		toRemoveAfterRebuild.push_back(entity);
	}

	if (dynamic->Exists(entity)) {
		dynamic->Remove(entity);
		assert(optimised->Exists(entity) == false);
	} else if (optimised->Exists(entity)) {
		optimised->Remove(entity);
	} else {
		ASSERT(false);
	}
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::SetMask(EntityType entity, MaskType mask)
{
	if (dynamic->Exists(entity)) {
		dynamic->SetMask(entity, mask);
	} else if (optimised->Exists(entity)) {
		optimised->SetMask(entity, mask);
		if (rebuild) {
			setMaskAfterRebuild[entity] = mask;
		}
	} else {
		ASSERT(false);
	}
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::TryIntegrateOptimised()
{
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

				for (auto entity : toRemoveAfterRebuild) {
					if (optimised->Exists(entity)) {
						optimised->Remove(entity);
					}
				}
				setMaskAfterRebuild.clear();

				for (auto it : setMaskAfterRebuild) {
					if (optimised->Exists(it.first)) {
						optimised->SetMask(it.first, it.second);
					}
				}
				toRemoveAfterRebuild.clear();

				for (auto it = dynamic->RestartIterator(); it->Valid();
					 it->Next()) {
					if (optimised->Exists(it->entity)) {
						auto a = it->aabb;
						auto b = optimised->GetAabb(it->entity);
						glm::vec3 c = glm::abs(a.min - b.min);
						glm::vec3 d = glm::abs(a.max - b.max);

						float len = c.x + c.y + c.z + d.x + d.y + d.z;
						if (len < 0.001) {
							toRemoveAfterRebuild.push_back(it->entity);
						} else {
							optimised->Remove(it->entity);
						}
					}
				}
				for (auto entity : toRemoveAfterRebuild) {
					dynamic->Remove(entity);
				}
				toRemoveAfterRebuild.clear();

				for (auto it = dynamic->RestartIterator(); it->Valid();
					 it->Next()) {
					assert(optimised->Exists(it->entity) == false);
				}
			}
			finishedRebuilding->store(false);
		}
	}
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::TryScheduleRebuild()
{
	if (rebuild) {
		if (finishedRebuilding->load() == false) {
			return;
		}
	}

	if (scheduleRebuildFunc && dbvhs[1] != nullptr) {
		rebuild = _rebuild.get();
		rebuild->Clear();
		for (auto it = optimised->RestartIterator(); it->Valid(); it->Next()) {
			rebuild->Add(it->entity, it->aabb, it->mask);
		}

		for (auto it = dynamic->RestartIterator(); it->Valid(); it->Next()) {
			rebuild->Add(it->entity, it->aabb, it->mask);
		}

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
}

SPP_TEMPLATE_DECL
int32_t ThreeStageDbvh<SPP_TEMPLATE_ARGS>::GetCount() const
{
	return dynamic->GetCount() + optimised->GetCount();
}

SPP_TEMPLATE_DECL
bool ThreeStageDbvh<SPP_TEMPLATE_ARGS>::Exists(EntityType entity) const
{
	return optimised->Exists(entity) || dynamic->Exists(entity);
}

SPP_TEMPLATE_DECL
Aabb ThreeStageDbvh<SPP_TEMPLATE_ARGS>::GetAabb(EntityType entity) const
{
	if (optimised->Exists(entity)) {
		return optimised->GetAabb(entity);
	} else if (dynamic->Exists(entity)) {
		return dynamic->GetAabb(entity);
	} else {
		ASSERT(!"Entity does not exist");
		return {};
	}
}

SPP_TEMPLATE_DECL
MaskType ThreeStageDbvh<SPP_TEMPLATE_ARGS>::GetMask(EntityType entity) const
{
	if (optimised->Exists(entity)) {
		return optimised->GetMask(entity);
	} else if (dynamic->Exists(entity)) {
		return dynamic->GetMask(entity);
	} else {
		ASSERT(!"Entity does not exist");
		return 0;
	}
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::IntersectAabb(AabbCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	TryIntegrateOptimised();

	dynamic->IntersectAabb(cb);
	optimised->IntersectAabb(cb);
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::IntersectRay(RayCallback &cb)
{
	if (cb.callback == nullptr) {
		return;
	}

	TryIntegrateOptimised();

	dynamic->IntersectRay(cb);
	optimised->IntersectRay(cb);
}

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::Rebuild()
{
	if (rebuild) {
		clear = true;
	}

	for (auto it = dynamic->RestartIterator(); it->Valid(); it->Next()) {
		optimised->Add(it->entity, it->aabb, it->mask);
	}
	optimised->Rebuild();

	dynamic->Clear();

	tests = 0;
	dynamicUpdates = 0;
	optimisedUpdates = 0;
	finishedRebuilding->store(false);
	tests = 0;
}

SPP_TEMPLATE_DECL
BroadphaseBaseIterator<SPP_TEMPLATE_ARGS> *ThreeStageDbvh<SPP_TEMPLATE_ARGS>::RestartIterator()
{
	iterator = {*this};
	return &iterator;
}

SPP_TEMPLATE_DECL
ThreeStageDbvh<SPP_TEMPLATE_ARGS>::Iterator::Iterator(ThreeStageDbvh &bp)
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

SPP_TEMPLATE_DECL
ThreeStageDbvh<SPP_TEMPLATE_ARGS>::Iterator::~Iterator() {}

SPP_TEMPLATE_DECL
bool ThreeStageDbvh<SPP_TEMPLATE_ARGS>::Iterator::Next()
{
	if (it == nullptr) {
		return false;
	}

	if (it->Next() == false) {
		switch (stage) {
		case 0:
			stage = 1;
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

SPP_TEMPLATE_DECL
bool ThreeStageDbvh<SPP_TEMPLATE_ARGS>::Iterator::FetchData()
{
	if (Valid()) {
		this->entity = it->entity;
		this->aabb = it->aabb;
		this->mask = it->mask;
		return true;
	}
	return false;
}

SPP_TEMPLATE_DECL
bool ThreeStageDbvh<SPP_TEMPLATE_ARGS>::Iterator::Valid() { return it; }

SPP_TEMPLATE_DECL
void ThreeStageDbvh<SPP_TEMPLATE_ARGS>::SetRebuildSchedulerFunction(
	void (*_ScheduleRebuildFunc)(
		std::shared_ptr<std::atomic<bool>> finishedRebuilding,
		std::shared_ptr<BroadphaseBase<SPP_TEMPLATE_ARGS>> dbvh, std::shared_ptr<void> data),
	std::shared_ptr<void> scheduleUpdateUserData)
{
	scheduleRebuildFunc = _ScheduleRebuildFunc;
	this->scheduleUpdateUserData = scheduleUpdateUserData;
}

SPP_DEFINE_VARIANTS(ThreeStageDbvh)

} // namespace spp
#undef assert
