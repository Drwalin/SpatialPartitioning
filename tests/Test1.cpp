#include <cstdio>

#include <random>
#include <vector>
#include <set>
#include <chrono>

#include "../include/spatial_partitioning/BroadPhaseBase.hpp"
#include "../include/spatial_partitioning/BruteForce.hpp"
#include "../include/spatial_partitioning/BvhMedianSplitHeap.hpp"

const int32_t TOTAL_ENTITIES = 100000;
const size_t TOTAL_AABB_TESTS = 10000;
const size_t BRUTE_FROCE_TESTS_COUNT_DIVISOR = 1;

std::mt19937_64 mt(12345);

std::vector<spp::Aabb> aabbsToTest;
auto SingleTest(spp::BroadphaseBase *broadphase,
const std::vector<spp::Aabb> &aabbsToTest, size_t testsCount)
{
	struct _Cb : public spp::IntersectionCallback {
		std::vector<uint64_t> entities;

	} cb;
	cb.mask = ~(uint32_t)0;
	typedef void (*CbT)(spp::IntersectionCallback *, uint64_t);
	cb.callback = (CbT) + [](_Cb *cb, uint64_t entity) {
		cb->entities.push_back(entity);
	};
	testsCount = std::min(testsCount, aabbsToTest.size());
	for (size_t i=0; i<testsCount; ++i) {
		cb.aabb = aabbsToTest[i];
		broadphase->IntersectAabb(cb);
	}

	return cb;
}

std::vector<spp::Aabb> globalAabbs;
std::vector<std::set<uint64_t>>
Test(std::vector<spp::BroadphaseBase *> broadphases, size_t testsCount)
{
	std::uniform_real_distribution<float> distPos(-300, 300);
	std::uniform_real_distribution<float> distSize(pow(0.2,1.0/3.0),1.0);
	std::vector<spp::Aabb> &aabbs = globalAabbs;
	size_t i = aabbs.size();
	aabbs.resize(testsCount);
	for (; i<aabbs.size(); ++i) {
		spp::Aabb &aabb = aabbs[i];
		glm::vec3 p = {distPos(mt), distPos(mt) / 8.0f, distPos(mt)};
		glm::vec3 s = {distSize(mt), distSize(mt), distSize(mt)};
		s = s * s * 10.0f;
		aabb = {p, p+s};
	}
	
	std::vector<std::set<uint64_t>> ents;
	for (int i = 0; i < broadphases.size(); ++i) {
		auto &it = broadphases[i];
		size_t tC = i?aabbs.size():aabbs.size()/BRUTE_FROCE_TESTS_COUNT_DIVISOR;
		auto beg = std::chrono::steady_clock::now();
		auto vec = SingleTest(it, aabbs, tC);
		auto end = std::chrono::steady_clock::now();
		ents.push_back(
			std::set<uint64_t>(vec.entities.begin(), vec.entities.end()));
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%i intersection test [count: %lu]: %.3f us/op\n", i, tC, us/double(tC));
		printf("    nodesTested: %lu,   testedCount: %lu     [count = %lu]:\n ",
			   vec.nodesTestedCount, vec.testedCount, ents.back().size());
		fflush(stdout);
	}

	printf("\n Assumes bruteforce is valid\n");
	for (int i = 1; i < broadphases.size(); ++i) {
		printf(" %i: result is: %s\n", i,
			   ents[0] == ents[i] ? "valid" : "different ... ERROR!!!");
	}

	return ents;
}

struct EntityData {
	spp::Aabb aabb;
	uint64_t id = 0;
	uint32_t mask = ~0;
};

int main()
{
	std::vector<EntityData> entities;
	entities.resize(TOTAL_ENTITIES);
	std::uniform_real_distribution<float> distPos(-300, 300);
	std::uniform_real_distribution<float> distSize(0.2, 10);
	uint64_t id = 1;
	for (auto &e : entities) {
		e.aabb.min = {distPos(mt), distPos(mt) / 8.0f, distPos(mt)};
		e.aabb.max = {distSize(mt), distSize(mt) * 2, distSize(mt)};
		e.aabb.max += e.aabb.min;
		e.id = id;
		++id;
	}
	entities[0].aabb = {{-200, -10, -200}, {200, 100, 200}};
	entities[1].aabb = {{150, 190, 150}, {200, 200, 200}};

	spp::BvhMedianSplitHeap bvh;
	spp::BruteForce bf;

	std::vector<spp::BroadphaseBase *> broadphases = {
		&bf,
		&bvh,
	};

	int broadphaseId = 1;
	for (auto bp : broadphases) {
		auto beg = std::chrono::steady_clock::now();
		for (const auto &e : entities) {
			bp->Add(e.id, e.aabb, e.mask);
		}
		auto end = std::chrono::steady_clock::now();
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%i adding time: %.3f us\n", broadphaseId, us);
		fflush(stdout);
		++broadphaseId;
	}

	broadphaseId = 1;
	for (auto bp : broadphases) {
		auto beg = std::chrono::steady_clock::now();
		bp->Rebuild();
		auto end = std::chrono::steady_clock::now();
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%i build: %.3f us\n", broadphaseId, us);
		fflush(stdout);
		++broadphaseId;
	}

	std::uniform_real_distribution<float> distDisp(-50, 50);
	for (int i = 0; i < 300; ++i) {
		auto & e = entities[mt()%entities.size()];
		glm::vec3 disp = {distPos(mt), distPos(mt) / 4.0f, distPos(mt)};
		e.aabb.min += disp;
		e.aabb.max += disp;
	}

	broadphaseId = 1;
	for (auto bp : broadphases) {
		auto beg = std::chrono::steady_clock::now();
		for (int i = 0; i < 300 && i < entities.size(); ++i) {
			bp->Update(entities[i].id, entities[i].aabb);
		}
		auto end = std::chrono::steady_clock::now();
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%i update data: %.3f us\n", broadphaseId, us);
		fflush(stdout);
		++broadphaseId;
	}

	broadphaseId = 1;
	for (auto bp : broadphases) {
		auto beg = std::chrono::steady_clock::now();
		bp->Rebuild();
		auto end = std::chrono::steady_clock::now();
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%i rebuild: %.3f us\n", broadphaseId, us);
		fflush(stdout);
		++broadphaseId;
	}

	printf("\nAfter rebuild:\n\n");

	std::vector<std::set<uint64_t>> ents = Test(broadphases, TOTAL_AABB_TESTS);

	std::vector<uint64_t> ee;
	std::vector<glm::vec3> vv;
	const size_t CCC = 300000;
	ee.resize(CCC);
	vv.resize(CCC);
	for (size_t i = 0; i < CCC; ++i) {
		ee[i] = (mt()%entities.size())%700;
		vv[i] = {distPos(mt), distPos(mt) / 4.0f, distPos(mt)};
	}
	
	bvh.SetAabbUpdatePolicy(spp::BvhMedianSplitHeap::ON_UPDATE_EXTEND_AABB);

	broadphaseId = 1;
	for (auto bp : broadphases) {
		auto beg = std::chrono::steady_clock::now();
		for (size_t i = 0; i < ee.size(); ++i) {
			spp::Aabb &a = entities[ee[i]].aabb;
			a.min += vv[i];
			a.max += vv[i];
			bp->Update(ee[i], a);
		}
		auto end = std::chrono::steady_clock::now();
		auto diff = end - beg;
		int64_t ns =
			std::chrono::duration_cast<std::chrono::nanoseconds, int64_t>(diff)
				.count();
		double us = double(ns) / 1000.0;
		printf("%i update data: %.3f us/op\n", broadphaseId, us/double(CCC));
		fflush(stdout);
		++broadphaseId;
	}

	printf("\nAfter updated without rebuild:\n\n");

	ents = Test(broadphases, TOTAL_AABB_TESTS);

	return 0;
}
