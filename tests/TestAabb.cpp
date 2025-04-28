#include <cstdio>

#include <random>

#include "../include/spatial_partitioning/Aabb.hpp"
#include "../src/bullet/btAabbUtil2.h"
#include "glm/geometric.hpp"

float INF = 3.0 / 0.0;

bool BulletRayTest(glm::vec3 start, glm::vec3 end, spp::Aabb aabb,
				   glm::vec3 &hitPoint, float &near)
{
	glm::vec3 a = start;
	glm::vec3 b = end;
	glm::vec3 c = aabb.min;
	glm::vec3 d = aabb.max;
	bullet::btVector3 norm;
	near = 1.0;
	if (bullet::btRayAabb({a.x, a.y, a.z}, {b.x, b.y, b.z}, {c.x, c.y, c.z},
						  {d.x, d.y, d.z}, near, norm)) {
		hitPoint = start + ((end - start) * near);
		return true;
	}
	near = -1;
	return false;
}

bool AabbRayTest(glm::vec3 start, glm::vec3 end, spp::Aabb aabb,
				 glm::vec3 &hitPoint, float &near)
{
	float far;
	if (aabb.SlowRayTest2(start, end, near, far)) {
		hitPoint = start + ((end - start) * near);
		return true;
	}
	near = -1;
	return false;
}

bool CenteredRayTest(glm::vec3 start, glm::vec3 end, spp::Aabb aabb,
					 glm::vec3 &hitPoint, float &near)
{
	float f;
	if (((spp::AabbCentered)aabb).SlowRayTestCenter(start, end, near, f)) {
		hitPoint = start + ((end - start) * near);
		return true;
	}
	near = -1;
	return false;
}

std::mt19937_64 mt(12345);

using FuncType = bool (*)(glm::vec3, glm::vec3, spp::Aabb, glm::vec3 &,
						  float &);

bool PrintRayTest(glm::vec3 start, glm::vec3 end, spp::Aabb aabb)
{
	spp::Aabb abm = aabb;
	float n[3];
	glm::vec3 p[3];
	bool res[3];
	bool pin[3];

	const float EPS = 0.001;

	abm.min -= glm::vec3{EPS, EPS, EPS};
	abm.max += glm::vec3{EPS, EPS, EPS};

	const FuncType func[3] = {BulletRayTest, AabbRayTest, CenteredRayTest};

	for (int i = 0; i < 3; ++i) {
		if ((res[i] = func[i](start, end, aabb, p[i], n[i]))) {
			pin[i] = abm.ContainsAll({p[i], p[i]});
		} else {
			n[i] = -1;
			p[i] = {INF, INF, INF};
			pin[i] = false;
		}
	}

	abm = aabb;

	if (res[0] == res[1] && res[0] == res[2] && pin[0] == pin[1] &&
		pin[0] == pin[2]) {
		return true;
	}

	for (int i = 0; i < 3; ++i) {
		if (i == 0) {
			printf("aabb: ");
		} else {
			printf("      ");
		}
		printf("{ %12.6f .. %12.6f }    ", abm.min[i], abm.max[i]);
		if (i == 0) {
			printf("start: ");
		} else {
			printf("       ");
		}
		printf("{ %12.6f }    ", start[i]);
		if (i == 0) {
			printf("end: ");
		} else {
			printf("     ");
		}
		printf("{ %12.6f }    ", end[i]);
		printf("\n");
	}

	for (int i = 0; i < 3; ++i) {
		printf("hit: {%12.6f %12.6f %12.6f}    "
			   "dist: %12.6f     "
			   "%s   %s\n",
			   p[i].x, p[i].y, p[i].z, n[i], res[i] ? "HIT   " : "MISS  ",
			   pin[i]	? "VALID"
			   : res[i] ? "INVALID"
						: "");
	}

	printf("\n");
	return false;
}

int totalTests = 0;
int failedTests = 0;

struct Sample {
	spp::Aabb aabb;
	glm::vec3 start, end;
};
void PrintRayTests(Sample sample)
{
	++totalTests;
	if (!PrintRayTest(sample.start, sample.end, (spp::Aabb)sample.aabb)) {
		++failedTests;
	}
}

void main2()
{
	PrintRayTests({{{8.16, 0.57, 4.64}, {11.59, 8.26, 7.13}},
				   {-1.41, -26.94, -325.14},
				   {3.08, -1.31, 8.82}});

	PrintRayTests({{{5.98, 1.01, -9.69}, {9.80, 7.85, -6.34}},
				   {-140.54, -38.46, -367.67},
				   {0.31, 0.42, -2.05}});

	PrintRayTests({{{-400, -10, -400}, {400, 100, 400}},
				   {-140.54, -38.46, -367.67},
				   {0.31, 0.42, -2.05}});

	PrintRayTests({{{505.91, 44.51, 516.16}, {508.41, 47.01, 518.66}},
				   {510.53, 44.70, 520.18},
				   {-5.93, 1.13, 8.57}});

	PrintRayTests({{{-482.57, -9.95, -452.50}, {317.43, 100.05, 347.50}},
				   {510.53, 44.70, 520.18},
				   {-5.93, 1.13, 8.57}});

	PrintRayTests({{{-407.68, -15.09, -220.11}, {392.32, 94.91, 579.89}},
				   {-409.09, -1.95, -322.72},
				   {-5.15, 2.23, -2.92}});

	PrintRayTests({{{-315.54, -1.51, -251.16}, {-313.06, 0.99, -248.66}},
				   {-409.09, -1.95, -322.72},
				   {-5.15, 2.23, -2.92}});

	PrintRayTests({{{51.45, -35.54, -111.60}, {53.96, -33.04, -109.11}},
				   {69.49, -45.62, -138.86},
				   {-9.72, 1.98, -8.09}});

	PrintRayTests({{{35.52, -31.22, -85.71}, {39.67, -21.38, -84.13}},
				   {69.49, -45.62, -138.86},
				   {-9.72, 1.98, -8.09}});

	PrintRayTests({{{-422.68, 6.42, -425.11}, {-420.20, 8.92, -422.59}},
				   {-421.21, 8.41, -424.90},
				   {-9.01, 1.08, 4.78}});

	PrintRayTests({{{-498.29, -20.57, -350.35}, {301.71, 89.43, 449.65}},
				   {-421.21, 8.41, -424.90},
				   {-9.01, 1.08, 4.78}});

	PrintRayTests({{{467.77, 44.41, 470.79}, {470.30, 46.91, 473.30}},
				   {509.77, 49.75, 511.41},
				   {2.29, -0.02, 2.25}});

	PrintRayTests({{{-367.85, -0.71, -402.84}, {432.15, 109.29, 397.16}},
				   {509.77, 49.75, 511.41},
				   {2.29, -0.02, 2.25}});

	PrintRayTests({{{258.14, -14.86, -279.04}, {260.66, -12.36, -276.56}},
				   {474.93, -22.97, -498.41},
				   {-8.15, -2.13, 1.75}});

	PrintRayTests({{{258.14, -14.86, -279.04}, {260.64, -12.35, -276.52}},
				   {474.93, -22.97, -498.41},
				   {-8.15, -2.13, 1.75}});
}

int main()
{
	printf(" TEST SUITE 1:\n");
	main2();

	printf("\n TEST SUITE 2:\n");
	spp::Aabb aabb;
	glm::vec3 start, end;
	float n, f;

	start = {-4, 8, 5};
	end = {27, 11, 2};
	aabb = {{14, 10, 3}, {18, 11, 5}};
	if (aabb.SlowRayTestCenter(start, end, n, f)) {
		glm::vec3 p = (start + glm::normalize(end - start) * n);
		printf(" git,    n: %f    f: %f      p: %.2f %.2f %.2f\n", n, f, p.x,
			   p.y, p.z);
	} else {
		printf("Not git\n");
	}

	start = {-4, 8, 0};
	end = {27, 11, 0};
	aabb = {{14, 10, -10}, {18, 11, 10}};
	if (aabb.SlowRayTestCenter(start, end, n, f)) {
		glm::vec3 p = (start + glm::normalize(end - start) * n);
		printf(" git,    n: %f    f: %f      p: %.2f %.2f %.2f\n", n, f, p.x,
			   p.y, p.z);
	} else {
		printf("Not git\n");
	}

	start = {27, 11, 0};
	end = {-4, 8, 0};
	aabb = {{14, 10, -10}, {18, 11, 10}};
	if (aabb.SlowRayTestCenter(start, end, n, f)) {
		glm::vec3 p = (start + glm::normalize(end - start) * n);
		printf(" git,    n: %f    f: %f      p: %.2f %.2f %.2f\n", n, f, p.x,
			   p.y, p.z);
	} else {
		printf("Not git\n");
	}

	if (failedTests == totalTests) {
		printf("\n\n   PASSED %i / %i ... OK\n", totalTests - failedTests,
			   totalTests);
	} else {
		printf("\n\n   PASSED %i / %i ... %i failed\n",
			   totalTests - failedTests, totalTests, failedTests);
	}

	return 0;
}
