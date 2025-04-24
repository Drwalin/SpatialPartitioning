#include <cstdio>

#include <random>

#include "../include/spatial_partitioning/Aabb.hpp"
#include "glm/geometric.hpp"

std::mt19937_64 mt(12345);


//     34864:       1 ==    1275
//     		-400.00  -10.00 -400.00
//     		400.00  100.00  400.00
//     		<->
//     		8.16    0.57    4.64
//     		11.59    8.26    7.13
//     		hit points:
//     		1.47  -10.48 -110.65
//     		-0.70  -22.86 -271.91
//     		start:
//     		-1.41  -26.94 -325.14
//     		-1.41  -26.94 -325.14
//     		end:
//     		3.08   -1.31    8.82
//     		3.08   -1.31    8.82
//     		dist:
//     		215.14
//     		53.39

//     37512:  612307 ==       1
//     		-2.07   -0.00    2.48
//     		2.25    9.93    5.19
//     		<->
//     		-400.00  -10.00 -400.00
//     		400.00  100.00  400.00
//     		hit points:
//     		262.15  -20.02  173.82
//     		146.66  -10.48  101.56
//     		start:
//     		447.42  -35.33  289.73
//     		447.42  -35.33  289.73
//     		end:
//     		0.07    1.63    9.85
//     		0.07    1.63    9.85
//     		dist:
//     		219.07
//     		355.64

//     37911:  263285 ==       1
//     		0.47   -0.08    0.89
//     		2.83    8.25    5.12
//     		<->
//     		-400.00  -10.00 -400.00
//     		400.00  100.00  400.00
//     		hit points:
//     		-377.30   29.90 -385.93
//     		-366.93   29.06 -375.81
//     		start:
//     		-407.92   32.40 -415.78
//     		-407.92   32.40 -415.78
//     		end:
//     		9.60   -1.68   -8.73
//     		9.60   -1.68   -8.73
//     		dist:
//     		42.84
//     		57.35

template<typename AABB=spp::Aabb>
void PrintRayTest(glm::vec3 start, glm::vec3 end, AABB aabb)
{
	
	glm::vec3 dir = end - start;
	float length = glm::length(dir);
	glm::vec3 dirNormalized = glm::normalize(dir);
	glm::vec3 invDir = glm::vec3(1.f, 1.f, 1.f) / dirNormalized;
	
	float near = 0, far = 0;
	bool res = aabb.FastRayTest(start, dir, invDir, length, near, far);
	
	float hitDistance = near;
	glm::vec3 hitPoint = start + dirNormalized * near;
// 	float factor = near / length;
	
	spp::Aabb abm = aabb;
	
	printf("Hit result: {%8.2f %8.2f %8.2f .. %8.2f %8.2f %8.2f}   "
			"HitPoint: {%8.2f %8.2f %8.2f}    "
			"start: {%8.2f %8.2f %8.2f}    "
			"end: {%8.2f %8.2f %8.2f}    "
			"dist: %8.2f     "
			"%s\n",
			abm.min.x, abm.min.y, abm.min.z,
			abm.max.x, abm.max.y, abm.max.z,
			hitPoint.x, hitPoint.y, hitPoint.z,
			start.x, start.y, start.z,
			end.x, end.y, end.z,
			hitDistance,
			res ? "HAS HIT" : "MISS");
}

struct Sample {
	spp::Aabb aabb;
	glm::vec3 start, end;
};
void PrintRayTests(Sample sample)
{
	PrintRayTest(sample.start, sample.end, (spp::Aabb)sample.aabb);
	PrintRayTest(sample.start, sample.end, (spp::AabbCentered)sample.aabb);
}

void main2()
{
	PrintRayTests({{{8.16, 0.57, 4.64}, {11.59, 8.26, 7.13}},
				   {-1.41, -26.94, -325.14},
				   {3.08, -1.31, 8.82}});
	
	
	
	
// 5.98    1.01   -9.69 
// 9.80    7.85   -6.34 
// <-> 
// -400.00  -10.00 -400.00 
// 400.00  100.00  400.00     
// hit points: 
// -126.87  -34.69 -332.17 
// -39.17  -10.48 -104.
// start: 
// -140.54  -38.46 -367.67 
// end:    
// 0.31    0.42   -2.05 
// dist:   
// 38.22 
// 283.38

	PrintRayTests({{{5.98, 1.01, -9.69}, {9.80, 7.85, -6.34}},
				   {-140.54, -38.46, -367.67},
				   {0.31, 0.42, -2.05}});
	
	PrintRayTests({{{-400,-10,-400}, {400,100,400}},
				   {-140.54, -38.46, -367.67},
				   {0.31, 0.42, -2.05}});
}

int main()
{
	printf(" TEST SUITE 1:\n");
	main2();
	
	printf("\n TEST SUITE 2:\n");
	spp::Aabb aabb;
	glm::vec3 start, end;
	float n, f;
	
	start = {-4,8,5};
	end = {27,11,2};
	aabb = {{14,10,3}, {18,11,5}};
	if (aabb.SlowRayTest(start, end, n, f)) {
		glm::vec3 p = (start + glm::normalize(end-start)*n);
		printf(" git,    n: %f    f: %f      p: %.2f %.2f %.2f\n", 
				n, f, p.x, p.y, p.z);
	} else {
		printf("Not git\n");
	}
	
	
	start = {-4,8,0};
	end = {27,11,0};
	aabb = {{14,10,-10}, {18,11,10}};
	if (aabb.SlowRayTest(start, end, n, f)) {
		glm::vec3 p = (start + glm::normalize(end-start)*n);
		printf(" git,    n: %f    f: %f      p: %.2f %.2f %.2f\n", 
				n, f, p.x, p.y, p.z);
	} else {
		printf("Not git\n");
	}
	
	
	start = {27,11,0};
	end = {-4,8,0};
	aabb = {{14,10,-10}, {18,11,10}};
	if (aabb.SlowRayTest(start, end, n, f)) {
		glm::vec3 p = (start + glm::normalize(end-start)*n);
		printf(" git,    n: %f    f: %f      p: %.2f %.2f %.2f\n", 
				n, f, p.x, p.y, p.z);
	} else {
		printf("Not git\n");
	}
	
	return 0;
}
