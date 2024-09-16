#include <cstdio>

#include <random>

#include "../include/spatial_partitioning/Aabb.hpp"
#include "glm/geometric.hpp"

std::mt19937_64 mt(12345);


int main()
{
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
