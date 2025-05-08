// This file is part of SpatialPartitioning.
// Copyright (c) 2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#pragma once

#include "../../glm/glm/ext/vector_float3.hpp"

namespace spp
{
struct RayInfo {
	RayInfo() = default;
	RayInfo(glm::vec3 start, glm::vec3 end) { Calc(start, end); }
	
	void Calc(glm::vec3 start, glm::vec3 end);
	
	glm::vec3 start;
	glm::vec3 end;
	glm::vec3 dir;
	glm::vec3 dirNormalized;
	glm::vec3 invDir;
	int signs[3];
	float length;
};
}
