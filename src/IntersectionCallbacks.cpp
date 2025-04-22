// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../include/spatial_partitioning/IntersectionCallbacks.hpp"

namespace spp
{
void RayCallback::InitVariables()
{
	if (initedVars == false) {
		initedVars = true;
		dir = end - start;
		length = glm::length(dir);
		dirNormalized = glm::normalize(dir);
		invDir = glm::vec3(1.f, 1.f, 1.f) / dirNormalized;
	}
}
}
