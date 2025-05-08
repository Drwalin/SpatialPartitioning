// This file is part of SpatialPartitioning.
// Copyright (c) 2024-2025 Marek Zalewski aka Drwalin
// You should have received a copy of the MIT License along with this program.

#include "../glm/glm/geometric.hpp"

#include "../include/spatial_partitioning/RayInfo.hpp"

namespace spp
{
void RayInfo::Calc(glm::vec3 start, glm::vec3 end)
{
	this->start = start;
	this->end = end;
	dir = end - start;
	length = glm::length(dir);
	dirNormalized = glm::normalize(dir);
	for (int i = 0; i < 3; ++i) {
		invDir[i] = dir[i] == 0.0f ? 1e18f : 1.0f / dir[i];
	}
	signs[0] = invDir[0] < 0.0 ? 1 : 0;
	signs[1] = invDir[1] < 0.0 ? 1 : 0;
	signs[2] = invDir[2] < 0.0 ? 1 : 0;
}
} // namespace spp
