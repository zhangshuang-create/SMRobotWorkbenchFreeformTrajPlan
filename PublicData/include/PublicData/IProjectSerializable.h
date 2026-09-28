#pragma once

#include <string>
#include "json.hpp"

namespace PublicData
{
	class IProjectSerializable
	{
	public:
		virtual ~IProjectSerializable() = default;

		virtual bool save(const std::string& projectDir, nlohmann::json& root) = 0;
		virtual bool load(const std::string& projectDir, const nlohmann::json& root) = 0;
	};
}