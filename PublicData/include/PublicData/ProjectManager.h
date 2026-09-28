#pragma once

#include <ProjectManager/ProjectManager_global.h>
#include <ProjectManager/IProjectSerializable.h>
#include <string>
#include <vector>

namespace PublicData
{
	class ProjectManager_SHAREDLIB ProjectManager
	{
	public:
		void setProjectDir(const std::string& dir);
		void registerModule(IProjectSerializable* module);
		bool saveProject();
		bool loadProject();

	private:
		std::string projectDir_;
		std::vector<IProjectSerializable*> modules_;
	};
}