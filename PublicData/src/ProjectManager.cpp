#include <ProjectManager/ProjectManager.h>
#include <fstream>
#include <ProjectManager/json.hpp>

using json = nlohmann::json;

namespace PublicData
{
	void ProjectManager::setProjectDir(const std::string& dir)
	{
		projectDir_ = dir;
	}

	void ProjectManager::registerModule(IProjectSerializable* module)
	{
		modules_.push_back(module);
	}

	bool ProjectManager::saveProject()
	{
		json root;

		for (auto* module : modules_)
		{
			if (module == nullptr || !module->save(projectDir_, root))
			{
				return false;
			}
		}

		std::ofstream file(projectDir_ + "/project.json");
		if (!file.is_open())
		{
			return false;
		}

		file << root.dump(4);
		return true;
	}

	bool ProjectManager::loadProject()
	{
		std::ifstream file(projectDir_ + "/project.json");
		if (!file.is_open())
		{
			return false;
		}

		json root;
		file >> root;

		for (auto* module : modules_)
		{
			if (module == nullptr || !module->load(projectDir_, root))
			{
				return false;
			}
		}

		return true;
	}
}