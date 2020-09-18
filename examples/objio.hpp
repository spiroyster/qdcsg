#ifndef QDCSG_OBJIO
#define QDCSG_OBJIO

#include "..\include\qdcsg.hpp"

#include <fstream>
#include <sstream>

namespace qdcsg
{
	static std::string readFile(const std::string& filepath)
	{
		std::ifstream file(filepath);
		if (!file)
			throw std::exception("Unable to read file");

		std::string str;

		file.seekg(0, std::ios::end);
		str.reserve(static_cast<size_t>(file.tellg()));
		file.seekg(0, std::ios::beg);

		str.assign((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());

		return str;
	}
	static std::shared_ptr<qdcsg::mesh> readOBJ(const std::string& filename)
	{
		std::istringstream syntax(readFile(filename));

		std::list<qdcsg::vertex> vertices;
		std::list<std::list<unsigned int>> triangles;
		
		std::string line;
		while (std::getline(syntax, line))
		{
			// if this line has a comment.. ignore it.
			if (line.find('#') != std::string::npos)
				continue;

			// look for v
			std::size_t v = line.find('v');
			if (v != std::string::npos)
			{
				std::istringstream iss(line.substr(v+1));

				float x = 0, y = 0, z = 0;
				iss >> std::ws >> x >> std::ws >> y >> std::ws >> z;
				vertices.push_back(qdcsg::vertex(x, y, z));
				continue;
			}

			// look for f
			std::size_t f = line.find('f');
			if (f != std::string::npos)
			{
				std::istringstream iss(line.substr(f+1));


				






				

				continue;
			}
		}

		std::shared_ptr<qdcsg::mesh> result(new qdcsg::mesh());

		{
			std::vector<qdcsg::vertex> verticesAsVector(vertices.begin(), vertices.end());
			result->reserve(triangles.size());
			std::for_each(triangles.begin(), triangles.end(), 
				[&result, &verticesAsVector](const std::list<unsigned int>& faces) 
			{
				std::vector<unsigned int> faceIndexesAsVector(faces.begin(), faces.end());
				result->push_back(qdcsg::triangle(verticesAsVector[faceIndexesAsVector[0]],
					verticesAsVector[faceIndexesAsVector[1]], verticesAsVector[faceIndexesAsVector[2]]));
			});
		}
		
		return result;
	}


	static void writeOBJ(const std::string& filename, std::shared_ptr<qdcsg::mesh>& mesh)
	{
		std::list<qdcsg::vertex> vertices;
		std::list<std::vector<unsigned int>> triangles;
		
		for (unsigned int t = 0; t < mesh->size(); ++t)
		{
			unsigned int tOffset = vertices.size();

			vertices.push_back((*mesh)[t].a_);
			vertices.push_back((*mesh)[t].b_);
			vertices.push_back((*mesh)[t].c_);

			triangles.push_back(std::vector<unsigned int>({ tOffset, tOffset +1, tOffset +2 }));
		}

		std::ostringstream oss;

		oss << "# qdcsg obj file. https://github.com/spiroyster/qdcsg\n";

		oss << "\n# vertices...\n";

		std::for_each(vertices.begin(), vertices.end(), 
			[&oss](const qdcsg::vertex& v) 
		{
			oss << "v " << v.x_ << " " << v.y_ << " " << v.z_ << '\n';
		});

		oss << "\n# triangles...\n";

		std::for_each(triangles.begin(), triangles.end(), 
			[&oss](const std::vector<unsigned int>& t)
		{
			oss << "f " << t[0] << " " << t[1] << " " << t[2] << '\n';
		});

		std::ofstream file(filename);
		if (file)
			file << oss.str();
		else
			std::exception("Unable to save obj file.");
	}



}


#endif // QDCSG_OBJIO