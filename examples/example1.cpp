#include "..\include\qdcsg.hpp"
#include "objio.hpp"

int main(int argc, char** argv)
{
	std::shared_ptr<qdcsg::mesh> sphere = qdcsg::readOBJ("sphere.obj");
	std::shared_ptr<qdcsg::mesh> cube = qdcsg::readOBJ("cube.obj");

	std::shared_ptr<qdcsg::mesh> result = qdcsg::Difference(*cube, *sphere);

	qdcsg::writeOBJ( "cubeDiffSphere.obj", result);

	return 0;
}