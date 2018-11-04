
#include "qdcsg.hpp"
#include "dump_vrml.h"


qdcsg::vertex add(const qdcsg::vertex& a, const qdcsg::vertex& b)
{
    return qdcsg::vertex(a.x_ + b.x_, a.y_ + b.y_, a.z_ + b.z_);
}

qdcsg::vertex scale(const qdcsg::vertex& v, float s)
{
    return qdcsg::vertex(v.z_ * s, v.y_ * s, v.z_ * s);
}

qdcsg::vertex rodrigues(const qdcsg::vertex& v, const qdcsg::vertex& axis, float angle)
{
    float c = static_cast<float>(cos(angle));
    qdcsg::vertex vRot = scale(v, c);
    vRot = add(scale(qdcsg::impl::cross(axis, v), static_cast<float>(sin(angle))), vRot);
    vRot = add(scale(axis, qdcsg::impl::dot(axis, v) * (1 - c)), vRot);
    return vRot;
}


qdcsg::mesh cuboid(const qdcsg::vertex& position, float w, float h, float l, unsigned int ID)
{
    qdcsg::mesh result;

    qdcsg::vertex min(-0.5f * w, -0.5f * h, -0.5f * l);
    qdcsg::vertex max(0.5f * w, 0.5f * h, 0.5f * l);

    min = add(min, position);
    max = add(max, position);

    result.vertices_.reserve(8);
    result.vertices_.push_back(qdcsg::vertex(min.x_, min.y_, min.z_)); 
    result.vertices_.push_back(qdcsg::vertex(min.x_, max.y_, min.z_));
    result.vertices_.push_back(qdcsg::vertex(max.x_, max.y_, min.z_));
    result.vertices_.push_back(qdcsg::vertex(max.x_, min.y_, min.z_));
    result.vertices_.push_back(qdcsg::vertex(min.x_, min.y_, max.z_));
    result.vertices_.push_back(qdcsg::vertex(min.x_, max.y_, max.z_));
    result.vertices_.push_back(qdcsg::vertex(max.x_, max.y_, max.z_));
    result.vertices_.push_back(qdcsg::vertex(max.x_, min.y_, max.z_));

    // ccw orientation
    result.triangles_.reserve(12);
    
    // bottom...
    result.triangles_.push_back(qdcsg::triangle(3, 0, 1, ID));
    result.triangles_.push_back(qdcsg::triangle(1, 2, 3, ID));
    
    // right...
    result.triangles_.push_back(qdcsg::triangle(7, 3, 2, ID));
    result.triangles_.push_back(qdcsg::triangle(2, 6, 7, ID));
    
    // front...
    result.triangles_.push_back(qdcsg::triangle(4, 0, 3, ID));
    result.triangles_.push_back(qdcsg::triangle(3, 7, 4, ID));
    
    // left...
    result.triangles_.push_back(qdcsg::triangle(5, 1, 0, ID));
    result.triangles_.push_back(qdcsg::triangle(0, 4, 5, ID));
    
    // back...
    result.triangles_.push_back(qdcsg::triangle(6, 2, 1, ID));
    result.triangles_.push_back(qdcsg::triangle(1, 5, 6, ID));
    
    // top...
    result.triangles_.push_back(qdcsg::triangle(5, 4, 7, ID));
    result.triangles_.push_back(qdcsg::triangle(7, 6, 5, ID));

    return result;
}




void DumpSurface(qdcsg::mesh& m, const std::string& filepath)
{
    VRMLDump::outputFile vrml(filepath.c_str());

    vrml.current_GeomMode(VRMLDump::faces);
    vrml.StartGeom();

    for (unsigned int v = 0; v < m.vertices_.size(); ++v)
        vrml.dump_vec3(m.vertices_[v].x_, m.vertices_[v].y_, m.vertices_[v].z_);

    for (unsigned int t = 0; t < m.triangles_.size(); ++t)
    {
        vrml.dump_index(m.triangles_[t].a_);
        vrml.dump_index(m.triangles_[t].b_);
        vrml.dump_index(m.triangles_[t].c_);
        vrml.dump_EndPoly();
    }

    vrml.StopGeom();
}


void DumpSurfaceWithColour(qdcsg::mesh& m, const std::string& filepath)
{
    VRMLDump::outputFile vrml(filepath.c_str());

    for (unsigned int t = 0; t < m.triangles_.size(); ++t)
    {
        const qdcsg::vertex& a = m.vertices_[m.triangles_[t].a_];
        const qdcsg::vertex& b = m.vertices_[m.triangles_[t].b_];
        const qdcsg::vertex& c = m.vertices_[m.triangles_[t].c_];

        switch (m.triangles_[t].id_)
        {
        case 1:
            vrml.current_Colour(1.0f, 0, 0);
            break;
        case 2:
            vrml.current_Colour(0, 0, 1.0f);
            break;
        }
        vrml.dump_triangle(a.x_, a.y_, a.z_, b.x_, b.y_, b.z_, c.x_, c.y_, c.z_);
    }
        
}



int main(int argc, char** agrv)
{
    // csg examples
    {
        qdcsg::mesh A = cuboid(qdcsg::vertex(), 1.0f, 1.0f, 1.0f, 1);
        qdcsg::mesh B = cuboid(qdcsg::vertex(0.5f, 0.5f, 0.5f), 1.0f, 1.0f, 1.0f, 2);

        DumpSurface(A, std::string("c:\\dump\\1a.wrl"));
        DumpSurface(B, std::string("c:\\dump\\1b.wrl"));

        DumpSurfaceWithColour(*qdcsg::Difference(A, B), std::string("c:\\dump\\1diff.wrl"));
        DumpSurfaceWithColour(*qdcsg::Intersection(A, B), std::string("c:\\dump\\1intersection.wrl"));
        DumpSurfaceWithColour(*qdcsg::Union(A, B), std::string("c:\\dump\\1union.wrl"));
    }
    
    // multi manifold
    {
        qdcsg::mesh A = cuboid(qdcsg::vertex(), 1.0f, 1.0f, 1.0f, 1);
        qdcsg::mesh B = cuboid(qdcsg::vertex(), 1.0f, 1.1f, 1.0f, 2);

        // rotate B via yAxis...
        std::for_each(B.vertices_.begin(), B.vertices_.end(), [](qdcsg::vertex& v) { v = rodrigues(v, qdcsg::vertex(1.0f, 0, 0), 3.141592654f/4.0f); });

        DumpSurfaceWithColour(A, std::string("c:\\dump\\2a.wrl"));
        DumpSurfaceWithColour(B, std::string("c:\\dump\\2b.wrl"));


        DumpSurfaceWithColour(*qdcsg::Difference(A, B), std::string("c:\\dump\\2diff.wrl"));
        DumpSurfaceWithColour(*qdcsg::Intersection(A, B), std::string("c:\\dump\\2intersection.wrl"));
        DumpSurfaceWithColour(*qdcsg::Union(A, B), std::string("c:\\dump\\2union.wrl"));
    }



    // carousel




    return 0;
}
