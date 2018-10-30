# About

qdcsg (Quick'n Dirty Constructive Solid Geometry) is a header only library for providing difference, union and intersection CSG routines, requiring only STL and C++11.

It is designed to be fast and robust for quick CSG operations on arbitary trinagulated 3D meshes. qdcsg (at time of writing) produces open manifolds only which is sufficient
for visualisation, it does not close these manifolds, nor does it remesh which are two things required for performing some computation functions on the resultant clean  mesh. For more
information, see below. The routines requird for this require retessellation, something which is current outside the scope of qdcsg, although there are plans to implement it in the future.

It is based on this paper... and uses BSP trees to speed things up.

# Usage

    #include "qdcsg.hpp"

 and you're off!

qdcsg namespace provides three functions 

    std::shared_ptr<mesh> difference(const mesh& A, const mesh& B);
    std::shared_ptr<mesh> intersection(const mesh& A, const mesh& B);
    std::shared_ptr<mesh> union(const mesh& A, const mesh& B);




# Input

The input data itself does not have to a closed volume or manifolds itself, however triangles should be well formed
tolerance...The vertices of the input mesh can be shared.

## Triangle ID

In order to retain information about which meshes have been sued to carve out. A triangle ID is retained and all triangles should 


# Output



# Known Issues

Robustness wrt on the plane scenarios.


# Manifolds


## Multiple manifolds



## Closing manifolds






# Future



Halfspace clipping routines - provide 
Point on plane scenarios
OpenMP
Stiener poitns for closing manifolds
Remeshing for cleaner meshes
Templating for custom triangle and vertex types
