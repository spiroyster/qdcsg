Quick n Dirty Constructive Solid Geometry. STL + header only + C++11

# About

qdcsg is a header only library for providing difference, union and intersection CSG routines, requiring only STL and C++11.
It is designed to be fast, easy to use, no dependencies (other than STL) and work on triangulated data. The output is visuallay correct although may require addition work (such as calculating manifolds and lower tessellation) if using the output for computational geometry or topologically processing.

# Usage

    #include "qdcsg.hpp"

 and you're off!

qdcsg namespace provides 3 functions, one for each routine.

    std::shared_ptr<mesh> Difference(const mesh& A, const mesh& B);
    std::shared_ptr<mesh> Intersection(const mesh& A, const mesh& B);
    std::shared_ptr<mesh> Union(const mesh& A, const mesh& B);

You need to populate the qdcsg mesh which is a striaght forward triangle indexed (a1, b1, c1, a2, b2, c2 ... an, bn, cn) and vertex container.

qdcsg works on the triangles, as a result can be quite liberal with what it accepts so can take either orientation, non-shared, non-closed, meshes which do not even have to be manfolds themseleves, although in the last case ymmv, since there would be a limited concept of inside/outside.

There is a validation routine which is performed on both the user input data, and any generated triangles (splits) which checks to ensure triangles are well formed and will ignore triangles with an area less than epsilon. See validateTriangle(...). The float operations work to a default tolerance of 0.001f. This can be user defined by asinging the static float epsilon.

    qdcsg::epsilon = 0.1f;


Have fun!


# Output

tessellation (ugly, but fast)
non mnaifold



## Multiple manifolds

Consider this situation. 

[MultiManifold result]

The resultant mesh is a single mesh, although there is no distinction between the 4 manifolds. This processing is currently outside the scope of qdcsg although future revisions may contain helpers in order to facilitate this. In order to deduce the manifolds, points need to be shared and then joins calculated. All triangles part of the same manifold will be joined in some manner (2 or joins depends on robustness of algorithms used). This process should also close resultant manifolds iff both the input meshes are closed manifolds themseleves.

## Triangle ID

An extra unsigned int for each triangle allows the owner (mesh) of triangles to be tracked when perofrming larger CSG operations on multiple objects. The triangle ID of an intersecting 

[Carousel]

# Known Issues

Robustness wrt on the plane scenarios. If two triangles are coplanar and on the same plane, this provides an interesting situation which is current ignored. This is satisfactory in most situations, but can become problmatic with certain geometry. For the BSP tree used, this is fine, however since no extra processing is done on these types of triangles, the result triangle will be the input triangle which depends on the onPlaneIsInside flag (see Difference/Union/Intersection functions). This is on my list to fix!


# Future

Point on the plane scenario
OpenMP support
Halfspace clipping routines
Stiener point insertion for closing manifolds
Remeshing for cleaner meshes
Templating for custom triangle and vertex types
