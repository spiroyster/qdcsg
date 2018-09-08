
#ifndef SoC_GEOMETRY_BSP_HPP
#define SoC_GEOMETRY_BSP_HPP

#include <vector>
#include <list>
#include <memory>
#include <algorithm>
#include <functional>

        namespace csg
        {

            class vertex
            {
            public:
                vertex(float x, float y, float z)
                    : x_(x), y_(y), z_(z)
                {
                }

                float x_, y_, z_;
            };

            class triangle
            {

            public:

                triangle(unsigned int a, unsigned int b, unsigned int c, unsigned int id)
                    : a_(a), b_(b), c_(c), id_(id)
                {
                }

                unsigned int a_, b_, c_, id_;
            };

            // difference...
            struct mesh
            {
                std::vector<vertex> vertices_;
                std::vector<triangle> triangles_;
            };





            namespace impl
            {


















                class bspTree
                {
                public:

                    class intersectionResult
                    {
                        float a_, b_, c_;
                    public:
                        intersectionResult(float a, float b, float c) : a_(a), b_(b), c_(c) {}

                        float A() const { return a_; }
                        float B() const { return b_; }
                        float C() const { return c_; }
                        int Direction(float v) const { return !v ? 0 : (v < 0 ? -1 : 1); }
                        bool IsOutside() const { if (IsOnPlane()) { return false; }  if (a_ >= 0 && b_ >= 0 && c_ >= 0) { return true; } return false; }
                        bool IsInside() const { if (IsOnPlane()) { return false; }  if (a_ <= 0 && b_ <= 0 && c_ <= 0) { return true; } return false; }
                        bool IsOnPlane() const { return !a_ && !b_ && !c_; }
                        bool AtLeastOneVertexIsOnPlane() const { return !a_ || !b_ || !c_; }
                        bool AtLeastOneVertexIsNotOnPlane() const { return a_ || b_ || c_; }
                    };

                    struct node
                    {
                        node(const vertex& a, const vertex& b, const vertex& c) {}

                        std::unique_ptr<node> inside_;
                        std::unique_ptr<node> outside_;

                        const vertex& GetPosition() const { return position_; }
                        const vertex& GetNormal() const { return normal_; }

                        float GetDirectionFromPlane(const vertex& v) const
                        {

                        }

                        vertex CalculateIntersection(const vertex& a, const vertex& b) const
                        {

                        }

                        intersectionResult calculateTriangleIntersection(const vertex& a, const vertex& b, const vertex& c) const
                        {

                        }

                    private:
                        vertex position_;
                        vertex normal_;
                    };

                    bspTree(const mesh& mesh);

                    // Intersect mesh...
                    void Intersect(mesh& mergeMesh, bool onPlaneIsInside) const
                    {

                    }

                    enum OIFlag
                    {
                        INSIDE,
                        OUTSIDE,
                        ONPLANE,
                        UNDEFINED
                    };

                    // traverse for vertex position...
                    OIFlag Traverse(const vertex& v) const
                    {

                    }

                    

                    std::shared_ptr<mesh> SplitTriangle(const intersectionResult& intersectionResult, const Vertex& A, const Vertex& B, const Vertex& C, unsigned int ID, const Vertex& planePosition, const Vertex& planeNormal) const;


                protected:
                    std::unique_ptr<Node> root_;
                };
            }

            

            std::list<std::shared_ptr<mesh>> meshDifference(const mesh& A, const mesh& B)
            {
                return std::list<std::shared_ptr<mesh>>();
            }

            std::list<std::shared_ptr<mesh>> meshIntersection(const mesh& A, const mesh& B)
            {
                return std::list<std::shared_ptr<mesh>>();
            }

            std::shared_ptr<mesh> meshUnion(const mesh& A, const mesh& B)
            {
                return std::shared_ptr<mesh>();
            }

            // intersection


            // uion
            


        }

#endif // SoC_GEOMETRY_BSP_HPP
