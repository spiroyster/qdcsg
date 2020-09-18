/*
    MIT License

    Copyright (c) 2018 Cordell Barron

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

*/


#ifndef QDCSG_HPP
#define QDCSG_HPP

#include <algorithm>
#include <list>
#include <memory>
#include <vector>

namespace qdcsg
{

    struct vertex
    {
        vertex() : x_(0), y_(0), z_(0)
        {
        }

        vertex(float x, float y, float z)
            : x_(x), y_(y), z_(z)
        {
        }

        float x_, y_, z_;
    };

    struct triangle
    {
        triangle() : a_(0), b_(0), c_(0), id_(0) {}

        triangle(unsigned int a, unsigned int b, unsigned int c, unsigned int id)
            : a_(a), b_(b), c_(c), id_(id)
        {
        }

        unsigned int a_, b_, c_, id_;
    };


    struct mesh
    {
        std::vector<vertex> vertices_;
        std::vector<triangle> triangles_;
    };

    struct result
    {
        std::vector<vertex> vertices_;
        std::vector<triangle> outsideTriangles_;
        std::vector<triangle> insideTriangles_;
    };

    static float epsilon = 0.001f;

    namespace impl
    {
        float magnitude(const vertex& v)
        {
            return sqrt((v.x_ * v.x_) + (v.y_ * v.y_) + (v.z_ * v.z_));
        }

        vertex unitise(const vertex& v)
        {
            float oneOverMagnitude = 1.0f / magnitude(v);
            return vertex(v.x_ * oneOverMagnitude, v.y_ * oneOverMagnitude, v.z_ * oneOverMagnitude);
        }

        vertex subtract(const vertex& a, const vertex& b)
        {
            return vertex(a.x_ - b.x_, a.y_ - b.y_, a.z_ - b.z_);
        }

        vertex cross(const vertex& a, const vertex& b)
        {
            return vertex((a.y_ * b.z_) - (a.z_ * b.y_), (a.z_ * b.x_) - (a.x_ * b.z_), (a.x_ * b.y_) - (a.y_ * b.x_));
        }

        float dot(const vertex& a, const vertex& b)
        {
            return (a.x_ * b.x_) + (a.y_ * b.y_) + (a.z_ * b.z_);
        }

        bool equals(const vertex& a, const vertex& b, float e)
        {
            return (abs(a.x_ - b.x_) < e && abs(a.y_ - b.y_) < e && abs(a.z_ - b.z_) < e);
        }

        bool equals(const vertex& a, const vertex& b)
        {
            return (a.x_ == b.x_ && a.y_ == b.y_ && a.z_ == b.z_);
        }

        float area(const vertex& a, const vertex& b, const vertex& c)
        {
            vertex ab = subtract(b, a);
            float abMag = magnitude(ab);
            vertex ac = subtract(a, c);
            float acMag = magnitude(ac);

            // if either points equal each other or all 3 are colinear... maybe use e?
            if ( equals(a, b, epsilon) || equals(a, c, epsilon) || equals(b, c, epsilon))
                return 0;

            vertex acUnit = unitise(ac);
            float cosThita = dot(unitise(ab), acUnit);


            vertex d(a.x_ + (acUnit.x_ * cosThita), a.y_ + (acUnit.y_ * cosThita), a.z_ + (acUnit.z_ * cosThita));

            vertex bd = subtract(b, d);
            float bdMag = magnitude(bd);

            return 0.5f * (bdMag * acMag);
        }

        bool validateTriangle(const triangle& t, const vertex& a, const vertex& b, const vertex& c)
        {
            if (t.a_ == t.b_ || t.a_ == t.c_ || t.b_ == t.c_)
                return false;

            if ( area(a, b, c) < epsilon)
                return false;

            return true;
        }

        vertex rayPlaneIntersection(const vertex& rP, const vertex& rD, const vertex& pP, const vertex& pN)
        {
            float dotrDpN = dot(rD, pN);
            if (dotrDpN)
            {
                float s = dot(subtract(pP, rP), pN) / dotrDpN;
                return vertex(rP.x_ + (rD.x_ * s), rP.y_ + (rD.y_ * s), rP.z_ + (rD.z_ * s));
            }
            throw std::exception("RayAndPlaneNormalAreParrallel");
        }

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
                int direction(float v) const { return !v ? 0 : (v < 0 ? -1 : 1); }
                bool isOutside() const { if (isOnPlane()) { return false; }  if (a_ >= 0 && b_ >= 0 && c_ >= 0) { return true; } return false; }
                bool isInside() const { if (isOnPlane()) { return false; }  if (a_ <= 0 && b_ <= 0 && c_ <= 0) { return true; } return false; }
                bool isOnPlane() const { return !a_ && !b_ && !c_; }
                bool atLeastOneVertexIsOnPlane() const { return !a_ || !b_ || !c_; }
                bool atLeastOneVertexIsNotOnPlane() const { return a_ || b_ || c_; }
            };

            struct node
            {
                node(const vertex& a, const vertex& b, const vertex& c)
                    : a_(a), b_(b), c_(c)
                {
                    // calculate the position and normal of this triangle and use as plane pose...
                    normal_ = unitise(cross(subtract(b, a), subtract(c, a)));

                    // vertices should already be present in tree... these are added upon construction...
                    if (std::isnan(normal_.x_) || std::isnan(normal_.y_) || std::isnan(normal_.z_))
                        throw std::exception("BSPTreeNodeUnableToConstruct");
                }

                std::unique_ptr<node> inside_;
                std::unique_ptr<node> outside_;

                const vertex& GetA() const { return a_; }
                const vertex& GetB() const { return b_; }
                const vertex& GetC() const { return c_; }
                const vertex& GetNormal() const { return normal_; }

                float directionFromPlane(const vertex& v) const
                {
                    if (equals(v, a_))
                        return 0;

                    vertex d = unitise(subtract(v, a_));
                    float thita = dot(d, normal_);

                    // clamp this value? i.e if close to plane, clamp to plane...
                    if (abs(thita) < epsilon)
                        thita = 0;

                    return thita;
                }

                vertex calculateIntersection(const vertex& a, const vertex& b) const
                {
                    vertex rayD = unitise(subtract(b, a));
                    return rayPlaneIntersection(a, rayD, a_, normal_);
                }

                intersectionResult calculateTriangleIntersection(const vertex& a, const vertex& b, const vertex& c) const
                {
                    return intersectionResult(directionFromPlane(a), directionFromPlane(b), directionFromPlane(c));
                }

            private:
                vertex normal_;
                vertex a_;
                vertex b_;
                vertex c_;
            };

            bspTree(const mesh& mesh)
            {
                if (mesh.vertices_.empty() || mesh.triangles_.empty())
                    return;

                // we create a working list of the mesh triangles...
                std::list<triangle> workingTriangles;
                std::vector<vertex> workingVertices = mesh.vertices_;

                // populate with both the mesh outisde and inside triangles...
                workingTriangles.insert(workingTriangles.end(), mesh.triangles_.begin(), mesh.triangles_.end());

                // Get the first triangle and add this as a node...
                triangle& tri = workingTriangles.front();

                // Add the first node...
                root_.reset(new node(workingVertices[tri.a_], workingVertices[tri.b_], workingVertices[tri.c_]));

                // remove this triangle from the working list...
                workingTriangles.pop_front();

                // We then need to iterate through the rest of the working triangles... traversing current tree, and then adding nodes as needed...
                while (!workingTriangles.empty())
                {
                    // get the current working triangle...
                    std::list<triangle>::iterator currentTriangle = workingTriangles.begin();

                    // start the traversal from the root node...
                    node* currentNode = root_.get();

                    while (currentNode)
                    {
                        // we first need to validate the triangle (ensure it is valid)
                        const vertex& a = workingVertices[currentTriangle->a_];
                        const vertex& b = workingVertices[currentTriangle->b_];
                        const vertex& c = workingVertices[currentTriangle->c_];

                        // validate the triangle...
                        if (!validateTriangle(*currentTriangle, a, b, c))
                        {
                            // if the triangle is invalid... we ignore it so remove from our working list and start traversal with next triangle...
                            workingTriangles.erase(currentTriangle);
                            currentNode = 0;
                            continue;
                        }

                        // Calculate the current triangle intersecton relative to the current node...
                        intersectionResult intersections = currentNode->calculateTriangleIntersection(a, b, c);

                        // If this triangle is outside of node (in front of node plane)
                        if (intersections.isOutside())
                        {
                            // follow the outside branch/path...
                            if (currentNode->outside_)
                                currentNode = currentNode->outside_.get();
                            else
                            {
                                // if there is no outside node, we need to create a new one with this triangles plane.
                                currentNode->outside_.reset(new node(a, b, c));
                                currentNode = 0;
                            }
                        }
                        // Otherwise if the current triangle is inside or onplane of current node... (for onplane, assume triangle is 'inside')
                        else if (intersections.isInside() || intersections.isOnPlane())
                        {
                            // follow the inside branch/path...
                            if (currentNode->inside_)
                                currentNode = currentNode->inside_.get();
                            else
                            {
                                // if there is no inside node, we need to create a new one with this triangles plane.
                                currentNode->inside_.reset(new node(a, b, c));
                                currentNode = 0;
                            }
                        }
                        // Otherwise this triangle straddles the current node plane... so we need to split accordinglly...
                        else
                        {
                            std::shared_ptr<result> splitResult = split(intersections, a, b, c, currentTriangle->id_, currentNode->GetA(), currentNode->GetNormal());

                            // Our split result will generate one or more triangles... if there is only one triangle, this means that one or more of the split triangles was invalid
                            if (splitResult->insideTriangles_.size() == 1 && splitResult->outsideTriangles_.empty())
                            {
                                // if this triangle is inside, follow the inside...
                                if (currentNode->inside_)
                                    currentNode = currentNode->inside_.get();
                                else
                                {
                                    // if there is no inside node, we need to create a new one with this triangles plane.
                                    currentNode->inside_.reset(new node(a, b, c));
                                    currentNode = 0;
                                }
                            }
                            else if (splitResult->outsideTriangles_.size() == 1 && splitResult->insideTriangles_.empty())
                            {
                                // if this triangle is outside, follow the inside...
                                if (currentNode->outside_)
                                    currentNode = currentNode->outside_.get();
                                else
                                {
                                    // if there is no inside node, we need to create a new one with this triangles plane.
                                    currentNode->outside_.reset(new node(a, b, c));
                                    currentNode = 0;
                                }
                            }
                            else
                            {
                                // Otherwise we need to add these split triangles to our workingtriangle list and start traversal again...

                                // add the vertices...
                                unsigned int vertexOffset = static_cast<unsigned int>(workingVertices.size());
                                workingVertices.insert(workingVertices.end(), splitResult->vertices_.begin(), splitResult->vertices_.end());

                                // add thes triangles to our working list, ammending the vertex offsets to cater for new vertices...
                                for (std::vector<qdcsg::triangle>::iterator insideItr = splitResult->insideTriangles_.begin(); insideItr != splitResult->insideTriangles_.end(); ++insideItr)
                                {
                                    insideItr->a_ += vertexOffset;
                                    insideItr->b_ += vertexOffset;
                                    insideItr->c_ += vertexOffset;
                                    workingTriangles.push_back(*insideItr);
                                }
                                for (std::vector<qdcsg::triangle>::iterator outsideItr = splitResult->outsideTriangles_.begin(); outsideItr != splitResult->outsideTriangles_.end(); ++outsideItr)
                                {
                                    outsideItr->a_ += vertexOffset;
                                    outsideItr->b_ += vertexOffset;
                                    outsideItr->c_ += vertexOffset;
                                    workingTriangles.push_back(*outsideItr);
                                }

                                currentNode = 0;// currentNode.reset();
                            }
                        }

                        // if there is no current node (it has been reset) this means we have either added new triangles to our working list (from a split)
                        // or we have added a node currently. So we need to start again with the next triangle
                        if (!currentNode)
                            workingTriangles.erase(currentTriangle);
                    }
                }
            }

            // Intersect mesh...
            std::shared_ptr<result> intersect(const mesh& intersectionMesh, bool onPlaneIsInside) const
            {

                std::shared_ptr<result> output(new result());

                // We need to create a working list of the current triangles from the mesh...
                std::list<triangle> workingTriangles(intersectionMesh.triangles_.begin(), intersectionMesh.triangles_.end());

                // copy the current vertices to our output...
                output->vertices_ = intersectionMesh.vertices_;

                // For each triangle from our working list... we push it through this tree and and either split triangles adding them to inside or outside...
                // or keep adding to inside or outside...
                while (!workingTriangles.empty())
                {
                    // get the triangle itr
                    std::list<triangle>::iterator currentTriangle = workingTriangles.begin();

                    const vertex& a = output->vertices_[currentTriangle->a_];
                    const vertex& b = output->vertices_[currentTriangle->b_];
                    const vertex& c = output->vertices_[currentTriangle->c_];

                    // start to traverse the tree to see if this triangle is inside or outside the tree..
                    node* currentNode = root_.get();

                    // start from root node, and traverse, calculating if triangle is infront or behind node plane...
                    while (currentNode)
                    {
                        // validate the triangle...
                        if (!validateTriangle(*currentTriangle, a, b, c))
                        {
                            // if the triangle is invalid... we ignore it so remove from our working list and start traversal with next working triangle...
                            workingTriangles.erase(currentTriangle);
                            currentNode = 0;// currentNode.reset();
                            continue;
                        }

                        // Calculate the current triangle intersecton relative to the current node...
                        intersectionResult intersections = currentNode->calculateTriangleIntersection(a, b, c);

                        // If this triangle is outside of node (in front of node plane)
                        if (intersections.isOutside())
                        {
                            // follow the outside branch/path...
                            if (currentNode->outside_)
                                currentNode = currentNode->outside_.get();
                            // if there is no outside branch... this triangle is on outside so add to our list...and start traversal again
                            else
                            {
                                output->outsideTriangles_.push_back(*currentTriangle);
                                currentNode = 0;// currentNode.reset();
                            }
                        }
                        // Otherwise if the current triangle is inside or onplane of current node... (for onplane, assume triangle is 'inside')
                        else if (intersections.isInside())
                        {
                            // follow the inside branch/path...
                            if (currentNode->inside_)
                                currentNode = currentNode->inside_.get();
                            // if there is no outside branch... this triangle is on outside so add to our list...
                            else
                            {
                                output->insideTriangles_.push_back(*currentTriangle);
                                currentNode = 0;// currentNode.reset();
                            }

                        }
                        else if (intersections.isOnPlane())
                        {
                            // we use the predicate to decide if to consider this triangle inside or outside when on the plane...
                            if (onPlaneIsInside)
                            {
                                // follow the inside branch/path...
                                if (currentNode->inside_)
                                    currentNode = currentNode->inside_.get();
                                // if there is no inside branch... this triangle is on outside so add to our list...
                                else
                                {
                                    output->insideTriangles_.push_back(*currentTriangle);
                                    currentNode = 0;// currentNode.reset();
                                }
                            }
                            else
                            {
                                // follow the outside branch/path...
                                if (currentNode->outside_)
                                    currentNode = currentNode->outside_.get();
                                // if there is no outside branch... this triangle is on outside so add to our list...
                                else
                                {
                                    output->outsideTriangles_.push_back(*currentTriangle);
                                    currentNode = 0;// currentNode.reset();
                                }
                            }
                        }


                        // Otherwise this triangle straddles the current node plane... so we need to split accordinglly...
                        else
                        {
                            // we split the triangles and generate the correct 
                            std::shared_ptr<result> splitResult = split(intersections, a, b, c, currentTriangle->id_, currentNode->GetA(), currentNode->GetNormal());

                            // Our split result will generate one or more triangles... if there is only one triangle, this means that one or more of the split triangles was invalid
                            if (splitResult->insideTriangles_.size() == 1 && splitResult->outsideTriangles_.empty())
                            {
                                // follow the inside branch/path...
                                if (currentNode->inside_)
                                    currentNode = currentNode->inside_.get();
                                // if there is no outside branch... this triangle is on outside so add to our list...
                                else
                                {
                                    output->insideTriangles_.push_back(*currentTriangle);
                                    currentNode = 0;
                                }
                            }
                            else if (splitResult->outsideTriangles_.size() == 1 && splitResult->insideTriangles_.empty())
                            {
                                // follow the outside branch/path...
                                if (currentNode->outside_)
                                    currentNode = currentNode->outside_.get();
                                // if there is no outside branch... this triangle is on outside so add to our list...
                                else
                                {
                                    output->outsideTriangles_.push_back(*currentTriangle);
                                    currentNode = 0;
                                }
                            }
                            else
                            {
                                // if these are new split triangles to add... we need to ammend them to our list for traversal...

                                // add the vertices...
                                unsigned int vertexOffset = static_cast<unsigned int>(output->vertices_.size());
                                output->vertices_.insert(output->vertices_.end(), splitResult->vertices_.begin(), splitResult->vertices_.end());

                                // add thes triangles to our working list, ammending the vertex offsets to cater for new vertices...
                                for (std::vector<triangle>::iterator insideItr = splitResult->insideTriangles_.begin(); insideItr != splitResult->insideTriangles_.end(); ++insideItr)
                                {
                                    insideItr->a_ += vertexOffset;
                                    insideItr->b_ += vertexOffset;
                                    insideItr->c_ += vertexOffset;
                                    workingTriangles.push_back(*insideItr);
                                }
                                for (std::vector<triangle>::iterator outsideItr = splitResult->outsideTriangles_.begin(); outsideItr != splitResult->outsideTriangles_.end(); ++outsideItr)
                                {
                                    outsideItr->a_ += vertexOffset;
                                    outsideItr->b_ += vertexOffset;
                                    outsideItr->c_ += vertexOffset;
                                    workingTriangles.push_back(*outsideItr);
                                }

                                currentNode = 0;
                            }
                        }

                        // if there is no current node (it has been reset) this means we have either added new triangles to our working list (from a split)
                        // or we have added a node currently. So we need to start again with the next triangle
                        if (!currentNode)
                            workingTriangles.erase(currentTriangle);

                    }
                }

                return output;
            }

            
            std::shared_ptr<result> split(const intersectionResult& intersections, const vertex& A, const vertex& B, const vertex& C, unsigned int ID, const vertex& planePosition, const vertex& planeNormal) const
            {
                std::shared_ptr<result> output(new result());

                // put the original three points from the triangle into the split result...
                output->vertices_.reserve(5);        // at most there can be 5 vertices...
                output->vertices_.push_back(A);
                output->vertices_.push_back(B);
                output->vertices_.push_back(C);

                // the indexes...
                unsigned int triangleA = 0, triangleB = 1, triangleC = 2, intersection1 = 3, intersection2 = 4;

                if (intersections.atLeastOneVertexIsOnPlane())
                {
                    // 1 intersection ... 2 triangles...
                    triangle insideTriangle, outsideTriangle;

                    if (!intersections.A())
                    {
                        // if a is on plane... b must be one side, and c the other... so use these to calculate the intersection point...
                        output->vertices_.push_back(rayPlaneIntersection(B, unitise(subtract(C, B)), planePosition, planeNormal));

                        // add the two triangles... if b is behind, a must be infront...
                        if (intersections.B() < 0)
                        {
                            insideTriangle = triangle(triangleA, triangleB, intersection1, ID);//output->CreateTriangle(Mesh::IOFlag::INSIDE, triangleA, triangleB, intersection1, ID);
                            outsideTriangle = triangle(intersection1, triangleC, triangleA, ID);
                        }
                        else
                        {
                            outsideTriangle = triangle(triangleA, triangleB, intersection1, ID);
                            insideTriangle = triangle(intersection1, triangleC, triangleA, ID);
                        }
                    }
                    else if (!intersections.B())
                    {
                        // if b is on plane... c must be one side, and a the other... so use these to calculate the intersection point...
                        output->vertices_.push_back(rayPlaneIntersection(C, unitise(subtract(A, C)), planePosition, planeNormal));

                        // add the two triangles... if c is behind, a must be infront...
                        if (intersections.C() < 0)
                        {
                            insideTriangle = triangle(triangleB, triangleC, intersection1, ID);
                            outsideTriangle = triangle(intersection1, triangleA, triangleB, ID);
                        }
                        else
                        {
                            outsideTriangle = triangle(triangleB, triangleC, intersection1, ID);
                            insideTriangle = triangle(intersection1, triangleA, triangleB, ID);
                        }
                    }
                    else if (!intersections.C())
                    {
                        // if c is on plane... b must be one side, and a the other... so use these to calculate the intersection point...
                        output->vertices_.push_back(rayPlaneIntersection(A, unitise(subtract(B, A)), planePosition, planeNormal));

                        // add the two triangles... if b is behind, a must be infront...
                        if (intersections.A() < 0)
                        {
                            insideTriangle = triangle(triangleC, triangleA, intersection1, ID);
                            outsideTriangle = triangle(intersection1, triangleB, triangleC, ID);
                        }
                        else
                        {
                            outsideTriangle = triangle(triangleC, triangleA, intersection1, ID);
                            insideTriangle = triangle(intersection1, triangleB, triangleC, ID);
                        }
                    }
                    else
                        throw std::exception("CSGSplitInvalid");

                    if (validateTriangle(insideTriangle, output->vertices_[insideTriangle.a_], output->vertices_[insideTriangle.b_], output->vertices_[insideTriangle.c_]))
                        output->insideTriangles_.push_back(insideTriangle);

                    if (validateTriangle(outsideTriangle, output->vertices_[outsideTriangle.a_], output->vertices_[outsideTriangle.b_], output->vertices_[outsideTriangle.c_]))
                        output->outsideTriangles_.push_back(outsideTriangle);
                }
                else
                {
                    // 2 intersections ... 3 triangles...

                    // first find out which is behind and which is infront...
                    int aDirection = intersections.direction(intersections.A());
                    int bDirection = intersections.direction(intersections.B());
                    int cDirection = intersections.direction(intersections.C());

                    int sum = aDirection + bDirection + cDirection;

                    // 1 intersection ... 2 triangles...

                    // if the split triangle is outside heavy...
                    if (sum > 0)
                    {
                        triangle insideTriangle, outsideTriangle1, outsideTriangle2;

                        // if a is behind...
                        if (aDirection < 0)
                        {
                            // b and c must be infront...
                            output->vertices_.push_back(rayPlaneIntersection(A, unitise(subtract(B, A)), planePosition, planeNormal));
                            output->vertices_.push_back(rayPlaneIntersection(A, unitise(subtract(C, A)), planePosition, planeNormal));

                            // calculate the inside triangle...
                            insideTriangle = triangle(triangleA, intersection1, intersection2, ID);// , result->vertices_[triangleA], result->vertices_[intersection1], result->vertices_[intersection2]);

                                                                                                                                        // calculate the outside triangles...
                            outsideTriangle1 = triangle(intersection1, triangleB, triangleC, ID);// , result->vertices_[intersection1], result->vertices_[triangleB], result->vertices_[triangleC]);
                            outsideTriangle2 = triangle(triangleC, intersection2, intersection1, ID);// , result->vertices_[triangleC], result->vertices_[intersection2], result->vertices_[intersection1]);
                        }
                        else if (bDirection < 0)
                        {
                            // c and a must be infront...
                            output->vertices_.push_back(rayPlaneIntersection(B, unitise(subtract(C, B)), planePosition, planeNormal));
                            output->vertices_.push_back(rayPlaneIntersection(B, unitise(subtract(A, B)), planePosition, planeNormal));

                            // calculate the inside triangle...
                            insideTriangle = triangle(triangleB, intersection1, intersection2, ID);// , result->vertices_[triangleB], result->vertices_[intersection1], result->vertices_[intersection2]);

                                                                                                                                        // calculate the outside triangles...
                            outsideTriangle1 = triangle(intersection1, triangleC, triangleA, ID);//, result->vertices_[intersection1], result->vertices_[triangleC], result->vertices_[triangleA]);
                            outsideTriangle2 = triangle(triangleA, intersection2, intersection1, ID);// , result->vertices_[triangleA], result->vertices_[intersection2], result->vertices_[intersection1]);
                        }
                        else if (cDirection < 0)
                        {
                            // a and b must be infront...
                            output->vertices_.push_back(rayPlaneIntersection(C, unitise(subtract(C, A)), planePosition, planeNormal));
                            output->vertices_.push_back(rayPlaneIntersection(C, unitise(subtract(C, B)), planePosition, planeNormal));

                            // calculate the inside triangle...
                            insideTriangle = triangle(triangleC, intersection1, intersection2, ID);// , result->vertices_[triangleC], result->vertices_[intersection1], result->vertices_[intersection2]);

                                                                                                                                        // calculate the outside triangles...
                            outsideTriangle1 = triangle(intersection1, triangleA, triangleB, ID);// , result->vertices_[intersection1], result->vertices_[triangleA], result->vertices_[triangleB]);
                            outsideTriangle2 = triangle(triangleB, intersection2, intersection1, ID);// , result->vertices_[triangleB], result->vertices_[intersection2], result->vertices_[intersection1]);
                        }

                        // if the inside triangle is invalid... add the original triangle as the outside triangle.
                        if (!validateTriangle(insideTriangle, output->vertices_[insideTriangle.a_], output->vertices_[insideTriangle.b_], output->vertices_[insideTriangle.c_]))
                            output->outsideTriangles_.push_back(triangle(triangleA, triangleB, triangleC, ID));// , result->vertices_[triangleA], result->vertices_[triangleB], result->vertices_[triangleC]));
                        else
                        {
                            output->insideTriangles_.push_back(insideTriangle);

                            // outside triangle1 and outside triangle2 must be valid
                            output->outsideTriangles_.push_back(outsideTriangle1);
                            output->outsideTriangles_.push_back(outsideTriangle2);
                        }
                    }

                    // Otherwise it is inside heavy...
                    else
                    {
                        triangle outsideTriangle, insideTriangle1, insideTriangle2;

                        // if a is infront...
                        if (aDirection > 0)
                        {
                            // b and c must be behind...
                            output->vertices_.push_back(rayPlaneIntersection(A, unitise(subtract(B, A)), planePosition, planeNormal));
                            output->vertices_.push_back(rayPlaneIntersection(A, unitise(subtract(C, A)), planePosition, planeNormal));

                            // calculate the outside triangle...
                            outsideTriangle = triangle(triangleA, intersection1, intersection2, ID);

                            // calculate the inside triangles...
                            insideTriangle1 = triangle(intersection1, triangleB, triangleC, ID);
                            insideTriangle2 = triangle(triangleC, intersection2, intersection1, ID);
                        }
                        else if (bDirection > 0)
                        {
                            // c and a must be behind...
                            output->vertices_.push_back(rayPlaneIntersection(B, unitise(subtract(C, B)), planePosition, planeNormal));
                            output->vertices_.push_back(rayPlaneIntersection(B, unitise(subtract(A, B)), planePosition, planeNormal));

                            // calculate the inside triangle...
                            outsideTriangle = triangle(triangleB, intersection1, intersection2, ID);

                            // calculate the outside triangles...
                            insideTriangle1 = triangle(intersection1, triangleC, triangleA, ID);
                            insideTriangle2 = triangle(triangleA, intersection2, intersection1, ID);
                        }
                        else if (cDirection > 0)
                        {
                            // a and b must be infront...
                            output->vertices_.push_back(rayPlaneIntersection(C, unitise(subtract(C, A)), planePosition, planeNormal));
                            output->vertices_.push_back(rayPlaneIntersection(C, unitise(subtract(C, B)), planePosition, planeNormal));

                            // calculate the inside triangle...
                            outsideTriangle = triangle(triangleC, intersection1, intersection2, ID);

                            // calculate the outside triangles...
                            insideTriangle1 = triangle(intersection1, triangleA, triangleB, ID);
                            insideTriangle2 = triangle(triangleB, intersection2, intersection1, ID);
                        }

                        // if the inside triangle is invalid... add the original triangle as the outside triangle.
                        if (!validateTriangle(outsideTriangle, output->vertices_[outsideTriangle.a_], output->vertices_[outsideTriangle.b_], output->vertices_[outsideTriangle.c_]))
                            output->insideTriangles_.push_back(triangle(triangleA, triangleB, triangleC, ID));
                        else
                        {
                            output->outsideTriangles_.push_back(outsideTriangle);

                            // outside triangle1 and outside triangle2 must be valid
                            output->insideTriangles_.push_back(insideTriangle1);
                            output->insideTriangles_.push_back(insideTriangle2);
                        }
                    }
                }
                return output;
            }

        private:
            std::unique_ptr<node> root_;
        };


        // Merge meshes...
        std::shared_ptr<mesh> merge(const result& A, const std::vector<triangle>& aTriangles, const result& B, const std::vector<triangle>& bTriangles, bool flipB)
        {
            std::shared_ptr<mesh> result(new mesh());

            unsigned int offset = static_cast<unsigned int>(A.vertices_.size());

            result->vertices_.insert(result->vertices_.end(), A.vertices_.begin(), A.vertices_.end());
            result->vertices_.insert(result->vertices_.end(), B.vertices_.begin(), B.vertices_.end());

            result->triangles_.insert(result->triangles_.end(), aTriangles.begin(), aTriangles.end());
            result->triangles_.insert(result->triangles_.end(), bTriangles.begin(), bTriangles.end());

            // ammend the triangle indexes...
            std::for_each(result->triangles_.begin() + aTriangles.size(), result->triangles_.end(), [&offset](triangle& t) { t.a_ += offset; t.b_ += offset; t.c_ += offset; });

            // if required, flip b's triangles...
            if (flipB)
                std::for_each(result->triangles_.begin() + aTriangles.size(), result->triangles_.end(), [](triangle& t) { unsigned int c = t.c_; t.c_ = t.b_; t.b_ = c; });

            // prune the mesh so that it only has vertices that are referenced by triangles...
            std::vector<int> newIndexes(result->vertices_.size(), -1);
            std::vector<vertex> newVertices;
            const std::vector<vertex>& currentVertices = result->vertices_;
            newVertices.reserve(result->vertices_.size());

            std::for_each(result->triangles_.begin(), result->triangles_.end(),
                [&currentVertices, &newIndexes, &newVertices](triangle& t)
            {
                if (newIndexes[t.a_] == -1)
                {
                    newVertices.push_back(currentVertices[t.a_]);
                    newIndexes[t.a_] = static_cast<int>(newVertices.size() - 1);
                }
                t.a_ = newIndexes[t.a_];

                if (newIndexes[t.b_] == -1)
                {
                    newVertices.push_back(currentVertices[t.b_]);
                    newIndexes[t.b_] = static_cast<int>(newVertices.size() - 1);
                }
                t.b_ = newIndexes[t.b_];

                if (newIndexes[t.c_] == -1)
                {
                    newVertices.push_back(currentVertices[t.c_]);
                    newIndexes[t.c_] = static_cast<int>(newVertices.size() - 1);
                }
                t.c_ = newIndexes[t.c_];
            });

            result->vertices_ = newVertices;

            return result;
        }
        
    }



    


    std::shared_ptr<mesh> Difference(const mesh& A, const mesh& B)
    {
        impl::bspTree treeA(A);
        impl::bspTree treeB(B);

        result intersectedB = *treeA.intersect(B, true);
        result intersectedA = *treeB.intersect(A, true);

        return impl::merge(intersectedA, intersectedA.outsideTriangles_, intersectedB, intersectedB.insideTriangles_, true);
    }

    std::shared_ptr<mesh> Intersection(const mesh& A, const mesh& B)
    {
        impl::bspTree treeA(A);
        impl::bspTree treeB(B);

        result intersectedB = *treeA.intersect(B, true);
        result intersectedA = *treeB.intersect(A, true);

        return impl::merge(intersectedA, intersectedA.insideTriangles_, intersectedB, intersectedB.insideTriangles_, false);
    }

    std::shared_ptr<mesh> Union(const mesh& A, const mesh& B)
    {
        impl::bspTree treeA(A);
        impl::bspTree treeB(B);

        result intersectedB = *treeA.intersect(B, true);
        result intersectedA = *treeB.intersect(A, true);

        return impl::merge(intersectedA, intersectedA.outsideTriangles_, intersectedB, intersectedB.outsideTriangles_, false);
    }

    


}

#endif // QDCSG_HPP
