#include "BSP.hpp"

#include <Framework\SoC\Maths\VectorOperationsInterface.hpp>
#include <Framework\SoC\Geometry\Exception.hpp>
#include <Framework\SoC\Geometry\MeshOperationsInterface.hpp>
#include <Framework\SoC\Maths\Trigonometry.hpp>

#include <Framework\SoC\Geometry\dump_vrml.h>

namespace SoC
{
    namespace Geometry
    {
        namespace BSP
        {

            Tree::Node::Node(const Vertex& a, const Vertex& b, const Vertex& c)
            {
                const SoC::Maths::VectorOperationsInterface& vectorOps = SoC::Maths::GetVectorOperations();

                // calculate the position and normal of this triangle and use as plane pose...
                position_ = a;
                normal_ = vectorOps.Unitise(vectorOps.Cross(vectorOps.Subtract(b, position_), vectorOps.Subtract(c, position_)));

                // vertices should already be present in tree... these are added upon construction...
                if (std::isnan(normal_.x_) || std::isnan(normal_.y_) || std::isnan(normal_.z_))
                    throw SoC::Geometry::Exception(SoC::Geometry::ExceptionID::BSPTreeNodeUnableToConstruct);
            }

            float Tree::Node::GetDirectionFromPlane(const Vertex& v) const
            {
                const SoC::Maths::VectorOperationsInterface& vectorOps = SoC::Maths::GetVectorOperations();

                if (vectorOps.ApproximatelyEquals(v, position_))
                    return 0;

                SoC::Maths::Vector3 d = vectorOps.Unitise(vectorOps.Subtract(v, position_));
                float thita = vectorOps.Dot(d, normal_);

                // clamp this value? i.e if close to plane, clamp to plane...
                if (abs(thita) < vectorOps.GetEpsilon())
                    thita = 0;

                return thita;
            }

            Vertex Tree::Node::CalculateIntersection(const Vertex& a, const Vertex& b) const
            {
                const SoC::Maths::VectorOperationsInterface& vectorOps = SoC::Maths::GetVectorOperations();
                Vertex rayD = vectorOps.Unitise(vectorOps.Subtract(b, a));
                return vectorOps.RayPlaneIntersectionPoint(a, rayD, position_, normal_);
            }

            Mesh::IntersectionResult Tree::Node::CalculateTriangleIntersection(const Vertex& a, const Vertex& b, const Vertex& c) const
            {
                return Mesh::IntersectionResult(GetDirectionFromPlane(a), GetDirectionFromPlane(b), GetDirectionFromPlane(c));
            }


            /*Tree::Tree(const Mesh& mesh)
            {
            Populate(mesh, [](const Vertex& a, const Vertex& b, const Vertex& c, std::list<std::shared_ptr<Triangle>>::iterator& triangle) { return new Node(a, b, c); });
            }*/

            Tree::Tree(const Mesh& mesh)
                : Tree(mesh, [](const Vertex& a, const Vertex& b, const Vertex& c, std::list<std::shared_ptr<Triangle>>::iterator& triangle) { return new Node(a, b, c); })
            {
            }


            // Construct tree...
            Tree::Tree(const Mesh& mesh, std::function<Node*(const Vertex&, const Vertex&, const Vertex&, std::list<std::shared_ptr<Triangle>>::iterator&)> CreateNode)
                //Tree::Tree(const Mesh& mesh)
                //void Tree::Populate(const Mesh& mesh, std::function<Tree::Node*(const Vertex&, const Vertex&, const Vertex&, std::list<std::shared_ptr<Triangle>>::iterator&)> CreateNode)
            {
                if (mesh.vertices_.empty())
                    return;

                // we create a working list of the mesh triangles...
                std::list<std::shared_ptr<Triangle>> workingTriangles;
                std::vector<Vertex> workingVertices = mesh.vertices_;

                // populate with both the mesh outisde and inside triangles...
                workingTriangles.insert(workingTriangles.end(), mesh.inside_.begin(), mesh.inside_.end());
                workingTriangles.insert(workingTriangles.end(), mesh.outside_.begin(), mesh.outside_.end());

                if (workingTriangles.empty())
                    return;

                // Get the first triangle and add this as a node...
                std::shared_ptr<Triangle> triangle = workingTriangles.front();

                // Add the first node...
                //root_.reset(new Node(workingVertices[triangle->a()], workingVertices[triangle->b()], workingVertices[triangle->c()]));
                root_.reset(CreateNode(workingVertices[triangle->a()], workingVertices[triangle->b()], workingVertices[triangle->c()], workingTriangles.begin()));

                // remove this triangle from the working list...
                workingTriangles.pop_front();

                // We then need to iterate through the rest of the working triangles... traversing current tree, and then adding nodes as needed...
                while (!workingTriangles.empty())
                {
                    // get the current working triangle...
                    std::list<std::shared_ptr<Triangle>>::iterator currentTriangle = workingTriangles.begin();

                    // start the traversal from the root node...
                    //std::shared_ptr<Node> currentNode = root_;
                    Node* currentNode = root_.get();

                    while (currentNode)
                    {
                        // we first need to validate the triangle (ensure it is valid)
                        const Vertex& A = workingVertices[(*currentTriangle)->a()];
                        const Vertex& B = workingVertices[(*currentTriangle)->b()];
                        const Vertex& C = workingVertices[(*currentTriangle)->c()];

                        // validate the triangle...
                        if (!mesh.ValidateTriangle(A, B, C, *(*currentTriangle)))
                        {
                            // if the triangle is invalid... we ignore it so remove from our working list and start traversal with next triangle...
                            workingTriangles.erase(currentTriangle);
                            currentNode = 0;// currentNode.reset();
                            continue;
                        }

                        // Calculate the current triangle intersecton relative to the current node...
                        Mesh::IntersectionResult intersectionResult = currentNode->CalculateTriangleIntersection(A, B, C);

                        // If this triangle is outside of node (in front of node plane)
                        if (intersectionResult.IsOutside())
                        {
                            // follow the outside branch/path...
                            if (currentNode->outside_)
                                currentNode = currentNode->outside_.get();
                            else
                            {
                                // if there is no outside node, we need to create a new one with this triangles plane.
                                //currentNode->outside_.reset(new Node(A, B, C));
                                currentNode->outside_.reset(CreateNode(A, B, C, currentTriangle));
                                currentNode = 0;// currentNode.reset();
                            }
                        }
                        // Otherwise if the current triangle is inside or onplane of current node... (for onplane, assume triangle is 'inside')
                        else if (intersectionResult.IsInside() || intersectionResult.IsOnPlane())
                        {
                            // follow the inside branch/path...
                            if (currentNode->inside_)
                                currentNode = currentNode->inside_.get();
                            else
                            {
                                // if there is no inside node, we need to create a new one with this triangles plane.
                                currentNode->inside_.reset(CreateNode(A, B, C, currentTriangle));
                                //currentNode->inside_.reset(new Node(A, B, C));
                                currentNode = 0;// currentNode.reset();
                            }
                        }
                        // Otherwise this triangle straddles the current node plane... so we need to split accordinglly...
                        else
                        {
                            std::shared_ptr<Mesh> splitResult = mesh.SplitTriangle(intersectionResult, A, B, C, (*currentTriangle)->ID(), currentNode->GetPosition(), currentNode->GetNormal());

                            // Our split result will generate one or more triangles... if there is only one triangle, this means that one or more of the split triangles was invalid
                            if (splitResult->inside_.size() == 1 && splitResult->outside_.empty())
                            {
                                // if this triangle is inside, follow the inside...
                                if (currentNode->inside_)
                                    currentNode = currentNode->inside_.get();
                                else
                                {
                                    // if there is no inside node, we need to create a new one with this triangles plane.
                                    currentNode->inside_.reset(CreateNode(A, B, C, currentTriangle));
                                    currentNode = 0;// currentNode.reset();
                                }
                            }
                            else if (splitResult->outside_.size() == 1 && splitResult->inside_.empty())
                            {
                                // if this triangle is outside, follow the inside...
                                if (currentNode->outside_)
                                    currentNode = currentNode->outside_.get();
                                else
                                {
                                    // if there is no inside node, we need to create a new one with this triangles plane.
                                    currentNode->outside_.reset(CreateNode(A, B, C, currentTriangle));
                                    currentNode = 0;// currentNode.reset();
                                }
                            }
                            else
                            {
                                // Otherwise we need to add these split triangles to our workingtriangle list and start traversal again...

                                // add the vertices...
                                unsigned int vertexOffset = workingVertices.size();
                                workingVertices.insert(workingVertices.end(), splitResult->vertices_.begin(), splitResult->vertices_.end());

                                // add thes triangles to our working list, ammending the vertex offsets to cater for new vertices...
                                for (std::list<std::shared_ptr<Triangle>>::iterator insideItr = splitResult->inside_.begin(); insideItr != splitResult->inside_.end(); ++insideItr)
                                {
                                    (*insideItr)->OffsetIndexes(vertexOffset);
                                    workingTriangles.push_back(*insideItr);
                                }
                                for (std::list<std::shared_ptr<Triangle>>::iterator outsideItr = splitResult->outside_.begin(); outsideItr != splitResult->outside_.end(); ++outsideItr)
                                {
                                    (*outsideItr)->OffsetIndexes(vertexOffset);
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

            Mesh::IOFlag Tree::Traverse(const Vertex& v) const
            {
                return Mesh::IOFlag::UNDEFINED;
            }

            void Tree::Intersect(Mesh& mergeMesh, bool onPlaneIsInside) const
            {
                Intersect(mergeMesh, onPlaneIsInside, std::function<void(const Mesh&, Node* currentNode)>());
            }

            // we intersect this tree using the mesh provided
            void Tree::Intersect(Mesh& mergeMesh, bool onPlaneIsInside, std::function<void(const Mesh&, Node* currentNode)> splitResultCallback) const
            {
                // We need to create a working list of the current triangles from the mesh...
                std::list<std::shared_ptr<Triangle>> workingTriangles;

                // add the current triangles to our working list...
                for (std::list<std::shared_ptr<Triangle>>::iterator tri = mergeMesh.inside_.begin(); tri != mergeMesh.inside_.end(); ++tri)
                    workingTriangles.push_back(*tri);
                for (std::list<std::shared_ptr<Triangle>>::iterator tri = mergeMesh.outside_.begin(); tri != mergeMesh.outside_.end(); ++tri)
                    workingTriangles.push_back(*tri);

                // clear our merge result (result) inside and outside... our working list has references to them so triangles won't be freed.
                mergeMesh.inside_.clear();
                mergeMesh.outside_.clear();

                // For each triangle from our working list... we push it through this tree and and either split triangles adding them to inside or outside...
                // or keep adding to inside or outside...
                while (!workingTriangles.empty())
                {
                    // get the triangle itr
                    std::list<std::shared_ptr<Triangle>>::iterator currentTriangle = workingTriangles.begin();

                    // start to traverse the tree to see if this triangle is inside or outside the tree..
                    //std::shared_ptr<Node> currentNode = root_;
                    Node* currentNode = root_.get();

                    // start from root node, and traverse, calculating if triangle is infront or behind node plane...
                    while (currentNode)
                    {
                        const Vertex& A = mergeMesh.vertices_[(*currentTriangle)->a()];
                        const Vertex& B = mergeMesh.vertices_[(*currentTriangle)->b()];
                        const Vertex& C = mergeMesh.vertices_[(*currentTriangle)->c()];

                        // validate the triangle...
                        if (!mergeMesh.ValidateTriangle(A, B, C, *(*currentTriangle)))
                        {
                            // if the triangle is invalid... we ignore it so remove from our working list and start traversal with next working triangle...
                            workingTriangles.erase(currentTriangle);
                            currentNode = 0;// currentNode.reset();
                            continue;
                        }

                        // Calculate the current triangle intersecton relative to the current node...
                        Mesh::IntersectionResult intersectionResult = currentNode->CalculateTriangleIntersection(A, B, C);

                        // If this triangle is outside of node (in front of node plane)
                        if (intersectionResult.IsOutside())
                        {
                            // follow the outside branch/path...
                            if (currentNode->outside_)
                                currentNode = currentNode->outside_.get();
                            // if there is no outside branch... this triangle is on outside so add to our list...and start traversal again
                            else
                            {
                                mergeMesh.outside_.push_back(*currentTriangle);
                                currentNode = 0;// currentNode.reset();
                            }
                        }
                        // Otherwise if the current triangle is inside or onplane of current node... (for onplane, assume triangle is 'inside')
                        else if (intersectionResult.IsInside())
                        {
                            // follow the inside branch/path...
                            if (currentNode->inside_)
                                currentNode = currentNode->inside_.get();
                            // if there is no outside branch... this triangle is on outside so add to our list...
                            else
                            {
                                mergeMesh.inside_.push_back(*currentTriangle);
                                currentNode = 0;// currentNode.reset();
                            }

                        }
                        else if (intersectionResult.IsOnPlane())
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
                                    mergeMesh.inside_.push_back(*currentTriangle);
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
                                    mergeMesh.outside_.push_back(*currentTriangle);
                                    currentNode = 0;// currentNode.reset();
                                }
                            }
                        }


                        // Otherwise this triangle straddles the current node plane... so we need to split accordinglly...
                        else
                        {
                            // we split the triangles and generate the correct 
                            std::shared_ptr<Mesh> splitResult = mergeMesh.SplitTriangle(intersectionResult, A, B, C, (*currentTriangle)->ID(), currentNode->GetPosition(), currentNode->GetNormal());

                            // Pass the split result to our callback if supplied...
                            /*if (splitResultCallback)
                            splitResultCallback(*splitResult, currentNode);*/
                            Node* currentNodeForCallback = currentNode;

                            // Our split result will generate one or more triangles... if there is only one triangle, this means that one or more of the split triangles was invalid
                            if (splitResult->inside_.size() == 1 && splitResult->outside_.empty())
                            {
                                // follow the inside branch/path...
                                if (currentNode->inside_)
                                    currentNode = currentNode->inside_.get();
                                // if there is no outside branch... this triangle is on outside so add to our list...
                                else
                                {
                                    mergeMesh.inside_.push_back(*currentTriangle);
                                    currentNode = 0; //currentNode.reset();
                                }
                            }
                            else if (splitResult->outside_.size() == 1 && splitResult->inside_.empty())
                            {
                                // follow the outside branch/path...
                                if (currentNode->outside_)
                                    currentNode = currentNode->outside_.get();
                                // if there is no outside branch... this triangle is on outside so add to our list...
                                else
                                {
                                    mergeMesh.outside_.push_back(*currentTriangle);
                                    currentNode = 0; // currentNode.reset();
                                }
                            }
                            else
                            {
                                // if these are new split triangles to add... we need to ammend them to our list for traversal...

                                // add the vertices...
                                unsigned int vertexOffset = mergeMesh.vertices_.size();
                                mergeMesh.vertices_.insert(mergeMesh.vertices_.end(), splitResult->vertices_.begin(), splitResult->vertices_.end());

                                // add thes triangles to our working list, ammending the vertex offsets to cater for new vertices...
                                for (std::list<std::shared_ptr<Triangle>>::iterator insideItr = splitResult->inside_.begin(); insideItr != splitResult->inside_.end(); ++insideItr)
                                {
                                    (*insideItr)->OffsetIndexes(vertexOffset);
                                    workingTriangles.push_back(*insideItr);
                                }
                                for (std::list<std::shared_ptr<Triangle>>::iterator outsideItr = splitResult->outside_.begin(); outsideItr != splitResult->outside_.end(); ++outsideItr)
                                {
                                    (*outsideItr)->OffsetIndexes(vertexOffset);
                                    workingTriangles.push_back(*outsideItr);
                                }

                                currentNode = 0;
                            }

                            // Pass the split result to our callback if supplied...
                            if (splitResultCallback)
                                splitResultCallback(*splitResult, currentNodeForCallback);
                        }

                        // if there is no current node (it has been reset) this means we have either added new triangles to our working list (from a split)
                        // or we have added a node currently. So we need to start again with the next triangle
                        if (!currentNode)
                            workingTriangles.erase(currentTriangle);

                    }
                }
            }


            TreeMutableMesh::TMMNode::TMMNode(const Vertex& A, const Vertex& B, const Vertex& C, std::list<std::shared_ptr<Triangle>>::iterator& triangle)
                : Node(A, B, C), triangle_(*triangle)
            {
            }

            TreeMutableMesh::TreeMutableMesh(Mesh& mesh)
                : Tree(mesh, [](const Vertex& a, const Vertex& b, const Vertex& c, std::list<std::shared_ptr<Triangle>>::iterator& triangle) { return new TMMNode(a, b, c, triangle); }), mesh_(mesh)
            {
            }




            namespace
            {



                // Given two points on triangles... find all the intersection points between theie edges and any triangles within...

                struct NodeTriangleIntersections
                {

                    NodeTriangleIntersections(TreeMutableMesh::TMMNode* node, const Vertex& intersectionPoint)
                        : node_(node)
                    {
                        intersections_.push_back(intersectionPoint);
                    }

                    TreeMutableMesh::TMMNode* node_;
                    std::list<Vertex> intersections_;
                };

                struct SingleNodeTriangleIntersection
                {
                    SingleNodeTriangleIntersection(TreeMutableMesh::TMMNode* node, const Vertex& intersectionPoint)
                        : node_(node), intersection_(intersectionPoint)
                    {
                    }

                    TreeMutableMesh::TMMNode* node_;
                    Vertex intersection_;
                };


                // Get the intersected triangle ... will be this same normal as currentNode...   triangle, node
                TreeMutableMesh::TMMNode* GetIntersectedTriangleAndNode(TreeMutableMesh::TMMNode* node, Mesh& mesh, const SoC::Maths::Vector3& intersectionPoint)
                {
                    TreeMutableMesh::TMMNode* currentNode = node;

                    while (currentNode)
                    {
                        Vertex& A = mesh.vertices_[currentNode->triangle_->a()];
                        Vertex& B = mesh.vertices_[currentNode->triangle_->b()];
                        Vertex& C = mesh.vertices_[currentNode->triangle_->c()];

                        // IF this intersection point is within the triangle...
                        if (SoC::Maths::GetVectorOperations().IsPointWithinTriangle(A, B, C, intersectionPoint))
                            return currentNode;
                        else
                        {
                            // get the next one... only need to check 'inside'
                            currentNode = static_cast<TreeMutableMesh::TMMNode*>(currentNode->inside_.get());

                            // if the next node does not have the same normal as current node, we are finished checking...
                            if (currentNode)
                                if (!SoC::Maths::GetVectorOperations().Equals(node->GetNormal(), currentNode->GetNormal()))
                                    return 0;
                        }
                    }

                    return 0;
                }


                std::list<TreeMutableMesh::TMMNode*> GetSameFacingNodes(TreeMutableMesh::TMMNode* node)
                {
                    std::list<TreeMutableMesh::TMMNode*> result;



                    TreeMutableMesh::TMMNode* currentNode = node;

                    while (currentNode)
                    {
                        result.push_back(currentNode);

                        // get the next one... only need to check 'inside'
                        currentNode = static_cast<TreeMutableMesh::TMMNode*>(currentNode->inside_.get());

                        // if the next node does not have the same normal as current node, we are finished checking...
                        if (currentNode)
                            if (!SoC::Maths::GetVectorOperations().Equals(node->GetNormal(), currentNode->GetNormal()))
                                currentNode = 0;
                    }

                    return result;
                }


            }

            std::list<TreeMutableMesh::TMMNode*> TreeMutableMesh::IsOnTriangle(const Vertex& v, bool onPlaneIsInside)
            {
                std::list<TreeMutableMesh::TMMNode*> result;

                // traverse the tree finding the triangle that v lies on...
                // start the traversal from the root node...
                Node* currentNode = root_.get();

                while (currentNode)
                {
                    float direction = currentNode->GetDirectionFromPlane(v);

                    // if outside...
                    if (direction > 0)
                    {
                        if (currentNode->outside_)
                            currentNode = currentNode->outside_.get();
                        else
                            currentNode = 0;
                    }
                    else if (direction < 0)
                    {
                        if (currentNode->inside_)
                            currentNode = currentNode->inside_.get();
                        else
                            currentNode = 0;
                    }
                    else
                    {
                        // Otherwise it is on the plane... keep checking all nodes on the plane (same normal as current node)
                        TMMNode* tmmNode = static_cast<TMMNode*>(currentNode);
                        if (tmmNode)
                        {
                            const Vertex& A = mesh_.vertices_[tmmNode->triangle_->a()];
                            const Vertex& B = mesh_.vertices_[tmmNode->triangle_->b()];
                            const Vertex& C = mesh_.vertices_[tmmNode->triangle_->c()];

                            if (SoC::Maths::GetVectorOperations().IsPointWithinTriangle(A, B, C, v))
                                result.push_back(tmmNode);
                        }

                        if (currentNode->inside_)
                            currentNode = currentNode->inside_.get();
                        else
                            currentNode = 0;

                        //// current node continues... following inside...
                        //if (onPlaneIsInside)
                        //{
                        //    if (currentNode->inside_)
                        //        currentNode = currentNode->inside_.get();
                        //    else
                        //        currentNode = 0;
                        //}
                        //else
                        //{
                        //    if (currentNode->outside_)
                        //        currentNode = currentNode->outside_.get();
                        //    else
                        //        currentNode = 0;
                        //}
                    }
                }

                return result;

            }



            namespace
            {
                void TessellateNodeAndTraverse(Tree::Node* node, Mesh& resultantMesh)
                {
                    TreeMutableMesh::TMMNode* tmmNode = static_cast<TreeMutableMesh::TMMNode*>(node);

                    if (tmmNode)
                    {
                        if (!tmmNode->intersections_.empty())
                        {
                            std::vector<SoC::Maths::Vector3> pointsToTessellate;

                            pointsToTessellate.push_back(resultantMesh.vertices_[tmmNode->triangle_->a()]);
                            pointsToTessellate.push_back(resultantMesh.vertices_[tmmNode->triangle_->b()]);
                            pointsToTessellate.push_back(resultantMesh.vertices_[tmmNode->triangle_->c()]);

                            // Calculate the pose using the original triangle vertices...
                            std::shared_ptr<SoC::Geometry::Pose> pose = SoC::Geometry::GetMeshOperations().GetCoplanarPose(pointsToTessellate);

                            // Add the additional intersection points...
                            pointsToTessellate.insert(pointsToTessellate.end(), tmmNode->intersections_.begin(), tmmNode->intersections_.end());

                            // clamp and remove duplicates...
                            {
                                Mesh intersectionPoints;
                                intersectionPoints.vertices_ = pointsToTessellate;
                                intersectionPoints.ClampPoints(SoC::Maths::GetVectorOperations().GetEpsilon());
                                pointsToTessellate = SoC::Geometry::GetMeshOperations().RemoveDuplicatePoints(intersectionPoints.vertices_);
                            }

                            {
                                std::ostringstream oss;
                                oss << "Tessellating triangle " << tmmNode->triangle_->a() << ", " << tmmNode->triangle_->b() << ", " << tmmNode->triangle_->c() << '\n';
                                for (unsigned int v = 0; v < pointsToTessellate.size(); ++v)
                                    oss << " " << v << ": " << pointsToTessellate[v].x_ << ", " << pointsToTessellate[v].y_ << ", " << pointsToTessellate[v].z_ << '\n';
                                OutputDebugString(oss.str().c_str());
                            }

                            std::shared_ptr<SoC::Geometry::Mesh> newTessellation = SoC::Geometry::GetMeshOperations().TessellateDelaunay(pointsToTessellate, *pose);

                            // get the original triangle to remove...
                            bool isInside = false;
                            if (1 == 1)
                            {
                                std::shared_ptr<Triangle> triangleToRemove = tmmNode->triangle_;

                                // check the insides for it...
                                std::list<std::shared_ptr<Triangle>>::iterator triangleToRemoveItr = std::find_if(resultantMesh.inside_.begin(), resultantMesh.inside_.end(),
                                    [&triangleToRemove](std::shared_ptr<Triangle>& triangle)
                                {
                                    if ((triangle->a() == triangleToRemove->a()) && (triangle->b() == triangleToRemove->b()) && (triangle->c() == triangleToRemove->c()))
                                        return true;
                                    return false;
                                });

                                // If found remove it!
                                if (triangleToRemoveItr != resultantMesh.inside_.end())
                                {
                                    resultantMesh.inside_.erase(triangleToRemoveItr);
                                    isInside = true;
                                }
                                // Otherwise check the outsides....
                                else
                                {
                                    triangleToRemoveItr = std::find_if(resultantMesh.outside_.begin(), resultantMesh.outside_.end(),
                                        [&triangleToRemove](std::shared_ptr<Triangle>& triangle)
                                    {
                                        if ((triangle->a() == triangleToRemove->a()) && (triangle->b() == triangleToRemove->b()) && (triangle->c() == triangleToRemove->c()))
                                            return true;
                                        return false;
                                    });

                                    if (triangleToRemoveItr == resultantMesh.outside_.end())
                                        throw SoC::Geometry::Exception(SoC::Geometry::ExceptionID::BSPUnableToFindTriangle);

                                    resultantMesh.outside_.erase(triangleToRemoveItr);
                                }
                            }

                            // add these triangles and vertices...
                            if (1 == 1)
                            {
                                unsigned int offsetIndex = resultantMesh.vertices_.size();
                                resultantMesh.vertices_.insert(resultantMesh.vertices_.end(), newTessellation->GetPoints().begin(), newTessellation->GetPoints().end());

                                for (unsigned int t = 0; t < newTessellation->GetTriangles().size(); ++t)
                                {
                                    SoC::Geometry::TriangleInterface::Triangle& tri = newTessellation->GetTriangles()[t];
                                    std::shared_ptr<Triangle> newTriangle(new Triangle(tri.a_ + offsetIndex, tri.b_ + offsetIndex, tri.c_ + offsetIndex, 3));
                                    if (isInside)
                                        resultantMesh.inside_.push_back(newTriangle);
                                    else
                                        resultantMesh.outside_.push_back(newTriangle);
                                }
                            }

                        }
                    }

                    if (node->inside_)
                        TessellateNodeAndTraverse(node->inside_.get(), resultantMesh);

                    if (node->outside_)
                        TessellateNodeAndTraverse(node->outside_.get(), resultantMesh);
                }


            }


            void TreeMutableMesh::IntersectAndMutate(Mesh& mergeMesh, bool onPlaneIsInside)
            {
                // we call intersect with our call back which we track the intersections added so we can  mutate our original mesh.
                //std::list<NodeTriangleIntersections> nodeTriangleIntersectionsList;
                std::map<TMMNode*, std::list<SoC::Maths::Vector3>> nodeTriangleIntersections;
                Mesh& thisMesh = mesh_;

                // Intrsect the mergeMesh with our tree...
                Intersect(mergeMesh, onPlaneIsInside, [&nodeTriangleIntersections, &thisMesh](const Mesh& splitResult, Node* currentNode) {});

                // THIS IS HEAVY :(

                // Now we need to check each of mergemesh vertices.... any that lie on a triangle of original mesh are added to that triangles intersections...
                for (std::vector<Vertex>::iterator mergeMeshVertexItr = mergeMesh.vertices_.begin(); mergeMeshVertexItr != mergeMesh.vertices_.end(); ++mergeMeshVertexItr)
                {
                    Vertex& vertex = *mergeMeshVertexItr;
                    std::list<TMMNode*> intersectedTriangles = IsOnTriangle(vertex, onPlaneIsInside);
                    std::for_each(intersectedTriangles.begin(), intersectedTriangles.end(), [&vertex](TMMNode* node) { node->intersections_.push_back(vertex); });

                    {
                        std::ostringstream oss;
                        oss << " " << mergeMeshVertexItr->x_ << ", " << mergeMeshVertexItr->y_ << ", " << mergeMeshVertexItr->z_ << " :";
                        if (!intersectedTriangles.empty())
                        {
                            for (std::list<TMMNode*>::iterator itr = intersectedTriangles.begin(); itr != intersectedTriangles.end(); ++itr)
                                oss << " " << (*itr)->triangle_->a() << ", " << (*itr)->triangle_->b() << ", " << (*itr)->triangle_->c() << "\n";
                        }
                        else
                            oss << '\n';
                        OutputDebugString(oss.str().c_str());
                    }
                }

                // Then we can iterate all our nodes/triangles and retessellate them...
                TessellateNodeAndTraverse(root_.get(), mesh_);

                mesh_.ShareVertices();

            }

            Mesh HalfSpace::GetPlane(const Vertex& hsPosition, const Vertex& hsNormal)
            {
                Mesh mesh;

                // construct three points on the plane...
                SoC::Geometry::OrthogonalAxis3D axis(hsNormal);

                const SoC::Maths::VectorOperationsInterface& vectorOps = SoC::Maths::GetVectorOperations();

                mesh.vertices_.push_back(hsPosition);
                mesh.vertices_.push_back(vectorOps.Add(hsPosition, axis.i()));
                mesh.vertices_.push_back(vectorOps.Add(hsPosition, axis.j()));
                mesh.inside_.push_back(mesh.CreateTriangle(Mesh::IOFlag::UNDEFINED, 0, 1, 2, 0));

                return mesh;
            }


            namespace
            {
                struct HalfSpaceFillReference
                {
                    HalfSpaceFillReference(std::shared_ptr<SoC::Geometry::BSP::Triangle>& triangle)
                        : triangle_(triangle), processed_(false)
                    {
                    }

                    std::shared_ptr<SoC::Geometry::BSP::Triangle> triangle_;
                    bool processed_;
                };
            }

            void HalfSpace::Intersect(Mesh& mergeMesh, bool onPlaneIsInside, bool fill) const
            {
                Tree::Intersect(mergeMesh, onPlaneIsInside);
                mergeMesh.ShareVertices();
            }

            Mesh::Mesh(const SoC::Geometry::Mesh& mesh, unsigned int id)
            {
                // add the points...
                vertices_.reserve(mesh.GetVertices().size());
                for (unsigned int v = 0; v < mesh.GetVertices().size(); ++v)
                    vertices_.push_back(mesh.GetPoints()[mesh.GetVertices()[v].p_]);

                // we then add the triangles... 
                for (unsigned int t = 0; t < mesh.GetTriangles().size(); ++t)
                {
                    const SoC::Geometry::TriangleInterface::Triangle& triangle = mesh.GetTriangles()[t];
                    inside_.push_back(CreateTriangle(SoC::Geometry::BSP::Mesh::IOFlag::INSIDE, triangle.a_, triangle.b_, triangle.c_, id));
                }
            }

            std::shared_ptr<SoC::Geometry::Mesh> Mesh::GetSoCMesh() const
            {
                std::shared_ptr<SoC::Geometry::Mesh> result(new SoC::Geometry::Mesh(vertices_, std::vector<SoC::Maths::Vector3>(), std::vector<SoC::Geometry::VertexInterface::Vertex>(), std::vector<SoC::Geometry::TriangleInterface::Triangle>()));

                result->GetVertices().reserve(vertices_.size());
                for (unsigned int v = 0; v < vertices_.size(); ++v)
                    result->GetVertices().push_back(SoC::Geometry::VertexInterface::Vertex(v, 0));

                // add the triangles... inside followed by outside...
                result->GetTriangles().reserve(inside_.size() + outside_.size());
                for (std::list<std::shared_ptr<SoC::Geometry::BSP::Triangle>>::const_iterator tri = inside_.begin(); tri != inside_.end(); ++tri)
                    result->GetTriangles().push_back(SoC::Geometry::TriangleInterface::Triangle((*tri)->a(), (*tri)->b(), (*tri)->c()));
                for (std::list<std::shared_ptr<SoC::Geometry::BSP::Triangle>>::const_iterator tri = outside_.begin(); tri != outside_.end(); ++tri)
                    result->GetTriangles().push_back(SoC::Geometry::TriangleInterface::Triangle((*tri)->a(), (*tri)->b(), (*tri)->c()));

                return SoC::Geometry::GetMeshOperations().GenerateVertices(*result, false);
            }

            std::shared_ptr<SoC::Geometry::Mesh> Mesh::GetSoCMesh(unsigned int ID) const
            {
                std::shared_ptr<SoC::Geometry::Mesh> result(new SoC::Geometry::Mesh(std::vector<SoC::Maths::Vector3>(), std::vector<SoC::Maths::Vector3>(), std::vector<SoC::Geometry::VertexInterface::Vertex>(), std::vector<SoC::Geometry::TriangleInterface::Triangle>()));

                // relevant triangles...
                std::list<std::shared_ptr<SoC::Geometry::BSP::Triangle>> triangles;

                // iterate through all the triangles.... triangles with ID equal to ID, we add to the mesh...
                for (std::list<std::shared_ptr<SoC::Geometry::BSP::Triangle>>::const_iterator tri = inside_.begin(); tri != inside_.end(); ++tri)
                {
                    if ((*tri)->ID() == ID)
                        triangles.push_back(*tri);
                }


                for (std::list<std::shared_ptr<SoC::Geometry::BSP::Triangle>>::const_iterator tri = outside_.begin(); tri != outside_.end(); ++tri)
                {
                    if ((*tri)->ID() == ID)
                        triangles.push_back(*tri);
                }

                // we can then spit just these out...
                result->GetPoints().reserve(triangles.size() * 3);
                result->GetVertices().reserve(triangles.size() * 3);
                result->GetTriangles().reserve(triangles.size());

                for (std::list<std::shared_ptr<SoC::Geometry::BSP::Triangle>>::const_iterator tri = triangles.begin(); tri != triangles.end(); ++tri)
                {
                    // add the points...
                    unsigned int vertexOffset = result->GetPoints().size();

                    result->GetPoints().push_back(vertices_[(*tri)->a()]);
                    result->GetPoints().push_back(vertices_[(*tri)->b()]);
                    result->GetPoints().push_back(vertices_[(*tri)->c()]);

                    result->GetVertices().push_back(SoC::Geometry::VertexInterface::Vertex(vertexOffset, 0));
                    result->GetVertices().push_back(SoC::Geometry::VertexInterface::Vertex(vertexOffset + 1, 0));
                    result->GetVertices().push_back(SoC::Geometry::VertexInterface::Vertex(vertexOffset + 2, 0));

                    // add the triangle...
                    result->GetTriangles().push_back(SoC::Geometry::TriangleInterface::Triangle(vertexOffset, vertexOffset + 1, vertexOffset + 2));
                }

                return SoC::Geometry::GetMeshOperations().GenerateVertices(*result, false);
            }

            void Mesh::RemoveIDs(unsigned int ID)
            {
                // remove all triangles with the correct ID...
                std::list<std::shared_ptr<SoC::Geometry::BSP::Triangle>>::const_iterator tri = inside_.begin();
                while (tri != inside_.end())
                {
                    if ((*tri)->ID() == ID)
                        tri = inside_.erase(tri);
                    else
                        ++tri;
                }

                tri = outside_.begin();
                while (tri != outside_.end())
                {
                    if ((*tri)->ID() == ID)
                        tri = outside_.erase(tri);
                    else
                        ++tri;
                }
            }

            // Mesh operations.... these are overridable by user for bespoke triangle information...
            std::shared_ptr<Triangle> Mesh::CreateTriangle(const Mesh::IOFlag& flag, unsigned int a, unsigned int b, unsigned int c, unsigned int ID) const
            {
                return std::shared_ptr<Triangle>(new Triangle(a, b, c, ID));
            }

            bool Mesh::ValidateTriangle(const Vertex& A, const Vertex& B, const Vertex& C, const Triangle& triangle) const
            {
                // Calculate the area and ensure triangle is large enough...
                float area = SoC::Maths::GetVectorOperations().Area(A, B, C);
                return (area > SoC::Maths::GetVectorOperations().GetEpsilon());
            }

            std::shared_ptr<Mesh> Mesh::SplitTriangle(const Mesh::IntersectionResult& intersectionResult, const Vertex& A, const Vertex& B, const Vertex& C, unsigned int ID, const Vertex& planePosition, const Vertex& planeNormal) const
            {
                std::shared_ptr<Mesh> result(new Mesh());

                const SoC::Maths::VectorOperationsInterface& vectorOps = SoC::Maths::GetVectorOperations();

                // put the original three points from the triangle into the split result...
                result->vertices_.reserve(5);        // at most there can be 5 vertices...
                result->vertices_.push_back(A);
                result->vertices_.push_back(B);
                result->vertices_.push_back(C);

                // the indexes...
                unsigned int triangleA = 0, triangleB = 1, triangleC = 2, intersection1 = 3, intersection2 = 4;

                if (intersectionResult.AtLeastOneVertexIsOnPlane())
                {
                    // 1 intersection ... 2 triangles...
                    std::shared_ptr<Triangle> insideTriangle;
                    std::shared_ptr<Triangle> outsideTriangle;

                    if (!intersectionResult.A())
                    {
                        // if a is on plane... b must be one side, and c the other... so use these to calculate the intersection point...
                        result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(B, vectorOps.Unitise(vectorOps.Subtract(C, B)), planePosition, planeNormal));

                        // add the two triangles... if b is behind, a must be infront...
                        if (intersectionResult.B() < 0)
                        {
                            insideTriangle = result->CreateTriangle(Mesh::IOFlag::INSIDE, triangleA, triangleB, intersection1, ID);
                            outsideTriangle = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, intersection1, triangleC, triangleA, ID);
                        }
                        else
                        {
                            outsideTriangle = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, triangleA, triangleB, intersection1, ID);
                            insideTriangle = result->CreateTriangle(Mesh::IOFlag::INSIDE, intersection1, triangleC, triangleA, ID);
                        }
                    }
                    else if (!intersectionResult.B())
                    {
                        // if b is on plane... c must be one side, and a the other... so use these to calculate the intersection point...
                        result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(C, vectorOps.Unitise(vectorOps.Subtract(A, C)), planePosition, planeNormal));

                        // add the two triangles... if c is behind, a must be infront...
                        if (intersectionResult.C() < 0)
                        {
                            insideTriangle = result->CreateTriangle(Mesh::IOFlag::INSIDE, triangleB, triangleC, intersection1, ID);
                            outsideTriangle = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, intersection1, triangleA, triangleB, ID);
                        }
                        else
                        {
                            outsideTriangle = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, triangleB, triangleC, intersection1, ID);
                            insideTriangle = result->CreateTriangle(Mesh::IOFlag::INSIDE, intersection1, triangleA, triangleB, ID);
                        }
                    }
                    else if (!intersectionResult.C())
                    {
                        // if c is on plane... b must be one side, and a the other... so use these to calculate the intersection point...
                        result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(A, vectorOps.Unitise(vectorOps.Subtract(B, A)), planePosition, planeNormal));

                        // add the two triangles... if b is behind, a must be infront...
                        if (intersectionResult.A() < 0)
                        {
                            insideTriangle = result->CreateTriangle(Mesh::IOFlag::INSIDE, triangleC, triangleA, intersection1, ID);
                            outsideTriangle = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, intersection1, triangleB, triangleC, ID);
                        }
                        else
                        {
                            outsideTriangle = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, triangleC, triangleA, intersection1, ID);
                            insideTriangle = result->CreateTriangle(Mesh::IOFlag::INSIDE, intersection1, triangleB, triangleC, ID);
                        }
                    }
                    else
                        throw SoC::Geometry::Exception(SoC::Geometry::ExceptionID::CSGSplitIntersectionInvalid);

                    if (result->ValidateTriangle(result->vertices_[insideTriangle->a()], result->vertices_[insideTriangle->b()], result->vertices_[insideTriangle->c()], *insideTriangle))
                        result->inside_.push_back(insideTriangle);

                    if (result->ValidateTriangle(result->vertices_[outsideTriangle->a()], result->vertices_[outsideTriangle->b()], result->vertices_[outsideTriangle->c()], *outsideTriangle))
                        result->outside_.push_back(outsideTriangle);
                }
                else
                {
                    // 2 intersections ... 3 triangles...

                    // first find out which is behind and which is infront...
                    int aDirection = intersectionResult.Direction(intersectionResult.A());
                    int bDirection = intersectionResult.Direction(intersectionResult.B());
                    int cDirection = intersectionResult.Direction(intersectionResult.C());

                    int sum = aDirection + bDirection + cDirection;

                    // 1 intersection ... 2 triangles...

                    // if the split triangle is outside heavy...
                    if (sum > 0)
                    {
                        std::shared_ptr<Triangle> insideTriangle, outsideTriangle1, outsideTriangle2;

                        // if a is behind...
                        if (aDirection < 0)
                        {
                            // b and c must be infront...
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(A, vectorOps.Unitise(vectorOps.Subtract(B, A)), planePosition, planeNormal));
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(A, vectorOps.Unitise(vectorOps.Subtract(C, A)), planePosition, planeNormal));

                            // calculate the inside triangle...
                            insideTriangle = result->CreateTriangle(Mesh::IOFlag::INSIDE, triangleA, intersection1, intersection2, ID);// , result->vertices_[triangleA], result->vertices_[intersection1], result->vertices_[intersection2]);

                                                                                                                                       // calculate the outside triangles...
                            outsideTriangle1 = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, intersection1, triangleB, triangleC, ID);// , result->vertices_[intersection1], result->vertices_[triangleB], result->vertices_[triangleC]);
                            outsideTriangle2 = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, triangleC, intersection2, intersection1, ID);// , result->vertices_[triangleC], result->vertices_[intersection2], result->vertices_[intersection1]);
                        }
                        else if (bDirection < 0)
                        {
                            // c and a must be infront...
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(B, vectorOps.Unitise(vectorOps.Subtract(C, B)), planePosition, planeNormal));
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(B, vectorOps.Unitise(vectorOps.Subtract(A, B)), planePosition, planeNormal));

                            // calculate the inside triangle...
                            insideTriangle = result->CreateTriangle(Mesh::IOFlag::INSIDE, triangleB, intersection1, intersection2, ID);// , result->vertices_[triangleB], result->vertices_[intersection1], result->vertices_[intersection2]);

                                                                                                                                       // calculate the outside triangles...
                            outsideTriangle1 = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, intersection1, triangleC, triangleA, ID);//, result->vertices_[intersection1], result->vertices_[triangleC], result->vertices_[triangleA]);
                            outsideTriangle2 = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, triangleA, intersection2, intersection1, ID);// , result->vertices_[triangleA], result->vertices_[intersection2], result->vertices_[intersection1]);
                        }
                        else if (cDirection < 0)
                        {
                            // a and b must be infront...
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(C, vectorOps.Unitise(vectorOps.Subtract(C, A)), planePosition, planeNormal));
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(C, vectorOps.Unitise(vectorOps.Subtract(C, B)), planePosition, planeNormal));

                            // calculate the inside triangle...
                            insideTriangle = result->CreateTriangle(Mesh::IOFlag::INSIDE, triangleC, intersection1, intersection2, ID);// , result->vertices_[triangleC], result->vertices_[intersection1], result->vertices_[intersection2]);

                                                                                                                                       // calculate the outside triangles...
                            outsideTriangle1 = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, intersection1, triangleA, triangleB, ID);// , result->vertices_[intersection1], result->vertices_[triangleA], result->vertices_[triangleB]);
                            outsideTriangle2 = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, triangleB, intersection2, intersection1, ID);// , result->vertices_[triangleB], result->vertices_[intersection2], result->vertices_[intersection1]);
                        }

                        // if the inside triangle is invalid... add the original triangle as the outside triangle.
                        if (!result->ValidateTriangle(result->vertices_[insideTriangle->a()], result->vertices_[insideTriangle->b()], result->vertices_[insideTriangle->c()], *insideTriangle))
                            result->outside_.push_back(result->CreateTriangle(Mesh::IOFlag::OUTSIDE, triangleA, triangleB, triangleC, ID));// , result->vertices_[triangleA], result->vertices_[triangleB], result->vertices_[triangleC]));
                        else
                        {
                            result->inside_.push_back(insideTriangle);

                            // outside triangle1 and outside triangle2 must be valid
                            result->outside_.push_back(outsideTriangle1);
                            result->outside_.push_back(outsideTriangle2);
                        }
                    }

                    // Otherwise it is inside heavy...
                    else
                    {
                        std::shared_ptr<Triangle> outsideTriangle, insideTriangle1, insideTriangle2;

                        // if a is infront...
                        if (aDirection > 0)
                        {
                            // b and c must be behind...
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(A, vectorOps.Unitise(vectorOps.Subtract(B, A)), planePosition, planeNormal));
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(A, vectorOps.Unitise(vectorOps.Subtract(C, A)), planePosition, planeNormal));

                            // calculate the outside triangle...
                            outsideTriangle = result->CreateTriangle(Mesh::IOFlag::OUTSIDE, triangleA, intersection1, intersection2, ID);

                            // calculate the inside triangles...
                            insideTriangle1 = result->CreateTriangle(BSP::Mesh::IOFlag::INSIDE, intersection1, triangleB, triangleC, ID);
                            insideTriangle2 = result->CreateTriangle(BSP::Mesh::IOFlag::INSIDE, triangleC, intersection2, intersection1, ID);
                        }
                        else if (bDirection > 0)
                        {
                            // c and a must be behind...
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(B, vectorOps.Unitise(vectorOps.Subtract(C, B)), planePosition, planeNormal));
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(B, vectorOps.Unitise(vectorOps.Subtract(A, B)), planePosition, planeNormal));

                            // calculate the inside triangle...
                            outsideTriangle = result->CreateTriangle(BSP::Mesh::IOFlag::OUTSIDE, triangleB, intersection1, intersection2, ID);

                            // calculate the outside triangles...
                            insideTriangle1 = result->CreateTriangle(BSP::Mesh::IOFlag::INSIDE, intersection1, triangleC, triangleA, ID);
                            insideTriangle2 = result->CreateTriangle(BSP::Mesh::IOFlag::INSIDE, triangleA, intersection2, intersection1, ID);
                        }
                        else if (cDirection > 0)
                        {
                            // a and b must be infront...
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(C, vectorOps.Unitise(vectorOps.Subtract(C, A)), planePosition, planeNormal));
                            result->vertices_.push_back(vectorOps.RayPlaneIntersectionPoint(C, vectorOps.Unitise(vectorOps.Subtract(C, B)), planePosition, planeNormal));

                            // calculate the inside triangle...
                            outsideTriangle = result->CreateTriangle(BSP::Mesh::IOFlag::OUTSIDE, triangleC, intersection1, intersection2, ID);

                            // calculate the outside triangles...
                            insideTriangle1 = result->CreateTriangle(BSP::Mesh::IOFlag::INSIDE, intersection1, triangleA, triangleB, ID);
                            insideTriangle2 = result->CreateTriangle(BSP::Mesh::IOFlag::INSIDE, triangleB, intersection2, intersection1, ID);
                        }

                        // if the inside triangle is invalid... add the original triangle as the outside triangle.
                        if (!result->ValidateTriangle(result->vertices_[outsideTriangle->a()], result->vertices_[outsideTriangle->b()], result->vertices_[outsideTriangle->c()], *outsideTriangle))
                            result->inside_.push_back(result->CreateTriangle(BSP::Mesh::IOFlag::INSIDE, triangleA, triangleB, triangleC, ID));
                        else
                        {
                            result->outside_.push_back(outsideTriangle);

                            // outside triangle1 and outside triangle2 must be valid
                            result->inside_.push_back(insideTriangle1);
                            result->inside_.push_back(insideTriangle2);
                        }
                    }
                }
                return result;
            }


            // And something to remove all bad triangles....
            namespace
            {
                struct VertexReference
                {
                    VertexReference(const SoC::Maths::Vector3& point, unsigned int meshVertexIndex) : point_(point), addedVertexIndex_(0), meshVertexIndex_(meshVertexIndex)
                    {
                    }

                    SoC::Maths::Vector3 point_;
                    unsigned int addedVertexIndex_;
                    unsigned int meshVertexIndex_;
                };

                // caluclate the joining edges of the of each triangle in the mesh...
                struct VertexUser
                {
                    std::list<std::shared_ptr<Triangle>> users_;
                };

                bool HasSameUser(std::shared_ptr<Triangle>& currentTriangle, const VertexUser& i, const VertexUser& j)
                {
                    // we need to ignore users with the requested ID as this will always give fals positive (this tringle must be used by all to get here...)

                    // first check a... b needs to have the same user as a for ab join...
                    for (std::list<std::shared_ptr<Triangle>>::const_iterator iUser = i.users_.begin(); iUser != i.users_.end(); ++iUser)
                    {
                        if ((*iUser) == currentTriangle)
                            continue;

                        for (std::list<std::shared_ptr<Triangle>>::const_iterator jUser = j.users_.begin(); jUser != j.users_.end(); ++jUser)
                        {
                            if ((*jUser) == currentTriangle)
                                continue;

                            if ((*iUser) == (*jUser))
                                return true;
                        }

                    }
                    return false;
                }

                void DumpTriangle(std::vector<Vertex>& vertices, std::list<std::shared_ptr<Triangle>>::iterator triangle, unsigned int numberOfJoins, unsigned int currentJoinCount)
                {
                    std::ostringstream oss;
                    //oss << "C:\\Dump\\Triangle" << joins << "Join" << (*inside)->a() << (*inside)->b() << (*inside)->c() << ".wrl";
                    oss << "C:\\Dump\\Triangle" << numberOfJoins << "Join" << currentJoinCount << ".wrl";
                    VRMLDump::outputFile vrml(oss.str().c_str());

                    vrml.current_GeomMode(VRMLDump::faces);

                    switch (numberOfJoins)
                    {
                    case 0: vrml.current_Colour(1.0f, 0.3f, 0.3f, 0.5f); break;
                    case 1: vrml.current_Colour(0.6f, 0.6f, 0.6f, 0.5f); break;
                    case 2: vrml.current_Colour(0.0f, 0.0f, 1.0f, 0.5f); break;
                    case 3: vrml.current_Colour(0.0f, 1.0f, 0.0f, 0.5f); break;
                    }

                    vrml.StartGeom();

                    vrml.dump_vec3(vertices[(*triangle)->a()].x_, vertices[(*triangle)->a()].y_, vertices[(*triangle)->a()].z_);
                    vrml.dump_vec3(vertices[(*triangle)->b()].x_, vertices[(*triangle)->b()].y_, vertices[(*triangle)->b()].z_);
                    vrml.dump_vec3(vertices[(*triangle)->c()].x_, vertices[(*triangle)->c()].y_, vertices[(*triangle)->c()].z_);

                    vrml.dump_index(0);
                    vrml.dump_index(1);
                    vrml.dump_index(2);
                    vrml.dump_EndPoly();

                    vrml.StopGeom();
                }
            }

            void Mesh::ShareVertices()
            {
                if (vertices_.empty())
                    return;

                const SoC::Maths::VectorOperationsInterface& vectorOps = SoC::Maths::GetVectorOperations();

                // create a vector of references...
                std::vector<Vertex> newVertices;

                // create a vector mirroring the mesh vertex list...
                std::vector<VertexReference> vertexReferences;
                vertexReferences.reserve(vertices_.size());
                for (unsigned int mv = 0; mv < vertices_.size(); ++mv)
                    vertexReferences.push_back(VertexReference(vertices_[mv], mv));

                // then sort the vertex reference by point (ascending)
                std::sort(vertexReferences.begin(), vertexReferences.end(),
                    [](VertexReference& a, VertexReference& b)
                {
                    if (a.point_.x_ == b.point_.x_)
                    {
                        if (a.point_.y_ == b.point_.y_)
                        {
                            return a.point_.z_ < b.point_.z_;
                        }

                        return a.point_.y_ < b.point_.y_;
                    }

                    return a.point_.x_ < b.point_.x_;
                });


                // Once sorted, we strategically add the points and vertices to our new result...
                newVertices.reserve(vertexReferences.size());
                VertexReference* added = &vertexReferences.front();

                unsigned int addedIndex = newVertices.size();;

                // add the first point and vertex...
                added->addedVertexIndex_ = addedIndex;
                newVertices.push_back(added->point_);
                ++addedIndex;

                for (unsigned int vr = 1; vr < vertexReferences.size(); ++vr)
                {
                    VertexReference& vertRef = vertexReferences[vr];

                    // if the points are not the same...
                    if (!vectorOps.Equals(vertRef.point_, added->point_))
                    {
                        vertRef.addedVertexIndex_ = addedIndex;
                        newVertices.push_back(vertRef.point_);
                        addedIndex = newVertices.size();

                        // added is our new vertexReference...
                        added = &vertRef;
                    }
                    else
                    {
                        // update this vertex references added vertex with the correct index...
                        vertRef.addedVertexIndex_ = added->addedVertexIndex_;
                    }
                }


                // We then reorder back into original order for the tringles to query and update with the correct reference...
                std::sort(vertexReferences.begin(), vertexReferences.end(),
                    [](VertexReference& a, VertexReference& b)
                {
                    return a.meshVertexIndex_ < b.meshVertexIndex_;
                });

                // then we can go through the triangles, updating them with the correct vertex index...

                // we need to repeat this for both inside and outside...
                for (std::list<std::shared_ptr<Triangle>>::iterator outside = outside_.begin(); outside != outside_.end(); ++outside)
                {
                    // query against the vertex references... and ammend the index...
                    (*outside).reset(new Triangle(vertexReferences[(*outside)->a()].addedVertexIndex_, vertexReferences[(*outside)->b()].addedVertexIndex_, vertexReferences[(*outside)->c()].addedVertexIndex_, (*outside)->ID()));
                }
                for (std::list<std::shared_ptr<Triangle>>::iterator inside = inside_.begin(); inside != inside_.end(); ++inside)
                {
                    // query against the vertex references... and ammend the index...
                    (*inside).reset(new Triangle(vertexReferences[(*inside)->a()].addedVertexIndex_, vertexReferences[(*inside)->b()].addedVertexIndex_, vertexReferences[(*inside)->c()].addedVertexIndex_, (*inside)->ID()));
                }

                vertices_ = newVertices;


            }



            Mesh::Joins Mesh::GetJoins()
            {
                Joins result;

                std::vector<VertexUser> vertexUsers(vertices_.size(), VertexUser());

                // Add the triangle users...
                for (std::list<std::shared_ptr<Triangle>>::iterator outside = outside_.begin(); outside != outside_.end(); ++outside)
                {
                    vertexUsers[(*outside)->a()].users_.push_back(*outside);
                    vertexUsers[(*outside)->b()].users_.push_back(*outside);
                    vertexUsers[(*outside)->c()].users_.push_back(*outside);
                }
                for (std::list<std::shared_ptr<Triangle>>::iterator inside = inside_.begin(); inside != inside_.end(); ++inside)
                {
                    vertexUsers[(*inside)->a()].users_.push_back(*inside);
                    vertexUsers[(*inside)->b()].users_.push_back(*inside);
                    vertexUsers[(*inside)->c()].users_.push_back(*inside);
                }

                // we do this by iterating each triangle...each triangle's adjacent vertex needs to use the same triangle...
                for (std::list<std::shared_ptr<Triangle>>::iterator outside = outside_.begin(); outside != outside_.end(); ++outside)
                {
                    unsigned int joins = 0;

                    const VertexUser& A = vertexUsers[(*outside)->a()];
                    const VertexUser& B = vertexUsers[(*outside)->b()];
                    const VertexUser& C = vertexUsers[(*outside)->c()];

                    joins += HasSameUser(*outside, A, B) ? 1 : 0;
                    joins += HasSameUser(*outside, B, C) ? 1 : 0;
                    joins += HasSameUser(*outside, C, A) ? 1 : 0;

                    result.triangles_[joins].push_back(*outside);
                }
                for (std::list<std::shared_ptr<Triangle>>::iterator inside = inside_.begin(); inside != inside_.end(); ++inside)
                {
                    unsigned int joins = 0;

                    const VertexUser& A = vertexUsers[(*inside)->a()];
                    const VertexUser& B = vertexUsers[(*inside)->b()];
                    const VertexUser& C = vertexUsers[(*inside)->c()];

                    joins += HasSameUser(*inside, A, B) ? 1 : 0;
                    joins += HasSameUser(*inside, B, C) ? 1 : 0;
                    joins += HasSameUser(*inside, C, A) ? 1 : 0;

                    result.triangles_[joins].push_back(*inside);
                }


                return result;
            }


            unsigned int Mesh::RemoveJoinedTriangles(bool noJoins, bool singleJoin, bool doubleJoins, bool tripleJoins)
            {
                // Ensure the vertices are shared...
                ShareVertices();

                // Get the joins...
                Joins joins = GetJoins();

                unsigned int totalTriangleCount = inside_.size() + outside_.size();

                // clear out inside triangles... we only populate the inside...
                inside_.clear();
                outside_.clear();

                // Add our joined triangles back...
                if (!noJoins)
                    inside_.insert(inside_.end(), joins.triangles_[0].begin(), joins.triangles_[0].end());
                if (!singleJoin)
                    inside_.insert(inside_.end(), joins.triangles_[1].begin(), joins.triangles_[1].end());
                if (!doubleJoins)
                    inside_.insert(inside_.end(), joins.triangles_[2].begin(), joins.triangles_[2].end());
                if (!tripleJoins)
                    inside_.insert(inside_.end(), joins.triangles_[3].begin(), joins.triangles_[3].end());

                return totalTriangleCount - inside_.size();
            }

            // Get shared vertices by different triangle ID's...
            std::list<std::pair<unsigned int, std::list<std::shared_ptr<Triangle>>>> Mesh::GetSharedVerticesWithDifferentTriangleIDs()
            {
                std::list<std::pair<unsigned int, std::list<std::shared_ptr<Triangle>>>> result;

                // Get the vertex users....
                std::vector<VertexUser> vertexUsers(vertices_.size(), VertexUser());

                // Add the triangle users...
                for (std::list<std::shared_ptr<Triangle>>::iterator outside = outside_.begin(); outside != outside_.end(); ++outside)
                {
                    vertexUsers[(*outside)->a()].users_.push_back(*outside);
                    vertexUsers[(*outside)->b()].users_.push_back(*outside);
                    vertexUsers[(*outside)->c()].users_.push_back(*outside);
                }
                for (std::list<std::shared_ptr<Triangle>>::iterator inside = inside_.begin(); inside != inside_.end(); ++inside)
                {
                    vertexUsers[(*inside)->a()].users_.push_back(*inside);
                    vertexUsers[(*inside)->b()].users_.push_back(*inside);
                    vertexUsers[(*inside)->c()].users_.push_back(*inside);
                }

                // iterate through all the vertex users-ices... any that have user from 2 or more triangles with different ID;s..
                //for (std::vector<VertexUser>::iterator itr = vertexUsers.begin(); itr != vertexUsers.end(); ++itr)
                for (unsigned int v = 0; v < vertexUsers.size(); ++v)
                {
                    VertexUser& vertexUser = vertexUsers[v];

                    if (vertexUser.users_.empty())
                        continue;

                    // get ID of the first user...
                    unsigned int firstUserID = vertexUser.users_.front()->ID();

                    bool include = false;
                    for (std::list<std::shared_ptr<Triangle>>::iterator tri = vertexUser.users_.begin(); tri != vertexUser.users_.end(); ++tri)
                    {
                        if (firstUserID != (*tri)->ID())
                        {
                            include = true;
                            break;
                        }
                    }

                    if (include)
                    {
                        std::list<std::shared_ptr<Triangle>> vertexUserTriangles;
                        for (std::list<std::shared_ptr<Triangle>>::iterator tri = vertexUser.users_.begin(); tri != vertexUser.users_.end(); ++tri)
                            vertexUserTriangles.push_back(*tri);
                        result.push_back(std::pair<unsigned int, std::list<std::shared_ptr<Triangle>>>(v, vertexUserTriangles));
                    }
                }

                return result;
            }


            namespace
            {
                bool IsApproximatelyEqualWithinTolerance(const SoC::Maths::Vector3& lhs, const SoC::Maths::Vector3& rhs, float tol)
                {
                    return (abs(lhs.x_ - rhs.x_) < tol && abs(lhs.y_ - rhs.y_) < tol && abs(lhs.z_ - rhs.z_) < tol);
                }

                // clamp points...
                void IterateAndClamp(std::vector<VertexReference>& vertexReferences, float tol)
                {
                    VertexReference* referencePoint = &vertexReferences.front();

                    // add the first point and vertex...
                    for (unsigned int vr = 1; vr < vertexReferences.size(); ++vr)
                    {
                        VertexReference& vertRef = vertexReferences[vr];

                        // if the points are approximately the same...
                        if (IsApproximatelyEqualWithinTolerance(vertRef.point_, referencePoint->point_, tol))
                            vertRef.point_ = referencePoint->point_;
                        else
                            referencePoint = &vertexReferences[vr];
                    }
                }

            }





            void Mesh::ClampPoints(float tolerance)
            {
                const SoC::Maths::VectorOperationsInterface& vectorOps = SoC::Maths::GetVectorOperations();

                // create a vector mirroring the mesh vertex list...
                std::vector<VertexReference> vertexReferences;
                vertexReferences.reserve(vertices_.size());
                for (unsigned int mv = 0; mv < vertices_.size(); ++mv)
                    vertexReferences.push_back(VertexReference(vertices_[mv], mv));


                // then sort the vertex reference by point (ascending) x -> y -> z
                std::sort(vertexReferences.begin(), vertexReferences.end(),
                    [](VertexReference& a, VertexReference& b)
                {
                    if (a.point_.x_ == b.point_.x_)
                        if (a.point_.y_ == b.point_.y_)
                            return a.point_.z_ < b.point_.z_;
                    return a.point_.y_ < b.point_.y_;
                    return a.point_.x_ < b.point_.x_;
                });

                // iterate through and clamp close values...
                IterateAndClamp(vertexReferences, tolerance);

                // reorder via z...
                std::sort(vertexReferences.begin(), vertexReferences.end(),
                    [](VertexReference& a, VertexReference& b)
                {
                    if (a.point_.z_ == b.point_.z_)
                        if (a.point_.x_ == b.point_.x_)
                            return a.point_.y_ < b.point_.y_;
                    return a.point_.x_ < b.point_.x_;
                    return a.point_.z_ < b.point_.z_;
                });

                // iterate through and clamp close values...
                IterateAndClamp(vertexReferences, tolerance);

                // reorder via y...
                std::sort(vertexReferences.begin(), vertexReferences.end(),
                    [](VertexReference& a, VertexReference& b)
                {
                    if (a.point_.y_ == b.point_.y_)
                        if (a.point_.z_ == b.point_.z_)
                            return a.point_.x_ < b.point_.x_;
                    return a.point_.z_ < b.point_.z_;
                    return a.point_.y_ < b.point_.y_;
                });

                // iterate through and clamp close values...
                IterateAndClamp(vertexReferences, tolerance);

                // put the vertex references back into our vertices...
                // We then reorder back into original order for the tringles to query and update with the correct reference...
                std::sort(vertexReferences.begin(), vertexReferences.end(),
                    [](VertexReference& a, VertexReference& b)
                {
                    return a.meshVertexIndex_ < b.meshVertexIndex_;
                });

                for (unsigned int v = 0; v < vertexReferences.size(); ++v)
                    vertices_[v] = vertexReferences[v].point_;

            }
        }
    }
}
