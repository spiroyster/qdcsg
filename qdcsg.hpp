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


#include <memory>
#include <list>
#include <vector>
#include <algorithm>

namespace qdcsg
{

	struct vertex { vertex(float x, float y, float z) : x_(x), y_(y), z_(z) {} float x_, y_, z_; };
	struct triangle { triangle(const vertex& a, const vertex& b, const vertex& c) : a_(a), b_(b), c_(c) {} vertex a_, b_, c_; };
	typedef std::vector<triangle> mesh;

	struct mergeResult
	{
		mesh outside_;
		mesh inside_;
	};

	struct halfSpaceClipResult
	{
		mesh triangles_;
		std::vector<vertex> intersectionPoints_;
	};

	namespace impl
	{
		static float magnitude(const vertex& v)
		{
			return sqrt((v.x_ * v.x_) + (v.y_ * v.y_) + (v.z_ * v.z_));
		}

		static vertex unitise(const vertex& v)
		{
			float oneOverMagnitude = 1.0f / magnitude(v);
			return vertex(v.x_ * oneOverMagnitude, v.y_ * oneOverMagnitude, v.z_ * oneOverMagnitude);
		}

		static vertex subtract(const vertex& a, const vertex& b)
		{
			return vertex(a.x_ - b.x_, a.y_ - b.y_, a.z_ - b.z_);
		}

		static vertex cross(const vertex& a, const vertex& b)
		{
			return vertex((a.y_ * b.z_) - (a.z_ * b.y_), (a.z_ * b.x_) - (a.x_ * b.z_), (a.x_ * b.y_) - (a.y_ * b.x_));
		}

		static float dot(const vertex& a, const vertex& b)
		{
			return (a.x_ * b.x_) + (a.y_ * b.y_) + (a.z_ * b.z_);
		}

		static bool equals(const vertex& a, const vertex& b, float e)
		{
			return (abs(a.x_ - b.x_) < e && abs(a.y_ - b.y_) < e && abs(a.z_ - b.z_) < e);
		}

		static bool equals(const vertex& a, const vertex& b)
		{
			return (a.x_ == b.x_ && a.y_ == b.y_ && a.z_ == b.z_);
		}

		static vertex add(const vertex& a, const vertex& b)
		{
			return vertex(a.x_ + b.x_, a.y_ + b.y_, a.z_ + b.z_);
		}

		static float area(const triangle& t)
		{
			vertex ab = subtract(t.b_, t.a_);
			float abMag = magnitude(ab);
			vertex ac = subtract(t.c_, t.a_);
			float acMag = magnitude(ac);

			vertex acUnit = unitise(ac);
			float cosThita = dot(unitise(ab), acUnit);

			// check if points are collinear.
			if (cosThita == 1.0f || cosThita == -1.0f)
				return 0;

			if (std::isnan(cosThita) || !acMag || !abMag)
				return 0;

			vertex d(t.a_.x_ + (acUnit.x_ * cosThita), t.a_.y_ + (acUnit.y_ * cosThita), t.a_.z_ + (acUnit.z_ * cosThita));

			vertex bd = subtract(t.b_, d);
			float bdMag = magnitude(bd);

			return 0.5f * (bdMag * acMag);
		}

		static bool validateTriangle(const triangle& t)
		{
			return area(t) > 0.001;
		}

		static vertex rayPlaneIntersection(const vertex& rP, const vertex& rD, const vertex& pP, const vertex& pN)
		{
			float dotrDpN = dot(rD, pN);
			if (dotrDpN)
			{
				float s = dot(subtract(pP, rP), pN) / dotrDpN;
				return vertex(rP.x_ + (rD.x_ * s), rP.y_ + (rD.y_ * s), rP.z_ + (rD.z_ * s));
			}
			throw std::exception("RayAndPlaneNormalAreParrallel");
		}

		static float directionFromPlane(const vertex& p, const vertex& n, const vertex& v)
		{
			if (equals(v, p))
				return 0;

			vertex d = unitise(subtract(v, p));
			return dot(d, n);
		}

		static vertex calculateNormal(const triangle& t)
		{
			return unitise(cross(subtract(t.c_, t.a_), subtract(t.b_, t.a_)));
		}


		struct splitResult
		{
			std::list<triangle> outside_;
			std::list<triangle> inside_;
		};

		/*namespace debug
		{
			static bool enabled = false;
			static unsigned int triangleCounter = 0;
			static unsigned int nodeCounter = 0;
			static std::string outputVertex(const vertex& v)
			{
				std::ostringstream oss;
				oss << v.x_ << ", " << v.y_ << ", " << v.z_;
				return oss.str();
			}

			static void dump_qdcsg_triangle(VRMLDump::outputFile& vrml, const qdcsg::triangle& t)
			{
				vrml.current_GeomMode(VRMLDump::faces);
				vrml.StartGeom();

				vrml.dump_vec3(t.a_.x_, t.a_.y_, t.a_.z_);
				vrml.dump_vec3(t.b_.x_, t.b_.y_, t.b_.z_);
				vrml.dump_vec3(t.c_.x_, t.c_.y_, t.c_.z_);

				vrml.dump_index(0);
				vrml.dump_index(1);
				vrml.dump_index(2);
				vrml.dump_EndPoly();

				vrml.StopGeom();
			}

			static void dump_qdcsg_mesh(VRMLDump::outputFile& vrml, const qdcsg::mesh& m)
			{
				vrml.current_GeomMode(VRMLDump::faces);
				vrml.StartGeom();

				for (unsigned int t = 0; t < m.size(); ++t)
				{
					vrml.dump_vec3(m[t].a_.x_, m[t].a_.y_, m[t].a_.z_);
					vrml.dump_vec3(m[t].b_.x_, m[t].b_.y_, m[t].b_.z_);
					vrml.dump_vec3(m[t].c_.x_, m[t].c_.y_, m[t].c_.z_);
				}
				for (unsigned int t = 0; t < m.size(); ++t)
				{
					vrml.dump_index((t * 3) + 0);
					vrml.dump_index((t * 3) + 1);
					vrml.dump_index((t * 3) + 2);
				}

				vrml.StopGeom();
			}

			static void dump_qdcsg_result(VRMLDump::outputFile& vrml, const qdcsg::mergeResult& r)
			{
				vrml.current_Colour(1.0f, 0, 0);
				dump_qdcsg_mesh(vrml, r.outside_);
				vrml.current_Colour(0, 1.0f, 0);
				dump_qdcsg_mesh(vrml, r.inside_);
			}


			static void dump_qdcsg_mesh(VRMLDump::outputFile& vrml, const std::list<qdcsg::triangle>& tl)
			{
				vrml.current_GeomMode(VRMLDump::faces);
				vrml.StartGeom();

				for (std::list<qdcsg::triangle>::const_iterator itr = tl.begin(); itr != tl.end(); ++itr)
				{
					vrml.dump_vec3(itr->a_.x_, itr->a_.y_, itr->a_.z_);
					vrml.dump_vec3(itr->b_.x_, itr->b_.y_, itr->b_.z_);
					vrml.dump_vec3(itr->c_.x_, itr->c_.y_, itr->c_.z_);
				}
				for (unsigned int t = 0; t < tl.size(); ++t)
				{
					vrml.dump_index((t * 3) + 0);
					vrml.dump_index((t * 3) + 1);
					vrml.dump_index((t * 3) + 2);
				}

				vrml.StopGeom();
			}

			static void dump_qdcsg_result(VRMLDump::outputFile& vrml, const qdcsg::impl::splitResult& r)
			{
				vrml.current_Colour(1.0f, 0, 0);
				dump_qdcsg_mesh(vrml, r.outside_);
				vrml.current_Colour(0, 1.0f, 0);
				dump_qdcsg_mesh(vrml, r.inside_);
			}
		}*/


		class bsp
		{
		public:

			class node : public triangle
			{
			public:
				node(const triangle& t) : triangle(t.a_, t.b_, t.c_), normal_(calculateNormal(t))
				{
					if (std::isnan(normal_.x_) || std::isnan(normal_.y_) || std::isnan(normal_.z_))
						throw std::exception("Unable to calculate normal correctly.");

				}

				vertex calculateIntersection(const vertex& a, const vertex& b) const
				{
					vertex rayD = unitise(subtract(b, a));
					return rayPlaneIntersection(a, rayD, a_, normal_);
				}

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

				intersectionResult calculateTriangleIntersection(const triangle& t) const
				{
					return intersectionResult(directionFromPlane(a_, normal_, t.a_), directionFromPlane(a_, normal_, t.b_), directionFromPlane(a_, normal_, t.c_));
				}

				const vertex& position() const { return a_; }
				const vertex& normal() const { return normal_; }

				std::unique_ptr<node> inside_;
				std::unique_ptr<node> outside_;

			private:
				vertex normal_;
			};

		private:
			node* followOutsideNode(node* nde, const triangle& tri)
			{
				if (nde->outside_)
					return nde->outside_.get();

				nde->outside_.reset(new node(tri));
				return 0;
			}
			node* followInsideNode(node* nde, const triangle& tri)
			{
				if (nde->inside_)
					return nde->inside_.get();

				nde->inside_.reset(new node(tri));
				return 0;
			}

		public:
			bsp(const mesh& m)
			{
				if (m.empty())
					return;

				// create a working list of the triangles. We validate as we go along.
				std::list<triangle> workingTriangles(m.begin(), m.end());

				// get the first valid triangle...
				while (!validateTriangle(workingTriangles.front()))
					workingTriangles.pop_front();

				// create out node with it...
				root_.reset(new node(workingTriangles.front()));

				while (!workingTriangles.empty())
				{
					// get the next valid working triangle...
					while (!validateTriangle(workingTriangles.front()))
						workingTriangles.pop_front();
					std::list<triangle>::iterator currentTriangle = workingTriangles.begin();

					// traverse the tree to find the location to place this triangle/node.
					node* currentNode = root_.get();

					while (currentNode)
					{

						// calculate the currentTriangle intersection of the currentNode.
						node::intersectionResult intersection = currentNode->calculateTriangleIntersection(*currentTriangle);

						// if the current triangle is outside the current node (triangle)...
						if (intersection.isOutside())
							currentNode = followOutsideNode(currentNode, *currentTriangle);
						// if the current triangle is inside or on the plane of the current node (triangle)...
						else if (intersection.isInside() || intersection.isOnPlane())
							currentNode = followInsideNode(currentNode, *currentTriangle);
						else
						{
							// otherwise the current triangle straddles the current node somehow so we need to split the triangle...
							splitResult splitTriangles = splitTriangle(intersection, currentNode, *currentTriangle);

							// invalid triangles won't be added, so we need to check there is only one triangle in the split...
							unsigned int workingTriangleCount = workingTriangles.size();

							if (splitTriangles.inside_.size() == 1 && splitTriangles.outside_.empty())
								currentNode = followInsideNode(currentNode, *currentTriangle);
							else if (splitTriangles.inside_.empty() && splitTriangles.outside_.size() == 1)
								currentNode = followOutsideNode(currentNode, *currentTriangle);
							else
							{
								// Add both these sets of triangles to our working list for tree traversal...
								workingTriangles.insert(workingTriangles.begin(), splitTriangles.inside_.begin(), splitTriangles.inside_.end());
								workingTriangles.insert(workingTriangles.begin(), splitTriangles.outside_.begin(), splitTriangles.outside_.end());

								// remove this triangle for our working list...
								currentNode = 0;
							}
						}

						// if the currentNode has been set to null, this means one or more working triangles have been added to the result.
						// We need to remove this current triangle from the working triangles list.
						if (!currentNode)
							workingTriangles.erase(currentTriangle);
					}
				}
			}

		protected:
			splitResult splitTriangle(const node::intersectionResult& intersection, const node* nde, const triangle& tri) const
			{
				splitResult result;

				// If there is at least one point on the plane... since this is split, this must mean that the triangle straddles...
				// calculate the single intersection point and the two triangles...
				if (intersection.atLeastOneVertexIsOnPlane())
				{
					// if vertex A is on the plane, intersection point must be along BC...
					if (!intersection.A())
					{
						vertex intersectionPoint = rayPlaneIntersection(tri.b_, unitise(subtract(tri.c_, tri.b_)), nde->position(), nde->normal());
						if (intersection.B() > 0)
						{
							result.outside_.push_back(triangle(tri.a_, tri.b_, intersectionPoint));
							result.inside_.push_back(triangle(intersectionPoint, tri.c_, tri.a_));
						}
						else
						{
							result.inside_.push_back(triangle(tri.a_, tri.b_, intersectionPoint));
							result.outside_.push_back(triangle(intersectionPoint, tri.c_, tri.a_));
						}
					}
					// if vertex B is on the plane, intersection point must be along CA...
					else if (!intersection.B())
					{
						vertex intersectionPoint = rayPlaneIntersection(tri.c_, unitise(subtract(tri.a_, tri.c_)), nde->position(), nde->normal());
						if (intersection.C() > 0)
						{
							result.outside_.push_back(triangle(tri.b_, tri.c_, intersectionPoint));
							result.inside_.push_back(triangle(intersectionPoint, tri.a_, tri.b_));
						}
						else
						{
							result.inside_.push_back(triangle(tri.b_, tri.c_, intersectionPoint));
							result.outside_.push_back(triangle(intersectionPoint, tri.a_, tri.b_));
						}
					}
					// otherwise C must be on the plane, intersection point must be along AB...
					else if (!intersection.C())
					{
						vertex intersectionPoint = rayPlaneIntersection(tri.a_, unitise(subtract(tri.b_, tri.a_)), nde->position(), nde->normal());
						if (intersection.A() > 0)
						{
							result.outside_.push_back(triangle(tri.c_, tri.a_, intersectionPoint));
							result.inside_.push_back(triangle(intersectionPoint, tri.b_, tri.c_));
						}
						else
						{
							result.inside_.push_back(triangle(tri.c_, tri.a_, intersectionPoint));
							result.outside_.push_back(triangle(intersectionPoint, tri.b_, tri.c_));
						}
					}
					else
						throw std::exception("Unable to split triangle");
				}
				else
				{
					// Otherwise this triangle straddles the plane along two edges... resulting in 3 triangles...
					if ((intersection.A() > 0 && intersection.B() > 0) || (intersection.A() < 0 && intersection.B() < 0))
					{
						vertex intersection1 = rayPlaneIntersection(tri.b_, unitise(subtract(tri.c_, tri.b_)), nde->position(), nde->normal());
						vertex intersection2 = rayPlaneIntersection(tri.c_, unitise(subtract(tri.a_, tri.c_)), nde->position(), nde->normal());

						result.outside_.push_back(triangle(tri.a_, tri.b_, intersection1));
						result.inside_.push_back(triangle(intersection1, tri.c_, intersection2));
						result.inside_.push_back(triangle(intersection2, tri.a_, intersection1));
					}
					else if ((intersection.B() > 0 && intersection.C() > 0) || (intersection.B() < 0 && intersection.C() < 0))
					{
						vertex intersection1 = rayPlaneIntersection(tri.c_, unitise(subtract(tri.a_, tri.c_)), nde->position(), nde->normal());
						vertex intersection2 = rayPlaneIntersection(tri.a_, unitise(subtract(tri.b_, tri.a_)), nde->position(), nde->normal());

						result.outside_.push_back(triangle(tri.b_, tri.c_, intersection1));
						result.inside_.push_back(triangle(intersection1, tri.a_, intersection2));
						result.inside_.push_back(triangle(intersection2, tri.b_, intersection1));
					}
					else if ((intersection.A() > 0 && intersection.C() > 0) || (intersection.A() < 0 && intersection.C() < 0))
					{
						vertex intersection1 = rayPlaneIntersection(tri.a_, unitise(subtract(tri.b_, tri.a_)), nde->position(), nde->normal());
						vertex intersection2 = rayPlaneIntersection(tri.b_, unitise(subtract(tri.c_, tri.b_)), nde->position(), nde->normal());

						result.outside_.push_back(triangle(tri.c_, tri.a_, intersection1));
						result.inside_.push_back(triangle(intersection1, tri.b_, intersection2));
						result.inside_.push_back(triangle(intersection2, tri.c_, intersection1));
					}
					else
						throw std::exception("Unable to split triangle");
				}

				for (std::list<triangle>::iterator itr = result.inside_.begin(); itr != result.inside_.end();)
				{
					if (!validateTriangle(*itr))
						itr = result.inside_.erase(itr);
					else
						++itr;
				}
				for (std::list<triangle>::iterator itr = result.outside_.begin(); itr != result.outside_.end();)
				{
					if (!validateTriangle(*itr))
						itr = result.outside_.erase(itr);
					else
						++itr;
				}

				return result;
			}

			std::unique_ptr<node> root_;
		};



		class csg : public bsp
		{
			node* followOutsideNode(node* nde, const triangle& tri, mergeResult& result)
			{
				if (nde->outside_)
					return nde->outside_.get();

				result.outside_.push_back(tri);
				return 0;
			}
			node* followInsideNode(node* nde, const triangle& tri, mergeResult& result)
			{
				if (nde->inside_)
					return nde->inside_.get();

				result.inside_.push_back(tri);
				return 0;
			}

		public:

			csg(const mesh& m) : bsp(m) {}

			std::shared_ptr<mergeResult> merge(const mesh& m)
			{
				std::shared_ptr<mergeResult> result(new mergeResult());

				if (m.empty())
					return result;

				// create a working list of the triangles. We validate as we go along.
				std::list<triangle> workingTriangles(m.begin(), m.end());

				// get the first valid triangle...
				while (!validateTriangle(workingTriangles.front()))
					workingTriangles.pop_front();

				// create out node with it...
				root_.reset(new node(workingTriangles.front()));

				while (!workingTriangles.empty())
				{
					// get the next valid working triangle...
					while (!validateTriangle(workingTriangles.front()))
						workingTriangles.pop_front();
					std::list<triangle>::iterator currentTriangle = workingTriangles.begin();

					// traverse the tree to find the location to place this triangle/node.
					node* currentNode = root_.get();

					while (currentNode)
					{
						// calculate the currentTriangle intersection of the currentNode.
						node::intersectionResult intersection = currentNode->calculateTriangleIntersection(*currentTriangle);

						// if the current triangle is outside the current node (triangle)...
						if (intersection.isOutside())
							currentNode = followOutsideNode(currentNode, *currentTriangle, *result);
						// if the current triangle is inside or on the plane of the current node (triangle)...
						else if (intersection.isInside())
							currentNode = followInsideNode(currentNode, *currentTriangle, *result);
						else if (intersection.isOnPlane())
						{
							// if the current triangle is coplanar to node... we need to perform coplanar split on the triangle...
							splitResult splitTriangles = splitCoplanar(currentNode, *currentTriangle);

							// since all inside triangles are effectively within the node triangle...we add them to outside
							result->outside_.insert(result->outside_.end(), splitTriangles.inside_.begin(), splitTriangles.inside_.end());
							currentNode = 0;

							// all the outside triangles need to be checked against the next 'inside' triangle node...
							// we only keep checking if there is a single outside triangle... 
							if (splitTriangles.outside_.size() == 1)
							{
								// check if the next inside is coplanar...
								if (currentNode->inside_)
								{
									if (equals(calculateNormal(splitTriangles.outside_.front()), currentNode->inside_.get()->normal()))
										currentNode = currentNode->inside_.get();
									else
										result->outside_.push_back(*currentTriangle);
								}
								// otherwise triagle is outside...
								else
									result->outside_.push_back(*currentTriangle);
							}
							else
								workingTriangles.insert(workingTriangles.end(), splitTriangles.outside_.begin(), splitTriangles.outside_.end());
						}
						else
						{
							// otherwise the current triangle straddles the current node somehow so we need to split the triangle...
							splitResult splitTriangles = splitTriangle(intersection, currentNode, *currentTriangle);

							// invalid triangles won't be added, so we need to check there is only one triangle in the split...
							if (splitTriangles.inside_.size() == 1 && splitTriangles.outside_.empty())
								currentNode = followInsideNode(currentNode, *currentTriangle, *result);
							else if (splitTriangles.outside_.empty() && splitTriangles.outside_.size() == 1)
								currentNode = followOutsideNode(currentNode, *currentTriangle, *result);
							else
							{
								// Add both these sets of triangles to our working list for tree traversal...
								workingTriangles.insert(workingTriangles.end(), splitTriangles.inside_.begin(), splitTriangles.inside_.end());
								workingTriangles.insert(workingTriangles.end(), splitTriangles.outside_.begin(), splitTriangles.outside_.end());

								// remove this triangle for our working list...
								currentNode = 0;
							}
						}

						// if the currentNode has been set to null, this means one or more working triangles have been added to the result.
						// We need to remove this current triangle from the working triangles list.
						if (!currentNode)
							workingTriangles.erase(currentTriangle);

					}
				}

				return result;
			}

		private:

			splitResult splitCoplanar(const node* nde, const triangle& tri) const
			{
				splitResult result;

				std::list<triangle> abTrianglesToTest, bcTrianglesToTest, caTrianglesToTest;

				// check triangle against node AB.
				{
					node edge(triangle(nde->a_, subtract(nde->b_, nde->a_), add(nde->b_, nde->normal())));
					while (!abTrianglesToTest.empty())
					{
						node::intersectionResult nodeEdgeIntersection = edge.calculateTriangleIntersection(tri);

						// if triangle is entirly outside... triangle is outside.
						if (nodeEdgeIntersection.isOutside())
							result.outside_.push_back(tri);
						else
						{
							// otherwise we need to split...
							splitResult split = splitTriangle(nodeEdgeIntersection, &edge, tri);

							result.outside_.insert(result.outside_.end(), split.outside_.begin(), split.outside_.end());
							bcTrianglesToTest = result.inside_;
						}

						abTrianglesToTest.pop_front();
					}
				}

				// check triangle against node BC.
				{
					node edge(triangle(nde->b_, subtract(nde->c_, nde->b_), add(nde->c_, nde->normal())));
					while (!bcTrianglesToTest.empty())
					{
						node::intersectionResult nodeEdgeIntersection = edge.calculateTriangleIntersection(tri);

						// if triangle is entirly outside... triangle is outside.
						if (nodeEdgeIntersection.isOutside())
							result.outside_.push_back(tri);
						else
						{
							// otherwise we need to split...
							splitResult split = splitTriangle(nodeEdgeIntersection, &edge, tri);

							result.outside_.insert(result.outside_.end(), split.outside_.begin(), split.outside_.end());
							caTrianglesToTest = result.inside_;
						}

						bcTrianglesToTest.pop_front();
					}
				}

				// check triangle against node CA.
				{
					node edge(triangle(nde->c_, subtract(nde->a_, nde->c_), add(nde->a_, nde->normal())));
					while (!caTrianglesToTest.empty())
					{
						node::intersectionResult nodeEdgeIntersection = edge.calculateTriangleIntersection(tri);

						// if triangle is entirly outside... triangle is outside.
						if (nodeEdgeIntersection.isOutside())
							result.outside_.push_back(tri);
						else
						{
							// otherwise we need to split...
							splitResult split = splitTriangle(nodeEdgeIntersection, &edge, tri);

							result.outside_.insert(result.outside_.end(), split.outside_.begin(), split.outside_.end());
							result.inside_.insert(result.inside_.end(), split.inside_.begin(), split.inside_.end());
						}

						caTrianglesToTest.pop_front();
					}
				}

				// if there are only inside triangles, the entire triangle is inside...
				if (result.outside_.empty())
					result.inside_ = std::list<triangle>({ tri });
				else if (result.inside_.empty())
					result.outside_ = std::list<triangle>({ tri });

				// validate the triangles...
				for (std::list<triangle>::iterator itr = result.inside_.begin(); itr != result.inside_.end();)
				{
					if (!validateTriangle(*itr))
						itr = result.inside_.erase(itr);
					else
						++itr;
				}
				for (std::list<triangle>::iterator itr = result.outside_.begin(); itr != result.outside_.end();)
				{
					if (!validateTriangle(*itr))
						itr = result.outside_.erase(itr);
					else
						++itr;
				}

				return result;
			}
		};




	}	// namespace impl

	// difference
	static std::shared_ptr<mesh> Difference(const mesh& a, const mesh& b)
	{
		std::shared_ptr<mesh> result(new mesh());

		std::shared_ptr<mergeResult> B = impl::csg(a).merge(b);
		std::shared_ptr<mergeResult> A = impl::csg(b).merge(a);

		// result is all the outside of A...
		result->insert(result->end(), A->outside_.begin(), A->outside_.end());

		// and all the inside of B flipped...
		std::for_each(B->inside_.begin(), B->inside_.end(),
			[](triangle& t)
		{
			vertex c = t.c_;
			t.c_ = t.a_;
			t.a_ = c;
		});
		result->insert(result->end(), B->inside_.begin(), B->inside_.end());

		return result;
	}

	// union
	static std::shared_ptr<mesh> Union(const mesh& a, const mesh& b)
	{
		std::shared_ptr<mesh> result(new mesh());

		std::shared_ptr<mergeResult> B = impl::csg(a).merge(b);
		std::shared_ptr<mergeResult> A = impl::csg(b).merge(a);

		// result is all the outside of A...
		result->insert(result->end(), A->outside_.begin(), A->outside_.end());

		// and all the outside of B...
		result->insert(result->end(), B->outside_.begin(), B->outside_.end());

		return result;
	}

	// intersection
	static std::shared_ptr<mesh> Intersection(const mesh& a, const mesh& b)
	{
		std::shared_ptr<mesh> result(new mesh());

		std::shared_ptr<mergeResult> B = impl::csg(a).merge(b);
		std::shared_ptr<mergeResult> A = impl::csg(b).merge(a);

		// result is all the inside of A...
		result->insert(result->end(), A->inside_.begin(), A->inside_.end());

		// and all the insdide of B...
		result->insert(result->end(), B->inside_.begin(), B->inside_.end());

		return result;
	}

	// half space clip

}

#endif // QDCSG_HPP

