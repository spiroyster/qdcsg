// VRML dump v2. Cordell's Useful Tools
// This helper allows 3D data to be dumped to VRML 2.0. It is limited
// by the restrictions of VRML for the output.
#ifndef __VRMLDUMP__
#define __VRMLDUMP__

#include <iostream>			// needed for 
#include <fstream>			// needed for file writing
#include <vector>

namespace VRMLDump 
{		// encapsulate our appl into the VRMLDump namespace
	enum geom_mode { points, lines, faces }; 
	static unsigned int count = 0;	// counter used externally for file names etc

class outputFile
{
	std::ofstream		m_file;
	geom_mode		m_mode;
	float			m_diffuse[4];
    float			m_emissive[4];
	std::vector<int>		m_indexes;
	int				m_geomAdded;
	void __PointSet() 
		{ m_file << "geometry PointSet {\n"; m_file << "coord Coordinate {\n"; m_file << "point [\n";}
	void __LineSet() 
		{ m_file << "geometry IndexedLineSet {\n"; m_file << "coord Coordinate {\n"; m_file << "point [\n"; }
	void __FaceSet() 
		{ m_file << "geometry IndexedFaceSet {\n"; m_file << "coord Coordinate {\n"; m_file << "point [\n"; }
	void __ShapeNode() 
		{
			m_file << "Shape {\n";
			m_file << "appearance Appearance {\n";
			m_file << "material Material {\n";
			m_file << "ambientIntensity 0.5000000\n";
			m_file << "diffuseColor " << m_diffuse[0] << " " << m_diffuse[1] << " " << m_diffuse[2] << '\n';
			m_file << "emissiveColor " << m_emissive[0] << " " << m_emissive[1] << " " << m_emissive[2] << '\n';
			m_file << "shininess 0.5000000\n";
			m_file << "specularColor 0.5647059 0.5647059 0.5647059\n";
			m_file << "transparency " << 1.0 - m_diffuse[3] << '\n';
			m_file << "}\n";
			m_file << "}\n";
			switch ( m_mode )
			{
			case VRMLDump::points:
				__PointSet(); return;
			case VRMLDump::lines:
				__LineSet(); return;
			case VRMLDump::faces:
				__FaceSet(); return;
			default:
				__PointSet(); return;
			}
		}
	void __CloseNode() 
		{ m_file << "}\n"; }
public:
	// VRML dump object... create one of these, and call the various methods for dumping data

	outputFile() : m_mode( VRMLDump::points ), m_file("c:/dump/dump.wrl"), m_geomAdded(0) 
		{ m_file << "#VRML V2.0 utf8\n"; current_Colour(); }

	outputFile( const char* name ) : m_mode( VRMLDump::points ), m_file(name), m_geomAdded(0) 
		{ m_file << "#VRML V2.0 utf8\n"; current_Colour(); }
	
	// start buffering geometry...
	void StartGeom()
		{
			m_indexes.clear();
			m_geomAdded = 0;
			__ShapeNode();
		}
	
	// stop buffering geometry...
	void StopGeom()
		{
			m_file << "]\n";
			__CloseNode();		// coord
			if ( m_mode == VRMLDump::lines || m_mode == VRMLDump::faces )
			{
				m_file << "coordIndex [\n";
				unsigned int size = static_cast<unsigned int>(m_indexes.size());
				if ( size > 0 )
					for ( unsigned int i = 0; i < size; ++i ) { m_file << m_indexes[i] << ",\n"; }
				else
					for ( unsigned int i = 0; i < static_cast<unsigned int>( m_geomAdded ); ++i ) { m_file << i << ",\n"; }				
				m_file << "]\n";
			}
			__CloseNode();		// geometry
			__CloseNode();		// shape
		}
	
	// Dump methods...
	void dump_index( const int index ) { m_indexes.push_back( index ); }
	void dump_vec3( const double x, const double y, const double z )
		{ m_file << x << " " << y << " " << z << ",\n"; ++m_geomAdded; }
	void dump_vec2( const double x, const double y )
		{ m_file << x << " " << y << ",\n"; ++m_geomAdded; }
	void dump_EndPoly() { m_indexes.push_back( -1 ); }
	
	// material/mode accessors/mutators
	const VRMLDump::geom_mode current_GeomMode() const { return m_mode; }
	void current_GeomMode( VRMLDump::geom_mode mode ) { m_mode = mode; }
    void current_Diffuse(float r, float g, float b, float a = 1.0f) { m_diffuse[0] = r; m_diffuse[1] = g; m_diffuse[2] = b; m_diffuse[3] = a; }
    void current_Emissive(float r, float g, float b, float a = 1.0f) { m_emissive[0] = r; m_emissive[1] = g; m_emissive[2] = b; m_emissive[3] = a; }
    void current_Colour() { current_Colour(0.5, 0.5, 0.5, 1.0); }
    void current_Colour(float r, float g, float b, float a = 1.0f) { m_emissive[0] = r; m_emissive[1] = g; m_emissive[2] = b; m_emissive[3] = a; m_diffuse[0] = r; m_diffuse[1] = g; m_diffuse[2] = b; m_diffuse[3] = a; }

	// CVector3D...
#ifdef CVECTOR3D_INCLUDED
	void dump_CVector3D( const Shoemaster::CVector3D& v )
		{ dump_vec3( v.x, v.y, v.z ); }
#endif

	// CVector2D...
#ifdef CVECTOR2D_INCLUDED
	void dump_CVector2D( const Shoemaster::CVector2D& v )
		{ dump_vec3( v.x, v.y, 0.0 ); }
#endif

#ifdef CVECTOR2DARRAY_INCLUDED
	void dump_CVector2DArray( const Shoemaster::CVector2DArray& va )
		{
			StartGeom();
			for ( unsigned int i = 0; i < va.size(); ++i )
				dump_CVector2D( va[i] );
			StopGeom();
		}
#endif

#ifdef sh_ARRAY_T_INCLUDED

	void dump_Point2_t( const point2_t& p )
		{ dump_vec3( static_cast<float>( p[0] ), static_cast<float>( p[1] ), 0.0 ); }
	
	void dump_Point3_t( const point3_t& p )
		{ dump_vec3( static_cast<float>( p[0] ), static_cast<float>( p[1] ), static_cast<float>( p[2] ) ); }

	void dump_Point2_array_t( const point2_array_t& a )
		{
			StartGeom();
			int a_size = a.used;
			for ( int i = 0; i < a_size; ++i )
				dump_Point2_t( a.array[i] );
			StopGeom();
		}
	void dump_Point3_array_t( const point3_array_t& a )
		{
			StartGeom();
			int a_size = a.used;
			for ( int i = 0; i < a_size; ++i )
				dump_Point3_t( a.array[i] );
			StopGeom();
		}
#endif

#ifdef CSURFACE_INCLUDED
	
	void dump_CSurface( Shoemaster::CSurface* surface )
	{
		current_GeomMode( VRMLDump::faces );
		StartGeom();

		for ( unsigned int i = 0; i < surface->GetNumberOfPoints(); ++i )
			dump_CVector3D( surface->GetPoint(i) );

		for ( unsigned int j = 0; j < surface->GetNumberOfTriangles(); ++j )
		{
			const Shoemaster::CSurface::Triangle& tri = surface->GetTriangle(j);
			
			const Shoemaster::CSurface::Vertex& v1 = surface->GetVertex( tri.vertex1 );
			const Shoemaster::CSurface::Vertex& v2 = surface->GetVertex( tri.vertex2 );
			const Shoemaster::CSurface::Vertex& v3 = surface->GetVertex( tri.vertex3 );

			dump_index( v1.point );
			dump_index( v2.point );
			dump_index( v3.point );
			dump_EndPoly();						
		}
		StopGeom();
	}

#endif

#ifdef CTRIANGULATEDSURFACE_INCLUDED
	
	void dump_CTriangulatedSurface( Shoemaster::CTriangulatedSurface* surface, bool faces = false )
	{
		current_GeomMode( faces ? VRMLDump::faces : VRMLDump::lines );
		StartGeom();

		for ( unsigned int i = 0; i < surface->GetNumberOfVertices(); ++i )
			dump_CVector3D( surface->GetVertex(i).point );

		for ( unsigned int j = 0; j < surface->GetNumberOfTriangles(); ++j )
		{
			const Shoemaster::CTriangulatedSurface::Triangle& tri = surface->GetTriangle(j);
			dump_index( tri.v1 );
			dump_index( tri.v2 );
			dump_index( tri.v3 );
			if ( !faces )
				dump_index( tri.v1 );
			dump_EndPoly();						
		}
		StopGeom();
	}

#endif

#ifdef SoC_GEOMETRY_MESH_HPP
    
    void dump_SoCMesh(const SoC::Geometry::Mesh& mesh, bool faces = false)
    {
        current_GeomMode(faces ? VRMLDump::faces : VRMLDump::lines);
        StartGeom();

        for (unsigned int i = 0; i < mesh.GetVertices().size(); ++i)
        {
            const SoC::Maths::Vector3& v3 = mesh.GetPoints()[mesh.GetVertices()[i].p_];
            dump_vec3(v3.x_, v3.y_, v3.z_);
        }

        for (unsigned int j = 0; j < mesh.GetTriangles().size(); ++j)
        {
            //const Shoemaster::CTriangulatedSurface::Triangle& tri = surface->GetTriangle(j);
            const SoC::Geometry::TriangleInterface::Triangle& tri = mesh.GetTriangles()[j];
            dump_index(mesh.GetVertices()[tri.a_].p_);
            dump_index(mesh.GetVertices()[tri.b_].p_);
            dump_index(mesh.GetVertices()[tri.c_].p_);
            if ( !faces )
                dump_index(mesh.GetVertices()[tri.a_].p_);
            dump_EndPoly();
        }
        StopGeom();
    }

#endif 

    void dump_line(float x1, float y1, float z1, float x2, float y2, float z2)
    {
        current_GeomMode(VRMLDump::lines);
        StartGeom();
        dump_vec3(x1, y1, z1);
        dump_vec3(x2, y2, z2);
        StopGeom();
    }

    void dump_triangle(float ax, float ay, float az, float bx, float by, float bz, float cx, float cy, float cz)
    {
        current_GeomMode(VRMLDump::faces);
        StartGeom();
        dump_vec3(ax, ay, az);
        dump_vec3(bx, by, bz);
        dump_vec3(cx, cy, cz);        
        StopGeom();
    }

};		// outputFile
}		// VRMLDump
#endif  // __VRMLDUMP__