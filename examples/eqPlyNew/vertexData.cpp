/*  
    vertexData.cpp
    Copyright (c) 2007, Tobias Wolf <twolf@access.unizh.ch>
    All rights reserved.  
    
    Implementation of the VertexData class.
*/


#include "vertexData.h"
#include "ply.h"
#include <cstdlib>
#include <algorithm>


#undef NDEBUG
#include <cassert>


using namespace std;
using namespace mesh;


/*  Contructor.  */
VertexData::VertexData()
{
    _boundingBox[0] = Vertex( 0.0f );
    _boundingBox[1] = Vertex( 0.0f );
}


/*  Read the vertex and (if available/wanted) color data from the open file.  */
void VertexData::readVertices( PlyFile* file, const int nVertices, 
                               const bool readColors )
{
    // temporary vertex structure for ply loading
    struct _Vertex
    {
        float           x;
        float           y;
        float           z;
        unsigned char   r;
        unsigned char   g;
        unsigned char   b;
    } vertex;

    PlyProperty vertexProps[] = 
    {
        { "x", PLY_FLOAT, PLY_FLOAT, offsetof( _Vertex, x ), 0, 0, 0, 0 },
        { "y", PLY_FLOAT, PLY_FLOAT, offsetof( _Vertex, y ), 0, 0, 0, 0 },
        { "z", PLY_FLOAT, PLY_FLOAT, offsetof( _Vertex, z ), 0, 0, 0, 0 },
        { "red", PLY_UCHAR, PLY_UCHAR, offsetof( _Vertex, r ), 0, 0, 0, 0 },
        { "green", PLY_UCHAR, PLY_UCHAR, offsetof( _Vertex, g ), 0, 0, 0, 0 },
        { "blue", PLY_UCHAR, PLY_UCHAR, offsetof( _Vertex, b ), 0, 0, 0, 0 }
    };
    
    // use all 6 properties when reading colors, only the first 3 otherwise
    int limit = readColors ? 6 : 3;
    for( int i = 0; i < limit; ++i ) 
        ply_get_property( file, "vertex", &vertexProps[i] );
    
    vertices.clear();
    vertices.reserve( nVertices );
    
    if( readColors )
    {
        colors.clear();
        colors.reserve( nVertices );
    }
    
    // read in the vertices, set color to default if none is specified
    for( int i = 0; i < nVertices; ++i )
    {
        ply_get_element( file, static_cast< void* >( &vertex ) );
        vertices.push_back( Vertex( vertex.x, vertex.y, vertex.z ) );
        if( readColors )
            colors.push_back( Color( vertex.r, vertex.g, vertex.b, 0 ) );
    }
}


/*  Read the index data from the open file.  */
void VertexData::readTriangles( PlyFile* file, const int nFaces )
{
    // temporary face structure for ply loading
    struct _Face
    {
        unsigned char   nVertices;
        int*            vertices;
    } face;

    PlyProperty faceProps[] = 
    {
        { "vertex_indices", PLY_INT, PLY_INT, offsetof( _Face, vertices ), 
          1, PLY_UCHAR, PLY_UCHAR, offsetof( _Face, nVertices ) }
    };
    
    ply_get_property( file, "face", &faceProps[0] );
    
    triangles.clear();
    triangles.reserve( nFaces );
    
    // read in the faces, asserting that they are only triangles
    for( int i = 0; i < nFaces; ++i )
    {
        ply_get_element( file, static_cast< void* >( &face ) );
        assert( face.nVertices == 3 && face.vertices != 0 );
        triangles.push_back( Triangle( face.vertices[0], 
                                       face.vertices[1],
                                       face.vertices[2] ) );
        
        // free the memory that was allocated by ply_get_element
        free( face.vertices );
    }
}


/*  Open a PLY file and read vertex, color and index data.  */
bool VertexData::readPlyFile( const char* filename, const bool ignoreColors )
{
    int     nPlyElems;
    char**  elemNames;
    int     fileType;
    float   version;
    bool    result = false;
    
    PlyFile* file = ply_open_for_reading( const_cast< char* >( filename ), 
                                          &nPlyElems, &elemNames, 
                                          &fileType, &version );
    if( !file )
        return result;
    assert( elemNames != 0 );
    
    #ifndef NDEBUG
    cout << filename << ": " << nPlyElems << " elements, file type = " 
         << fileType << ", version = " << version << endl;
    #endif
    
    for( int i = 0; i < nPlyElems; ++i )
    {
        int nElems;
        int nProps;
        
        PlyProperty** props = ply_get_element_description( file, elemNames[i], 
                                                           &nElems, &nProps );
        assert( props != 0 );
        
        #ifndef NDEBUG
        cout << "element " << i << ": name = " << elemNames[i] << ", "
             << nProps << " properties, " << nElems << " elements" << endl;
        for( int j = 0; j < nProps; ++j )
        {
            cout << "element " << i << ", property " << j << ": "
                 << "name = " << props[j]->name << endl;
        }
        #endif
        
        if( equal_strings( elemNames[i], "vertex" ) )
        {
            bool hasColors = false;
            // determine if the file stores vertex colors
            for( int j = 0; j < nProps; ++j )
                if( equal_strings( props[j]->name, "red" ) )
                    hasColors = true;
            
            #ifndef NDEBUG
            if( ignoreColors )
                cout << "colors in file ignored per request" << endl;
            #endif
            
            readVertices( file, nElems, hasColors && !ignoreColors );
            assert( vertices.size() == static_cast< size_t >( nElems ) );
            if( hasColors && !ignoreColors )
                assert( colors.size() == static_cast< size_t >( nElems ) );
        }
        else if( equal_strings( elemNames[i], "face" ) )
        {
            readTriangles( file, nElems );
            assert( triangles.size() == static_cast< size_t >( nElems ) );
            result = true;
        }
        
        // free the memory that was allocated by ply_get_element_description
        for( int j = 0; j < nProps; ++j )
            free( props[j] );
        free( props );
    }
    
    ply_close( file );
    
    // free the memory that was allocated by ply_open_for_reading
    for( int i = 0; i < nPlyElems; ++i )
        free( elemNames[i] );
    free( elemNames );
    
    return result;
}


/*  Calculate the face or vertex normals of the current vertex data.  */
void VertexData::calculateNormals( const bool vertexNormals )
{
    #ifndef NDEBUG
    int wrongNormals = 0;
    #endif
     
    normals.clear();
    if( vertexNormals )
    {
        normals.reserve( vertices.size() );
        
        // initialize all normals to zero
        for( size_t i = 0; i < vertices.size(); ++i )
            normals.push_back( Normal( 0, 0, 0 ) );
    }
    else
        normals.reserve( triangles.size() );
    
    // iterate over all triangles and add their normals to adjacent vertices
    Normal  triangleNormal;
    Index   i0, i1, i2;
    for( size_t i = 0; i < triangles.size(); ++i )
    {
        i0 = triangles[i][0];
        i1 = triangles[i][1];
        i2 = triangles[i][2];
        triangleNormal.normal( vertices[i0], vertices[i1], vertices[i2] );
        
        // count emtpy normals in debug mode
        #ifndef NDEBUG
        if( triangleNormal.length() == 0.0f )
            ++wrongNormals;
        #endif
         
        if( vertexNormals )
        {
            normals[i0] += triangleNormal; 
            normals[i1] += triangleNormal; 
            normals[i2] += triangleNormal;
        }
        else
            normals.push_back( triangleNormal ); 
    }
    
    // normalize all the normals
    if( vertexNormals )
        for( size_t i = 0; i < vertices.size(); ++i )
            normals[i].normalise();
    
    #ifndef NDEBUG
    cout << wrongNormals << " faces had no valid normal" << endl;
    #endif 
}


/*  Calculate the bounding box of the current vertex data.  */
void VertexData::calculateBoundingBox()
{
    _boundingBox[0] = vertices[0];
    _boundingBox[1] = vertices[0];
    for( size_t v = 1; v < vertices.size(); ++v )
        for( size_t i = 0; i < 3; ++i )
        {
            _boundingBox[0][i] = min( _boundingBox[0][i], vertices[v][i] );
            _boundingBox[1][i] = max( _boundingBox[1][i], vertices[v][i] );
        }
}


/*  Scales the data to be within +- baseSize/2 (default 2.0) coordinates.  */
void VertexData::scale( const float baseSize )
{
    // calculate bounding box if not yet done
    if( _boundingBox[0].length() == 0.0f && _boundingBox[1].length() == 0.0f )
        calculateBoundingBox();
    
    // find largest dimension and determine scale factor
    float factor = 0.0f;
    for( size_t i = 0; i < 3; ++i )
        factor = max( factor, _boundingBox[1][i] - _boundingBox[0][i] );
    factor = baseSize / factor;
    
    // determine scale offset
    Vertex offset;
    for( size_t i = 0; i < 3; ++i )
        offset[i] = ( _boundingBox[0][i] + _boundingBox[1][i] ) * 0.5f;
    
    // scale the data
    for( size_t v = 0; v < vertices.size(); ++v )
        for( size_t i = 0; i < 3; ++i )
        {
            vertices[v][i] -= offset[i];
            vertices[v][i] *= factor;
        }
    
    // scale the bounding box
    for( size_t v = 0; v < 2; ++v )
        for( size_t i = 0; i < 3; ++i )
        {
            _boundingBox[v][i] -= offset[i];
            _boundingBox[v][i] *= factor;
        }
}


/*  Helper structure to sort Triangles with standard library sort function.  */
struct _TriangleSort
{
    _TriangleSort( const VertexData& data, const Axis axis ) : _data( data ),
                                                               _axis( axis ) {}
    
    bool operator() ( const Triangle& t1, const Triangle& t2 )
    {
        // references to both triangles' three vertices
        const Vertex& v11 = _data.vertices[ t1[0] ];
        const Vertex& v12 = _data.vertices[ t1[1] ];
        const Vertex& v13 = _data.vertices[ t1[2] ];
        const Vertex& v21 = _data.vertices[ t2[0] ];
        const Vertex& v22 = _data.vertices[ t2[1] ];
        const Vertex& v23 = _data.vertices[ t2[2] ];
        
        // compare first by given axis
        int axis = _axis;
        do
        {
            // within one axis, test the different vertices in succession
            if( v11[axis] != v21[axis] )
                return ( v11[axis] < v21[axis] );
            if( v12[axis] != v22[axis] )
                return ( v12[axis] < v22[axis] );
            if( v13[axis] != v23[axis] )
                return ( v13[axis] < v23[axis] );
            
            // if still equal, move on to the next axis
            axis = ( axis + 1 ) % 3;
        }
        while( axis != _axis );
        
        return false;
    }
    
    const VertexData&   _data;
    const Axis          _axis;
};


/*  Sort the index data from start to start + length along the given axis.  */
void VertexData::sort( const Index start, const Index length, const Axis axis )
{
    assert( start >= 0);
    assert( length > 0 );
    assert( start + length <= triangles.size() );
    
    std::sort( triangles.begin() + start, 
               triangles.begin() + start + length,
               _TriangleSort( *this, axis ) );
}
