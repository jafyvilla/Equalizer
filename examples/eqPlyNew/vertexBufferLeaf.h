/*  
    vertexBufferLeaf.h
    Copyright (c) 2007, Tobias Wolf <twolf@access.unizh.ch>
    All rights reserved.  
    
    Header file of the VertexBufferLeaf class.
*/


#ifndef MESH_VERTEXBUFFERLEAF_H
#define MESH_VERTEXBUFFERLEAF_H


#include "vertexBufferBase.h"


namespace mesh 
{
    
    
    /*  The class for kd-tree leaf nodes.  */
    class VertexBufferLeaf : public VertexBufferBase
    {
    public:
        VertexBufferLeaf( VertexBufferData& data) : _globalData( data ),
                                                    _vertexStart( 0 ), 
                                                    _indexStart( 0 ),
                                                    _indexLength( 0 ) {}
        virtual ~VertexBufferLeaf() {}
        virtual void render( VertexBufferState& state, 
                             bool performCulling = true ) const;
        
    protected:
        virtual void toStream( std::ostream& os );
        virtual void fromMemory( char** addr, VertexBufferData& globalData );
        virtual void setupTree( VertexData& data, const Index start,
                                const Index length, const Axis axis,
                                VertexBufferData& globalData );
        virtual BoundingBox updateBoundingSphere();
        virtual void updateRange();
        
    private:
        void renderImmediate( VertexBufferState& state ) const;
        void renderDisplayList( VertexBufferState& state ) const;
        void renderVertexArray( VertexBufferState& state ) const;
        void renderBufferObject( VertexBufferState& state ) const;
        
        VertexBufferData&   _globalData;
        Index               _vertexStart;
        ShortIndex          _vertexLength;
        Index               _indexStart;
        Index               _indexLength;
    };
    
    
}


#endif // MESH_VERTEXBUFFERLEAF_H
