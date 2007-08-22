
/* Copyright (c) 2006, Stefan Eilemann <eile@equalizergraphics.com> 
   All rights reserved. */

#ifndef EQ_VOL_NODE_H
#define EQ_VOL_NODE_H

#include "eqVol.h"
#include "initData.h"

#include <eq/eq.h>

namespace eqVol
{
	using namespace eq;
	
    class Node : public eq::Node
    {
    public:
        const InitData& getInitData() const { return _initData; }
        const Model*    getModel() const    { return _model; }

		virtual eq::FrameData* getFrameData( const uint32_t id, const uint32_t version );


    protected:
        virtual ~Node(){}

        virtual bool configInit( const uint32_t initID );
        virtual bool configExit();

    private:
 		
		InitData _initData;
        Model*   _model;
    };
}

#endif // EQ_VOL_NODE_H
