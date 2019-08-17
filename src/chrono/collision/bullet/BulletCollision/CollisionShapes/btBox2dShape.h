/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef OBB_BOX_2D_SHAPE_H
#define OBB_BOX_2D_SHAPE_H

#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMinMax.h"

///The btBox2dShape is a box primitive around the origin, its sides axis aligned with length specified by half extents, in local shape coordinates. When used as part of a btCollisionObject or btRigidBody it will be an oriented box in world space.
class btBox2dShape: public btPolyhedralConvexShape
{

	//btVector3	m_boxHalfExtents1; //use m_implicitShapeDimensions instead

	btVector3 m_centroid;
	btVector3 m_vertices[4];
	btVector3 m_normals[4];

public:

	btVector3 getHalfExtentsWithMargin() const
	{
		btVector3 halfExtents = getHalfExtentsWithoutMargin();
		btVector3 margin(getMargin(),getMargin(),getMargin());
		halfExtents += margin;
		return halfExtents;
	}
	
	const btVector3& getHalfExtentsWithoutMargin() const
	{
		return m_implicitShapeDimensions;//changed in Bullet 2.63: assume the scaling and margin are included
	}
	

	btVector3	localGetSupportingVertex(const btVector3& vec) const override
	{
		btVector3 halfExtents = getHalfExtentsWithoutMargin();
		btVector3 margin(getMargin(),getMargin(),getMargin());
		halfExtents += margin;
		
		return btVector3(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
			btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
			btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
	}

	SIMD_FORCE_INLINE  btVector3	localGetSupportingVertexWithoutMargin(const btVector3& vec)const override
	{
		const btVector3& halfExtents = getHalfExtentsWithoutMargin();
		
		return btVector3(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
			btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
			btFsels(vec.z(), halfExtents.z(), -halfExtents.z()));
	}

	void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const override
	{
		const btVector3& halfExtents = getHalfExtentsWithoutMargin();
	
		for (int i=0;i<numVectors;i++)
		{
			const btVector3& vec = vectors[i];
			supportVerticesOut[i].setValue(btFsels(vec.x(), halfExtents.x(), -halfExtents.x()),
				btFsels(vec.y(), halfExtents.y(), -halfExtents.y()),
				btFsels(vec.z(), halfExtents.z(), -halfExtents.z())); 
		}

	}


	btBox2dShape( const btVector3& boxHalfExtents) 
		: btPolyhedralConvexShape(),
		m_centroid(0,0,0)
	{
		m_vertices[0].setValue(-boxHalfExtents.getX(),-boxHalfExtents.getY(),0);
		m_vertices[1].setValue(boxHalfExtents.getX(),-boxHalfExtents.getY(),0);
		m_vertices[2].setValue(boxHalfExtents.getX(),boxHalfExtents.getY(),0);
		m_vertices[3].setValue(-boxHalfExtents.getX(),boxHalfExtents.getY(),0);

		m_normals[0].setValue(0,-1,0);
		m_normals[1].setValue(1,0,0);
		m_normals[2].setValue(0,1,0);
		m_normals[3].setValue(-1,0,0);

		m_shapeType = BOX_2D_SHAPE_PROXYTYPE;
		btVector3 margin(getMargin(),getMargin(),getMargin());
		m_implicitShapeDimensions = (boxHalfExtents * m_localScaling) - margin;
	};

	void setMargin(btScalar collisionMargin) override
	{
		//correct the m_implicitShapeDimensions for the margin
		btVector3 oldMargin(getMargin(),getMargin(),getMargin());
		btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		
		btConvexInternalShape::setMargin(collisionMargin);
		btVector3 newMargin(getMargin(),getMargin(),getMargin());
		m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;

	}
	void	setLocalScaling(const btVector3& scaling) override
	{
		btVector3 oldMargin(getMargin(),getMargin(),getMargin());
		btVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		btVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		btConvexInternalShape::setLocalScaling(scaling);

		m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;

	}

	void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const override;

	

	void	calculateLocalInertia(btScalar mass,btVector3& inertia) const override;





	int	getVertexCount() const
	{
		return 4;
	}

	int getNumVertices()const override
	{
		return 4;
	}

	const btVector3* getVertices() const
	{
		return &m_vertices[0];
	}

	const btVector3* getNormals() const
	{
		return &m_normals[0];
	}







	void getPlane(btVector3& planeNormal,btVector3& planeSupport,int i ) const override
	{
		//this plane might not be aligned...
		btVector4 plane ;
		getPlaneEquation(plane,i);
		planeNormal = btVector3(plane.getX(),plane.getY(),plane.getZ());
		planeSupport = localGetSupportingVertex(-planeNormal);
	}


	const btVector3& getCentroid() const
	{
		return m_centroid;
	}
	
	int getNumPlanes() const override
	{
		return 6;
	}	
	
	

	int getNumEdges() const override
	{
		return 12;
	}


	void getVertex(int i,btVector3& vtx) const override
	{
		btVector3 halfExtents = getHalfExtentsWithoutMargin();

		vtx = btVector3(
				halfExtents.x() * (1-(i&1)) - halfExtents.x() * (i&1),
				halfExtents.y() * (1-((i&2)>>1)) - halfExtents.y() * ((i&2)>>1),
				halfExtents.z() * (1-((i&4)>>2)) - halfExtents.z() * ((i&4)>>2));
	}
	

	virtual void	getPlaneEquation(btVector4& plane,int i) const
	{
		btVector3 halfExtents = getHalfExtentsWithoutMargin();

		switch (i)
		{
		case 0:
			plane.setValue(btScalar(1.),btScalar(0.),btScalar(0.),-halfExtents.x());
			break;
		case 1:
			plane.setValue(btScalar(-1.),btScalar(0.),btScalar(0.),-halfExtents.x());
			break;
		case 2:
			plane.setValue(btScalar(0.),btScalar(1.),btScalar(0.),-halfExtents.y());
			break;
		case 3:
			plane.setValue(btScalar(0.),btScalar(-1.),btScalar(0.),-halfExtents.y());
			break;
		case 4:
			plane.setValue(btScalar(0.),btScalar(0.),btScalar(1.),-halfExtents.z());
			break;
		case 5:
			plane.setValue(btScalar(0.),btScalar(0.),btScalar(-1.),-halfExtents.z());
			break;
		default:
			btAssert(0);
		}
	}

	
	void getEdge(int i,btVector3& pa,btVector3& pb) const override
	//virtual void getEdge(int i,Edge& edge) const
	{
		int edgeVert0 = 0;
		int edgeVert1 = 0;

		switch (i)
		{
		case 0:
				edgeVert0 = 0;
				edgeVert1 = 1;
			break;
		case 1:
				edgeVert0 = 0;
				edgeVert1 = 2;
			break;
		case 2:
			edgeVert0 = 1;
			edgeVert1 = 3;

			break;
		case 3:
			edgeVert0 = 2;
			edgeVert1 = 3;
			break;
		case 4:
			edgeVert0 = 0;
			edgeVert1 = 4;
			break;
		case 5:
			edgeVert0 = 1;
			edgeVert1 = 5;

			break;
		case 6:
			edgeVert0 = 2;
			edgeVert1 = 6;
			break;
		case 7:
			edgeVert0 = 3;
			edgeVert1 = 7;
			break;
		case 8:
			edgeVert0 = 4;
			edgeVert1 = 5;
			break;
		case 9:
			edgeVert0 = 4;
			edgeVert1 = 6;
			break;
		case 10:
			edgeVert0 = 5;
			edgeVert1 = 7;
			break;
		case 11:
			edgeVert0 = 6;
			edgeVert1 = 7;
			break;
		default:
			btAssert(0);

		}

		getVertex(edgeVert0,pa );
		getVertex(edgeVert1,pb );
	}




	
		bool isInside(const btVector3& pt,btScalar tolerance) const override
	{
		btVector3 halfExtents = getHalfExtentsWithoutMargin();

		//btScalar minDist = 2*tolerance;
		
		bool result =	(pt.x() <= (halfExtents.x()+tolerance)) &&
						(pt.x() >= (-halfExtents.x()-tolerance)) &&
						(pt.y() <= (halfExtents.y()+tolerance)) &&
						(pt.y() >= (-halfExtents.y()-tolerance)) &&
						(pt.z() <= (halfExtents.z()+tolerance)) &&
						(pt.z() >= (-halfExtents.z()-tolerance));
		
		return result;
	}


	//debugging
	const char*	getName()const override
	{
		return "Box2d";
	}

	int		getNumPreferredPenetrationDirections() const override
	{
		return 6;
	}
	
	void	getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const override
	{
		switch (index)
		{
		case 0:
			penetrationVector.setValue(btScalar(1.),btScalar(0.),btScalar(0.));
			break;
		case 1:
			penetrationVector.setValue(btScalar(-1.),btScalar(0.),btScalar(0.));
			break;
		case 2:
			penetrationVector.setValue(btScalar(0.),btScalar(1.),btScalar(0.));
			break;
		case 3:
			penetrationVector.setValue(btScalar(0.),btScalar(-1.),btScalar(0.));
			break;
		case 4:
			penetrationVector.setValue(btScalar(0.),btScalar(0.),btScalar(1.));
			break;
		case 5:
			penetrationVector.setValue(btScalar(0.),btScalar(0.),btScalar(-1.));
			break;
		default:
			btAssert(0);
		}
	}

};

#endif //OBB_BOX_2D_SHAPE_H


