/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef OBB_TRIANGLE_MINKOWSKI_H
#define OBB_TRIANGLE_MINKOWSKI_H

#include "btConvexShape.h"
#include "btBoxShape.h"

ATTRIBUTE_ALIGNED16(class) btTriangleShape : public btPolyhedralConvexShape
{


public:

	btVector3	m_vertices1[3];

	int getNumVertices() const override
	{
		return 3;
	}

	btVector3& getVertexPtr(int index)
	{
		return m_vertices1[index];
	}

	const btVector3& getVertexPtr(int index) const
	{
		return m_vertices1[index];
	}
	void getVertex(int index,btVector3& vert) const override
	{
		vert = m_vertices1[index];
	}

	int getNumEdges() const override
	{
		return 3;
	}
	
	void getEdge(int i,btVector3& pa,btVector3& pb) const override
	{
		getVertex(i,pa);
		getVertex((i+1)%3,pb);
	}


	void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax)const override 
	{
//		btAssert(0);
		getAabbSlow(t,aabbMin,aabbMax);
	}

	btVector3 localGetSupportingVertexWithoutMargin(const btVector3& dir)const override 
	{
		btVector3 dots(dir.dot(m_vertices1[0]), dir.dot(m_vertices1[1]), dir.dot(m_vertices1[2]));
	  	return m_vertices1[dots.maxAxis()];

	}

	void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const override
	{
		for (int i=0;i<numVectors;i++)
		{
			const btVector3& dir = vectors[i];
			btVector3 dots(dir.dot(m_vertices1[0]), dir.dot(m_vertices1[1]), dir.dot(m_vertices1[2]));
  			supportVerticesOut[i] = m_vertices1[dots.maxAxis()];
		}

	}

	btTriangleShape() : btPolyhedralConvexShape ()
    {
		m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
	}

	btTriangleShape(const btVector3& p0,const btVector3& p1,const btVector3& p2) : btPolyhedralConvexShape ()
    {
		m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
        m_vertices1[0] = p0;
        m_vertices1[1] = p1;
        m_vertices1[2] = p2;
    }


	void getPlane(btVector3& planeNormal,btVector3& planeSupport,int i) const override
	{
		getPlaneEquation(i,planeNormal,planeSupport);
	}

	int	getNumPlanes() const override
	{
		return 1;
	}

	void calcNormal(btVector3& normal) const
	{
		normal = (m_vertices1[1]-m_vertices1[0]).cross(m_vertices1[2]-m_vertices1[0]);
		normal.normalize();
	}

	virtual void getPlaneEquation(int i, btVector3& planeNormal,btVector3& planeSupport) const
	{
		(void)i;
		calcNormal(planeNormal);
		planeSupport = m_vertices1[0];
	}

	void	calculateLocalInertia(btScalar mass,btVector3& inertia) const override
	{
		(void)mass;
		btAssert(0);
		inertia.setValue(btScalar(0.),btScalar(0.),btScalar(0.));
	}

			bool isInside(const btVector3& pt,btScalar tolerance) const override
	{
		btVector3 normal;
		calcNormal(normal);
		//distance to plane
		btScalar dist = pt.dot(normal);
		btScalar planeconst = m_vertices1[0].dot(normal);
		dist -= planeconst;
		if (dist >= -tolerance && dist <= tolerance)
		{
			//inside check on edge-planes
			int i;
			for (i=0;i<3;i++)
			{
				btVector3 pa,pb;
				getEdge(i,pa,pb);
				btVector3 edge = pb-pa;
				btVector3 edgeNormal = edge.cross(normal);
				edgeNormal.normalize();
				btScalar dist = pt.dot( edgeNormal);
				btScalar edgeConst = pa.dot(edgeNormal);
				dist -= edgeConst;
				if (dist < -tolerance)
					return false;
			}
			
			return true;
		}

		return false;
	}
		//debugging
		const char*	getName()const override
		{
			return "Triangle";
		}

		int		getNumPreferredPenetrationDirections() const override
		{
			return 2;
		}
		
		void	getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const override
		{
			calcNormal(penetrationVector);
			if (index)
				penetrationVector *= btScalar(-1.);
		}


};

#endif //OBB_TRIANGLE_MINKOWSKI_H

