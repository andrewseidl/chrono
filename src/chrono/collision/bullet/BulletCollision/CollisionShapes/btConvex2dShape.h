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

#ifndef BT_CONVEX_2D_SHAPE_H
#define BT_CONVEX_2D_SHAPE_H

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

///The btConvex2dShape allows to use arbitrary convex shapes are 2d convex shapes, with the Z component assumed to be 0.
///For 2d boxes, the btBox2dShape is recommended.
class btConvex2dShape : public btConvexShape
{
	btConvexShape*	m_childConvexShape;

	public:
	
	btConvex2dShape(	btConvexShape* convexChildShape);
	
	~btConvex2dShape() override;
	
	btVector3	localGetSupportingVertexWithoutMargin(const btVector3& vec)const override;

	btVector3	localGetSupportingVertex(const btVector3& vec)const override;

	void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const override;

	void	calculateLocalInertia(btScalar mass,btVector3& inertia) const override;

	btConvexShape*	getChildShape() 
	{
		return m_childConvexShape;
	}

	const btConvexShape*	getChildShape() const
	{
		return m_childConvexShape;
	}

	const char*	getName()const override 
	{
		return "Convex2dShape";
	}
	


	///////////////////////////


	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const override;

	void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const override;

	void	setLocalScaling(const btVector3& scaling) override ;
	const btVector3& getLocalScaling() const override ;

	void	setMargin(btScalar margin) override;
	btScalar	getMargin() const override;

	int		getNumPreferredPenetrationDirections() const override;
	
	void	getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const override;


};

#endif //BT_CONVEX_2D_SHAPE_H
