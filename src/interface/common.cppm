// SPDX-FileCopyrightText: (c) 2019 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module;

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <mathutil/uvec.h>

export module pragma.modules.bullet:common;

export {
	namespace uvec
	{
		btVector3 create_bt(const Vector3 &v);
		Vector3 create(const btVector3 &v);
	};

	namespace uquat
	{
		btQuaternion create_bt(const Quat &v);
		Quat create(const btQuaternion &v);
	};

	/* Defined in bullet/src/BulletCollision/CollisionDispatch/btCollisionWorld.cpp */
	struct btBridgedManifoldResultCustom : public btManifoldResult
	{

		btCollisionWorld::ContactResultCallback&	m_resultCallback;

		btBridgedManifoldResultCustom( const btCollisionObjectWrapper* obj0Wrap,const btCollisionObjectWrapper* obj1Wrap,btCollisionWorld::ContactResultCallback& resultCallback );

		virtual void addContactPoint(const btVector3& normalOnBInWorld,const btVector3& pointInWorld,btScalar depth);

	};
	struct btSingleContactCallbackCustom : public btBroadphaseAabbCallback
	{

		btCollisionObject* m_collisionObject;
		btCollisionWorld*	m_world;
		btCollisionWorld::ContactResultCallback&	m_resultCallback;


		btSingleContactCallbackCustom(btCollisionObject* collisionObject, btCollisionWorld* world,btCollisionWorld::ContactResultCallback& resultCallback);

		virtual bool	process(const btBroadphaseProxy* proxy);
	};
}
