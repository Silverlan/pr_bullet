// SPDX-FileCopyrightText: (c) 2019 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

#include "overlapfiltercallback.hpp"
#include "collision_object.hpp"
#include <pragma/entities/entity_component_manager.hpp>
#include <pragma/physics/physobj.h>
#include <pragma/entities/baseentity.h>
#include <pragma/entities/components/base_physics_component.hpp>

#define VERBOSE_OBJECT_COLLISIONS 0

#if VERBOSE_OBJECT_COLLISIONS == 1
static std::string get_proxy_object_name(btBroadphaseProxy* proxy,BaseEntity **ptrEnt=nullptr)
{
	std::stringstream src;
	if(proxy->m_clientObject == nullptr)
		src<<"[btBroadphaseProxy] "<<typeid(*proxy).name()<<" ("<<proxy->m_uniqueId<<")";
	else
	{
		auto *o = static_cast<btCollisionObject*>(proxy->m_clientObject);
		auto *t = static_cast<PhysCollisionObject*>(o->getUserPointer());
		if(t == nullptr)
			src<<"[btCollisionObject] "<<typeid(*o).name();
		else
		{
			auto *phys = static_cast<PhysObj*>(t->userData);
			if(phys == nullptr)
				src<<"[PhysCollisionObject] "<<typeid(*t).name()<<" ("<<t->GetPos()<<")";
			else
			{
				auto *ent = phys->GetOwner();
				if(ptrEnt != nullptr)
					*ptrEnt = ent;
				src<<"[Entity] "<<ent->GetClass()<<" ("<<ent->GetName()<<")";
			}
		}
	}
	return src.str();
}
#endif

bool PhysOverlapFilterCallback::needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
{
	auto bShouldCollide = (proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) != 0;
	bShouldCollide = bShouldCollide && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);

#if VERBOSE_OBJECT_COLLISIONS == 1
	BaseEntity *ent0 = nullptr;
	BaseEntity *ent1 = nullptr;
	auto srcName = get_proxy_object_name(proxy0,&ent0);
	auto tgtName = get_proxy_object_name(proxy1,&ent1);
	static auto bSkip = false;
	if(bSkip == false && ent0 != ent1)
	{
		bSkip = true;
		auto r = needBroadphaseCollision(proxy0,proxy1);
		bSkip = false;
		if(r == true)
			std::cout<<"Collision between "<<srcName<<" and "<<tgtName<<": "<<r<<std::endl;
	}
#endif
	if(proxy0->m_clientObject != nullptr && proxy1->m_clientObject != nullptr)
	{
		auto *o1 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
		auto *o2 = static_cast<btCollisionObject*>(proxy1->m_clientObject);
		if(o1->isStaticObject() && o2->isStaticObject())
			return false; // Both are static, don't need to check for collisions
		auto *t1 = static_cast<pragma::physics::ICollisionObject*>(o1->getUserPointer());
		auto *t2 = static_cast<pragma::physics::ICollisionObject*>(o2->getUserPointer());
		if(t1 != nullptr && t2 != nullptr)
		{
			auto *phys1 = t1->GetPhysObj();
			auto *phys2 = t2->GetPhysObj();
			if(phys1 != nullptr && phys2 != nullptr)
			{
				bShouldCollide = (phys1->GetCollisionFilter() &phys2->GetCollisionFilterMask()) != CollisionMask::None;
				bShouldCollide = bShouldCollide && ((phys2->GetCollisionFilterMask() &phys1->GetCollisionFilter()) != CollisionMask::None);
				auto *ent1 = &phys1->GetOwner()->GetEntity();
				auto pPhysComponent1 = ent1->GetPhysicsComponent();
				auto *ent2 = &phys2->GetOwner()->GetEntity();
				auto pPhysComponent2 = ent2->GetPhysicsComponent();
				return (!pPhysComponent1 || pPhysComponent1->ShouldCollide(phys1,t1,ent2,phys2,t2,bShouldCollide)) && 
					(!pPhysComponent2 || pPhysComponent2->ShouldCollide(phys2,t2,ent1,phys1,t1,bShouldCollide));
			}
		}
	}
	// Just use the default behavior
	return bShouldCollide;
}
