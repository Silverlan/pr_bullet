// SPDX-FileCopyrightText: (c) 2019 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

#ifndef __PR_PX_MATERIAL_HPP__
#define __PR_PX_MATERIAL_HPP__

#include <pragma/physics/phys_material.hpp>
#include <memory>

namespace pragma::physics
{
	class BtEnvironment;
	class BtMaterial
		: public pragma::physics::IMaterial
	{
	public:
		BtMaterial(BtEnvironment &env,float staticFriction,float dynamicFriction,float restitution);
		virtual float GetStaticFriction() const override;
		virtual void SetStaticFriction(float friction) override;
		virtual float GetDynamicFriction() const override;
		virtual void SetDynamicFriction(float friction) override;
		virtual float GetRestitution() const override;
		virtual void SetRestitution(float restitution) override;
	private:
		float m_staticFriction = 0.f;
		float m_dynamicFriction = 0.f;
		float m_restitution = 0.f;
	};
};

#endif
