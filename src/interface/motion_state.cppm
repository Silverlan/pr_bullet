// SPDX-FileCopyrightText: (c) 2019 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module;

#include <LinearMath/btMotionState.h>
#include <LinearMath/btTransform.h>

export module pragma.modules.bullet:motion_state;

export import pragma.math;

export {
	namespace pragma::physics {
		class BtCollisionObject;
	};
	class SimpleMotionState : public btMotionState {
	  public:
		SimpleMotionState(pragma::physics::BtCollisionObject &o);
		virtual void getWorldTransform(btTransform &worldTrans) const override;
		virtual void setWorldTransform(const btTransform &worldTrans) override;
		pragma::physics::BtCollisionObject &collisionObject;
	};

	class KinematicMotionState : public SimpleMotionState {
	  public:
		KinematicMotionState(pragma::physics::BtCollisionObject &o, const umath::Transform &initialTransform = {});
		virtual ~KinematicMotionState() override;

		umath::Transform &GetWorldTransform();
		const umath::Transform &GetWorldTransform() const;
	  private:
		virtual void getWorldTransform(btTransform &worldTrans) const override;
		virtual void setWorldTransform(const btTransform &worldTrans) override;
		umath::Transform m_transform = {};
	};
}
