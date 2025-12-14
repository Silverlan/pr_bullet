// SPDX-FileCopyrightText: (c) 2019 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module;

#include <LinearMath/btTransform.h>

module pragma.modules.bullet;

import :motion_state;

KinematicMotionState::KinematicMotionState(pragma::physics::BtCollisionObject &o, const pragma::math::Transform &initialTransform) : SimpleMotionState {o}, m_transform {initialTransform} {}
KinematicMotionState::~KinematicMotionState() {}
void KinematicMotionState::getWorldTransform(btTransform &worldTrans) const { worldTrans = pragma::physics::BtEnvironment::CreateBtTransform(m_transform); }
void KinematicMotionState::setWorldTransform(const btTransform &worldTrans) { SimpleMotionState::setWorldTransform(worldTrans); /*m_transform.SetTransform(worldTrans);*/ }
pragma::math::Transform &KinematicMotionState::GetWorldTransform() { return m_transform; }
const pragma::math::Transform &KinematicMotionState::GetWorldTransform() const { return const_cast<KinematicMotionState *>(this)->GetWorldTransform(); }
