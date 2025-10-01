// SPDX-FileCopyrightText: (c) 2019 Silverlan <opensource@pragma-engine.com>
// SPDX-License-Identifier: MIT

module;

#include "module_definitions.hpp"

export module pragma.modules.bullet;

export import :collision_object;
export import :common;
export import :constraint;
export import :controller;
export import :debug;
export import :environment;
export import :kinematic_character_controller;
export import :material;
export import :motion_state;
export import :overlap_filter_callback;
export import :shape;

extern "C"
{
	PRAGMA_EXPORT void initialize_physics_engine(NetworkState &nw,std::unique_ptr<pragma::physics::IEnvironment,void(*)(pragma::physics::IEnvironment*)> &outEnv)
	{
		auto env = std::unique_ptr<pragma::physics::IEnvironment,void(*)(pragma::physics::IEnvironment*)>{
			new pragma::physics::BtEnvironment{nw},[](pragma::physics::IEnvironment *env) {
			env->OnRemove();
			delete env;
		}};
		if(env->Initialize() == false)
			env = nullptr;
		outEnv = std::move(env);
	}
};
