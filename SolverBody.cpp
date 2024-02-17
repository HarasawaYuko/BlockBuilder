#include "SolvetBody.h"
#include "UIMaterial.h"

SolverBody::SolverBody(const float ang , const float mInv , const float iInv) 
	:delta_linear_vel_(Vec2(0.f , 0.f)) , delta_rota_vel_(0.f) , angle_(ang) , mass_inv_(mInv) , inertia_inv_(iInv)
{}

SolverBody::SolverBody(const Object* obj) 
{
	angle_ = obj->getAngleRad();
	if (obj->isActive()) {
		mass_inv_ = 1.f / obj->getMass();
		inertia_inv_ = 1.f / obj->getIne();
	}
	else {
		mass_inv_ = 0.f;
		inertia_inv_ = 0.f;
	}
}

std::string SolverBody::toString()const {
	std::string str;
	str += "épê®:" + std::to_string(angle_);
	str += " mInv:" + std::to_string(mass_inv_);
	str += " iInv" + std::to_string(inertia_inv_);
	return str;
}