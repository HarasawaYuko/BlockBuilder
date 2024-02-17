#include "Solver.h"
#include "SolvetBody.h"
#include "Constant.h"

static float bias = Constant::BIAS;//�S����f��bias

Matrix getRtilda(const Vec2&);
void computeConstraint(Constraint& , SolverBody& , SolverBody& , const Object* , const Object* , const ContactPoint& );

void Solver::solve(World* world) {
	const std::vector<Object*>& objects = world->objects_;
	const std::vector<Pair>& pairs = world->pairs_;
	//�\���o�[�{�f�B���쐬
	std::vector<SolverBody> solverbodies;
	for (const auto obj : objects) {
		SolverBody body = SolverBody(obj);
		solverbodies.push_back(body);
	}

/****�S����ݒ�*****/
	for (const auto& pair : pairs) {
		//�v���L�V���쐬
		Collision* col = pair.getCol();//�Փ�
		const Object* objA = pair.getObj0();
		const Object* objB = pair.getObj1();
		SolverBody bodyA = solverbodies[objA->getIndex()];
		SolverBody bodyB = solverbodies[objB->getIndex()];

		//���C�W���ƒ��˕Ԃ�W���̐ݒ�
		col->setFri(sqrt(objA->getFri() * objB->getFri()));

		//���˕Ԃ�W���͐V�K�Ɍ��o���ꂽ�Փ˂݂̂ɓK�p
		float restitution;
		if (pair.getType() == Keep) {
			restitution = 0.f;
		}
		else {
			restitution = 0.5f * (objA->getE() + objB->getE());
		}

		//�S���̐ݒ�
		for (int j = 0; j < col->getContactNum(); j++) {
			ContactPoint& cp = col->getCp(j);

			//���Α��x(���[���h���W)���v�Z
			//���i���x���v�Z
			Vec2 vA = objA->getVelocity();
			Vec2 vB = objB->getVelocity();
			//��]���x���v�Z
			vA = vA + (objA->rotationVelocityVec(cp.pointA_ ,objA->getAngVelRad()) );
			vB = vB + (objB->rotationVelocityVec(cp.pointB_ ,objB->getAngVelRad()) );
			//���Α��x
			Vec2 Vab = vA - vB;

			//�S�����������߂̗͂ɗp����s��K���v�Z
			//K�͕����̎��ŗ��p�ł���̂ŁA��Ɍv�Z���Ă���
			Matrix K_matrix = Matrix(2, 2);
			float mass_sum = bodyA.mass_inv_ + bodyB.mass_inv_;
			Matrix mass_matrix = Matrix(2, 2);
			mass_matrix.identity();
			K_matrix = K_matrix + (mass_matrix * mass_sum);

			const Vec2 rA = cp.pointA_.rotationCCW(objA->getAngleRad());
			const Vec2 rB = cp.pointB_.rotationCCW(objB->getAngleRad());
			Matrix rAmat = getRtilda(rA) * (bodyA.inertia_inv_);
			Matrix rBmat = getRtilda(rB) * (bodyB.inertia_inv_);
			K_matrix = (K_matrix + rAmat) + rBmat;

			//constraint�ɋL�^
			{//��������
				//�S�������擾
				Vec2 axis = cp.normal_vec_;
				//�S�����̕���
				Matrix axis_ = Matrix(axis);
				Matrix denom_ = (K_matrix.product(axis_)).trans().product(axis_);
				float denom = denom_.matrix_[0][0];//����
				//���������̍S�����Z�b�g
				cp.constraint_[0].f_denominator_ = 1.f / denom;
				cp.constraint_[0].f_ = -(1.0f + restitution) * Vab.dot(axis);//���x�␳(f�̕���)
				cp.constraint_[0].f_ -= (bias * min(0.0f, cp.depth_ + 0.1f )) / (1.f / (float)FPS);//�ʒu�␳ �߂荞�݉����p
				cp.constraint_[0].f_ *= cp.constraint_[0].f_denominator_;//�����������
				//�S���͂̍ő�l�ƍŏ��l��ݒ�
				cp.constraint_[0].lower_f_ = 0.0f;
				cp.constraint_[0].upper_f_ = FLT_MAX;
				cp.constraint_[0].axis_ = axis;
			}
			{//���C����
				Vec2 axis_vec = cp.normal_vec_.normalCCW();
				//�S�����̕���
				Matrix axis_matrix = Matrix(axis_vec);
				Matrix denom_matrix = (K_matrix.product(axis_matrix)).trans().product(axis_matrix);
				float denom_scalar = denom_matrix.matrix_[0][0];
				cp.constraint_[1].f_denominator_ = 1.f / denom_scalar;
				cp.constraint_[1].f_ = -Vab.dot(axis_vec);       
				cp.constraint_[1].f_ *= cp.constraint_[1].f_denominator_;//�����������
				cp.constraint_[1].axis_ = axis_vec;
			}
		}
	}

/********�S���̉��Z*******/
	for (int k = 0; k < 10; k++) {
		for (const auto& pair : pairs) {
			Collision* col = pair.getCol();
			Object* objA = pair.getObj0();
			Object* objB = pair.getObj1();
			SolverBody& bodyA = solverbodies[objA->getIndex()];
			SolverBody& bodyB = solverbodies[objB->getIndex()];
			for (int it_contact = 0; it_contact < col->getContactNum(); it_contact++) {
				ContactPoint& cp = col->getCp(it_contact);
				//��������
				computeConstraint(cp.constraint_[0] , bodyA , bodyB , objA , objB , cp);
				//���C�͂�ݒ�
				float maxFriction = col->getFri() * abs(cp.constraint_[0].accum_impulse_);
				cp.constraint_[1].lower_f_ = -maxFriction;
				cp.constraint_[1].upper_f_ = maxFriction;
				computeConstraint(cp.constraint_[1], bodyA, bodyB, objA, objB, cp);
			}
		}
	}
	//���x���X�V
	for (int i = 0; i < objects.size(); i++) {
		objects[i]->addVel(solverbodies[i].delta_linear_vel_);
		objects[i]->addVelAng(solverbodies[i].delta_rota_vel_);
	}
}

Matrix getRtilda(const Vec2& r) {
	Matrix mat = Matrix(2 ,2);
	mat.matrix_[0][0] = r.y_ * r.y_;
	mat.matrix_[1][0] = -1 * r.y_ * r.x_;
	mat.matrix_[0][1] = -1 * r.y_ * r.x_;
	mat.matrix_[1][1] = r.x_ * r.x_;
	return mat;
}

void computeConstraint(Constraint& constraint , SolverBody& bodyA , SolverBody& bodyB , const Object* objA , const Object* objB , const ContactPoint& cp) {
	Vec2 rA = cp.pointA_.rotationCCW(bodyA.angle_);
	Vec2 rB = cp.pointB_.rotationCCW(bodyB.angle_);
	float deltaImpulse = constraint.f_;//���͂��擾
	Vec2 deltaVelocityA = bodyA.delta_linear_vel_ + objA->rotationVelocityVec(cp.pointA_, bodyA.delta_rota_vel_);//A�̑��x�ω���
	Vec2 deltaVelocityB = bodyB.delta_linear_vel_ + objB->rotationVelocityVec(cp.pointB_, bodyB.delta_rota_vel_);//B�̑��x�ω���
	//�S���͂��Z�o
	deltaImpulse -= constraint.f_denominator_ * constraint.axis_.dot(deltaVelocityA - deltaVelocityB);
	float oldImpulse = constraint.accum_impulse_;
	constraint.accum_impulse_ = clamp(oldImpulse + deltaImpulse, constraint.lower_f_, constraint.upper_f_);
	deltaImpulse = constraint.accum_impulse_ - oldImpulse;
	//�X�V���鑬�x���v�Z
	bodyA.delta_linear_vel_ = bodyA.delta_linear_vel_ + (constraint.axis_ * (deltaImpulse * bodyA.mass_inv_));
	bodyA.delta_rota_vel_ += (rA.cross(constraint.axis_ * deltaImpulse)) * bodyA.inertia_inv_;
	bodyB.delta_linear_vel_ = bodyB.delta_linear_vel_ - (constraint.axis_ * (deltaImpulse * bodyB.mass_inv_));
	bodyB.delta_rota_vel_ -= (rB.cross(constraint.axis_ * deltaImpulse)) * bodyB.inertia_inv_;
}