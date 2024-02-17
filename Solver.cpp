#include "Solver.h"
#include "SolvetBody.h"
#include "Constant.h"

static float bias = Constant::BIAS;//拘束力fのbias

Matrix getRtilda(const Vec2&);
void computeConstraint(Constraint& , SolverBody& , SolverBody& , const Object* , const Object* , const ContactPoint& );

void Solver::solve(World* world) {
	const std::vector<Object*>& objects = world->objects_;
	const std::vector<Pair>& pairs = world->pairs_;
	//ソルバーボディを作成
	std::vector<SolverBody> solverbodies;
	for (const auto obj : objects) {
		SolverBody body = SolverBody(obj);
		solverbodies.push_back(body);
	}

/****拘束を設定*****/
	for (const auto& pair : pairs) {
		//プロキシを作成
		Collision* col = pair.getCol();//衝突
		const Object* objA = pair.getObj0();
		const Object* objB = pair.getObj1();
		SolverBody bodyA = solverbodies[objA->getIndex()];
		SolverBody bodyB = solverbodies[objB->getIndex()];

		//摩擦係数と跳ね返り係数の設定
		col->setFri(sqrt(objA->getFri() * objB->getFri()));

		//跳ね返り係数は新規に検出された衝突のみに適用
		float restitution;
		if (pair.getType() == Keep) {
			restitution = 0.f;
		}
		else {
			restitution = 0.5f * (objA->getE() + objB->getE());
		}

		//拘束の設定
		for (int j = 0; j < col->getContactNum(); j++) {
			ContactPoint& cp = col->getCp(j);

			//相対速度(ワールド座標)を計算
			//並進速度を計算
			Vec2 vA = objA->getVelocity();
			Vec2 vB = objB->getVelocity();
			//回転速度を計算
			vA = vA + (objA->rotationVelocityVec(cp.pointA_ ,objA->getAngVelRad()) );
			vB = vB + (objB->rotationVelocityVec(cp.pointB_ ,objB->getAngVelRad()) );
			//相対速度
			Vec2 Vab = vA - vB;

			//拘束を解くための力に用いる行列Kを計算
			//Kは複数の軸で流用できるので、先に計算しておく
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

			//constraintに記録
			{//反発方向
				//拘束軸を取得
				Vec2 axis = cp.normal_vec_;
				//拘束式の分母
				Matrix axis_ = Matrix(axis);
				Matrix denom_ = (K_matrix.product(axis_)).trans().product(axis_);
				float denom = denom_.matrix_[0][0];//分母
				//反発方向の拘束をセット
				cp.constraint_[0].f_denominator_ = 1.f / denom;
				cp.constraint_[0].f_ = -(1.0f + restitution) * Vab.dot(axis);//速度補正(fの分母)
				cp.constraint_[0].f_ -= (bias * min(0.0f, cp.depth_ + 0.1f )) / (1.f / (float)FPS);//位置補正 めり込み解消用
				cp.constraint_[0].f_ *= cp.constraint_[0].f_denominator_;//分母を加える
				//拘束力の最大値と最小値を設定
				cp.constraint_[0].lower_f_ = 0.0f;
				cp.constraint_[0].upper_f_ = FLT_MAX;
				cp.constraint_[0].axis_ = axis;
			}
			{//摩擦方向
				Vec2 axis_vec = cp.normal_vec_.normalCCW();
				//拘束式の分母
				Matrix axis_matrix = Matrix(axis_vec);
				Matrix denom_matrix = (K_matrix.product(axis_matrix)).trans().product(axis_matrix);
				float denom_scalar = denom_matrix.matrix_[0][0];
				cp.constraint_[1].f_denominator_ = 1.f / denom_scalar;
				cp.constraint_[1].f_ = -Vab.dot(axis_vec);       
				cp.constraint_[1].f_ *= cp.constraint_[1].f_denominator_;//分母を加える
				cp.constraint_[1].axis_ = axis_vec;
			}
		}
	}

/********拘束の演算*******/
	for (int k = 0; k < 10; k++) {
		for (const auto& pair : pairs) {
			Collision* col = pair.getCol();
			Object* objA = pair.getObj0();
			Object* objB = pair.getObj1();
			SolverBody& bodyA = solverbodies[objA->getIndex()];
			SolverBody& bodyB = solverbodies[objB->getIndex()];
			for (int it_contact = 0; it_contact < col->getContactNum(); it_contact++) {
				ContactPoint& cp = col->getCp(it_contact);
				//反発方向
				computeConstraint(cp.constraint_[0] , bodyA , bodyB , objA , objB , cp);
				//摩擦力を設定
				float maxFriction = col->getFri() * abs(cp.constraint_[0].accum_impulse_);
				cp.constraint_[1].lower_f_ = -maxFriction;
				cp.constraint_[1].upper_f_ = maxFriction;
				computeConstraint(cp.constraint_[1], bodyA, bodyB, objA, objB, cp);
			}
		}
	}
	//速度を更新
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
	float deltaImpulse = constraint.f_;//撃力を取得
	Vec2 deltaVelocityA = bodyA.delta_linear_vel_ + objA->rotationVelocityVec(cp.pointA_, bodyA.delta_rota_vel_);//Aの速度変化量
	Vec2 deltaVelocityB = bodyB.delta_linear_vel_ + objB->rotationVelocityVec(cp.pointB_, bodyB.delta_rota_vel_);//Bの速度変化量
	//拘束力を算出
	deltaImpulse -= constraint.f_denominator_ * constraint.axis_.dot(deltaVelocityA - deltaVelocityB);
	float oldImpulse = constraint.accum_impulse_;
	constraint.accum_impulse_ = clamp(oldImpulse + deltaImpulse, constraint.lower_f_, constraint.upper_f_);
	deltaImpulse = constraint.accum_impulse_ - oldImpulse;
	//更新する速度を計算
	bodyA.delta_linear_vel_ = bodyA.delta_linear_vel_ + (constraint.axis_ * (deltaImpulse * bodyA.mass_inv_));
	bodyA.delta_rota_vel_ += (rA.cross(constraint.axis_ * deltaImpulse)) * bodyA.inertia_inv_;
	bodyB.delta_linear_vel_ = bodyB.delta_linear_vel_ - (constraint.axis_ * (deltaImpulse * bodyB.mass_inv_));
	bodyB.delta_rota_vel_ -= (rB.cross(constraint.axis_ * deltaImpulse)) * bodyB.inertia_inv_;
}