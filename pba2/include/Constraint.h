#ifndef __PBA_CONSTRAINT_H__
#define __PBA_CONSTRAINT_H__

#include "Vector.h"
#include "Matrix.h"
#include "DynamicalState.h"
#include "LinearAlgebra.h"
#include <iostream>

namespace pba {
	class ConstraintBase {
	public:
		ConstraintBase() : Ks(0.0), Kf(0.0) {};

		virtual double compute(DynamicalState& s) { std::cout << "calling ConstraintBase::compute(DynamicalState) based class virtual method\n"; return 0; }

		virtual Vector grad(DynamicalState& s, int index) { std::cout << "calling ConstraintBase::grad(DynamicState, int) base class virtual method\n"; return Vector(0, 0, 0); }

		virtual Matrix gradgrad(DynamicalState& s, int i, int j) { std::cout << "calling ConstraintBase::gradgrad(DynamicalState, double) base class virtual method\n"; return unitMatrix() * 0.0; }

		virtual void solve(DynamicalState& s, double tol, int maxloop) { std::cout << "calling ConstrainBase::solve(DynamicalState, double, int) base class virtual method\n"; }

		virtual ~ConstraintBase() {}

		double get_Ks() const { return Ks; }
		double get_Kf() const { return Kf; }

		void set_Ks(double v) { Ks = v; }
		void set_Kf(double v) { Kf = v; }
	private:
		double Ks, Kf;
	};
typedef std::shared_ptr<ConstraintBase> Constraint;

	class MultiConstraintBase
	{
	public:
		MultiConstraintBase() {}

		double compute(DynamicalState& s, size_t alpha) { return constraints[alpha]->compute(s); }

		Vector grad(DynamicalState& s, size_t alpha, int index) { return constraints[alpha]->grad(s, index); }

		Matrix gradgrad(DynamicalState& s, size_t alpha, int i, int j) { return constraints[alpha]->gradgrad(s, i, j); }

		void solve(DynamicalState& s, double tol, int maxloop);

		~MultiConstraintBase() {}

		void addConstraint(Constraint& c) { constraints.push_back(c); }

		const Constraint get_constraint(size_t c) const { return constraints[c]; }
		Constraint get_constraint(size_t c) { return constraints[c]; }

		size_t nb() const { return constraints.size(); }

	private:
		std::vector<Constraint> constraints;
	};
	typedef std::shared_ptr<MultiConstraintBase> MultiConstraint;

	MultiConstraint CreateMultiConstraint();
}

#endif __PBA_CONSTRAINT_H__