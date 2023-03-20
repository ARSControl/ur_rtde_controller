#include "polyfit/polyfit.h"

PolyFit::PolyFit()
{
}

PolyFit::~PolyFit()
{
}

bool PolyFit::computePolynomials(const trajectory &traj)
{
	std::vector<polynomial> polynomials_(traj.points[0].position.size());
	for (uint k = 0; k < traj.points[0].position.size(); k++)
	{
		double toll = 1e-6;
		double error = 1.0;
		double error_old = 2.0;
		int counter = 0;
		int n = 2;
		while (error > toll)
		{
			n++;
			Eigen::MatrixXd A;
			Eigen::MatrixXd B;

			for (uint i = 0; i < traj.points.size(); i++)
			{
				Eigen::MatrixXd subA;
				Eigen::MatrixXd subB;
				if (traj.points[i].acceleration.size())
				{
					subA.resize(3, n);
					subB.resize(1, 3);
				}
				else if (traj.points[i].velocity.size())
				{
					subA.resize(2, n);
					subB.resize(1, 2);
				}
				else
				{
					subA.resize(1, n);
					subB.resize(1, 1);
				}
				subA.setZero();
				subB.setZero();

				double t = traj.points[i].time;
				for (uint j = 0; j < n + 1; j++)
				{
					subA(1, j) = pow(t, n - j);
					if (traj.points[i].velocity.size() && n - j - 1 > 0)
						subA(2, j) = (n - j) * std::pow(t, n - j - 1);
					if (traj.points[i].acceleration.size() && n - j - 1 > 0)
						subA(3, j) = (n - j) * (n - j - 1) * std::pow(t, n - j - 2);
				}
				subB(1, 1) = traj.points[i].position[k];
				if (traj.points[i].velocity.size())
					subB(1, 2) = traj.points[i].velocity[k];
				if (traj.points[i].acceleration.size())
					subB(1, 3) = traj.points[i].acceleration[k];
				A.conservativeResize(A.rows() + subA.rows(), n);
				B.conservativeResize(B.rows() + subB.rows(), 1);
				A.block(A.rows() - subA.rows(), 0, subA.rows(), n) = subA;
				B.block(B.rows() - subB.rows(), 0, subB.rows(), 1) = subB;
			}
			auto x = A.bdcSvd().solve(B);
			error = (A * x - B).norm();
			if (error < toll)
			{
				polynomials_[k].n = n;
				polynomials_[k].coefficients.resize(n + 1);
				for (uint i = 0; i < n + 1; i++)
					polynomials_[k].coefficients[i] = x(i, 0);
				polynomials_[k].final_time = traj.points.end()->time;
			}
			if (std::fabs(error - error_old) < toll)
				counter++;
			else
				counter = 0;
			if (counter > 100)
			{
				polynomials_.resize(0);
				return false;
			}
			error_old = error;
		}
	}
	return true;
}

Eigen::VectorXd PolyFit::evaluatePolynomials(const double &t)
{
	Eigen::VectorXd pol_eval(polynomials_.size());
	for (uint i = 0; i < polynomials_.size(); i++)
		pol_eval(i) = evaluatePolynomial(polynomials_[i], t);
}

double PolyFit::evaluatePolynomial(const polynomial &p, const double &t)
{
	double eval_time = std::min(p.final_time, t);
	double eval_pol = *p.coefficients.end();
	for (uint i = 0; i < p.n; i++)
		eval_pol += p.coefficients[i] * std::pow(eval_time, p.n - i);

	return eval_pol;
}

Eigen::VectorXd PolyFit::evaluatePolynomialsDer(const double &t)
{
	Eigen::VectorXd pol_eval(polynomials_.size());
	for (uint i = 0; i < polynomials_.size(); i++)
		pol_eval(i) = evaluatePolynomialDer(polynomials_[i], t);
	return pol_eval;
}

double PolyFit::evaluatePolynomialDer(const polynomial &p, const double &t)
{
	double eval_time = std::min(p.final_time, t);
	double deval_pol = *p.coefficients.end() - 1;
	for (uint i = 0; i < p.n - 1; i++)
		deval_pol += (p.n - i) * p.coefficients[i] * std::pow(eval_time, p.n - i - 1);

	return deval_pol;
}

Eigen::VectorXd PolyFit::evaluatePolynomialsDDer(const double &t)
{
	Eigen::VectorXd ddpol_eval(polynomials_.size());
	for (uint i = 0; i < polynomials_.size(); i++)
		ddpol_eval(i) = evaluatePolynomialDDer(polynomials_[i], t);
	return ddpol_eval;
}

double PolyFit::evaluatePolynomialDDer(const polynomial &p, const double &t)
{
	double eval_time = std::min(p.final_time, t);
	double ddeval_pol = *p.coefficients.end() - 2;
	for (uint i = 0; i < p.n - 2; i++)
		ddeval_pol += (p.n - i - 1) * (p.n - i) * p.coefficients[i] * std::pow(eval_time, p.n - i - 2);

	return ddeval_pol;
}

double PolyFit::evaluateMaxPolynomials(const double &ts)
{
	double max = 0.0;
	for (auto &p : polynomials_)
		for (double t = 0.0; t < p.final_time; t += ts)
			max = std::max(max, (evaluatePolynomials(t).cwiseAbs()).maxCoeff());
	return max;
}

double PolyFit::evaluateMaxPolynomialsDer(const double &ts)
{
	double max = 0.0;
	for (auto &p : polynomials_)
		for (double t = 0.0; t < p.final_time; t += ts)
			max = std::max(max, (evaluatePolynomialsDer(t).cwiseAbs()).maxCoeff());
	return max;
}

double PolyFit::evaluateMaxPolynomialsDDer(const double &ts)
{
	double max = 0.0;
	for (auto &p : polynomials_)
		for (double t = 0.0; t < p.final_time; t += ts)
			max = std::max(max, (evaluatePolynomialsDDer(t).cwiseAbs()).maxCoeff());
	return max;
}