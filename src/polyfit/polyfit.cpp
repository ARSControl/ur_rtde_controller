#include "polyfit/polyfit.h"

PolyFit::PolyFit()
{
}

PolyFit::~PolyFit()
{
}

bool PolyFit::computePolynomials(const trajectory &traj)
{
    polynomials_.resize(traj.points[0].position.size());
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

            // First pass: collect sub-blocks and count rows
            std::vector<Eigen::MatrixXd> subAs, subBs;
            int total_rows = 0;

            for (const auto& pt : traj.points)
            {
                int rows = pt.acceleration.size() ? 3 :
                           pt.velocity.size() ? 2 : 1;

                Eigen::MatrixXd subA = Eigen::MatrixXd::Zero(rows, n + 1);
                Eigen::MatrixXd subB = Eigen::MatrixXd::Zero(rows, 1);

                double t = pt.time;
                for (int j = 0; j < n + 1; j++)
                {
                    subA(0, j) = std::pow(t, n - j);
                    if (rows > 1 && n - j - 1 >= 0)
                        subA(1, j) = (n - j) * std::pow(t, n - j - 1);
                    if (rows > 2 && n - j - 2 >= 0)
                        subA(2, j) = (n - j) * (n - j - 1) * std::pow(t, n - j - 2);
                }

                subB(0, 0) = pt.position[k];
                if (rows > 1) subB(1, 0) = pt.velocity[k];
                if (rows > 2) subB(2, 0) = pt.acceleration[k];

                subAs.push_back(std::move(subA));
                subBs.push_back(std::move(subB));
                total_rows += rows;
            }

            // Allocate A and B once with known total size
            Eigen::MatrixXd A(total_rows, n + 1);
            Eigen::MatrixXd B(total_rows, 1);

            // Fill A and B
            int current_row = 0;
            for (size_t i = 0; i < subAs.size(); ++i)
            {
                A.block(current_row, 0, subAs[i].rows(), n + 1) = subAs[i];
                B.block(current_row, 0, subBs[i].rows(), 1) = subBs[i];
                current_row += subAs[i].rows();
            }

            // Solve least squares
			Eigen::MatrixXd x(n + 1, 1);
			x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
            error = (A * x - B).norm();

            if (std::isnan(error))
            {
                error = error_old;
                continue;
            }

            if (error < toll)
            {
                polynomials_[k].n = n;
                polynomials_[k].coefficients.resize(n + 1);
                for (int i = 0; i < n + 1; i++)
                    polynomials_[k].coefficients[i] = x(i, 0);
                polynomials_[k].final_time = traj.points.back().time;
            }

            if (std::fabs(error - error_old) < 1000.0 * toll)
                counter++;
            else
                counter = 0;

            if (counter > 10)
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
	return pol_eval;
}

double PolyFit::evaluatePolynomial(const polynomial &p, const double &t)
{
	double eval_time = std::min(p.final_time, t);
	double eval_pol = p.coefficients.back();
	for (int i = 0; i < p.n; i++)
		eval_pol += p.coefficients[i] * std::pow(eval_time, p.n - i);
	return eval_pol;
}

Eigen::VectorXd PolyFit::evaluatePolynomialsDer(const double &t)
{
	Eigen::VectorXd dpol_eval(polynomials_.size());
	for (uint i = 0; i < polynomials_.size(); i++)
		dpol_eval(i) = evaluatePolynomialDer(polynomials_[i], t);
	return dpol_eval;
}

double PolyFit::evaluatePolynomialDer(const polynomial &p, const double &t)
{
	double eval_time = std::min(p.final_time, t);
	double deval_pol = (p.coefficients.size() >= 2) ? p.coefficients[p.coefficients.size() - 2] : 0.0;
	for (int i = 0; i < p.n; i++)
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
	double ddeval_pol = (p.coefficients.size() >= 3) ? p.coefficients[p.coefficients.size() - 3] : 0.0;
	for (int i = 0; i < p.n - 2; i++)
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

Eigen::VectorXd PolyFit::getLastPoint()
{
	if (polynomials_.empty())
		return Eigen::VectorXd(); // or throw, or return a NaN vector
	return evaluatePolynomials(polynomials_.front().final_time);
}

double PolyFit::getFinalTime()
{
	if (polynomials_.empty())
		return 0.; // or throw, or return a NaN vector
	return polynomials_.front().final_time;
}
