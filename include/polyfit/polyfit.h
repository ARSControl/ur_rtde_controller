#ifndef POLYFIT_H
#define POLYFIT_H

#include <Eigen/Dense>

class PolyFit
{

public:
  PolyFit();
  ~PolyFit();

  struct polynomial
  {
    int n;
    std::vector<double> coefficients;
    double final_time;
  };

  struct point
  {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    double time;
  };

  struct trajectory
  {
    std::vector<point> points;
  };

  bool computePolynomials(const trajectory &traj);
  Eigen::VectorXd evaluatePolynomials(const double &t);
  Eigen::VectorXd evaluatePolynomialsDer(const double &t);
  Eigen::VectorXd evaluatePolynomialsDDer(const double &t);
  double evaluateMaxPolynomials(const double &ts);
  double evaluateMaxPolynomialsDer(const double &ts);
  double evaluateMaxPolynomialsDDer(const double &ts);

private:
  std::vector<polynomial> polynomials_;
  double evaluatePolynomial(const polynomial &p, const double &t);
  double evaluatePolynomialDer(const polynomial &p, const double &t);
  double evaluatePolynomialDDer(const polynomial &p, const double &t);
};

#endif /* POLYFIT_H */
