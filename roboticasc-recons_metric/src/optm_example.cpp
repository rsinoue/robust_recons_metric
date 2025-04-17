#include "Eigen/Dense"
#include <nlopt.hpp>
#include <iostream>
#include <math.h>

using namespace std;
using namespace Eigen;


typedef struct {
    double a, b;
} my_constraint_data;

double myfunc(unsigned n, const double *x, double *grad, void *my_func_data);
double myconstraint(unsigned n, const double *x, double *grad, void *data);

int main(int argc, char **argv)
{

	cout << "Teste" << endl;

	MatrixXd teste;

	Vector3d vteste(4,5,6);

	teste.resize(3,10);

	teste = teste.Ones(3,100);

	teste.block(0,1,3,1) = vteste;

	cout << teste.block(0,1,3,1) << endl;
//    nlopt::opt opt(nlopt::LD_MMA, 2);

//    std::vector<double> lb(2);
//    lb[0] = -HUGE_VAL; lb[1] = 0;
//    opt.set_lower_bounds(lb);

//    opt.set_min_objective(myfunc, NULL);

//    my_constraint_data data[2] = { {2,0}, {-1,1} };
//    opt.add_inequality_constraint(myconstraint, &data[0], 1e-8);
//    opt.add_inequality_constraint(myconstraint, &data[1], 1e-8);

//    opt.set_xtol_rel(1e-4);

//    std::vector<double> x(2);
//    x[0] = 1.234; x[1] = 5.678;
//    double minf;
//    nlopt::result result = opt.optimize(x, minf);

  	return 0;
}


double myfunc(unsigned n, const double *x, double *grad, void *my_func_data)
{
    if (grad) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}


double myconstraint(unsigned n, const double *x, double *grad, void *data)
{
    my_constraint_data *d = (my_constraint_data *) data;
    double a = d->a, b = d->b;
    if (grad) {
        grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
        grad[1] = -1.0;
    }
    return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);
 }



