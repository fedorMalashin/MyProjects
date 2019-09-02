#include <iostream>
#include <cmath>
#include <fstream>
#define _USE_MATH_DEFINES

const double eps = 0.000001;

double InterpolateLagrangePolynomial (double x, double* x_values, double* y_values, int size)
{
	double lagrange_pol = 0;
	double basics_pol;

	for (int i = 0; i < size; i++)
	{
		basics_pol = 1;
		for (int j = 0; j < size; j++)
		{
			if (j == i) continue;
			basics_pol *= (x - x_values[j])/(x_values[i] - x_values[j]);		
		}
		lagrange_pol += basics_pol*y_values[i];
	}
	return lagrange_pol;
}

double testF(double x)
{
	return  x*x*x + 3*x*x + 3*x + 1; // for example
}

double fx(double x, double y, double* x_values, double* y_values, int size) 
{ 
	return x - y + 30 * sin(InterpolateLagrangePolynomial(x, x_values, y_values, size));
} // вычисляемая функция

double dfx(double x, double y, double* x_values, double* y_values, int size) 
{ 
	double df_val = (InterpolateLagrangePolynomial(x + 2, x_values, y_values, size) - (InterpolateLagrangePolynomial(x, x_values, y_values, size))) / 2.0;
 	return 1 + 30 * cos(InterpolateLagrangePolynomial(x, x_values, y_values, size)) * df_val;
}
 // производная функции

typedef double(*function)(double x, double y, double* x_values, double* y_values, int size); // задание типа function

double solve(function fx, function dfx, double x0, double y, double* x_values, double* y_values, int size) {
  double x1  = x0 - fx(x0, y, x_values, y_values, size)/dfx(x0, y, x_values, y_values, size); // первое приблжение
  while (fabs(x1-x0)>eps) { // пока не достигнута точность 0.000001
    x0 = x1;
    x1 = x1 - fx(x1, y, x_values, y_values, size)/dfx(x1, y, x_values, y_values, size); // последующие приближения
  }
  return x1;
}


int main(void)
{
	const int size = 8;
	double x_values[size] = {10, 20, 30, 40, 50, 70, 80, 90};
	double y_values[size];

	std::ofstream fout("points.txt");

	for (int i = 0; i < size; i++)
		y_values[i] = atan(x_values[i] / 95.0);

	for (int i = 0; i < size; i++)
		fout << x_values[i] << "   " << y_values[i] << std::endl;

	fout.close();

	std::cout << InterpolateLagrangePolynomial(15, x_values, y_values, size) * 180 / M_PI << std::endl;

	std::cout << atan(15 / 95.0) * 180 / M_PI << std::endl;

	std::ofstream fout_2("points_sin.txt");
	
	double x_values_sin[12] = {0, 5, 9, 25, 40, 60, 80, 90, 100, 110, 130, 145};
	double y_values_sin[12];

	for (int i = 0; i < 12; i++)
	{
	double interp = InterpolateLagrangePolynomial(x_values_sin[i], x_values, y_values, size);
		y_values_sin[i] = x_values_sin[i] - 10 + 50 * sin(interp);
	}

	std::cout << solve(fx, dfx, 20, 40, x_values, y_values, size) << std::endl;

	for (int i = 0; i < 12; i++)
		fout_2 << x_values_sin[i] << "   " << y_values_sin[i] << std::endl;

	fout_2.close();

	return 0;
}