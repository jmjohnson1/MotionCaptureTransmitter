#include "Eigen/Dense"
#include <iostream>

using namespace Eigen;

int main() {
	Vector3f vec;
	vec << 1, 2, 3;
	std::cout << "Initial: " << vec << std::endl;

	vec << 4, 5, 6;
	std::cout << "New: " << vec << std::endl;

	return 0;
}
