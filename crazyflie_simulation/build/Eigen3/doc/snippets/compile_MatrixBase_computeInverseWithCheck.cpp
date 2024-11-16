static bool eigen_did_assert = false;
#define eigen_assert(X)                                                                \
  if (!eigen_did_assert && !(X)) {                                                     \
    std::cout << "### Assertion raised in " << __FILE__ << ":" << __LINE__ << ":\n" #X \
              << "\n### The following would happen without assertions:\n";             \
    eigen_did_assert = true;                                                           \
  }

#include <iostream>
#include <cassert>
#include <Eigen/Eigen>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

using namespace Eigen;
using namespace std;

int main(int, char**) {
  cout.precision(3);
// intentionally remove indentation of snippet
{
Matrix3d m = Matrix3d::Random();
cout << "Here is the matrix m:" << endl << m << endl;
Matrix3d inverse;
bool invertible;
m.computeInverseWithCheck(inverse, invertible);
if (invertible) {
  cout << "It is invertible, and its inverse is:" << endl << inverse << endl;
} else {
  cout << "It is not invertible." << endl;
}

}
  return 0;
}
