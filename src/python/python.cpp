#include <boost/python.hpp>
#include <boost/numpy/eigen_numpy.h>

using namespace boost::python;

void python_Environment();
void python_CollisionChecker();
void python_ObjectPool();

BOOST_PYTHON_MODULE(kenv_ext)
{
    SetupEigenConverters();
    python_Environment();
    python_CollisionChecker();
    python_ObjectPool();
}
