#include <iostream>
#include <boost/python.hpp>
#include <boost/eigen_numpy.h>
#include <openrave/openrave.h>

void python_OREnvironment();

static void test_environment(OpenRAVE::EnvironmentBasePtr env)
{
    std::cout << "Environment(" << env << ")" << std::endl;
}

BOOST_PYTHON_MODULE(or_kenv_ext)
{
    using namespace boost::python;

    SetupEigenConverters();
    import("kenv");

    python_OREnvironment();

    // TODO: Testing OpenRAVE Boost.Python bindings.
    def("test_environment", &test_environment);
}
