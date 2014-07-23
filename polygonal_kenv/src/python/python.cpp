#include <boost/python.hpp>

using namespace boost::python;

void python_PolygonalEnvironment();

BOOST_PYTHON_MODULE(polygonal_kenv_ext)
{
    import("kenv");
    python_PolygonalEnvironment();
}
