#include <boost/python.hpp>
#include <boost/eigen_numpy.h>

using namespace boost::python;  
                                       
void python_GazeboEnvironment();
                               
BOOST_PYTHON_MODULE(gz_kenv_ext)
{
    import("kenv");       
    python_GazeboEnvironment();
}
