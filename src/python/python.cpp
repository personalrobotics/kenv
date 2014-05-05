#include <boost/python.hpp>
using namespace boost::python;  
                                       
void python_GazeboEnvironment();
                               
BOOST_PYTHON_MODULE(gz_kenv_ext)
{
    import("gz_kenv_ext");       
    python_GazeboEnvironment();
}
