#include <boost/python.hpp>
#include "OREnvironment.h"

void python_OREnvironment()
{
    // TODO: This should be or_kenv.
    using namespace kenv;
    using namespace boost::python;

    /* TODO: Wrap the OpenRAVE-specific methods:
     *  - OREnvironment::__init__
     *  - OREnvironment::getOREnvironment
     *  - ORObject::getKinBody
     */
    class_<OREnvironment, OREnvironment::Ptr, boost::noncopyable,
           bases<kenv::Environment> >("OREnvironment")
        ;

    class_<ORObject, ORObject::Ptr, boost::noncopyable,
           bases<kenv::Object> >("ORObject", no_init)
        ;

    class_<ORLink, ORLink::Ptr, boost::noncopyable,
           bases<kenv::Link> >("ORLink", no_init)
        ;
}
