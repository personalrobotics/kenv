#include <boost/python.hpp>
#include "ObjectPool.h"

using namespace boost::python;
using namespace kenv;

void python_ObjectPool()
{
    class_<ObjectPool, boost::noncopyable, ObjectPool::Ptr>("ObjectPool",
            init<kenv::Environment::Ptr, std::string>())
        .def("Create", &ObjectPool::Create)
        .def("__len__", &ObjectPool::size)
        ;

    class_<MultiObjectPool, boost::noncopyable, MultiObjectPool::Ptr>("MultiObjectPool",
            init<kenv::Environment::Ptr>())
        .def("Create", &MultiObjectPool::Create)
        .def("__len__", &MultiObjectPool::size)
        ;
}
