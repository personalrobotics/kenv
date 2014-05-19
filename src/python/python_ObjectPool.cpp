#include <boost/python.hpp>
#include "ObjectPool.h"
#include "python_pickle_helpers.h"

using namespace boost::python;
using namespace kenv;

void python_ObjectPool()
{
    class_<ObjectPool, boost::noncopyable, ObjectPool::Ptr>("ObjectPool",
            init<kenv::Environment::Ptr, std::string>())
        .def_pickle(kenv_util::pickle_init_wrapper<ObjectPool>())
        .def("Create", &ObjectPool::Create)
        .def("__len__", &ObjectPool::size)
        ;

    class_<MultiObjectPool, boost::noncopyable, MultiObjectPool::Ptr>("MultiObjectPool",
            init<kenv::Environment::Ptr>())
        .def_pickle(kenv_util::pickle_init_wrapper<MultiObjectPool>())
        .def("Create", &MultiObjectPool::Create)
        .def("__len__", &MultiObjectPool::size)
        ;
}
