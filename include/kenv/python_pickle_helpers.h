#ifndef PYTHON_PICKLE_HELPERS_H_
#define PYTHON_PICKLE_HELPERS_H_
#include <boost/python/tuple.hpp> 
#include "python_tuple_helpers.h"

namespace util {

template <class T>
struct pickle_init_wrapper : public boost::python::pickle_suite {
    pickle_init_wrapper()
    {
        boost::python::to_python_converter<typename T::arguments_tuple,
                                           util::tuple_converter<typename T::arguments_tuple> >(); 
    }

    static boost::python::tuple getinitargs(T const &instance)
    {
        return boost::python::tuple(instance.constructor_arguments());
    }
};

template <class T>
struct empty_pickle_wrapper : public boost::python::pickle_suite {
    static boost::python::tuple getinitargs(T const &instance)
    {
        return boost::python::tuple();
    }
};

}

#endif
