#ifndef PYTHON_PICKLE_HELPERS_H_
#define PYTHON_PICKLE_HELPERS_H_
#include <sstream>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
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

template <class T, class InputArchive, class OutputArchive>
struct pickle_serialization_wrapper : public pickle_init_wrapper<T> {
    static boost::python::tuple getstate(T const &instance)
    {
        std::ostringstream ostream;
        OutputArchive archive(ostream);
        archive & instance;
        return boost::python::make_tuple(ostream.str());
    }

    static void setstate(T &instance, boost::python::tuple state)
    {
        BOOST_ASSERT(boost::python::len(state) == 1);
        std::string const content = boost::python::extract<std::string>(state[0]);
        std::istringstream istream(content);
        InputArchive archive(istream);
        archive & instance;
    }
};

template <class T>
struct pickle_binary_serialization {
    typedef boost::archive::binary_iarchive iarchive;
    typedef boost::archive::binary_oarchive oarchive;
    typedef pickle_serialization_wrapper<T, iarchive, oarchive> wrapper;
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
