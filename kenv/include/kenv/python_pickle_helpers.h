#ifndef PYTHON_PICKLE_HELPERS_H_
#define PYTHON_PICKLE_HELPERS_H_
#include <sstream>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/python/tuple.hpp> 
#include "python_tuple_helpers.h"

namespace kenv_util {

template <class T>
struct pickle_init_wrapper : public boost::python::pickle_suite {
    pickle_init_wrapper()
    {
        boost::python::to_python_converter<typename T::arguments_tuple,
                                           kenv_util::tuple_converter<typename T::arguments_tuple> >(); 
    }

    static boost::python::tuple getinitargs(T const &instance)
    {
        return boost::python::tuple(instance.constructor_arguments());
    }
};

template <class T, class InputArchive, class OutputArchive>
struct pickle_serialization_wrapper : public pickle_init_wrapper<T> {
    static boost::python::tuple getstate(boost::python::object py_instance)
    {
        T const &instance = boost::python::extract<T const &>(py_instance);


        std::ostringstream ostream;
        OutputArchive archive(ostream);
        archive & instance;
        return boost::python::make_tuple(py_instance.attr("__dict__"),
                                         ostream.str());
    }

    static void setstate(boost::python::object py_instance, boost::python::tuple state)
    {
        T &instance = boost::python::extract<T &>(py_instance);
        boost::python::dict py_dict = boost::python::extract<boost::python::dict>(py_instance.attr("__dict__"));
        py_dict.update(state[0]);

        BOOST_ASSERT(boost::python::len(state) == 2);
        std::string const content = boost::python::extract<std::string>(state[1]);
        std::istringstream istream(content);
        InputArchive archive(istream);
        archive & instance;
    }

    static bool getstate_manages_dict() { return true; }
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
