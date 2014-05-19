#ifndef PYTHON_TUPLE_HELPERS_H_
#define PYTHON_TUPLE_HELPERS_H_
#include <boost/typeof/typeof.hpp>
#include <boost/python/tuple.hpp> 
#include <boost/python/module.hpp> 
#include <boost/python/refcount.hpp> 

namespace kenv_util {

inline boost::python::tuple tuple_to_python(boost::tuples::null_type) 
{ 
    return boost::python::tuple(); 
} 

template <class H, class T> 
boost::python::tuple tuple_to_python(boost::tuples::cons<H, T> const &x) 
{ 
    boost::python::list temp_list;
    temp_list.extend(boost::python::make_tuple(x.get_head()));
    temp_list.extend(tuple_to_python(x.get_tail()));
    return boost::python::tuple(temp_list);
} 

template <class T> 
struct tuple_converter 
{ 
    static PyObject *convert(T const &x) 
    { 
        return boost::python::incref(tuple_to_python(x).ptr()); 
    } 
}; 

}

#endif
