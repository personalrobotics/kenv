#ifndef PYTHON_TUPLE_HELPERS_H_
#define PYTHON_TUPLE_HELPERS_H_
#include <boost/typeof/typeof.hpp>
#include <boost/python/tuple.hpp> 
#include <boost/python/module.hpp> 
#include <boost/python/refcount.hpp> 

namespace util {

boost::python::tuple tuple_to_python(boost::tuples::null_type) 
{ 
    return boost::python::tuple(); 
} 

template <class H, class T> 
boost::python::tuple tuple_to_python(boost::tuples::cons<H, T> const &x) 
{ 
    BOOST_AUTO(head_tuple, boost::python::make_tuple(x.get_head()));
    BOOST_AUTO(tail_tuple, tuple_to_python(x.get_tail()));
    BOOST_AUTO(full_tuple, head_tuple.attr("__add__")(tail_tuple));
    return boost::python::extract<boost::python::tuple>(full_tuple);
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
