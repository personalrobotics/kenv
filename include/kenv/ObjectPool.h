#ifndef OBJECTPOOL_H_
#define OBJECTPOOL_H_

#include <map>
#include <set>
#include <vector>
#include <boost/enable_shared_from_this.hpp>
#include <boost/tuple/tuple.hpp>
#include "Environment.h"

namespace kenv {

class ObjectPool : public boost::enable_shared_from_this<ObjectPool> {
public:
    typedef boost::shared_ptr<ObjectPool> Ptr;
    typedef boost::shared_ptr<ObjectPool const> ConstPtr;
    typedef boost::tuple<Environment::Ptr, std::string> arguments_tuple;

    ObjectPool(Environment::Ptr env, std::string const &type);
    virtual ~ObjectPool();
    Object::Ptr Create();

    size_t size() const;
    arguments_tuple constructor_arguments() const;

private:
    Environment::Ptr env_;
    std::string type_;
    std::vector<Object::Ptr> owned_objects_;
    std::vector<Object *> inactive_objects_;
    std::set<Object *> active_objects_;

    void ObjectDeleter(Object *object);
};

class MultiObjectPool {
public:
    typedef boost::shared_ptr<MultiObjectPool> Ptr;
    typedef boost::shared_ptr<MultiObjectPool const> ConstPtr;
    typedef boost::tuple<Environment::Ptr> arguments_tuple;

    MultiObjectPool(Environment::Ptr env);
    Object::Ptr Create(std::string const &type);

    size_t size() const;
    arguments_tuple constructor_arguments() const;

private:
    Environment::Ptr env_;
    std::map<std::string, ObjectPool::Ptr> pools_;
};

}

#endif
