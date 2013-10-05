#ifndef OBJECTPOOL_H_
#define OBJECTPOOL_H_

#include <set>
#include <vector>
#include <boost/enable_shared_from_this.hpp>
#include "Environment.h"

namespace kenv {

class ObjectPool : public boost::enable_shared_from_this<ObjectPool> {
public:
    typedef boost::shared_ptr<ObjectPool> Ptr;
    typedef boost::shared_ptr<ObjectPool const> ConstPtr;

    ObjectPool(Environment::Ptr env, std::string const &type);
    virtual ~ObjectPool();
    Object::Ptr Create();

private:
    Environment::Ptr env_;
    std::string type_;
    std::vector<Object::Ptr> owned_objects_;
    std::vector<Object *> inactive_objects_;
    std::set<Object *> active_objects_;

    void ObjectDeleter(Object *object);
};

}

#endif
