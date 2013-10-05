#include <iostream>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include "ObjectPool.h"

namespace kenv {

ObjectPool::ObjectPool(Environment::Ptr env, std::string const &type)
    : env_(env)
    , type_(type)
{
    BOOST_ASSERT(env);
}

ObjectPool::~ObjectPool()
{
    if (!active_objects_.empty()) {
        std::cout << "Warning: Destroying a pool containing active objects." << std::endl;
    }
}

Object::Ptr ObjectPool::Create()
{
    if (inactive_objects_.empty()) {
        std::string const object_name = boost::str(boost::format("%s_pool1") % type_);
        Object::Ptr object = env_->createObject(type_, object_name, true);
        owned_objects_.push_back(object);
        inactive_objects_.push_back(object.get());
    }

    Object *object = inactive_objects_.back();
    inactive_objects_.pop_back();
    active_objects_.insert(object);

    object->enable(true);
    object->setVisible(true);
    return Object::Ptr(object, boost::bind(&ObjectPool::ObjectDeleter, this, _1));
}

void ObjectPool::ObjectDeleter(Object *object)
{
    size_t const num_erased = active_objects_.erase(object);
    if (num_erased == 1) {
        object->enable(false);
        object->setVisible(false);
        inactive_objects_.push_back(object);
    } else {
        std::cout << "Warning: Object " << object << " is not a member of this pool." << std::endl;
    }
}

}
