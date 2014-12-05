#include <iostream>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/typeof/typeof.hpp>
#include "ObjectPool.h"

namespace kenv {

/*
 * ObjectPool
 */
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

size_t ObjectPool::size() const
{
    return owned_objects_.size();
}

auto ObjectPool::constructor_arguments() const -> arguments_tuple
{
    return boost::make_tuple(env_, type_);
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

/*
 * MultiObjectPool
 */
MultiObjectPool::MultiObjectPool(Environment::Ptr env)
    : env_(env)
{
    BOOST_ASSERT(env);
}

Object::Ptr MultiObjectPool::Create(std::string const &type)
{
    ObjectPool::Ptr pool;
    
    BOOST_AUTO(it, pools_.find(type));
    if (it == pools_.end()) {
        it = pools_.insert(std::make_pair(type, boost::make_shared<ObjectPool>(env_, type))).first;
    }

    return it->second->Create();
}

size_t MultiObjectPool::size() const
{
    size_t num_objects = 0;

    typedef std::pair<std::string, ObjectPool::Ptr> PoolEntry;
    BOOST_FOREACH (PoolEntry const &pool_entry, pools_) {
        num_objects += pool_entry.second->size();
    }

    return num_objects;
}

auto MultiObjectPool::constructor_arguments() const -> arguments_tuple
{
    return boost::make_tuple(env_);
}

}
