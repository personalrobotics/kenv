    #ifndef COLLISIONCHECKER_H_
#define COLLISIONCHECKER_H_

#include "Environment.h"

namespace kenv {

class CollisionChecker {
public:
    typedef boost::shared_ptr<CollisionChecker> Ptr;
    typedef boost::shared_ptr<CollisionChecker const> ConstPtr;

    virtual bool checkCollision(Object::Ptr obj1, Object::Ptr oj2,
                                std::vector<Contact> *contacts = NULL,
                                std::vector<std::pair<Link::Ptr, Link::Ptr> > *links = NULL) const = 0;
};

class DefaultCollisionChecker : virtual public CollisionChecker {
public:
    typedef boost::shared_ptr<DefaultCollisionChecker> Ptr;
    typedef boost::shared_ptr<DefaultCollisionChecker const> ConstPtr;

    virtual bool checkCollision(Object::Ptr obj1, Object::Ptr oj2,
                                std::vector<Contact> *contacts = NULL,
                                std::vector<std::pair<Link::Ptr, Link::Ptr> > *links = NULL) const;
};

}

#endif
