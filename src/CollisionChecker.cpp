#include <kenv/CollisionChecker.h>

namespace kenv {

bool DefaultCollisionChecker::checkCollision(Object::Ptr obj1, Object::Ptr obj2,
                                             std::vector<Contact> *contacts,
                                             Link::Ptr *link1, Link::Ptr *link2) const
{
    return obj1->checkCollision(obj2, contacts, link1, link2);
}

}
