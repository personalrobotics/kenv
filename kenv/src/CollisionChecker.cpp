#include <kenv/CollisionChecker.h>

namespace kenv {

bool DefaultCollisionChecker::checkCollision(Object::Ptr obj1, Object::Ptr obj2,
                                             std::vector<Contact> *contacts,
                                             std::vector<std::pair<Link::Ptr, Link::Ptr> > *links) const
{
    return obj1->checkCollision(obj2, contacts, links);
}

}
