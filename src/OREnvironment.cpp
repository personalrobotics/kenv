#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/assert.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/typeof/typeof.hpp>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <openrave-core.h>
#include <openrave/openrave.h>
#include <iostream>
#include <kenv/OREnvironment.h>
#include <kenv/or_conversions.h>

namespace kenv {

/*
 * ORLogger
 */
void ORLogger::debug(std::string const &msg)
{
    RAVELOG_DEBUG("%s\n", msg.c_str());
}

void ORLogger::info(std::string const &msg)
{
    RAVELOG_INFO("%s\n", msg.c_str());
}

void ORLogger::warning(std::string const &msg)
{
    RAVELOG_WARN("%s\n", msg.c_str());
}

void ORLogger::error(std::string const &msg)
{
    RAVELOG_ERROR("%s\n", msg.c_str());
}

void ORLogger::fatal(std::string const &msg)
{
    RAVELOG_FATAL("%s\n", msg.c_str());
}

/*
 * ORLink
 */
ORLink::ORLink(boost::weak_ptr<ORObject> object, OpenRAVE::KinBody::LinkPtr link)
    : object_(object)
    , link_(link)
{
    BOOST_ASSERT(link);
}

Object::Ptr ORLink::getObject(void) const
{
    return object_.lock();
}

std::string ORLink::getName(void) const
{
    return link_->GetName();
}

Eigen::Affine3d ORLink::getTransform(void) const
{
    OpenRAVE::Transform or_tf = link_->GetTransform();
    return kenv::toEigen(or_tf);
}

void ORLink::enable(bool flag)
{
    link_->Enable(flag);
}

AlignedBox3d ORLink::computeLocalAABB() {
	OpenRAVE::AABB aabb = link_->ComputeLocalAABB();
	Eigen::Vector3d minpt, maxpt;
	minpt << aabb.pos.x-aabb.extents.x,
				aabb.pos.y-aabb.extents.y,
				aabb.pos.z-aabb.extents.z;
	maxpt << aabb.pos.x+aabb.extents.x,
			 aabb.pos.y+aabb.extents.y,
			 aabb.pos.z+aabb.extents.z;

	return AlignedBox3d(minpt, maxpt);
}

/*
 * OREnvironment
 */
OREnvironment::OREnvironment(void)
    : env_(OpenRAVE::RaveCreateEnvironment())
{
}

OREnvironment::OREnvironment(OpenRAVE::EnvironmentBasePtr or_env)
    : env_(or_env)
{
}

Object::Ptr OREnvironment::getObject(std::string const &name)
{
    OpenRAVE::KinBodyPtr kinbody = env_->GetKinBody(name);
    if (kinbody) {
        BOOST_AUTO(result, objects_.insert(std::make_pair(kinbody, ORObject::Ptr())));
        if (result.second) {
            // TODO: Use a special name for "unknown" types.
            result.first->second = boost::make_shared<ORObject>(shared_from_this(), kinbody, "");
        }
        return result.first->second;
    }
    return Object::Ptr();
}

Object::Ptr OREnvironment::createObject(std::string const &type, std::string const &name, bool anonymous)
{
    // Create the OpenRAVE object and add it to the environment.
    OpenRAVE::KinBodyPtr kinbody;
    try {
        kinbody = env_->ReadKinBodyXMLFile(OpenRAVE::KinBodyPtr(), type);
    } catch (OpenRAVE::openrave_exception const &e) {
        RAVELOG_ERROR("Unable to create object of type [%s]: %s\n", type.c_str(), e.what());
        return Object::Ptr();
    }

    if (!kinbody) {
        RAVELOG_ERROR("Unable to create object of type [%s]: An unknown error has occured.\n", type.c_str());
        return Object::Ptr();
    }

    // Add the kinbody to the environment.
    kinbody->SetName(name);
    env_->Add(kinbody, anonymous);

    ORObject::Ptr object;
    try {
        object = boost::make_shared<ORObject>(shared_from_this(), kinbody, type);
        object->initialize();
    } catch (OpenRAVE::openrave_exception const &e) {
        RAVELOG_ERROR("Unable to add object to the environment.\n");
        return Object::Ptr();
    }

    // The name might have changed if anonymous is true.
    std::string const actual_name = kinbody->GetName();
    objects_[kinbody] = object;
    return object;
}

void OREnvironment::remove(Object::Ptr object)
{
    std::string const &name = object->getName();

    ORObject::Ptr or_object = boost::dynamic_pointer_cast<ORObject>(object);
    BOOST_ASSERT(or_object);
    OpenRAVE::KinBodyPtr kinbody = or_object->getKinBody();

    BOOST_AUTO(it, objects_.find(kinbody));
    if (it == objects_.end()) {
        RAVELOG_ERROR("There is no object named '%s' in the environment.\n", name.c_str());
        return;
    }

    objects_.erase(it);
    env_->Remove(kinbody);
    //kinbody->Destroy();
}

boost::shared_ptr<void> OREnvironment::drawLine(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                                                double width, Eigen::Vector4d const &color)
{
    using namespace boost::assign;

    std::vector<Eigen::Vector3d> points;
    points += start, end;
    return drawLineStrip(points, width, color);
}

boost::shared_ptr<void> OREnvironment::drawLineStrip(std::vector<Eigen::Vector3d> const &points,
                                                     double width, Eigen::Vector4d const &color)
{
    using namespace boost::assign;
    
    std::vector<float> raw_points;
    BOOST_FOREACH (Eigen::Vector3d const &point, points) {
        raw_points += point[0], point[1], point[2];
    }

    int const num_points = static_cast<int>(points.size());
    return env_->drawlinestrip(&raw_points.front(), num_points, 3 * sizeof(float), width, kenv::toOR(color));
}

boost::shared_ptr<void> OREnvironment::drawLineList(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > const &lines,
                                                    double width, Eigen::Vector4d const &color)
{
    using namespace boost::assign;
    
    std::vector<float> raw_points;
    Eigen::Vector3d point1, point2;
    BOOST_FOREACH (boost::tie(point1, point2), lines) {
        raw_points += point1[0], point1[1], point1[2];
        raw_points += point2[0], point2[1], point2[2];
    }

    int const num_points = static_cast<int>(2 * lines.size());
    return env_->drawlinelist(&raw_points.front(), num_points, 3 * sizeof(float), width, kenv::toOR(color));
}


boost::shared_ptr<void> OREnvironment::drawArrow(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                                                 double width, Eigen::Vector4d const &color)
{
    return env_->drawarrow(toOR(start), toOR(end), width, toOR(color));
}

boost::shared_ptr<void> OREnvironment::drawPoints(std::vector<Eigen::Vector3d> const &points,
                                                  float point_size, Eigen::Vector4d const &color)
{
    std::vector<float> raw_points(3 * points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        raw_points[3 * i + 0] = points[i][0];
        raw_points[3 * i + 1] = points[i][1];
        raw_points[3 * i + 2] = points[i][2];
    }
    int const num_points = static_cast<int>(points.size());
    return env_->plot3(&raw_points.front(), num_points, 3 * sizeof(float), point_size, toOR(color));
}

boost::shared_ptr<void> OREnvironment::drawPlane(Eigen::Affine3d const &origin,
                                                 float width, float height,
                                                 boost::multi_array<float, 3> const &texture)
{
    // Eigen::AngleAxisd(M_PI_2, O.row(2)) * 
	Eigen::Quaterniond const q(origin.rotation());
    OpenRAVE::Vector const trans = toOR(origin.translation());
    OpenRAVE::Vector const rot(q.w(), q.x(), q.y(), q.z());
    OpenRAVE::Transform const or_origin(rot, trans);

    // OpenRAVE expects half extents, not extents.
    OpenRAVE::Vector const extents(0.5 * width, 0.5 * height, 0);
	return env_->drawplane(or_origin, extents, texture);
}


OpenRAVE::EnvironmentBasePtr OREnvironment::getOREnvironment(void) const
{
    return env_;
}

void OREnvironment::addType(std::string const &type, std::string const &path)
{
    BOOST_AUTO(it, types_.insert(std::make_pair(type, path)));
    if (!it.second) {
        RAVELOG_ERROR("There is already a type named '%s'.\n", type.c_str());
    }
}

ORViewer::Ptr OREnvironment::attachViewer(void)
{
    return boost::make_shared<ORViewer>(env_);
}

void ORViewer::redraw(void)
{
    or_viewer_->GetEnv()->UpdatePublishedBodies();
}

/*
 * ORObject
 */
ORObject::ORObject(boost::weak_ptr<OREnvironment> parent, OpenRAVE::KinBodyPtr kinbody,
                   std::string const &type)
    : parent_(parent)
    , kinbody_(kinbody)
    , type_(type)
{
    BOOST_ASSERT(!parent_.expired());
    BOOST_ASSERT(kinbody_);
}

void ORObject::initialize(void)
{
    ORObject::Ptr this_object = shared_from_this();

    BOOST_FOREACH (OpenRAVE::KinBody::LinkPtr or_link, kinbody_->GetLinks()) {
        std::string const name = or_link->GetName();
        ORLink::Ptr link = boost::make_shared<ORLink>(this_object, or_link);
        links_.insert(std::make_pair(name, link));
    }
}

Environment::Ptr ORObject::getEnvironment(void) const {
    return parent_.lock();
}

std::string ORObject::getName(void) const
{
    return kinbody_->GetName();
}

std::string ORObject::getType(void) const
{
    return type_;
}

std::string ORObject::getKinematicsGeometryHash(void) const
{
    return kinbody_->GetKinematicsGeometryHash();
}

OpenRAVE::KinBodyPtr ORObject::getKinBody(void) const
{
    return kinbody_;
}

bool ORObject::checkCollision(Object::ConstPtr entity, std::vector<Contact> *contacts,
                              std::vector<std::pair<Link::Ptr, Link::Ptr> > *links) const
{
    ORObject::ConstPtr or_entity = boost::dynamic_pointer_cast<ORObject const>(entity);
    BOOST_ASSERT(or_entity);
    BOOST_ASSERT(kinbody_->GetEnv() == or_entity->kinbody_->GetEnv());
    OpenRAVE::EnvironmentBasePtr env = kinbody_->GetEnv();
    OpenRAVE::KinBodyConstPtr other_kinbody = or_entity->kinbody_;

    // Run a simple collision check if we don't care about the contacts or the
    // full set of pairwise link collisions.
    if (!contacts && !links) {
        OpenRAVE::CollisionReportPtr report = boost::make_shared<OpenRAVE::CollisionReport>();
        return env->CheckCollision(kinbody_, other_kinbody, report);
    }

    // Request contacts if necessary.
    int const collision_options = (contacts) ? OpenRAVE::CO_Contacts : 0;
    OpenRAVE::CollisionCheckerBasePtr collision_checker = env->GetCollisionChecker();
    OpenRAVE::CollisionOptionsStateSaver collision_saver(collision_checker, collision_options);

    // Clear the output parameters.
    if (contacts) {
        contacts->clear();
    }
    if (links) {
        links->clear();
    }

    // ODE short-circuits and only returns contacts for the first pair of
    // colliding links that it detects. We'll work around that by running a
    // separate collision check for each pair of links. See OpenRAVE bug #266.
    std::string const collision_checker_type = collision_checker->GetXMLId();
    if (collision_checker_type == "ode") {
        bool is_collision = false;
        BOOST_FOREACH (OpenRAVE::KinBody::LinkConstPtr link1, kinbody_->GetLinks())
        BOOST_FOREACH (OpenRAVE::KinBody::LinkConstPtr link2, other_kinbody->GetLinks()) {
            OpenRAVE::CollisionReportPtr report = boost::make_shared<OpenRAVE::CollisionReport>();
            bool const is_link_collision = env->CheckCollision(link1, link2, report);
            is_collision = is_collision || is_link_collision;

            // Build a list of colliding links.
            if (is_link_collision && links) {
                Link::Ptr link1 = getLink(report->plink1->GetName());
                Link::Ptr link2 = getLink(report->plink2->GetName());
                links->push_back(std::make_pair(link1, link2));
            }

            // Add the contacts to the list.
            if (contacts) {
                contacts->reserve(contacts->size() + report->contacts.size());
                BOOST_FOREACH (OpenRAVE::CollisionReport::CONTACT const &or_contact, report->contacts) {
                    Contact contact;
                    contact.position = toEigen3(or_contact.pos);
                    // Contact normals appear to be returned in the wrong direction - reverse before adding
                    contact.normal = toEigen3(-or_contact.norm);
                    contacts->push_back(contact);
                }
            }
        }
        return is_collision;
    }
    // PQP seems to return all of the contacts. However, the CollisionReport
    // only contains one pair of links. We'll register a collision callback to
    // catch the intermediate results.
    else if (collision_checker_type == "pqp" || collision_checker_type == "bullet") {
        OpenRAVE::CollisionReportPtr report = boost::make_shared<OpenRAVE::CollisionReport>();
        OpenRAVE::UserDataPtr handle = env->RegisterCollisionCallback(
            boost::bind(&ORObject::checkCollisionCallback, this, _1, _2, links));
        bool const is_collision = env->CheckCollision(kinbody_, other_kinbody, report);

        if (contacts) {
            contacts->reserve(report->contacts.size());
            BOOST_FOREACH (OpenRAVE::CollisionReport::CONTACT const &or_contact, report->contacts) {
                Contact contact;
                contact.position = toEigen3(or_contact.pos);
                // Contact normals appear to be returned in the wrong direction - reverse before adding
                contact.normal = toEigen3(-or_contact.norm); 
                contacts->push_back(contact);
            }
        }
        return is_collision;
    } else {
        // TODO: There's no reason not to support PQP here.
        throw std::runtime_error("Only ODE is supported when requesting contacts.");
    }
} 

OpenRAVE::CollisionAction ORObject::checkCollisionCallback(
        OpenRAVE::CollisionReportPtr report, bool is_physics,
        std::vector<std::pair<Link::Ptr, Link::Ptr> > *links) const
{
    if (is_physics || !links) {
        return OpenRAVE::CA_DefaultAction;
    } else if (report->plink1 && report->plink2) {
        Link::Ptr link1 = getLink(report->plink1->GetName());
        Link::Ptr link2 = getLink(report->plink2->GetName());
        links->push_back(std::make_pair(link1, link2));
    }
    return OpenRAVE::CA_Ignore;
}

void ORObject::enable(bool flag)
{
    kinbody_->Enable(flag);
}

void ORObject::setVisible(bool flag)
{
    kinbody_->SetVisible(flag);
}

Eigen::Affine3d ORObject::getTransform(void) const
{
    OpenRAVE::Transform or_tf = kinbody_->GetTransform();
    return toEigen(or_tf);
}

void ORObject::setTransform(Eigen::Affine3d const &tf)
{
    OpenRAVE::TransformMatrix or_matrix = toOR(tf);

    OpenRAVE::Transform or_tf(or_matrix);
    // FIXME: This is a hack that should not be necessary.
    if (or_tf.rot.lengthsqr4() == 0) {
        or_tf.rot.y = or_tf.rot.z = or_tf.rot.w = 0.0;
        or_tf.rot.x = 1.0;
        RAVELOG_WARN("Attempted to set degenerate rotation matrix; assuming identity.\n");
    } else {
        or_tf.rot.normalize4();
    }
    kinbody_->SetTransform(or_tf);
}

AlignedBox3d ORObject::getAABB(void) const
{
    OpenRAVE::AABB const or_aabb = kinbody_->ComputeAABB();
    OpenRAVE::Vector const or_min = or_aabb.pos - or_aabb.extents;
    OpenRAVE::Vector const or_max = or_aabb.pos + or_aabb.extents;
    return AlignedBox3d(toEigen3(or_min), toEigen3(or_max));
}

void ORObject::setTransparency(double x)
{
    BOOST_ASSERT(0.0 <= x && x <= 1.0);

    BOOST_FOREACH (OpenRAVE::KinBody::LinkPtr link, kinbody_->GetLinks()) {
        BOOST_FOREACH (OpenRAVE::KinBody::Link::GeometryPtr geom, link->GetGeometries()) {
            geom->SetTransparency(x);
        }
    }
}

void ORObject::setColor(Eigen::Vector4d const &color)
{
    OpenRAVE::Vector const or_color = toOR(color);

    BOOST_FOREACH (OpenRAVE::KinBody::LinkPtr link, kinbody_->GetLinks()) {
        BOOST_FOREACH (OpenRAVE::KinBody::Link::GeometryPtr geom, link->GetGeometries()) {
            geom->SetDiffuseColor(or_color);
        }
    }
}

std::vector<Link::Ptr> ORObject::getLinks(void) const
{
    std::vector<Link::Ptr> links;
    links.reserve(links_.size());

    std::string name;
    ORLink::Ptr link;
    BOOST_FOREACH (boost::tie(name, link), links_) {
        links.push_back(link);
    }
    return links;
}

Link::Ptr ORObject::getLink(std::string const name) const
{
    BOOST_AUTO(it, links_.find(name));
    if (it != links_.end()) {
        return it->second;
    } else {
        return Link::Ptr();
    }
}

Eigen::VectorXd ORObject::getDOFValues(void) const
{
    std::vector<double> or_dof_values;
    kinbody_->GetDOFValues(or_dof_values);

    Eigen::VectorXd dof_values(or_dof_values.size());
    for (size_t i = 0; i < or_dof_values.size(); ++i) {
        dof_values[i] = or_dof_values[i];
    }
    return dof_values;
}

void ORObject::setDOFValues(Eigen::VectorXd const &dof_values)
{
    std::vector<double> or_dof_values(dof_values.size());
    for (int i = 0; i < dof_values.size(); ++i) {
        or_dof_values[i] = dof_values[i];
    }
    kinbody_->SetDOFValues(or_dof_values, OpenRAVE::KinBody::CLA_Nothing);
}


/*
 * ORViewer
 */
ORViewer::ORViewer(OpenRAVE::EnvironmentBasePtr or_env)
    : is_running_(true)
{
    thread_ = boost::thread(boost::bind(&ORViewer::mainLoop, this, or_env));
}

bool ORViewer::isOpen(void) const
{
    return is_running_;
}

void ORViewer::waitForClose(void)
{
    if (is_running_) {
        thread_.join();
    }
}

void ORViewer::mainLoop(OpenRAVE::EnvironmentBasePtr or_env)
{
    or_viewer_ = OpenRAVE::RaveCreateViewer(or_env, "qtcoin");
    BOOST_ASSERT(or_viewer_);
    or_env->AddViewer(or_viewer_);
    or_viewer_->main(true);
    is_running_ = false;
}

}

