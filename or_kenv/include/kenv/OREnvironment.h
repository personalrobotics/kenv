#ifndef ORENVIRONMENT_H_
#define ORENVIRONMENT_H_

#include <boost/enable_shared_from_this.hpp>
#include <boost/filesystem.hpp>
#include <openrave/openrave.h>
#include <kenv/Environment.h>

namespace kenv {

class OREnvironment;
class ORObject;
class ORRobot;

class ORViewer : private boost::noncopyable {
public:
    typedef boost::shared_ptr<ORViewer> Ptr;
    typedef boost::shared_ptr<ORViewer const> ConstPtr;

    ORViewer(OpenRAVE::EnvironmentBasePtr or_env);
    bool isOpen(void) const;
    void waitForClose(void);
    void redraw(void);

private:
    OpenRAVE::ViewerBasePtr or_viewer_;
    boost::thread thread_;
    bool is_running_;

    void mainLoop(OpenRAVE::EnvironmentBasePtr or_env);
};

class ORLogger : public Logger {
public:
    virtual void debug(std::string const &msg);
    virtual void info(std::string const &msg);
    virtual void warning(std::string const &msg);
    virtual void error(std::string const &msg);
    virtual void fatal(std::string const &msg);
};

class ORLink : virtual public Link {
public:
    typedef boost::shared_ptr<ORLink> Ptr;
    typedef boost::shared_ptr<ORLink const> ConstPtr;

    ORLink(boost::weak_ptr<ORObject> robot, OpenRAVE::KinBody::LinkPtr link);
    virtual Object::Ptr getObject(void) const;

    virtual std::string getName(void) const;
    virtual Eigen::Affine3d getTransform(void) const;
    virtual void enable(bool flag);

    AlignedBox3d computeLocalAABB();

private:
    boost::weak_ptr<ORObject> object_;
    OpenRAVE::KinBody::LinkPtr link_;
};

class ORObject : virtual public Object, public boost::enable_shared_from_this<ORObject> {
public:
    typedef boost::shared_ptr<ORObject> Ptr;
    typedef boost::shared_ptr<ORObject const> ConstPtr;

    ORObject(boost::weak_ptr<OREnvironment> parent, OpenRAVE::KinBodyPtr kinbody,
             std::string const &type);
    virtual void initialize(void);
    OpenRAVE::KinBodyPtr getKinBody(void) const;

    virtual Environment::Ptr getEnvironment(void) const;
    virtual std::string getName(void) const;
    virtual std::string getType(void) const;
    virtual std::string getKinematicsGeometryHash(void) const;

    virtual void saveState(void);
    virtual void restoreState(void);

    virtual bool checkCollision(Object::ConstPtr entity, std::vector<Contact> *contacts = NULL,
                                std::vector<std::pair<Link::Ptr, Link::Ptr> > *links = NULL) const;
    virtual bool checkCollision(std::vector<Contact> *contacts = NULL, 
                                std::vector<std::pair<Link::Ptr, Link::Ptr> > *links = NULL) const;

    virtual std::vector<Link::Ptr> getLinks(void) const;
    virtual Link::Ptr getLink(std::string const name) const;

    virtual void enable(bool flag);
    virtual void setVisible(bool flag);

    virtual Eigen::Affine3d getTransform(void) const;
    virtual void setTransform(Eigen::Affine3d const &tf);
    virtual AlignedBox3d getAABB(void) const;

    virtual Eigen::VectorXd getDOFValues(void) const;
    virtual void setDOFValues(Eigen::VectorXd const &dof_values);

    virtual void setColor(Eigen::Vector4d const &color);
    virtual void setTransparency(double p);

protected:
    boost::weak_ptr<OREnvironment> parent_;
    OpenRAVE::KinBodyPtr kinbody_;
    std::stack<OpenRAVE::KinBody::KinBodyStateSaverPtr> stateSavers_;
    std::map<std::string, ORLink::Ptr> links_;
    std::string type_;

    OpenRAVE::CollisionAction checkCollisionCallback(OpenRAVE::CollisionReportPtr report, bool is_physics,
                                                     std::vector<std::pair<Link::Ptr, Link::Ptr> > *links) const;
};

/**
 * A kenv wrapper for an OpenRAVE manipulator
 */
class ORManipulator : public kenv::Manipulator, public::boost::enable_shared_from_this<ORManipulator> {

public:
	/**
	 * Shared Ptr
	 */
	typedef boost::shared_ptr<ORManipulator> Ptr;

	/**
	 * Const Shared Ptr
	 */
	typedef boost::shared_ptr<ORManipulator const> ConstPtr;

	/**
	 * Constructor
	 * @param robot The robot this manipulator belongs to
	 * @param manip The OpenRAVE Manipulator to decorate
	 */
	ORManipulator(boost::shared_ptr<ORRobot> robot, OpenRAVE::RobotBase::ManipulatorPtr manip);

	/**
	 * @return The decorated OpenRAVE manipulator
	 */
	OpenRAVE::RobotBase::ManipulatorPtr getORManipulator() const;

    /**
     * @return The environment this manipulator is in
     */
    boost::shared_ptr<Environment> getEnvironment(void) const;

    /**
     * @return The robot this manipulator is attached ot
     */
    boost::shared_ptr<Robot> getRobot(void) const;

	/**
	 * @return The end-effector for this manipulator
	 */
    virtual Eigen::Affine3d getEndEffectorTransform(void) const;

	/**
	 * @return The Jacobian associated with the current pose of the robot.  
	 *   The Jacobian is in world frame.
	 */
	virtual Jacobian::Ptr getJacobian(void) const;

	/**
	 * Calculate an IK solution for the given end-effector transform
	 *
	 * @param ee_pose The end-effector transform
	 * @param ik The ik
	 * @param check_collision If true, return collision free ik
	 * @return True if a solution is found
	 */
	virtual bool findIK(const Eigen::Affine3d &ee_pose, Eigen::VectorXd &ik, bool check_collision = false) const;

    /**
     * Sets the dof values on the manipulator
     * @param dof_values The values for the manipulator
     * @param dof_indices The associated indices
     */
    virtual void setDOFValues(const Eigen::VectorXd& dof_values, const Eigen::VectorXi& dof_indices);

    /**
     * @return A list of DOF values
     */
    virtual Eigen::VectorXd getDOFValues() const;

    /**
     * @return A list of dof indices
     */
    virtual Eigen::VectorXi getDOFIndices() const;

    /**
     * @param lower The returned lower limits
     * @param higher The returned upper limits
     */
    virtual void getDOFLimits(Eigen::VectorXd& lower, Eigen::VectorXd& higher) const;

    /**
     * Check the given arm configuration for joint limit violations
     *
     * @param dof_values The configuration to check
     */
    virtual bool checkLimits(const Eigen::VectorXd& dof_values) const;


private: 
	boost::shared_ptr<ORRobot> robot_;
	OpenRAVE::RobotBase::ManipulatorPtr manip_;
};

/**
 * A kenv wrapper for an OpenRAVE robot
 */
class ORRobot : virtual public Robot, public ORObject {
    public:
	    /**
		 * Shared Ptr
		 */
        typedef boost::shared_ptr<ORRobot> Ptr;

		/**
		 * Const Shared Ptr
		 */
        typedef boost::shared_ptr<ORRobot const> ConstPtr;

		/**
		 * Constructor
		 * @param parent The environment this robot is a member of
		 * @param robot The OpenRAVE Robot to decorate
		 * @param type TODO? What is this used for?
		 */
        ORRobot(boost::weak_ptr<OREnvironment> parent, OpenRAVE::RobotBasePtr robot, std::string const &type);

		/**
		 * @return The OpenRAVE KinBody that this object decorates
		 */
        OpenRAVE::RobotBasePtr getORRobot(void) const;

		/**
		 * Save the current state of this robot
		 */
        virtual void saveState();

		/**
		 * @return The active manipulator on the robot
		 */
		virtual Manipulator::Ptr getActiveManipulator();

		/**
		 * @return The joint values for all joints marked active on the robot
		 */
        virtual Eigen::VectorXd getActiveDOFValues() const;

		/**
		 * @return The indices of the joints marked active on the robot
		 */
        virtual Eigen::VectorXi getActiveDOFIndices() const;

		/**
		 * @param dof_values The joint values to set
		 * @param dof_indices The indices of each joint to set
		 */
		virtual void setDOFValues(const Eigen::VectorXd &dof_values, const Eigen::VectorXi &dof_indices);

		/**
		 * @param lower A vector to fill with the lower limits for the joints marked active on the robot 
		 * @param higher A vector to fill with the upper limits for the joints marked active on the robot 
		 */
        virtual void getActiveDOFLimits(Eigen::VectorXd& lower, Eigen::VectorXd& higher) const;

		/**
		 * @return True if the robot is in self collision, False otherwise
		 */
		virtual bool checkSelfCollision() const;
		
		/**
		 * @param dof_values The dof_values to check
		 * @return True if the given dof_values are within limits, false otherwise
		 */
		virtual bool checkLimits(const Eigen::VectorXd &dof_values) const;


    private:
        OpenRAVE::RobotBasePtr robot_;
};

class OREnvironment : public Environment, public boost::enable_shared_from_this<OREnvironment> {
public:
    typedef boost::shared_ptr<OREnvironment> Ptr;
    typedef boost::shared_ptr<OREnvironment const> ConstPtr;

    OREnvironment(void);
    OREnvironment(OpenRAVE::EnvironmentBasePtr or_env);
    OpenRAVE::EnvironmentBasePtr getOREnvironment(void) const;
    virtual Object::Ptr getObject(std::string const &name);
    virtual void getObjects(std::vector<Object::Ptr>& objects);
    virtual Object::Ptr createObject(std::string const &type, std::string const &name, bool anonymous = false);
    virtual void remove(Object::Ptr object);

    virtual Robot::Ptr getRobot(std::string const &name);
    virtual Robot::Ptr createRobot(std::string const &type, std::string const &name, bool anonymous = false);

    virtual void runWorld(int);

    virtual boost::recursive_try_mutex& getMutex();
    virtual void saveFullState();
    virtual void restoreFullState();
    
    virtual bool checkCollision(Object::ConstPtr entity, std::vector<Contact> *contacts = NULL) const;
    virtual bool checkCollision(Object::ConstPtr obj1, Object::ConstPtr obj2, std::vector<Contact> *contacts = NULL) const;
    virtual bool checkCollision(Object::ConstPtr entity, const std::vector<Object::ConstPtr> &objects, std::vector< std::vector<Contact> > *contacts = NULL) const;

    virtual Handle drawLine(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                            double width, Eigen::Vector4d const &color);
    virtual Handle drawLineList(std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d> > const &lines,
                                double width, Eigen::Vector4d const &color);
    virtual Handle drawLineStrip(std::vector<Eigen::Vector3d> const &points,
                                 double width, Eigen::Vector4d const &color);
    virtual Handle drawArrow(Eigen::Vector3d const &start, Eigen::Vector3d const &end,
                             double width, Eigen::Vector4d const &color);
    virtual Handle drawPoints(std::vector<Eigen::Vector3d> const &points,
                              float point_size, Eigen::Vector4d const &color);

    virtual Handle drawPlane( const Eigen::Affine3d& origin, float width, float height,
    						const boost::multi_array<float,3>& texture);

    void addType(std::string const &type, std::string const &kinbody_path);

    ORViewer::Ptr attachViewer(void);

private:
    OpenRAVE::EnvironmentBasePtr env_;
    std::map<std::string, std::string> types_;

    typedef std::map<OpenRAVE::KinBodyPtr, ORObject::Ptr> ObjectMapType;
    ObjectMapType objects_;

    bool addObject(ORObject::Ptr object, std::string const name, bool anonymous);
};

}

#endif
