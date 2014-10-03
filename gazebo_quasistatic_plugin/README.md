Dependencies
------------
This packages depends on [Gazebo](http://gazebosim.org/) version 3 or 4. If you
do not already have Gazebo installed, you can do so by following the
[installation instructions](http://gazebosim.org/tutorials?tut=install&cat=get_started)
on the Gazebo website. Make sure you install the development headers in
`libgazebo4-dev` so you can develop plugins.

We generate [Boost.Python](www.boost.org/libs/python/doc/) to generate bindings
for several of our packages. These bindings also depend on
[Boost.NumPy](https://github.com/personalrobotics/Boost.NumPy) and
[Boost.NumPy_Eigen](https://github.com/personalrobotics/Boost.NumPy_Eigen) for
conversion between [NumPy](http://www.numpy.org/) datatypes (in Python) and
[Eigen](http://eigen.tuxfamily.org/) Eigen datatypes (in C++).

Unfortunately, Debian packages are not available for these dependencies.  You
can easily build Debian packages youself by running:
  
    # Boost.NumPy
    $ cd /tmp
    $ git clone https://github.com/personalrobotics/Boost.NumPy.git
    $ cd Boost.NumPy
    $ cmake -DCPACK_GENERATOR=DEB .
    $ make && make package
    $ sudo dpkg -i boost-numpy-*.deb
    
    # Boost.NumPy_Eigen
    $ cd /tmp
    $ git clone https://github.com/personalrobotics/Boost.NumPy_Eigen.git
    $ cd Boost.NumPy_Eigen
    $ cmake -DCPACK_GENERATOR=DEB .
    $ make && make package
    $ sudo dpkg -i boost-numpy-eigen-*.deb

Installation
------------
Checkout this Git repository into a Catkin workspace. Build the plugin:

    $ catkin_make --only-pkg-with-deps gazebo_quasistatic_plugin

This builds a plugin called `libquasistatic_plugin.so`. Add the directory
that contains this library to the Gazebo plugin search path:

    $ export GAZEBO_PLUGIN_PATH=<workspace>/devel/lib:${GAZEBO_PLUGIN_PATH}

This currently does not install the protobuf headers into the correct
include directory. You must do so manually:

    $ mkdir -p <workspace>/devel/include
    $ cp -r <workspace>/build/kenv/gazebo_quasistatic_plugin/proto_msg/* <workspace>/devel/include
    
**TODO:** How do we change the output directory for `protobuf_generate_cpp` in CMake?

Usage
-----
To set objects usign quasistatic physics using SDF:

Plugin Tag for gazebo_quasistatic_plugin must have 2 types of elements. 1 of tag type <pusher> and 1 or more of tag type <pushee>. 

Using the protobuf transport layer:

library is called libquasistatic_msgs.so and is apart of the gazebo_quasistatic_plugin package

proto_msg/set_pusher_request.proto
proto_msg/add_pushee_request.proto

The topic names respectively are
quasistatic_set_pusher
quasistatic_add_pushee

Example World
-------------
Build the test plugin:

    $ catkin_make --only-pkg-with-deps test_plugin

Run the test world:

    $ roscd gazebo_quasistatic_plugin
    $ gazebo models/blank_world.world
