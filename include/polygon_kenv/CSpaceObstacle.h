#ifndef CSPACEOBSTACLE_H_
#define CSPACEOBSTACLE_H_

#include <geos/geom/Geometry.h>

geos::geom::Geometry *ComputeCSpaceObstacle(geos::geom::Geometry *robot,
                                            geos::geom::Geometry *obstacle);

#endif
