/*
 * PathIntegrationMechanism.h
 *
 *  Created on: 22.10.2015
 *      Author: meicron
 */

#ifndef PATHINTEGRATIONMECHANISM_H_

#include "utils/ann-framework/ann.h"
#include "utils/ann-framework/circann.h"
#include <osg/Vec3f>


enum {HD, G, M};

class PathIntegration : public CircANN {
public:
    /**
     * The constructor.
     *
     * @param numneurons Number of neurons
     */
	PathIntegration(int numneurons=18, double noise=0.05, NoiseType type=correlated, double leak=0);

    /**
     * The destructor.
     */
	~PathIntegration();

    /**
     * Returns the angle of the home vector.
     *
     * @return (double)
     */
	double getHVAngle();

    /**
     * Returns the length of the home vector.
     *
     * @return (double)
     */
	double getHVLength();

    /**
     * Returns the home vector.
     *
     * @return (osg::Vec3)
     */
	osg::Vec3f getHV();

	void step(double angle, double speed);

private:
	CircANN * head_direction;
	CircANN * gater;
	CircANN * memory;
};


#endif /* PATHINTEGRATIONMECHANISM_H_ */
