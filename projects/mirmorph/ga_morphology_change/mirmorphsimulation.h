#ifndef MIRMORPHSIMULATION_H
#define MIRMORPHSIMULATION_H

#include <stdio.h>
#include <stdlib.h>

#include <ode-dbl/ode.h>
#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <ode_robots/mirmorph.h>
#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>

#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include <selforg/trackrobots.h>
#include <selforg/abstractcontroller.h>

#include <mirmorphrpos.h>

#include <ea.h>

#include <controllers/mirmorph/ga_morphology_change/ico_controller.h>

class MirmorphSimulation : public lpzrobots::Simulation{
public:
    MirmorphSimulation(const Chromosome<double>& chromosome);

    virtual ~MirmorphSimulation();

    double getResults(int argc, char **argv);

private:
    virtual void start(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle, lpzrobots::GlobalData& global);

    virtual void addCallback(lpzrobots::GlobalData& global, bool draw, bool pause, bool control);

    virtual bool restart(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle, lpzrobots::GlobalData& global);

    virtual bool command(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle, lpzrobots::GlobalData& global, int key, bool down);

    void createMirmorph(int orientation);

    void cleanSimulation();

    double escale(double input, double min, double max);

    double average_filter(std::vector<double> values, double new_value, int size);

    lpzrobots::OdeAgent* _agent;
    lpzrobots::MirmorphRPos* _robot;
    IcoController* _controller;
    One2OneWiring* _wiring;
    std::vector<lpzrobots::AbstractObstacle*> _spheres;
    std::vector<lpzrobots::FixedJoint*> _fixator;

    Chromosome<double> _chromosome;

    lpzrobots::GlobalData* _globalData;

    bool _repeat_experiment;
    int _repeat_number;
    double _time_factor;
    bool _reboot_sim;
    bool _bad_robot_conf;
    int _number_spheres;
    bool _use_box;
    bool _use_rough_ground;

    bool _size_test;

    double _x[2];
    double _y[2];
    double _traveled_distance;
    double _traveled_distance5s;
    double _one_shot_step;
    double _IR_max[8];
    double _tilt[2];
    std::vector<double> _tilt_vector;
    std::vector<double> _speeds;
    double _max_realTime;
    std::vector<std::vector<double>> _results;
    std::ofstream _results_log;
};

#endif // MIRMORPHSIMULATION_H
