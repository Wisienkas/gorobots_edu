#include "mirmorphsimulation.h"

MirmorphSimulation::MirmorphSimulation(const Chromosome<double> &chromosome){
    setTitle("Mirmorph simulation");

    _chromosome = chromosome;
}

MirmorphSimulation::~MirmorphSimulation(){

}

void MirmorphSimulation::start(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle, lpzrobots::GlobalData& global){
    _globalData = &global;
    _controller = NULL;

    setCameraHomePos(lpzrobots::Pos(-1.14383, 10.1945, 42.7865), lpzrobots::Pos(179.991, -77.6244, 0));
    _globalData->odeConfig.noise = 0.02;
    _globalData->odeConfig.setParam("controlinterval", 1);
    _globalData->odeConfig.setParam("simstepsize", 0.01);
    _globalData->odeConfig.setParam("realtimefactor", 16.0);
    simulation_time_reached = false;
    inTaskedMode = false;
    _size_test = false;

    _repeat_experiment = true;
    _repeat_number = 5;
    _time_factor = 0.5;
    _reboot_sim = false;
    _bad_robot_conf = false;
    _number_spheres = 1;
    _use_box = true;
    _use_rough_ground = true;

    _x[0] = 15.0;
    _x[1] = 0.0;
    _y[0] = 0.0;
    _y[1] = 0.0;
    _traveled_distance = 0;
    _traveled_distance5s = 0;
    _one_shot_step = 0;
    for(int i=0; i<8; i++){
        _IR_max[i] = 0.0;
    }
    _tilt[0] = -10.0;
    _tilt[1] = 10.0;
    _tilt_vector.clear();

    _max_realTime = 0;

    _results.resize(_repeat_number);

    auto playground = new lpzrobots::Playground(odeHandle,
                                            osgHandle.changeColor(lpzrobots::Color(0.6, 0.0, 0.6)),
                                            osg::Vec3(35, 0.5, 2.0), 1, false);
    playground->setPosition(osg::Vec3(4, 0, 0.0));
    _globalData->obstacles.push_back(playground);

    lpzrobots::PassiveSphere* s1;
    for(int i=0; i<_number_spheres; i++){
        s1 = new lpzrobots::PassiveSphere(odeHandle, osgHandle, 0.5);
        //s1->setTexture("Images/dusty.rgb");
        if(i==0){
            s1->setPosition(osg::Vec3(-3.0, 5.0, 3.0));
            s1->setColor(lpzrobots::Color(0,1,0));
        }
        else if(i==1){
            s1->setPosition(osg::Vec3(-5.0, 5.0, 2.0));
            s1->setColor(lpzrobots::Color(1,0,0));
        }
        else if(i==2){
            s1->setPosition(osg::Vec3(0.0, -5.0, 2.5));
            s1->setColor(lpzrobots::Color(0,0,1));
        }
        else if(i==3){
            s1->setPosition(osg::Vec3(-5.0, -5.0, 2.5));
            s1->setColor(lpzrobots::Color(1,1,0));
        }
        _spheres.push_back(s1);
        _globalData->obstacles.push_back(s1);
        _fixator.push_back(new lpzrobots::FixedJoint(s1->getMainPrimitive(), _globalData->environment));
        _fixator.back()->init(odeHandle, osgHandle);
    }

    if(_use_box){
        auto box = new lpzrobots::PassiveBox(odeHandle, osgHandle, osg::Vec3(6.5, 1.0, 2.0));
        box->setColor(lpzrobots::Color(1,0,0));
        box->setPose(osg::Matrix::rotate(M_PI/2, 0, 0, 1)*osg::Matrix::translate(7.0, 0.0, 1.5));
        _globalData->obstacles.push_back(box);
        _fixator.push_back(new lpzrobots::FixedJoint(box->getMainPrimitive(), _globalData->environment));
        _fixator.back()->init(odeHandle, osgHandle);
    }

    if(_use_rough_ground){
        lpzrobots::Substance roughterrainSubstance(1.0, 0.1, 500.0, 0.0);
        lpzrobots::OdeHandle oodeHanlde = odeHandle;
        oodeHanlde.substance = roughterrainSubstance;
        auto terrainground = new lpzrobots::TerrainGround(oodeHanlde, osgHandle.changeColor(lpzrobots::Color(83.0/255.0, 48.0/255.0, 0.0)), "rough6.ppm", "", 34.9, 34.9, 0.2);
        terrainground->setPose(osg::Matrix::translate(4, 0, 0.19));
        _globalData->obstacles.push_back(terrainground);
    }

    createMirmorph(0);
}

void MirmorphSimulation::addCallback(lpzrobots::GlobalData &global, bool draw, bool pause, bool control){
    _x[1] = _x[0];
    _y[1] = _y[0];
    Position pos = _globalData->agents.back()->getRobot()->getPosition();
    _x[0] = pos.x;
    _y[0] = pos.y;
    _traveled_distance += sqrt(pow(_x[0]-_x[1],2)+pow(_y[0]-_y[1],2));

    bool reached = false;
    if(sqrt(pow(-3-pos.x,2)+pow(5-pos.y,2))<0.7){
        reached = true;
    }

    Position vel = _globalData->agents.back()->getRobot()->getSpeed();
    _speeds.push_back(sqrt(pow(vel.x, 2)+pow(vel.y, 2)));
    double avg_speed = 0;
    for(uint i=0; i<_speeds.size(); i++){
        avg_speed += _speeds[i];
    }
    avg_speed /= _speeds.size();

    for(int i=0; i<8; i++){
        if(_controller->fitnessIR.at(i) > _IR_max[i]){
            _IR_max[i] = _controller->fitnessIR.at(i);
        }
    }

    double tilt_value = average_filter(_tilt_vector, _controller->orientation.at(2), 5);
    if(tilt_value > _tilt[0]){
        _tilt[0] = tilt_value;
    }
    else if(tilt_value < _tilt[1]){
        _tilt[1] = tilt_value;
    }

    if(truerealtimefactor>_max_realTime){
        _max_realTime = truerealtimefactor;
    }

    matrix::Matrix rotation = _globalData->agents.back()->getRobot()->getOrientation();
    double angle_x = atan2(rotation.val(2,1), rotation.val(2,2));

    bool not_moving = false;
    if((_globalData->sim_step/100)%5==0 && (_globalData->sim_step/100)!=0 && (_globalData->sim_step/100)!=_one_shot_step){
        _one_shot_step = (_globalData->sim_step/100);
        if(_traveled_distance-_traveled_distance5s<1.0 && _traveled_distance!=_traveled_distance5s){
            not_moving = true;
        }
        _traveled_distance5s = _traveled_distance;
    }

    if(_repeat_experiment){
        if(_globalData->sim_step>=(_time_factor*60.0*100.0) || _reboot_sim || _bad_robot_conf || reached || (angle_x<0.5 && angle_x>-0.5) || not_moving){
            _controller->stop = true;
            simulation_time_reached = true;
            _reboot_sim = false;

            std::vector<double> partial_result;
            partial_result.resize(15);

            partial_result[0] = min(_globalData->sim_step/(100.0), 30);
            partial_result[1] = _traveled_distance;
            partial_result[2] = max(_controller->distance/10, 0.3);
            partial_result[11] = 0;
            for(int i=0; i<8; i++){
                partial_result[i+3] = _IR_max[i];
                if(_IR_max[i] > partial_result[11]){
                    partial_result[11] = _IR_max[i];
                }
            }

            partial_result[12] = abs(_tilt[0]-_tilt[1]);

            partial_result[13] = _max_realTime;

            partial_result[14] = avg_speed;

            if(_bad_robot_conf || not_moving || (angle_x<0.5 && angle_x>-0.5)){
                partial_result[0] = -1;
            }

            _results[this->currentCycle-1] = partial_result;
        }
    }
}

bool MirmorphSimulation::restart(const lpzrobots::OdeHandle &odeHandle, const lpzrobots::OsgHandle &osgHandle, lpzrobots::GlobalData &global){
    if(this->currentCycle == _repeat_number){
//        delete _robot;
//        delete _wiring;
//        delete _controller;

//        _globalData->agents.erase(_globalData->agents.begin());

        return false;
    }
    else{
        lpzrobots::OdeAgent* agent = *_globalData->agents.begin();

        delete _robot;
        delete _wiring;

        _globalData->agents.erase(_globalData->agents.begin());

        _x[0] = 15.0;
        _x[1] = 0.0;
        _y[0] = 0.0;
        _y[1] = 0.0;
        _traveled_distance = 0;
        _traveled_distance5s = 0;
        _one_shot_step = 0;
        _speeds.clear();
        for(int i=0; i<8; i++){
            _IR_max[i] = 0.0;
        }
        _tilt[0] = -10.0;
        _tilt[1] = 10.0;
        _tilt_vector.clear();

        _max_realTime = 0;

        createMirmorph(currentCycle);

        _controller->stop = false;

        return true;
    }
}

bool MirmorphSimulation::command(const lpzrobots::OdeHandle& odeHandle, const lpzrobots::OsgHandle& osgHandle, lpzrobots::GlobalData& global, int key, bool down){
    if(down){
        std::vector<double> f;
        switch((char)key){
        case 'r':
            _reboot_sim = true;
            break;

        default:
            return false;
        }
    }
    return false;
}

void MirmorphSimulation::createMirmorph(int orientation){
    _bad_robot_conf = false;

    if(!_controller){
        _controller = new IcoController("MirmorphController", "Mirmorphcontroller");
    }

    _wiring = new One2OneWiring(new ColorUniformNoise(0.1));

    lpzrobots::MirmorphRPosConf fconf = lpzrobots::MirmorphRPos::getDefaultConf();

    fconf.mirconf.length = escale(_chromosome.getChromosome()[0], 0.8, 3.2);
    fconf.mirconf.width = escale(_chromosome.getChromosome()[1], 0.6, 2.4);
    fconf.mirconf.height = escale(_chromosome.getChromosome()[2], 0.36, 1.44);
    for(uint i=0; i<4; i++){
        fconf.mirconf.radius[i] = escale(_chromosome.getChromosome()[3], 0.2, 3*fconf.mirconf.length/8.0);
        fconf.mirconf.wheelthickness[i] = escale(_chromosome.getChromosome()[4], 0.1, 0.6);
        fconf.mirconf.wheelheights[i] = escale(_chromosome.getChromosome()[5], 0, fconf.mirconf.height/2.0);
    }
    fconf.frontIR_angle = escale(_chromosome.getChromosome()[6], -M_PI/2, M_PI/2);
    fconf.sideTopIR_angle = escale(_chromosome.getChromosome()[7], 0, M_PI);
    fconf.sideBottomIR_angle = escale(_chromosome.getChromosome()[8], 0, M_PI);
    fconf.backIR_angle = escale(_chromosome.getChromosome()[9], -M_PI/2, M_PI/2);
    fconf.irRangeFront = 4;
    fconf.irRangeSide = 4;
    fconf.irRangeBack = 3;

    if(_size_test){
        fconf = lpzrobots::MirmorphRPos::getDefaultConf();
        fconf.mirconf.length = escale(1, 0.8, 3.2);
        fconf.mirconf.width = escale(1, 0.6, 2.4);
        fconf.mirconf.height = escale(1, 0.36, 1.44);
        for(uint i=0; i<4; i++){
            fconf.mirconf.radius[i] = escale(1, 0.2, 3*fconf.mirconf.length/8.0);
            fconf.mirconf.wheelthickness[i] = escale(1, 0.1, 0.6);
            fconf.mirconf.wheelheights[i] = escale(1, 0, fconf.mirconf.height/2.0);
        }
        fconf.frontIR_angle = escale(0.395868, -M_PI/2, M_PI/2);
        fconf.sideTopIR_angle = escale(0.233922, 0, M_PI);
        fconf.sideBottomIR_angle = escale(0.841476, 0, M_PI);
        fconf.backIR_angle = escale(0.634882, -M_PI/2, M_PI/2);
        fconf.irRangeFront = 4;
        fconf.irRangeSide = 4;
        fconf.irRangeBack = 3;
    }

    for(int i=0; i<_number_spheres; i++){
        fconf.rpos_sensor_references.push_back(_spheres.at(i)->getMainPrimitive());
    }

    if(fconf.mirconf.height/2.0+0.1 > fconf.mirconf.wheelheights[0]+fconf.mirconf.radius[0]){
        _bad_robot_conf = true;
    }

    _robot = new lpzrobots::MirmorphRPos(odeHandle, osgHandle, fconf, "Mirmorph");

    double random_or = (MAX_OR-MIN_OR)*orientation/(_repeat_number-1)+MIN_OR;
    double robot_z = fconf.mirconf.height/2.0 + fconf.mirconf.radius[0] + 0.3;
    lpzrobots::Pos pos(15.0, 0.0, robot_z);
    _robot->place(osg::Matrix::rotate(random_or, 0, 0, 1) * osg::Matrix::translate(pos));

    _agent = new lpzrobots::OdeAgent(*_globalData);
    _agent->init(_controller, _robot, _wiring);
    _globalData->agents.push_back(_agent);
    _globalData->configs.push_back(_agent);
}

double MirmorphSimulation::getResults(int argc, char** argv){
    this->run(argc, argv);

    std::vector<double> fitness;
    double temp_fitness;
    _results_log.open("results.log", std::ios_base::out | std::ios_base::app);

    for(uint i=0; i<_results.size(); i++){
        if(_results[i][0]==-1){
            temp_fitness = 0.05;
        }
        else{
            temp_fitness = 0;
            temp_fitness += _results[i][14]*log10(100/_results[i][2])/2.8;
            temp_fitness += _results[i][14]*log10(1/_results[i][11]);
            temp_fitness += _results[i][14]*log10(2/_results[i][12])/1.3;
        }

        for(uint j=0; j<_results[i].size(); j++){
            _results_log << _results[i][j] << ",";
        }
        _results_log << temp_fitness << std::endl;

        fitness.push_back(temp_fitness);
    }

    //_results_log << "- - - -" << std::endl;
    _results_log.close();

    double weighted = 0;
    double divider = 0;

    for(unsigned int i=0; i<fitness.size(); i++){
        weighted += (i+1)*fitness[i];
        divider += (i+1);
    }
    weighted /= divider;

    std::sort(fitness.begin(), fitness.end());

    for(uint i=0; i<fitness.size(); i++){
        std::cout << fitness[i] << " ";
    }
    std::cout << std::endl;

    std::cout << weighted << std::endl;
    std::cout << "----------------------------------" << std::endl << std::endl;

    return weighted;

//    std::sort(fitness.begin(), fitness.end());

//    for(uint i=0; i<fitness.size(); i++){
//        std::cout << fitness[i] << " ";
//    }
//    std::cout << std::endl;

//    std::cout << fitness[fitness.size()/2] << std::endl;
//    std::cout << "----------------------------------" << std::endl << std::endl;

//    return fitness[fitness.size()/2];
}

double MirmorphSimulation::escale(double input, double min, double max){
    double output;

    output = input * (max-min) + min;

    return output;
}

double MirmorphSimulation::average_filter(std::vector<double> values, double new_value, int size){
    double average = 0;
    if(values.size() < size){
        values.push_back(new_value);

        for(uint i=0; i<values.size(); i++){
            average += values[i];
        }
        average /= values.size();
    }
    else{
        values.erase(values.begin());
        values.push_back(new_value);

        for(int i=0; i<size; i++){
            average += values[i];
        }
        average /= size;
    }

    return average;
}
