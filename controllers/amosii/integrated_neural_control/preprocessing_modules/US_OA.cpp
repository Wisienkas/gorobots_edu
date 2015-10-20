/*
 * US_OA.cpp
 *
 *  Created on: Feb 21, 2015
 *      Author: degoldschmidt
 */

#include "US_OA.h"


/// Obstacle avoidance modes
/// case 1: sum each recurrent and each inhibitory  weights;;
/// case 2: sum only recurrent weights;;
/// case 3: sum only inhibitory weights;
/// case 4: learn recurrent, inhibitory weights static!!!
/// case 5: learn inhibitory, recurrent weights static!!!
/// case 6: full static MRC
/// case 7: BRAITENBERG
/// case 8: winner takes it all
/// case 9: default: sum nothing

US_Obstacleavoidance::US_Obstacleavoidance(int mode){

    //write OA values on the console if TRUE
    debug=0;

    //Neuron 0 and 1 are input layer neurons, 2 and 3 are processing neurons. Neurons 5 and 6 are output layer.
    setNeuronNumber(6);
    setTransferFunction(getNeuron(0),identityFunction());
    setTransferFunction(getNeuron(1),identityFunction());
    setTransferFunction(getNeuron(4),identityFunction());
    setTransferFunction(getNeuron(5),identityFunction());



    neuron_weightsum_inhib=0;
    //learning and forgetting parameters for adaptive MRC
    mu    =0.0065;    //TODO: learn1 = new SPaSS(mu, gamma, vt);
    gamma =0.0003;

    mu2   =0.015;    //TODO: learn2 = new SPaSS(mu2, gamma2, vt2);
    gamma2=0.0003;

    //Reflex cut, below this limit the incoming signal is ignored
    reflex_cut=0.2;

    //Initialization of the input output layer
    i1=0.;
    i2=0.;
    u1=0.;
    u2=0.;
    v1=0.;
    v2=0.;

    //Initial synapse values of the adaptive MRC for neuron 2 and 3
    weight_neuron1=2.;
    weight_neuron2=2.;
    weight_neuron3=3.;
    weight_neuron4=3.;

    //Weight for input layer for neuron 0 and 1
    gain=4.7;

    //Threshold for cutting off output signals of neuron 2 and 3.
    e=0.009;

    //Different "modes" the network can be connected. Described in the constructor above.
    modes = mode;

    switch (mode) {

    case 1:
        neuron_weightsum_inhib=(weight_neuron3+weight_neuron4)*0.5;
        //recurrent
        w(2, 2,  0.5*(weight_neuron1+weight_neuron2));
        w(3, 3,  0.5*(weight_neuron1+weight_neuron2));
        //inhibitory
        w(3, 2,  -neuron_weightsum_inhib);
        w(2, 3,  -neuron_weightsum_inhib);
        break;

    case 2:
        //recurrent
        w(2, 2,  0.5*(weight_neuron1+weight_neuron2));
        w(3, 3,  0.5*(weight_neuron1+weight_neuron2));
        //inhibitory
        w(3, 2,  -weight_neuron3);
        w(2, 3,  -weight_neuron4);
        break;

    case 3://Best algorithm, adaptive two recurrent network, Eduard Thesis
        neuron_weightsum_inhib=(weight_neuron3+weight_neuron4)*0.5;
        //recurrent
        w(2, 2,  weight_neuron1);
        w(3, 3,  weight_neuron2);
        //inhibitory
        w(3, 2,  -neuron_weightsum_inhib);
        w(2, 3,  -neuron_weightsum_inhib);
        break;

    case 4:
        //recurrent
        w(2, 2,  weight_neuron1);
        w(3, 3,  weight_neuron2);
        //inhibitory
        w(3, 2,  -3.5);
        w(2, 3,  -3.5);
        break;

    case 5:
        //recurrent
        w(2, 2,  2.4);
        w(3, 3,  2.4);
        //inhibitory
        w(3, 2,  -weight_neuron3);
        w(2, 3,  -weight_neuron4);
        break;

    case 6:// MRC non adaptive
        //recurrent
        w(2, 2,  2.4);//2.4
        w(3, 3,  2.4);//2.4
        //inhibitory
        w(3, 2,  -3.5);//-3.5
        w(2, 3,  -3.5);//-3.5
        break;

    case 7://Braitenberg
        //recurrent
        w(2, 2,  0);
        w(3, 3,  0);
        //inhibitory
        w(3, 2,  0);
        w(2, 3,  0);
        break;

    case 8:
        //recurrent
        w(2, 2,  0);
        w(3, 3,  0);
        //inhibitory
        w(3, 2,  -weight_neuron3);
        w(2, 3,  -weight_neuron4);
        break;

    default:
        //recurrent
        w(2, 2,  weight_neuron1);
        w(3, 3,  weight_neuron2);
        //inhibitory
        w(3, 2,  -weight_neuron3);
        w(2, 3,  -weight_neuron4);
        break;
    }

    //Scaling term in my tezlaff learning!
    //Negative will lead to a fixpoint for the synaptic weight at 0, so the agent forgets all learned weights
    vt = -0.01;
    vt2= -0.01;

    //Initialize OA CPG like network
    w(2,0,gain);
    w(3,1,gain);
    w(4,2,    1);
    w(5,3,    1);
}

US_Obstacleavoidance::~US_Obstacleavoidance(){

}

void US_Obstacleavoidance::step(){
    //2013 by Eduard Grinke, BA-Thesis

    //Input conversion of neuron 0 and 1 to 0...1
    i1=0.5*(getOutput(0)+1.);                        //
    i2=0.5*(getOutput(1)+1.);
    //Output conversion of neuron 2 and 3 to 0...1
    u1=0.5*(getOutput(2)+1.);
    u2=0.5*(getOutput(3)+1.);
    //Output of neuron 2 and 3 conversion to 0...1
    v1=0.5*(getOutput(2)+1);
    v2=0.5*(getOutput(3)+1);

    //Reflexcut for input signal
    if(i1>reflex_cut){i1_refl=1.;} else{i1_refl=0.;}
    if(i2>reflex_cut){i2_refl=1.;} else{i2_refl=0.;}
    //Cut-off for neurons 2 and 3 output at time "t"
    if(u1<e)u1=0.;
    if(u2<e)u2=0.;
    //Cut-off for neurons 2 and 3 output at time "t+1"
    if(v1<e)v1=0.;
    if(v2<e)v2=0.;

    updateActivities();
    updateOutputs();

    //Applying tetzlaffs modified adaptive algorithm
    double dweight1 = mu *u1*v1*i1_refl+ gamma* (vt-v1)  *weight_neuron1* weight_neuron1;    // TODO: learn1->step()
    double dweight2 = mu *u2*v2*i2_refl+ gamma* (vt-v2)  *weight_neuron2* weight_neuron2;
    double dweight3 = mu2*u1*v1*i1_refl+ gamma2*(vt2-v1) *weight_neuron3* weight_neuron3;    // TODO: learn2->step()
    double dweight4 = mu2*u2*v2*i2_refl+ gamma2*(vt2-v2) *weight_neuron4* weight_neuron4;

    // TODO: dw() for different cases
    //recurrent
    dw(2, 2,  dweight1);
    dw(3, 3,  dweight2);
    //inhibitory
    dw(3, 2,  -(dweight3+dweight4)*0.5);
    dw(2, 3,  -(dweight3+dweight4)*0.5);

    updateWeights();

    //OA values to Console output
    if(debug==1){
        cout<<setprecision(8)<<"i1:"<<i1<<" "<<"u1: "<<u1<<"\t v1: " << v1 <<"\t w1: "<< ANN::getWeight(2,2)<<endl;
        cout<<setprecision(8)<<"i2:"<<i2<<" "<<"u2: "<<u2<<"\t v2: " << v2 <<"\t w2: "<< ANN::getWeight(3,3)<<endl;
        cout<<setprecision(8)<<"\t w3: "<< ANN::getWeight(3,2)<<endl;
        cout<<setprecision(8)<<"\t w4: "<< ANN::getWeight(2,3)<<endl;
    }
}

