/*****************************************************************************
 *  Copyright (C) 2012 by Timo Nachstedt                                     *
 *                                                                           *
 *  This program is free software: you can redistribute it and/or modify     *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  This program is distributed in the hope that it will be useful,          *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                           *
 ****************************************************************************/


#ifndef ANN_H_
#define ANN_H_

#include <list>
#include <vector>
#include "transferfunction.h"

// forward declarations
class Neuron;
class Synapse;

/**
 * Artificial Neural Network Class
 *
 * This is a class to represent a time-discrete recurrent neural network.
 * You can subclass this class to obtain classes for special network
 * configurations (such as VRN, PSN, CPG ...). It should be sufficient to
 * include this file into your project. Under normal circumstances you don't
 * need to know anything about the underlying neuron and synapses classes.
 */
class ANN {
public:
    /**
     * The constructor.
     */
    ANN();

    /**
     * The destructor.
     */
    virtual ~ANN();

    /**
     * Returns the activity of the neuron with the given number
     *
     * This method can be used to get the activity value of the neuron with
     * the given number directly belonging to this network. You cannot access
     * the activity values of neurons of other networks or subnetworks with this
     * method. If this is necessary use the method of the specific network
     * or ANN::getBias(Neuron*) with a pointer to the desired neuron
     * instead.
     *
     * @param neuron index of the neuron
     * @return activity value
     */
    const double& getActivity(const int neuron) const;

    /**
     * Returns the activity of the given neuron
     *
     * You can use this function to retrieve the activity of any neuron without
     * having to include the neuron.h header file. If you don't mind including
     * the additional header you can as well use Neuron::getActivity().
     *
     * @param neuron pointer to the neuron
     * @return activity value
     */
    static const double& getActivity(Neuron const * neuron);

    /**
     * Returns the bias of the neuron with the given number
     *
     * This method can be used to get the bias value of the neuron with
     * the given number directly belonging to this network. You cannot access
     * the bias values of neurons of other networks or subnetworks with this
     * method. If this is necessary use the method of the specific network
     * or ANN::getBias(Neuron*) with a pointer to the desired neuron
     * instead.
     *
     * @param neuron index of the neuron
     * @return bias value
     */
    const double& getBias(const int neuron) const;

    /**
     * Returns the bias of the given neuron
     *
     * You can use this function to retrieve the bias of any neuron without
     * having to include the neuron.h header file. If you don't mind including
     * the additional header you can as well use Neuron::getBias().
     *
     * @param neuron pointer to the neuron
     * @return bias value
     */
    static const double& getBias(Neuron const * neuron);

    /**
     * Returns a pointer to the default transfer function of the network
     *
     * @return pointer to the default transfer function
     */
    TransferFunction const* getDefaultTransferFunction() const;

    /**
     * Returns a pointer to the neuron with the given number
     *
     * This method can be used to get direct access to a neuron belonging to
     * this network or to store a pointer to a specific neuron when working
     * with multiple or sub networks.
     *
     * @param index index of the neuron
     * @return pointer to the neuron
     */
    Neuron* getNeuron(unsigned int const index);

    /**
     * Returns the number of neurons of this network
     *
     * This method returns the number of neurons directly belonging to this
     * network. Neurons of sub networks are not considered. Use
     * ANN::getTotalNeuronNumber() instead if you want to consider neurons
     * of all sub networks as well.
     *
     * @return number of neurons
     */
    unsigned int getNeuronNumber() const;

    /**
     * Returns the output of the neuron with the given number
     *
     * This method can be used to get the output value of the neuron with
     * the given number directly belonging to this network. You cannot access
     * the output values of neurons of other networks or subnetworks with this
     * method. If this is necessary use the method of the specific network
     * or ANN::getOutput(Neuron*) with a pointer to the desired neuron
     * instead.
     *
     * @param neuron index of the neuron
     * @return output value
     */
    const double& getOutput(const int neuron) const;

    /**
     * Returns the output of the given neuron
     *
     * You can use this function to retrieve the output of any neuron without
     * having to include the neuron.h header file. If you don't mind including
     * the additional header you can as well use Neuron::getOutput().
     *
     * @param neuron pointer to the neuron
     * @return output value
     */
    static const double& getOutput(Neuron const * neuron);

    /**
     * Returns synapse between two neurons of this network
     *
     * This methods returns the synapse between the given presynaptic and
     * postsynaptic neuron directly belonging to this network. If no such
     * synapse exists NULL is returned.
     *
     * @param post index of postsynaptic neuron
     * @param pre index of presynaptic neuron
     * @return pointer to synapse or NULL if not existent
     */
    Synapse* getSynapse(const unsigned int& post, const unsigned int& pre)const;

    /**
     * Returns synapse between any two neurons
     *
     * This method returns the synapse between the given presynaptic and
     * postsynaptic neuron that do not have to belong to the same network. If
     * no such synapse exists NULL is returned.
     *
     * @param post pointer to postsynaptic neuron
     * @param pre pointer to presynaptic neuron
     * @return pointer to synapse or NULL if not existent
     */
    static Synapse* getSynapse(Neuron const * const post,
            Neuron const * const pre);

    /**
     * Returns the number of neurons of this network including sub networks
     *
     * This method returns the number of neurons belonging to this
     * network and all of its sub networks (and their sub networks and so
     * on...). Use ANN::getNeuronNumber() instead if you only want to know the
     * number of neurons belonging directly to this network (without considering
     * sub networks).
     *
     * @return number of neurons
     */
    unsigned int getTotalNeuronNumber() const;

    /**
     * Returns the weight of the synapse between two neurons of this network
     *
     * This method returns the weight of the synapse between two neurons defined
     * by their indexes or 0 if their is no such synapse. This only works for
     * neurons directly belonging to this network. If you want to retrieve
     * synaptic weights between neurons of different or sub networks you have to
     * use ANN::getWeight(Neuron*, Neuron*).
     *
     * @param post index of the postsynaptic neuron
     * @param pre  index of the presynaptic neuron
     * @post weight of the synapse (or 0 if not present)
     */
    const double getWeight(const int& post, const int& pre) const;

    /**
     * Returns the weight of the synapse between any two neurons
     *
     * This method returns the weight of the synapse between the two given
     * neurons or 0 if their is no such synapse. For neurons directly belonging
     * to the same network you can as well use ANN::getWeight(int, int).
     *
     * @param post pointer to the postsynaptic neuron
     * @param pre  pointer to the presynaptic neuron
     * @return weight of the synapse (or 0 if not present)
     */
    const double getWeight(Neuron const * post, Neuron const * pre) const;

    /**
     * Return pointer to a identity function object
     *
     * @return pointer to identity function object
     */
    static LinearFunction const * const identityFunction();

    /**
     * Return pointer to a LogisticFunction object
     *
     * @return pointer to LogisticFunction object
     */
    static LogisticFunction const * const logisticFunction();

    /**
     * Returns a pointer to the neuron with the given number
     *
     * This is an abbreviation for Neuron* getNeuron(unsigned int const index)
     *
     * @param index index of the neuron
     * @return pointer to the neuron
     */
    Neuron* n(unsigned int const index);

    /**
     * Do stuff at the end of simulation step
     *
     * When using ANN::step() this method is called in every simulation step
     * after updating the neuron outputs. Overload it if you need to do some
     * stuff in every time step. The default implementation does nothing.
     */
    virtual void postProcessing();

    /**
     * Removes neuron from the network
     *
     * This method removes the given neuron from the network. All the synapses
     * attached to the neuron are removed as well. The neuron is only removed
     * if it indeed belongs to the network on which this method was called.
     *
     * @param neuron pointer to the neuron
     */
    void removeNeuron(Neuron const * neuron);

    /**
     * Sets the activity of the neuron with the given index
     *
     * This method can be used to set the activity value of the neuron with the
     * given index directly belonging to this network. You cannot set the
     * activity values of neurons of other networks or subnetworks with this
     * method. If this is necessary use the method of the specific network or
     * ANN::setActivity(Neuron*,double) with a pointer to the desired neuron
     * instead.
     *
     * @param neuron    index of the neuron
     * @param aactivity new activity value
     */
    void setActivity(const int& neuron, const double& aactivity);

    /**
     * Sets the input of the given neuron
     *
     * You can use this function to set the activity of any neuron without
     * having to include the neuron.h header file. If you don't mind including
     * the additional header you can as well use Neuron::setActivity(double).
     *
     * @param neuron    pointer to the neuron
     * @param aactivity new activity value
     */
    static void setActivity(Neuron * neuron, const double& aactivity);

    /**
     * Sets the transfer functions of all neurons
     *
     * With this method you can set the transfer function used for all neurons.
     * You can decide whether also the transfer functions of neurons belonging
     * to subnets should be included in the setting.
     * If you want to change also which transfer function will be used for
     * neurons added in the future use ANN::setDefaultTransferFunction()
     *
     * @param func           pointer to the TransferFunction object
     * @param includeSubnets decides whether neurons of subnets are also
     *                       affected
     */
    void setAllTransferFunctions(TransferFunction const * const func,
            const bool& includeSubnets=true);

    /**
     * Sets the bias of the given neuron
     *
     * You can use this function to set the bias of any neuron without
     * having to include the neuron.h header file. If you don't mind including
     * the additional header you can as well use Neuron::setBias(double).
     *
     * @param neuron pointer to the neuron
     * @param abias  new bias value
     */
    static void setBias(Neuron * neuron, const double& abias);

    /**
     * Sets the bias of the neuron with the given index
     *
     * This method can be used to set the bias of the neuron with the given
     * index directly belonging to this network. You cannot set the bias values
     * of neurons of other networks or subnetworks with this method. If this is
     * necessary use the method of the specific network or ANN::setBias(Neuron*)
     * with a pointer to the desired neuron instead.
     *
     * @param neuron index of the neuron
     * @param abias  new bias value
     */
    void setBias(const int& neuron, const double& abias);

    /**
     * Sets the default transfer function of this network
     *
     * With this method you can define the default transfer function used for
     * new neurons of this network. This only affects neurons which are created
     * after calling ANN::setDefaultTransferFunction().
     *
     * @param func pointer to the new default transfer function
     */
    void setDefaultTransferFunction(TransferFunction const * const func);

    /**
     * Sets the input of the neuron with the given index
     *
     * This method can be used to set the external input to the neuron with the
     * given index directly belonging to this network. You cannot set the input
     * values of neurons of other networks or subnetworks with this method. If
     * this is necessary use the method of the specific network or
     * ANN::setInput(Neuron*,double) with a pointer to the desired neuron
     * instead.
     *
     * @param neuron index of the neuron
     * @param ainput new input value
     */
    void setInput(const int& neuron, const double& ainput);

    /**
     * Sets the input of the given neuron
     *
     * You can use this function to set the input of any neuron without
     * having to include the neuron.h header file. If you don't mind including
     * the additional header you can as well use Neuron::setInput(double).
     *
     * @param neuron pointer to the neuron
     * @param ainput new input value
     */
    static void setInput(Neuron * neuron, const double ainput);

    /**
     * Sets the output of the neuron with the given index
     *
     * This method can be used to set the output value of the neuron with the
     * given index directly belonging to this network. You cannot set the output
     * values of neurons of other networks or subnetworks with this method. If
     * this is necessary use the method of the specific network or
     * ANN::setOutput(Neuron*,double) with a pointer to the desired neuron
     * instead.
     *
     * @param neuron  index of the neuron
     * @param aoutput new output value
     */
    void setOutput(const int& neuron, const double& aoutput);

    /**
     * Sets the output of the given neuron
     *
     * You can use this function to set the output value of any neuron without
     * having to include the neuron.h header file. If you don't mind including
     * the additional header you can as well use Neuron::setOutput(double).
     *
     * @param neuron  pointer to the neuron
     * @param aoutput new output value
     */
    static void setOutput(Neuron * neuron, const double aoutput);

    /**
     * Sets transfer function for a neuron of this net
     *
     * This method allows you to set an individual transfer function for a
     * neuron belonging to this network. You cannot set the transfer function
     * of neurons of other networks or subnetworks with this method. If
     * this is necessary use the method of the specific network or
     * ANN::setTransferFunction(Neuron*,TransferFunction) with a pointer to the
     * desired neuron instead.
     *
     * @param neuron index of the neuron
     * @param func   pointer to the transfer function object
     */
    void setTransferFunction(const int neuron,
            TransferFunction const * const func);

    /**
     * Sets the transfer function of the given neuron
     *
     * You can use this function to set the transfer function of any neuron
     * without having to include the neuron.h header file. If you don't mind
     * including the additional header you can as well use
     * Neuron::setTransferFunction(TransferFunction*).
     *
     * @param neuron pointer to the neuron
     * @param func   pointer to the transfer function object
     */
    void setTransferFunction(Neuron * neuron,
            TransferFunction const * const func);

    /**
     * Sets the weight of the synapse between any two neurons
     *
     * Use this method to set the weight of the synapse between any two given
     * neurons. The neurons do not have to belong to the same network.
     * If a synapse between those two neurons already exists its weight
     * is altered. If no synapse exists a new synapse with the given weight is
     * created. For neurons directly belonging to the same network you can as
     * well use ANN::setWeight(int, int, double).
     *
     * @param post   pointer to the postsynaptic neuron
     * @param pre    pointer to the presynaptic neuron
     * @param weight new weight of the synapse
     */
    static void setWeight(Neuron* post, Neuron* pre, const double weight);

    /**
     * Sets the weight of the synapse between two neurons of this network
     *
     * Use this method to set the weight of the synapse between the neurons
     * with the defined indexes of this network. If a synapse between those two
     * neurons already exists its weight is altered. If no synapse exists a new
     * synapse with the given weight is created. For belonging to different
     * networks use ANN::setWeight(Neuron*, Neuron*, double).
     *
     * @param post   index of the postsynaptic neuron
     * @param pre    index of the presynaptic neuron
     * @param weight new weight of the synapse
     */
    void setWeight(const int post, const int pre, const double weight);

    /**
     * Does one simulation step
     *
     * This a convenience function which calls the following functions in the
     * given order:
     * - ANN::updateActivities()
     * - ANN::updateWeights()
     * - ANN::updateOutputs()
     */
    virtual void step();

    /**
     * Returns pointer to a TanhFunction object
     *
     * @return pointer to TanhFunction object
     */
    static TanhFunction const * const tanhFunction();

    /**
     * Updates neuron activities
     *
     * This method iterates over all neurons in this network and all sub
     * networks and updates their activity values based on all incoming
     * synaptic weights and the presynaptic outputs.
     */
    virtual void updateActivities();

    /**
     * Updates neuron outputs
     *
     * This method iterates over all neurons in this network and all sub
     * networks and updates their output values based on their activity values.
     */
    virtual void updateOutputs();

    /**
     * Updates synaptic weights
     *
     * This method can be overriden if you want to include synaptic plasticity.
     * The default implementation does nothing.
     */
    virtual void updateWeights();
protected:
    /**
     * Defines the neuron number of this network
     *
     * With this function you can set the desired number of neurons for this
     * network. If less neurons are currently present neurons are added, if more
     * neurons are present the neurons with the highest indexes are removed.
     *
     * @param anumber new number of neurons directly belonging to this net
     */
    void setNeuronNumber(const unsigned int& anumber);

    /**
     * Adds a neuron to the network
     *
     * This methods creates a new neuron and adds it to the network.
     *
     * @return pointer to the newly created neuron
     */
    Neuron* addNeuron();

    /**
     * Adds a sub network to this network
     *
     * This method adds the given network as a sub network of this network.
     * This basically means that activities, outputs and weights are updated
     * synchronously
     *
     * @param subnet Pointer to the ANN that should be added as sub network
     */
    void addSubnet(ANN* subnet);

    /**
     * Adds a synapse between two neurons
     *
     * This method creates a new synapse between the two given neurons.
     *
     * @param post pointer to the postsynaptic neuron
     * @param pre  pointer to the presynaptic neuron
     * @return pointer to the newly created synapse
     */
    static Synapse* addSynapse(Neuron* post, Neuron* pre);

    /**
     * Sets the bias of a neuron of this network
     *
     * This method is an abbreviation of ANN::setBias(int, double).
     *
     * @param neuron index of the neuron
     * @param abias new bias value of this neuron
     */
    void b(const int neuron, const double& abias);

    /**
     * Sets the bias of a any neuron
     *
     * This method is an abbreviation of ANN::setBias(Neuron*, double).
     *
     * @param neuron pointer to the neuron
     * @param abias new bias value of this neuron
     */
    void b(Neuron* neuron, const double& abias);

    /**
     * Returns the bias of a neuron of this network
     *
     * This method is an abbreviation of ANN::getBias(int).
     *
     * @param  neuron index of the neuron
     * @return bias value of the neuron
     */
    const double& b(const int neuron);

    /**
     * Sets the synaptic weight between two neurons of this network
     *
     * This method is an abbreviation of ANN::setWeight(int,int,double).
     *
     * @param post    index of the postsynaptic neuron
     * @param pre     index of the presynaptic neuron
     * @param aweight new weight of the synapse
     */
    void w(const int& post, const int& pre, const double& aweight);

    /**
     * Sets the synaptic weight between any two neurons
     *
     * This method is an abbreviation of
     * ANN::setWeight(Neuron*,Neuron*,double).
     *
     * @param post    pointer to the postsynaptic neuron
     * @param pre     pointer to the presynaptic neuron
     * @param aweight new weight of the synapse
     */
    void w(Neuron* post, Neuron* pre, const double& aweight);

    /**
     * Returns the weight of a synapse between two neurons of this network
     *
     * This method is an abbreviation of ANN::getWeight(int,int).
     *
     * @param post index of the postsynaptic neuron
     * @param pre  index of the presynaptic neuron
     * @return synaptic weight or 0 if synapse not present
     */
    const double w(const int& post, const int& pre);

private:
    // f(x) = tanh(x)
    static TanhFunction const * const tanhFunctionPointer;

    // f(x) = 1./(1+exp(-x))
    static LogisticFunction const * const logisticFunctionPointer;

    // f(x) = x
    static LinearFunction const * const identityFunctionPointer;

    typedef std::vector <Neuron*>   NeuronList;
    typedef std::list <ANN*>        AnnList;
    NeuronList              neurons;
    AnnList                 subnets;
    TransferFunction const* defaultTransferFunction;
};


#endif /* ANN_H_ */
