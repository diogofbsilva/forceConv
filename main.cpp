/*
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo, Matteo Fumagalli
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
@ingroup icub_module

\defgroup forceConversion forceConversion

Converts the wrench from the F/T Sensor and the force 
from the hand tactile sensors to the base reference frame

\author Diogo Silva

\section intro_sec Description

This module estimates two external wrenches on the iCub base reference frame, given the wrench obtained from
the 6-axis force/torque (FT) sensor's measurements, which are
acquired through an input YARP port and the other measure is obtained from the tactile sensors force estimated.

\section lib_sec Libraries
- skinDynLib
- iDyn library.

\section example_sec Example
By launching the following command:

\code
forceConversion --rate 10
\endcode

\author Diogo Silva

This file can be edited at src/forceConversion/main.cpp.
*/

#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <iostream>
#include <iomanip>
#include <cstring>

#include "forceConv.h" 

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinDynLib;
using namespace std;

class ForceConversion: public RFModule
{
private:

    Property OptionsRightArm;
    Property OptionsLeftArm;
    bool     torso_enabled;
    bool     left_arm_enabled;
    bool     right_arm_enabled;
    bool     dummy_ft;

      // true: when skin detects no contact, the ext contact is supposed at the end effector
                                    // false: ext contact is supposed at the last location where skin detected a contact

    //dataFilter *inertialFilter{}; será necessário??
    Port rpcPort;

    baseConversion *base_conv;
    IThreeAxisLinearAccelerometers* m_iAcc{nullptr};
    IThreeAxisGyroscopes* m_iGyro{nullptr};
    ISixAxisForceTorqueSensors* m_left_arm_FT{nullptr};
    ISixAxisForceTorqueSensors* m_right_arm_FT{nullptr};
    PolyDriver  dd_left_arm_FT_MASClient;
    PolyDriver  dd_right_arm_FT_MASClient;
    PolyDriver *dd_right_arm;
    PolyDriver *dd_left_arm;

public:

    ForceConversion()
        {
            base_conv=nullptr;
            left_arm_enabled = true;
            torso_enabled = true;
            right_arm_enabled = true;
            dd_right_arm=nullptr;
            dd_left_arm=nullptr;
            dummy_ft = false;
        }
    
    virtual bool createDriver(PolyDriver *&_dd, Property options)
    {
        int trials=0;
        double start_time = yarp::os::Time::now();

        do
        {
            double current_time = yarp::os::Time::now();

            //remove previously existing drivers
            if (_dd)
            {
                delete _dd;
                _dd=0;
            }

            //creates the new device driver
            _dd = new PolyDriver(options);
            bool connected =_dd->isValid();

            //check if the driver is connected
            if (connected) break;
        
            //check if the timeout (60s) is expired
            if (current_time-start_time > 60.0)
            {
                yError("It is not possible to instantiate the device driver. I tried %d times!\n", trials);
                return false;
            }

            yarp::os::Time::delay(5);
            trials++;
            yWarning("\nUnable to connect the device driver, trying again...\n");
        }
        while (true);

        IEncoders *encs;

        bool ok = true;
        ok = ok & _dd->view(encs);
        if (!ok)
        {
            yError("one or more devices has not been viewed\nreturning...");
            return false;
        }

        return true;
    }
    
    bool configure(ResourceFinder &rf) override
    // Epa estas opções todas estão a mais, é eliminar isto??? É PRECISO??
    // ISTO AQUI VAI muita coisa para o lixo!!
    {
        //---------------------LOCAL NAME-----------------------//
        string local_name = "forceConversion";
        if (rf.check("local"))
        {
            local_name=rf.find("local").asString();
        }


        //-----------------GET THE ROBOT NAME-------------------//
        string robot_name;
        if (rf.check("robot"))
             robot_name = rf.find("robot").asString();
        else robot_name = "icub";


        //-----------------CHECK IF AUTOCONNECT IS ON-----------//
        bool autoconnect;
        if (rf.check("autoconnect"))
        {
            yInfo("'autoconnect' option enabled.\n");
            autoconnect = true;
        }
        else
        {
            yInfo("disabled \n");
            autoconnect = true;
        }

        //--------------- GAINS -------------------------//

        float gain_thumb;
        float gain_middle;
        float gain_index;
        float gain_ring;
        float gain_little;
        float gain_palm;

        if (rf.check("gain_thumb"))
        {
            gain_thumb = rf.find("gain_thumb").asFloat32();
        }else{
            gain_thumb = 1.0;
            yError ("'gain_thumb' parameter is deprecated. Use 1.0 instead");
        }

        if (rf.check("gain_index"))
        {
            gain_index = rf.find("gain_index").asFloat32();
        }else{
            gain_index = 1.0;
            yError ("'gain_index' parameter is deprecated. Use 1.0 instead");
        }

        if (rf.check("gain_middle"))
        {
            gain_middle = rf.find("gain_middle").asFloat32();
        }else{
            gain_middle = 1.0;
            yError ("'gain_middle' parameter is deprecated. Use 1.0 instead");
        }

        if (rf.check("gain_ring"))
        {
            gain_ring = rf.find("gain_ring").asFloat32();
        }else{
            gain_ring = 1.0;
            yError ("'gain_ring' parameter is deprecated. Use 1.0 instead");
        }

        if (rf.check("gain_little"))
        {
            gain_little = rf.find("gain_little").asFloat32();
        }else{
            gain_little = 1.0;
            yError ("'gain_little' parameter is deprecated. Use 1.0 instead");
        }

        if (rf.check("gain_palm"))
        {
            gain_palm = rf.find("gain_palm").asFloat32();
        }else{
            gain_palm = 1.0;
            yError ("'gain_palm' parameter is deprecated. Use 1.0 instead");
        }


        //---------------------RATE/PERIOD-----------------------------//
        int rate = 10;
        if (rf.check("period"))
        {
            rate = rf.find("period").asInt32();
            yInfo("rateThread working at %d ms\n", rate);
        }
        else
        {
            yInfo("Could not find period in the config file\nusing 10ms as default");
            rate = 10;
        }

        if (rf.check("rate"))
        {
            yError ("'rate' parameter is deprecated. Use 'period' instead");
            return false;
        }
        //std::string remoteInertialName{"/"+robot_name+"/head/inertials"};

        // ---------- Necessary to read the left arm force torque data -------------//

        /*

        if (left_arm_enabled)
        {
            OptionsRightArm.put("device","remote_controlboard");
            OptionsRightArm.put("local","/"+local_name+"/left_arm/client");
            OptionsRightArm.put("remote","/"+robot_name+"/left_arm");

            if (!createDriver(dd_left_arm, OptionsLeftArm))
            {
                yError("unable to create right arm device driver...quitting\n");
                return false;
            }
        }

        */
        

        // ---------- Necessary to read the left arm force torque data -------------//

        /* NO ICUB
        if (right_arm_enabled)
        {
            OptionsRightArm.put("device","remote_controlboard");
            OptionsRightArm.put("local","/"+local_name+"/right_arm/client");
            OptionsRightArm.put("remote","/"+robot_name+"/right_arm");

            if (!createDriver(dd_right_arm, OptionsRightArm))
            {
                yError("unable to create right arm device driver...quitting\n");
                return false;
            }
        }
        */

        

        //---------------------FLAGS-------------------------//
        int FT = 0;
        int skin = 0;
        if (rf.check("FT"))
        {
            FT = rf.find("FT").asInt32();
            yInfo("Using FT: %d\n", FT);
        }

        if (rf.check("skin"))
        {
            skin = rf.find("skin").asInt32();
            yInfo("Using Skin: %d\n", skin);
        }

        string arm;
        if (rf.check("arm"))
        {
            arm = rf.find("arm").asString();
        }
        else arm = "left";

        //--------------------CHECK FT SENSOR------------------------
        /* ROBOT REAL
        
        if (!dummy_ft)
        {
            if ((dd_right_arm && !Network::exists("/" + robot_name + "/right_arm/analog:o")))
                {     
                    yError("Unable to detect the presence of F/T sensors in your iCub...quitting\n");
                    return false;
                }
        }  
        */


        //---------------OPEN RPC PORT--------------------//
        string rpcPortName = "/"+local_name+"/rpc:i";
        rpcPort.open(rpcPortName);
        attach(rpcPort);

        //--------------------------THREAD--------------------------
        base_conv = new baseConversion(rate, FT, skin, arm, robot_name, local_name, autoconnect,m_left_arm_FT, m_right_arm_FT, gain_index, gain_little, gain_middle, gain_thumb, gain_ring, gain_palm);

        yInfo("thread istantiated...\n");
        Time::delay(2.0);

        if(!base_conv->start()) { // this calls threadInit() and it if returns true, it then calls run()
            yError() << "[forceConv] Could not start the compensator thread.";
            return false;
        }else{
            yInfo("thread started\n");
        }
        return true;
    }

    bool close() override
    {
        //The order of execution of the following closures is important, do not change it.
        yInfo("Closing forceConversion module... \n");

        if (base_conv)
          {
            thread_status_enum thread_status = base_conv->getThreadStatus();
            if (thread_status!=STATUS_DISCONNECTED)
            {
                yInfo("Setting the icub in stiff mode\n");
                base_conv->setStiffMode();
            }
            yInfo("Stopping the base_conv thread...");
            base_conv->stop();
            yInfo("base_conv thread stopped\n");
            delete base_conv;
            base_conv=nullptr;
          }

        yInfo("Closing the rpc port \n");
        rpcPort.close();

        yInfo("forceConversion module was closed successfully! \n");
        return true;
    }

    double getPeriod() override
    {
        return 1.0;
    }
    bool updateModule() override
    {
        double avgTime, stdDev, period;
        period = base_conv->getPeriod();
        base_conv->getEstimatedPeriod(avgTime, stdDev);
        if(avgTime > 1.3 * period){
        //   yDebug("(real period: %3.3f +/- %3.3f; expected period %3.3f)\n", avgTime, stdDev, period);
        }

        static unsigned long int alive_counter = 0;
        static double curr_time = Time::now();
        if (Time::now() - curr_time > 60)
        {
            yInfo ("forceConversion is alive! running for %ld mins.\n",++alive_counter);
            curr_time = Time::now();
        }

        if (base_conv==nullptr)
            return false;
        thread_status_enum thread_status = base_conv->getThreadStatus();
        if (thread_status==STATUS_OK)
            return true;
        else if (thread_status==STATUS_DISCONNECTED)
        {
            yError ("forceConversion module lost connection with iCubInterface, now closing...\n");
            return false;
        }
        else
        {
            yInfo("forceConversion module was closed successfully! \n");
            return true;
        }

    }

};

int main(int argc, char * argv[])
{
    ResourceFinder rf;
    rf.setDefaultContext("forceConversion");
    rf.setDefaultConfigFile("/home/vislab/thesis_diogo_silva/Code_Implementation/forceConversion/forceConversion_sim.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout << "For now we do not provide help, SORRY :(" << endl;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return 1;
    }

    ForceConversion obs;
    return obs.runModule(rf);
}