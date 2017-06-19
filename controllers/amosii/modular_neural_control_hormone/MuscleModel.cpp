/*
 * MuscleModel.cpp
 *
 *  Created on: May 26, 2014
 *  Author: Subhi Shaker Barikhan
 */

#include "MuscleModel.h"

MuscleModel::MuscleModel(int amosVersion) {
	//----------------muscle initializations---------------//

		for (int i =0;i<6;i++)
		{
		K1[i] = 3;//3;
		K2[i] = 6;//6;

		D1[i] = 1;
		D2[i] = 1;


		}


		K2[1] = K2[1] -1;
		K2[4] = K2[4] -1;

	  K2[2] = K2[2] -2;
	  K2[5] = K2[5] -2;


	dSWFTiOffset = FTIUP;//0.4;//0.4;//0.5;//More less dSWFTiOffset is , more high legs swing.
	dSWFTiFac = -0.1;

	Raw_R_fs_old.resize(3);
	Raw_L_fs_old.resize(3);
	Raw_R_fs.resize(3);
	Raw_L_fs.resize(3);
	m_reflex_muscle.resize(19);
	m_Prereflex.resize(19);
	JointAng.resize(19);
	PreJointAng.resize(19);
	JointVel.resize(19);
	PreJointVel.resize(19);

	m_preN.resize(19);//pre motor outputs (19 motors)
	m_preold.resize(19);//pre motor outputs old (19 motors)

		//----------------muscle initializations---------------//

};

MuscleModel::~MuscleModel() {

};

double MuscleModel::VirtualMusclesAcce1(double Ang, double DAng, double t, double Exforce, double MuscleAct, double DisVe, double K, double D, double Leng, double Theta1, double MusclActFac) //step function for FTi joints,
	//see RungeKutta41(…)
	      {


	        double ExVec,ExforceTor,PasiForceTor,ActiForceTor,NeedAngle;

	        ExVec = (Leng + Radius);//*cos(Ang) + DisVe;

	        ExforceTor = Exforce * sin(Ang) * ExVec;
	        PasiForceTor = (2 * ((D * DAng) + (K * Ang)) * Radius) * Radius;
	        ActiForceTor = MuscleAct * Radius *MusclActFac;

	        NeedAngle = (ExforceTor + ActiForceTor - PasiForceTor)/(Mass * Radius * Radius / 2.0);

	        return NeedAngle;

	      }


double  MuscleModel::VirtualMusclesAcceN(double Ang, double DAng, double t, double Exforce, double MuscleAct, double DisVe, double K, double D, double Leng, double Theta1, double MusclActFac) //step function for CTr // joints, see RungeKutta42(…)
  {
    double ExVec,ExforceTor,PasiForceTor,ActiForceTor,NeedAngle;

    ExVec = (Leng + Radius) * cos(Ang) + DisVe;

    ExforceTor = Exforce * ExVec;//Exforce * cos(Theta1) * ExVec;
    PasiForceTor = (2 * ((D * DAng) + (K * Ang)) * Radius) * Radius;
    ActiForceTor = MuscleAct * Radius * MusclActFac;

    NeedAngle = (ExforceTor + ActiForceTor - PasiForceTor)/(Mass * Radius * Radius / 2.0);

    return NeedAngle;

  }

MusclePara MuscleModel::RungeKutta41(double x, double v, double dt, double PreExforce, double Exforce, double PreMuscleAct, double MuscleAct, double DisVec, double K, double D, double Leng, double Theta1, double MusclActFac) // for FTi joints
	      {
	        double x1,v1,a1,x2,v2,a2,x3,v3,a3,x4,v4,a4,xf,vf;

	        x1 = x;
	        v1 = v;
	        a1 = VirtualMusclesAcce1(x1, v1, dt, PreExforce, PreMuscleAct, DisVec, K, D, Leng, Theta1, MusclActFac);

	        x2 = x + (0.5 * v1 * dt);
	        v2 = v + (0.5 * a1 * dt);
	        a2 = VirtualMusclesAcce1(x2, v2, dt/2.0, (PreExforce + Exforce)/2.0, (PreMuscleAct + MuscleAct)/2.0, DisVec, K, D, Leng,Theta1, MusclActFac);

	        x3 = x + (0.5 * v2 * dt);
	        v3 = v + (0.5 * a2 * dt);
	        a3 = VirtualMusclesAcce1(x3, v3, dt/2.0, (PreExforce + Exforce)/2.0, (PreMuscleAct + MuscleAct)/2.0, DisVec, K, D, Leng,Theta1, MusclActFac);

	        x4 = x + (v3 * dt);
	        v4 = v + (a3 * dt);
	        a4 = VirtualMusclesAcce1(x4, v4, dt, Exforce, MuscleAct, DisVec, K, D, Leng,Theta1, MusclActFac);

	        xf = x + (dt/6.0) * (v1 + (2 * v2) + (2 * v3) + v4);
	        vf = v + (dt/6.0) * (a1 + (2 * a2) + (2 * a3) + a4);

	        MusclePara MusPara1 = {xf, vf};

	        return MusPara1;

	      }

MusclePara MuscleModel::RungeKutta42(double x, double v, double dt, double PreExforce, double Exforce, double PreMuscleAct, double MuscleAct, double DisVec, double K, double D, double Leng, double Theta1, double MusclActFac) // for CTr joints
	        {
	          double x1,v1,a1,x2,v2,a2,x3,v3,a3,x4,v4,a4,xf,vf;

	          x1 = x;
	          v1 = v;
	          a1 = VirtualMusclesAcceN(x1, v1, dt, PreExforce, PreMuscleAct, DisVec, K, D, Leng, Theta1, MusclActFac);

	          x2 = x + (0.5 * v1 * dt);
	          v2 = v + (0.5 * a1 * dt);
	          a2 = VirtualMusclesAcceN(x2, v2, dt/2.0, (PreExforce + Exforce)/2.0, (PreMuscleAct + MuscleAct)/2.0, DisVec, K, D, Leng,Theta1, MusclActFac);

	          x3 = x + (0.5 * v2 * dt);
	          v3 = v + (0.5 * a2 * dt);
	          a3 = VirtualMusclesAcceN(x3, v3, dt/2.0, (PreExforce + Exforce)/2.0, (PreMuscleAct + MuscleAct)/2.0, DisVec, K, D, Leng,Theta1, MusclActFac);

	          x4 = x + (v3 * dt);
	          v4 = v + (a3 * dt);
	          a4 = VirtualMusclesAcceN(x4, v4, dt, Exforce, MuscleAct, DisVec, K, D, Leng,Theta1, MusclActFac);

	          xf = x + (dt/6.0) * (v1 + (2 * v2) + (2 * v3) + v4);
	          vf = v + (dt/6.0) * (a1 + (2 * a2) + (2 * a3) + a4);

	          MusclePara MusPara2 = {xf, vf};


	          return MusPara2;

	        }

double MuscleModel::Scaling(double Input, double MaxIn, double MinIn, double MaxOut, double MinOut)
     {

//        double dReVal = (MinOut + ((Input - MinIn)/(MaxIn - MinIn)) * (MaxOut - MinOut));
//
//        if (dReVal > 1)
//        {
//          dReVal = 1;
//        }
//
//        if (dReVal < -1)
//        {
//          dReVal = -1;
//        }

       return (MinOut + ((Input - MinIn)/(MaxIn - MinIn)) * (MaxOut - MinOut));


//        return dReVal;

     }

std::vector<double> MuscleModel::getMuscleReflex(std::vector<double> m_reflex, const std::vector<double> inreflex)
{

	  Raw_R_fs_old.at(0) = Raw_R_fs.at(0); //R0_fs = 19
	  Raw_R_fs_old.at(1) = Raw_R_fs.at(1); //R1_fs = 20
	  Raw_R_fs_old.at(2) = Raw_R_fs.at(2); //R2_fs = 21
	  Raw_L_fs_old.at(0) = Raw_L_fs.at(0); //L0_fs = 22
	  Raw_L_fs_old.at(1) = Raw_L_fs.at(1); //L1_fs = 23
	  Raw_L_fs_old.at(2) = Raw_L_fs.at(2); //L2_fs = 24


	Raw_R_fs.at(0) = inreflex[R0_fs]/FSSCAL; //R0_fs = 19
	Raw_R_fs.at(1)= inreflex[R1_fs]/FSSCAL; //R1_fs = 20
	Raw_R_fs.at(2)= inreflex[R2_fs]/FSSCAL; //R2_fs = 21
	Raw_L_fs.at(0)= inreflex[L0_fs]/FSSCAL; //L0_fs = 22
	Raw_L_fs.at(1)= inreflex[L1_fs]/FSSCAL; //L1_fs = 23
	Raw_L_fs.at(2)= inreflex[L2_fs]/FSSCAL; //L2_fs = 24
	//save current neural activations

	   for (unsigned int i = TR0_m; i < BJ_m; i++) {

	    m_preold.at(i) = m_preN.at(i);
	    }

	//if ((ISMUSCLE ==1) && (amosVersion ==1))
	//{
	  for (unsigned int i = CR0_m; i < FR0_m; i++)
	    {

	    m_reflex.at(i) = m_reflex.at(i) + 0.31;
	    }

	  for (unsigned int i = FR0_m; i < BJ_m; i++)
	    {

	    m_reflex.at(i) = ((m_reflex.at(i) + 0.76)/0.49) * 0.30 - 0.98;
	    }

	//}
	 // get current neural activations
		for (unsigned int i = TR0_m; i < BJ_m; i++)
		{

		m_preN.at(i) = m_reflex.at(i);
		}
	//-----------------muscle implementations----------------------

		// for joints in the right front leg
		if (m_preN.at(TR0_m) < m_preold.at(TR0_m))// TR joint
		// stance phases
		{
		// for a FTi joint

		  MusclePara FR0Out,CR0Out;

		     MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FR0_m), PreJointVel.at(FR0_m),
		         SampleTime , Raw_R_fs_old.at(0),Raw_R_fs.at(0), m_preold.at(FR0_m), m_preN.at(FR0_m),
		         0, K1[0], D1[0], HipLength, 0, FMusclActFac);

		  FR0Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
		  PreJointAng.at(FR0_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

		m_reflex_muscle.at(FR0_m) = Scaling(FR0Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
		     FR0Out.Vel = ParaR11.Vel;
		     PreJointVel.at(FR0_m) =  ParaR11.Vel;
		// should save the velocity (i.e., Vel)

		     double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


		     // for a knee joint
		     MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR0_m), PreJointVel.at(CR0_m)
		         , SampleTime , Raw_R_fs_old.at(0), Raw_R_fs.at(0), m_preold.at(CR0_m), m_preN.at(CR0_m),
		         DisVe, K2[0], D2[0], KneeLength, 0, CMusclActFac);

		     CR0Out.Pos = ParaR12.Pos;
		     PreJointAng.at(CR0_m) = ParaR12.Pos;
		     m_reflex_muscle.at(CR0_m) = Scaling(CR0Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
		     CR0Out.Vel = ParaR12.Vel;
		     PreJointVel.at(CR0_m) = ParaR12.Vel;

		if (m_reflex_muscle.at(CR0_m) > MINCTROUT)
		{
		m_reflex_muscle.at(CR0_m) = MINCTROUT;

		}
		}

		else
		//swing phases
		{
		  m_reflex_muscle.at(FR0_m) = dSWFTiFac * m_reflex.at(FR0_m)-dSWFTiOffset;
		  PreJointAng.at(FR0_m) = Scaling(m_reflex.at(FR0_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
		  PreJointVel.at(FR0_m) = 0;
		  m_reflex_muscle.at(CR0_m) = m_reflex.at(CR0_m);
		  PreJointAng.at(CR0_m) = Scaling(m_reflex.at(CR0_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
		  PreJointVel.at(CR0_m) = 0;

		}


		// for joints in the right middle leg
		  if (m_preN.at(TR1_m) < m_preold.at(TR1_m))// TR joint
		  // stance phases
		  {
		  // for a FTi joint

		    MusclePara FR1Out,CR1Out;

		       MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FR1_m), PreJointVel.at(FR1_m), SampleTime , Raw_R_fs_old.at(1),
		           Raw_R_fs.at(1), m_preold.at(FR1_m), m_preN.at(FR1_m), 0, K1[1], D1[1], HipLength, 0, FMusclActFac);

		    FR1Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
		    PreJointAng.at(FR1_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

		  m_reflex_muscle.at(FR1_m) = Scaling(FR1Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
		       FR1Out.Vel = ParaR11.Vel;
		       PreJointVel.at(FR1_m) =  ParaR11.Vel;
		  // should save the velocity (i.e., Vel)

		       double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


		       // for a knee joint
		       MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR1_m), PreJointVel.at(CR1_m), SampleTime
		           , Raw_R_fs_old.at(1), Raw_R_fs.at(1), m_preold.at(CR1_m), m_preN.at(CR1_m),
		           DisVe, K2[1], D2[1], KneeLength, 0, CMusclActFac);

		       CR1Out.Pos = ParaR12.Pos;
		       PreJointAng.at(CR1_m) = ParaR12.Pos;
		       m_reflex_muscle.at(CR1_m) = Scaling(CR1Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
		       CR1Out.Vel = ParaR12.Vel;
		       PreJointVel.at(CR1_m) = ParaR12.Vel;

		  if (m_reflex_muscle.at(CR1_m) > MINCTROUT)
		  {
		  m_reflex_muscle.at(CR1_m) = MINCTROUT;

		  }
		  }

		  else
		  //swing phases
		  {
		    m_reflex_muscle.at(FR1_m) = dSWFTiFac * m_reflex.at(FR1_m)-dSWFTiOffset;
		    PreJointAng.at(FR1_m) = Scaling(m_reflex.at(FR1_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
		    PreJointVel.at(FR1_m) = 0;
		    m_reflex_muscle.at(CR1_m) = m_reflex.at(CR1_m);
		    PreJointAng.at(CR1_m) = Scaling(m_reflex.at(CR1_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
		    PreJointVel.at(CR1_m) = 0;

		  }


		  // for joints in the right hind leg
		      if (m_preN.at(TR2_m) < m_preold.at(TR2_m))// TR joint
		      // stance phases
		      {
		      // for a FTi joint

		        MusclePara FR2Out,CR2Out;

		           MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FR2_m), PreJointVel.at(FR2_m), SampleTime , Raw_R_fs_old.at(2),
		               Raw_R_fs.at(2), m_preold.at(FR2_m), m_preN.at(FR2_m), 0, K1[2], D1[2], HipLength, 0, FMusclActFac);

		        FR2Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
		        PreJointAng.at(FR2_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

		      m_reflex_muscle.at(FR2_m) = Scaling(FR2Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
		           FR2Out.Vel = ParaR11.Vel;
		           PreJointVel.at(FR2_m) =  ParaR11.Vel;
		      // should save the velocity (i.e., Vel)

		           double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


		           // for a knee joint
		           MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CR2_m), PreJointVel.at(CR2_m), SampleTime
		               , Raw_R_fs_old.at(2), Raw_R_fs.at(2), m_preold.at(CR2_m), m_preN.at(CR2_m),
		               DisVe, K2[2], D2[2], KneeLength, 0, CMusclActFac);

		           CR2Out.Pos = ParaR12.Pos;
		           PreJointAng.at(CR2_m) = ParaR12.Pos;
		           m_reflex_muscle.at(CR2_m) = Scaling(CR2Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
		           CR2Out.Vel = ParaR12.Vel;
		           PreJointVel.at(CR2_m) = ParaR12.Vel;

		      if (m_reflex_muscle.at(CR2_m) > MINCTROUT)
		      {
		      m_reflex_muscle.at(CR2_m) = MINCTROUT;

		      }
		      }

		      else
		      //swing phases
		      {
		        m_reflex_muscle.at(FR2_m) = dSWFTiFac * m_reflex.at(FR2_m)-dSWFTiOffset;
		        PreJointAng.at(FR2_m) = Scaling(m_reflex.at(FR2_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
		        PreJointVel.at(FR2_m) = 0;
		        m_reflex_muscle.at(CR2_m) = m_reflex.at(CR2_m);
		        PreJointAng.at(CR2_m) = Scaling(m_reflex.at(CR2_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
		        PreJointVel.at(CR2_m) = 0;

		      }






		      // for joints in the left front leg
		        if (m_preN.at(TL0_m) < m_preold.at(TL0_m))// TR joint
		        // stance phases
		        {
		        // for a FTi joint

		          MusclePara FL0Out,CL0Out;

		             MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FL0_m), PreJointVel.at(FL0_m),
		                 SampleTime , Raw_L_fs_old.at(0),Raw_L_fs.at(0), m_preold.at(FL0_m), m_preN.at(FL0_m),
		                 0, K1[3], D1[3], HipLength, 0, FMusclActFac);

		          FL0Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
		          PreJointAng.at(FL0_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

		        m_reflex_muscle.at(FL0_m) = Scaling(FL0Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
		             FL0Out.Vel = ParaR11.Vel;
		             PreJointVel.at(FL0_m) =  ParaR11.Vel;
		        // should save the velocity (i.e., Vel)

		             double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


		             // for a knee joint
		             MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CL0_m), PreJointVel.at(CL0_m)
		                 , SampleTime , Raw_L_fs_old.at(0), Raw_L_fs.at(0), m_preold.at(CL0_m), m_preN.at(CL0_m),
		                 DisVe, K2[3], D2[3], KneeLength, 0, CMusclActFac);

		             CL0Out.Pos = ParaR12.Pos;
		             PreJointAng.at(CL0_m) = ParaR12.Pos;
		             m_reflex_muscle.at(CL0_m) = Scaling(CL0Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
		             CL0Out.Vel = ParaR12.Vel;
		             PreJointVel.at(CL0_m) = ParaR12.Vel;

		        if (m_reflex_muscle.at(CL0_m) > MINCTROUT)
		        {
		        m_reflex_muscle.at(CL0_m) = MINCTROUT;

		        }
		        }

		        else
		        //swing phases
		        {
		          m_reflex_muscle.at(FL0_m) = dSWFTiFac * m_reflex.at(FL0_m)-dSWFTiOffset;
		          PreJointAng.at(FL0_m) = Scaling(m_reflex.at(FL0_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
		          PreJointVel.at(FL0_m) = 0;
		          m_reflex_muscle.at(CL0_m) = m_reflex.at(CL0_m);
		          PreJointAng.at(CL0_m) = Scaling(m_reflex.at(CL0_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
		          PreJointVel.at(CL0_m) = 0;

		        }

		        // for joints in the left middle leg
		                  if (m_preN.at(TL1_m) < m_preold.at(TL1_m))// TR joint
		                  // stance phases
		                  {
		                  // for a FTi joint

		                    MusclePara FL1Out,CL1Out;

		                       MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FL1_m), PreJointVel.at(FL1_m),
		                           SampleTime , Raw_L_fs_old.at(1),Raw_L_fs.at(1), m_preold.at(FL1_m), m_preN.at(FL1_m),
		                           0, K1[4], D1[4], HipLength, 0, FMusclActFac);

		                    FL1Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
		                    PreJointAng.at(FL1_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

		                  m_reflex_muscle.at(FL1_m) = Scaling(FL1Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
		                       FL1Out.Vel = ParaR11.Vel;
		                       PreJointVel.at(FL1_m) =  ParaR11.Vel;
		                  // should save the velocity (i.e., Vel)

		                       double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


		                       // for a knee joint
		                       MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CL1_m), PreJointVel.at(CL1_m)
		                           , SampleTime , Raw_L_fs_old.at(1), Raw_L_fs.at(1), m_preold.at(CL1_m), m_preN.at(CL1_m),
		                           DisVe, K2[4], D2[4], KneeLength, 0, CMusclActFac);

		                       CL1Out.Pos = ParaR12.Pos;
		                       PreJointAng.at(CL1_m) = ParaR12.Pos;
		                       m_reflex_muscle.at(CL1_m) = Scaling(CL1Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
		                       CL1Out.Vel = ParaR12.Vel;
		                       PreJointVel.at(CL1_m) = ParaR12.Vel;

		                  if (m_reflex_muscle.at(CL1_m) > MINCTROUT)
		                  {
		                  m_reflex_muscle.at(CL1_m) = MINCTROUT;

		                  }
		                  }

		                  else
		                  //swing phases
		                  {
		                    m_reflex_muscle.at(FL1_m) = dSWFTiFac * m_reflex.at(FL1_m)-dSWFTiOffset;
		                    PreJointAng.at(FL1_m) = Scaling(m_reflex.at(FL1_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
		                    PreJointVel.at(FL1_m) = 0;
		                    m_reflex_muscle.at(CL1_m) = m_reflex.at(CL1_m);
		                    PreJointAng.at(CL1_m) = Scaling(m_reflex.at(CL1_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
		                    PreJointVel.at(CL1_m) = 0;

		                  }
		                  // for joints in the left hind leg
		                                if (m_preN.at(TL2_m) < m_preold.at(TL2_m))// TR joint
		                                // stance phases
		                                {
		                                // for a FTi joint

		                                  MusclePara FL2Out,CL2Out;

		                                     MusclePara ParaR11 = RungeKutta41(PreJointAng.at(FL2_m), PreJointVel.at(FL2_m),
		                                         SampleTime , Raw_L_fs_old.at(2),Raw_L_fs.at(2), m_preold.at(FL2_m), m_preN.at(FL2_m),
		                                         0, K1[5], D1[5], HipLength, 0, FMusclActFac);

		                                  FL2Out.Pos = ParaR11.Pos;// here Pos is the angle of the hip joint.
		                                  PreJointAng.at(FL2_m) = ParaR11.Pos;// should save the angle (i.e., Pos)

		                                m_reflex_muscle.at(FL2_m) = Scaling(FL2Out.Pos, FRMAXANG, FRMINANG, FRMAXOUT, FRMINOUT);
		                                     FL2Out.Vel = ParaR11.Vel;
		                                     PreJointVel.at(FL2_m) =  ParaR11.Vel;
		                                // should save the velocity (i.e., Vel)

		                                     double DisVe = (HipLength + Radius) * sin(ParaR11.Pos);


		                                     // for a knee joint
		                                     MusclePara ParaR12 = RungeKutta42(PreJointAng.at(CL2_m), PreJointVel.at(CL2_m)
		                                         , SampleTime , Raw_L_fs_old.at(2), Raw_L_fs.at(2), m_preold.at(CL2_m), m_preN.at(CL2_m),
		                                         DisVe, K2[5], D2[5], KneeLength, 0, CMusclActFac);

		                                     CL2Out.Pos = ParaR12.Pos;
		                                     PreJointAng.at(CL2_m) = ParaR12.Pos;
		                                     m_reflex_muscle.at(CL2_m) = Scaling(CL2Out.Pos, CRMAXANG, CRMINANG, CRMAXOUT, CRMINOUT);
		                                     CL2Out.Vel = ParaR12.Vel;
		                                     PreJointVel.at(CL2_m) = ParaR12.Vel;

		                                if (m_reflex_muscle.at(CL2_m) > MINCTROUT)
		                                {
		                                m_reflex_muscle.at(CL2_m) = MINCTROUT;

		                                }
		                                }

		                                else
		                                //swing phases
		                                {
		                                  m_reflex_muscle.at(FL2_m) = dSWFTiFac * m_reflex.at(FL2_m)-dSWFTiOffset;
		                                  PreJointAng.at(FL2_m) = Scaling(m_reflex.at(FL2_m), FRMAXOUT, FRMINOUT, FRMAXANG, FRMINANG);
		                                  PreJointVel.at(FL2_m) = 0;
		                                  m_reflex_muscle.at(CL2_m) = m_reflex.at(CL2_m);
		                                  PreJointAng.at(CL2_m) = Scaling(m_reflex.at(CL2_m), CRMAXOUT, CRMINOUT, CRMAXANG, CRMINANG);
		                                  PreJointVel.at(CL2_m) = 0;

		                                }

		                                for(int i=TR0_m;i<CR0_m;i++)
		                                {
		                                	m_reflex_muscle.at(i)=m_reflex.at(i);
		                                }


  return m_reflex_muscle;
}

