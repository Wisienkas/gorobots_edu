
#include <string.h> 
#include <stdio.h>   
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>

#include "dmp.h"

using namespace std;

DMP::DMP(){
   alpha_z = ALPHA_Z;
   beta_z = BETA_Z;
   alpha_v = ALPHA_V;
   alpha_z = ALPHA_Z;
   alpha_w = ALPHA_W;
   c = NULL;
   sigma = NULL;
   w = NULL;
}

DMP::~DMP(){
   delete [] c;
   c = NULL;
   delete [] sigma;
   sigma = NULL;
   delete [] w;
   w = NULL;
}

void DMP::set_weights_from_file(const char* fname){
   string tempStr;
   int i=0;
   fstream fileIn(fname);
   while (!fileIn.eof()){
     getline(fileIn,tempStr);
     sscanf(tempStr.c_str(),"%f", &w[i++]);
   }
   fileIn.close();
}

void DMP::init_dmp(float start, float goal, float total_t, float delta_t, float temp_scaling, int n_kernels, float width){
   s=start;
   g=goal;
   T=total_t;
   dt=delta_t;
   tau=temp_scaling;
   n=n_kernels;
   v=1;
   r=s;
   f=0;
   y=s;
   z=0;

   c = new float[n];  
   sigma = new float[n];
   w = new float[n];
  
   for (int i=0; i<n; i++){
      c[i]=i/(float)(n-1);
      sigma[i]=width;
      w[i]=0;
   }
}

void DMP::calculate_one_step_dmp(float t){
   float dv;
   float dr;
   float dy;
   float dz;
   float a;
   float psi;
   float sum_psi;
   
   if (t==0){
      v=1;
      r=s;
      y=s;
      z=0;
      f=0; 
   }
   else{
      a=exp((alpha_v/dt)*(tau*T-t));
      dv=-(alpha_v*a)/((1+a)*(1+a));
      if (isnan(dv)){
         dv=0;
      }    
      v=v+dv;

      if (t<=tau*T){
         dr=(1/tau)*(dt/T)*(g-s);
      }
      else{
         dr=0;
      }
      r=r+dr;      
      
      f=0;
      sum_psi=0;
      for (int i=0; i<n; i++){
         a=t/(tau*T)-c[i];
         psi=exp(-a*a/(2*sigma[i]*sigma[i])); 
         sum_psi=sum_psi+psi;
         f=f+psi*w[i]*v;
      }
      f=f/sum_psi;      
      
      dz=(1/tau)*(alpha_z*(beta_z*(r-y)-z)+f);
      z=z+dz;
      
      dy=(1/tau)*z; 
      y=y+dy;      
   }
}

