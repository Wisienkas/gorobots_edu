
#define ALPHA_Z 0.75
#define BETA_Z ALPHA_Z/4
#define ALPHA_V 1.0
#define ALPHA_W 1.0
#define DMP_DIM 2

class DMP {
   public:
      DMP();
      ~DMP();

      void set_alpha_z(float x) {alpha_z=x;}
      void set_beta_z(float x)  {beta_z=x;}
      void set_alpha_v(float x) {alpha_v=x;}
      void set_alpha_w(float x) {alpha_w=x;}
      void set_s(float x) {s=x;}
      void set_g(float x) {g=x;}
      void set_T(float x) {T=x;}
      void set_dt(float x) {dt=x;}
      void set_tau(float x) {tau=x;}
      void set_n(float x) {n=x;}
      void set_c(int i, float x) {c[i]=x;}
      void set_sigma(int i, float x) {sigma[i]=x;}
      void set_w(int i, float x) {w[i]=x;}
       
      float get_f() {return f;}
      float get_v() {return v;}
      float get_r() {return r;}
      float get_y() {return y;}
      float get_z() {return z;}
      float get_w(int i) {return w[i];}
      
      void init_dmp(float start, float goal, float total_t, float delta_t, float temp_scaling, int n_kernels, float width);
      void set_weights_from_file(const char* file_name);
      void calculate_one_step_dmp(float time_step);
      
   private:
      float s;
      float g;
      float T;
      float dt;
      float tau;
      int n;
      float* c;
      float* sigma;
      float* w;
      float alpha_z;
      float beta_z;
      float alpha_v;
      float alpha_w;
      float v;
      float r;
      float f;
      float y;
      float z;
      
};

