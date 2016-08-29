fub_mpc_planner::fub_mpc_planner()
{
  f = boost::bind(&senario::callback,this,_1,_2);
  server.setCallback(f);
}
void fub_mpc_lqr_planner::config_callback(fub_mpc_planner::paramConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f",config.vx_min,config.vx_max,
    config.vy_min,config.vy_max,config.ax_min,config.ax_max,config.ay_min,config.ay_max,config.left_boundry_y,config.right_boundry_y);
  delta_t=config.delta_t;
}
void fub_mpc_planner::init_state(int ndim, ColumnVector& x)
{

  y=0;
  y_ref=4;

  // if (abs(x[2]-0)>abs(x[2]-4))
  //   y_ref=4;
  // else
  //   y_ref=0;

  x(1) = 100; //vx
  x(2) = 0; //vy
  x(3) = 0; //y-y_ref
  if (obstacles_num>0)
  { 
    for (int i=0;i<obstacles_num;i++)
    {
      x(4+i*4) = -10;
      x(5+i*4) = 10;
      x(6+i*4) = 0;
      x(7+i*4) = 0;
    }
  }
}
void fub_mpc_planner::system_dynamic(int mode, int ndim, const ColumnVector& x, double& fx, ColumnVector& gx, SymmetricMatrix& Hx, int& result)
{
  ColumnVector x_next(ndim)

  x_next(1) = x(1);
  x_next(2) = x(2);
  x_next(3) = x(3)+delta_t*x(2);

  for (int i=0;i<obstacles_num;i++)
  {
    x_next(4+i*2) = x(4+i*2)+(ax-ax_obs)*delta_t;
    x_next(5+i*2) = x(5+i*2)+x(4+i*2)*delta_t;
    x_next(6+i*2) = x(6+i*2)+(ay-ay_obs)*delta_t;
    x_next(7+i*2) = x(7+i*2)+x(6+i*2)*delta_t;
  }

  if 
  y_ref=min()
  

  if (mode & NLPFunction) {
    fx  = alfa*(pow((x_next(2)-y_ref),2))+beta*(pow(ax,2)+pow(ay,2));
    result = NLPFunction;
  }
  if (mode & NLPGradient) {
    gx(1) = 0;
    gx(2) = 2*alfa*(x_next(2)-y_ref);
    gx(3) = 0;
    for (int i=0;i<obstacles_num;i++)
    {
      gx(4+i*2) = 0;
      gx(5+i*2) = 0;
      gx(6+i*2) = 0;
      gx(7+i*2) = 0;
    }
    result = NLPGradient;
  }
//   The various Matrix objects have two indices, are indexed from 1,
// and they use parentheses around // the index.

  if (mode & NLPHessian) {
   

    Hx(2,1) = 0;
    Hx(2,2) = 2*alfa;

    result = NLPHessian;
  }
}

void fub_mpc_planner::system_cost_function(int mode, int ndim, const ColumnVector& x, ColumnVector& cx, Matrix& cgx, OptppArray<SymmetricMatrix>& cHx, int& result)
{ // Hock and Schittkowski's Problem 65 
  double f1, f2, f3, x1, x2, x3;
  SymmetricMatrix Htmp(ndim);

  if (ndim < 3)
     exit(1);

  x1 = x(1);
  x2 = x(2);
  x3 = x(3)+delta_t*x(2);

  for (int i=0;i<obstacles_num;i++)
  {
    x4 = x(3+i);
    x5=  x(3+i)*delta_t;
  }

  f1 = x1;
  f2 = x2;
  f3 = x3;

  if (mode & NLPFunction) {
    cx(1)  = alfa*(pow((x2-y_ref),2))+S*(pow(ax,2)+pow(ay,2));
    result = NLPFunction;
  }
  if (mode & NLPGradient) {
    cgx(1,1) = -2*x1;
    cgx(2,1) = -2*x2;
    cgx(3,1) = -2*x3;
    result = NLPGradient;
  }
  if (mode & NLPHessian) {
    Htmp(1,1) = -2;
    Htmp(1,2) = 0.0;
    Htmp(1,3) = 0.0;
    Htmp(2,1) = 0.0;
    Htmp(2,2) = -2;
    Htmp(2,3) = 0.0;
    Htmp(3,1) = 0.0;
    Htmp(3,2) = 0.0;
    Htmp(3,3) = -2;

    cHx[0] = Htmp;
    result = NLPHessian;
  }
}
int main ()
{
  ros::init(argc, argv, "fub_mpc_planner");

  ros::NodeHandle nh;
  fub_mpc_planner fub_mpc_planner(nh);
//TODO: obstacle reading from fub_senario
  int obstacles_num= 1;
  float obstacle_vel=100;
  float obstacle_acc=0;
  float v_obstacle_min=obstacle_vel-obstacle_acc*delta_t;
  float v_obstacle_max=obstacle_vel+obstacle_acc*delta_t;

  int ndim = 3+4*obstacles_num;
  ColumnVector lower(ndim), upper(ndim); 

// Here is one way to assign values to a ColumnVector.
  vx_min=vx-a_min*delta_t;

  lower << vx_min << vy_min << ay_min;
  upper << vx_max << vy_max << ay_max ;


  //obstacle constrains
  for (int i=0;i<obstacles_num;i++)
  {
    //vx_min, x_min, vy_min, y_min
    lower << -300 << -1000 << -300 <<1000;
    //vx_max, x_max, vy_max, y_max
    upper << -300 << -1000 << -300 <<1000 ;
  }
  

  Constraint c1 = new BoundConstraint(ndim, lower, upper);

  NLP* mpc_planner = new NLP(new NLF2(ndim, 1, constrains, system_dynamic));
  Constraint nleqn = new NonLinearInequality(mpc_planner);
  CompoundConstraint* constraints = new CompoundConstraint(nleqn, c1);
  NLF2 nips(ndim, hs65, system_dynamic, constraints);
  OptNIPS objfcn(&nips);

// The "0" in the second argument says to create a new file.  A "1"
// would signify appending to an existing file.

  objfcn.setOutputFile("example1.out", 0);
  objfcn.setFcnTol(1.0e-06);
  objfcn.setMaxIter(1500);
  objfcn.setMeritFcn(ArgaezTapia);
  objfcn.optimize();
  objfcn.printStatus("Solution from nips");
  //std::string f=objfcn.printStatus("Optimization method");
  std::cout<<"ah"<<std::endl;
  objfcn.cleanup();
   ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
