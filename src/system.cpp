#include <fub_mpc_planner/fub_mpc_lqr_planner.h>
fub_mpc_lqr_planner::fub_mpc_lqr_planner(ros::NodeHandle nh)
{
  f = boost::bind(&fub_mpc_lqr_planner::config_callback,this,_1,_2);
  server.setCallback(f);

}
void fub_mpc_lqr_planner::config_callback(fub_mpc_planner::paramConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f",config.vx_min,config.vx_max,
    config.vy_min,config.vy_max,config.ax_min,config.ax_max,config.ay_min,config.ay_max,config.left_boundry_y,config.right_boundry_y);
  delta_t=config.delta_t;
  int ndim=init_state();
  std::cout<<"ndim: "<< ndim <<std::endl;
  system_dynamic(ndim);
  optimize(ndim);

}
int fub_mpc_lqr_planner::init_state()
{

//============================================================
  //TODO: obstacle reading from fub_senario
  int obstacles_num= 1;
  // float obstacle_vel=100;
  // float obstacle_acc=0;
  // float v_obstacle_min=obstacle_vel-obstacle_acc*delta_t;
  // float v_obstacle_max=obstacle_vel+obstacle_acc*delta_t;

  int ndim = 3+4*obstacles_num;
  x.resize(ndim);
  x=Eigen::VectorXd::Zero(ndim);
//============================================================
  //TODO: define constains
  // Eigen::VectorXd lower(ndim), upper(ndim);
  // vx_min=vx-a_min*delta_t;
  // lower << vx_min << vy_min << ay_min;
  // upper << vx_max << vy_max << ay_max ;


  //TODO: obstacle constrains
  // for (int i=0;i<obstacles_num;i++)
  // {
  //   //vx_min, x_min, vy_min, y_min
  //   lower << -300 << -1000 << -300 <<1000;
  //   //vx_max, x_max, vy_max, y_max
  //   upper << -300 << -1000 << -300 <<1000 ;
  // }
//============================================================
  //initial car and obstacles positions and velocities

  y=0;
  y_ref=4;

  // if (abs(x[2]-0)>abs(x[2]-4))
  //   y_ref=4;
  // else
  //   y_ref=0;

  x[0] = 100; //vx
  x[1] = 0; //vy
  x[2] = 0-4; //y-y_ref
  if (obstacles_num>0)
  {
    for (int i=0;i<obstacles_num;i++)
    {
      x[3+i*4] = -10;
      x[4+i*4] = 10;
      x[5+i*4] = 0;
      x[6+i*4] = 0;
    }
  }

  // W=Eigen::MatrixXd::Zero(ndim,1);
  // W<<0,0,0;
  // if (obstacles_num>0)
  //   for (int i=0;i<obstacles_num;i++)
  //   {
  //     Eigen::MatrixXd W_obstacle;
  //     W_obstacle=Eigen::MatrixXd::Zero(4,1);
  //     W_obstacle<<0,0,0,0;
  //     A.block<4,1>(3+i*4,0)=W_obstacle;
  //   }

  return ndim;

}
void fub_mpc_lqr_planner::system_dynamic(int ndim)
{
  int obstacles_num=(ndim-3)/4;

  A=Eigen::MatrixXd::Zero(ndim,ndim);
  A.block<3,3>(0,0)<< 1,0,0,0,1,0,0,delta_t,1;
  if (obstacles_num>0)
    for (int i=0;i<obstacles_num;i++)
    {
      Eigen::MatrixXd A_obstacle;
      A_obstacle=Eigen::MatrixXd::Zero(4,4);
      A_obstacle<<1,0,0,0,
      delta_t,1,0,0,
      0,0,1,0,
      0,0,delta_t,1;
      A.block<4,4>(3+i*4,3+i*4)=A_obstacle;
    }

  B=Eigen::MatrixXd::Zero(ndim,2);

  B.block<3,2>(0,0) << 1,0,0,1,0,0;
  if (obstacles_num>0)
    for (int i=0;i<obstacles_num;i++)
    {
      Eigen::MatrixXd B_obstacle;
      B_obstacle=Eigen::MatrixXd::Zero(4,2);
      B_obstacle<<1,0,0,0,0,1,0,0;
      B.block<4,2>(3+i*4,0)=B_obstacle;
    }



  //check controllability of system full rank [B A*B A2*B A3*B ... An*B ] (ndim x ndim*2)
  controllability=Eigen::MatrixXd::Zero(ndim,ndim*2);

  controllability.block(0,0,ndim,2)=B;
  Eigen::MatrixXd A_i;
  A_i=A;
  for (int i=1;i<ndim;i++)
  {
    //controllability.block<ndim,2>(0,2*i)
    controllability.block(0,2*i,ndim,2)=A_i*B;

    A_i=A*A_i;
  }
  std::cout << "Here is the matrix A:\n" << A<< std::endl;
  std::cout << "Here is the matrix B:\n" << B<< std::endl;
  std::cout << "Here is the matrix controllability:\n" << controllability<< std::endl;
  Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(controllability);
  auto rank = lu_decomp.rank();

  if (rank < ndim)
    std::cout<<"rank: "<< rank <<"system is not controllable"<<std::endl;
  else
    std::cout<<"rank: "<< rank <<"system is controllable"<<std::endl;


  // Q, R: J=sum (alfa(xT * Q * x) + beta(uT * R * u))
  Q=Eigen::MatrixXd::Zero(ndim,ndim);
  Q(2,2)=10; //alfa=1
  Q(1,1)=100; //alfa=1
  //Q(0,0)=0.01; //alfa=1
  R=Eigen::MatrixXd::Zero(2,2);
  R<<1,0,0,10;//beta=1
}

void fub_mpc_lqr_planner::optimize(int ndim)
{
  K=Eigen::MatrixXd::Zero(2,ndim);
  P=Eigen::MatrixXd::Zero(ndim,ndim);
  for (int iteration=0;iteration<10;iteration++)
  {
    K=((R+B.transpose()*P*B).inverse())*(B.transpose()*P*A);
   // P=(A+B*K.transpose()).transpose()*P*(A+B*K.transpose())+K*R*K.transpose()+Q;
    P=A.transpose()*P*A-A.transpose()*P*B*K+Q;
    U=-1*K*x;
     std::cout << "Here is the matrix K:\n" << K<< std::endl;
    // if (U(1)>8)
    // {

    //   K(0,3)=0.1;

    // }
    std::cout << "Here is the matrix U:\n" << U<< std::endl;
    // if (U(2)>8)
    // {
    //   U(2)=8;
    // }
    x=A*x+B*U;
    // if (x(1)*delta_t>100)
    //   x(1)=100;
    // if (abs(x(3)-4)<abs(x(3)))
    //   x(3)=x(3)-4;
    std::cout << "Here is the matrix X:\n" << x<< std::endl;
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "fub_mpc_lqr_planner");

  ros::NodeHandle nh;
  fub_mpc_lqr_planner fub_mpc_lqr_planner(nh);
  fub_mpc_lqr_planner.optimize(7);
  std::cout<<"run"<<std::endl;
  //ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
