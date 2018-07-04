#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Eigen>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>

#define JOINT_NUM (3)
#define D2R (M_PI/180.0)

#define FK_SOLVER (1)
#define IK_VEL_SOLVER (1)

int main (int argc, char **argv)
{
    ros::init(argc, argv, "kdl_manipulator_node");
    ros::NodeHandle node;

    KDL::JntArray m_joint_position;
    m_joint_position.resize(JOINT_NUM);
    m_joint_position.data = Eigen::Vector3d(M_PI/4, -M_PI/2, 0.0); //M_PI/4, -M_PI/2, 0.0

    KDL::Chain m_chain;

// Set Kinematics Tree
//0
    m_chain.addSegment(KDL::Segment("base", // Origin
                                       KDL::Joint(KDL::Joint::None),
                                       KDL::Frame(KDL::Vector(0.0 , 0.0, 0.0)), // from base to first joint
                                       KDL::RigidBodyInertia(0.0,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//1
    m_chain.addSegment(KDL::Segment("first_link",
                                       KDL::Joint(KDL::Joint::RotX),
                                       KDL::Frame(KDL::Vector(0.0 , 0.0 , 1.0)), // KDL::Frame(KDL::Rotation::EulerZYX(0.0, 0.0, M_PI/4)) * KDL::Frame(KDL::Vector(0.0 , 0.0 , 1.0))
                                       KDL::RigidBodyInertia(0.0, // !!!
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );

//2
    m_chain.addSegment(KDL::Segment("sec_link",
                                       KDL::Joint(KDL::Joint::RotX),
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 1.0)),
                                       KDL::RigidBodyInertia(0.0, // !!!
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );
//3
    m_chain.addSegment(KDL::Segment("tip",
                                       KDL::Joint(KDL::Joint::RotX), // RotX
                                       KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)),
                                       KDL::RigidBodyInertia(0.0,
                                                             KDL::Vector(0.0, 0.0, 0.0),
                                                             KDL::RotationalInertia(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                                                             )
                                       )
                          );

// Set Joint Limits
    std::vector<double> min_position_limit, max_position_limit;
    min_position_limit.push_back(-180.0); max_position_limit.push_back(180.0); //
    min_position_limit.push_back(-180.0); max_position_limit.push_back(180.0); //
    min_position_limit.push_back(-180.0); max_position_limit.push_back(180.0); //

//Convert angles to rads
    KDL::JntArray min_joint_position_limit(JOINT_NUM), max_joint_position_limit(JOINT_NUM);
    for (int index=0; index<JOINT_NUM; index++)
    {
      min_joint_position_limit(index) = min_position_limit[index]*D2R; //D2R - degrees to radians
      //ROS_INFO("joint [%d] min: %f", index, min_joint_position_limit(index));
      max_joint_position_limit(index) = max_position_limit[index]*D2R;
      //ROS_INFO("joint [%d] max: %f", index, max_joint_position_limit(index));
    }

//Solvers
    KDL::ChainFkSolverPos_recursive *m_fk_solver_;
    KDL::ChainIkSolverVel_pinv *m_ik_vel_solver_;
    KDL::ChainIkSolverPos_NR *m_ik_pos_solver_;

    m_fk_solver_ = new KDL::ChainFkSolverPos_recursive(m_chain); // forward kinematics solver

//inverse kinematics solvers
    m_ik_vel_solver_ = new KDL::ChainIkSolverVel_pinv(m_chain);

    m_ik_pos_solver_ = new KDL::ChainIkSolverPos_NR(m_chain,
                                                         //min_joint_position_limit, max_joint_position_limit,
                                                         *m_fk_solver_,
                                                         *m_ik_vel_solver_
                                                         //,300, 0.000001
                                                    );


//Body target pose

    KDL::Frame tip_desired_pose = KDL::Frame(KDL::Rotation::RPY(0.0, 0.0, 0.0), KDL::Vector(0.0, 1.0, 1.0));

    //std::cout<<tip_desired_pose(0,0)<<" "<<tip_desired_pose(0,1)<<" "<<tip_desired_pose(0,2)<<" "<<tip_desired_pose(0,3)<<std::endl;
    //std::cout<<tip_desired_pose(1,0)<<" "<<tip_desired_pose(1,1)<<" "<<tip_desired_pose(1,2)<<" "<<tip_desired_pose(1,3)<<std::endl;
    //std::cout<<tip_desired_pose(2,0)<<" "<<tip_desired_pose(2,1)<<" "<<tip_desired_pose(2,2)<<" "<<tip_desired_pose(2,3)<<std::endl;
    //std::cout<<tip_desired_pose(3,0)<<" "<<tip_desired_pose(3,1)<<" "<<tip_desired_pose(3,2)<<" "<<tip_desired_pose(3,3)<<std::endl;

    KDL::JntArray m_desired_joint_position;
    m_desired_joint_position.resize(JOINT_NUM);



    int ik_pose_err = m_ik_pos_solver_->CartToJnt(m_joint_position, tip_desired_pose, m_desired_joint_position);

    if (ik_pose_err != 0)
    {
      ROS_WARN("IK ERR : %s", m_ik_pos_solver_->strError(ik_pose_err));
      //return 0;
    }

    // output
    if(ik_pose_err>=0){

      std::vector<double_t> m_output;
      m_output.resize(JOINT_NUM);

      for (int i=0; i<JOINT_NUM; i++)
      {
        m_output[i] = m_desired_joint_position(i);

      }

      ROS_INFO("Inverse Kinematics:");
      ROS_INFO("Joint angles: %f, %f, %f",
               m_output[0]*180/M_PI,m_output[1]*180/M_PI,m_output[2]*180/M_PI);
    }
//IK_Velocities
  if(IK_VEL_SOLVER){
        KDL::Vector des_trans_vel(0.1,0.1,0.1); // Cartesian velocities
        KDL::Vector des_rot_vel(0.1,0.1,0.1);

//      KDL::Twist des_vel((0.1,0.1,0.1),(0.1,0.1,0.1));
        KDL::Twist des_cart_vel(des_trans_vel,des_rot_vel);

        KDL::JntArray jnt_vel = KDL::JntArray(JOINT_NUM);

        int vel_err = m_ik_vel_solver_->CartToJnt(m_joint_position,des_cart_vel,jnt_vel); // jnt_vel - output

        if (vel_err < 0)
        {
          ROS_WARN("IK ERR : %s", m_ik_pos_solver_->strError(vel_err));
          //return 0;
        }

        if(vel_err >= 0)
        {
            std::vector<double_t> jnt_vel_out;
            jnt_vel_out.resize(JOINT_NUM);

            for (unsigned i=0;i<JOINT_NUM;i++){
               jnt_vel_out[i] = jnt_vel(i);
            }

            ROS_INFO("Joint velocities: %f, %f, %f",
                          jnt_vel_out[0],jnt_vel_out[1],jnt_vel_out[2]);
        }

  }

//FK
  if (FK_SOLVER){

      KDL::JntArray jnt_fk = KDL::JntArray(JOINT_NUM);
      jnt_fk(0) = 0;
      jnt_fk(1) = 0;
      jnt_fk(2) = 0;

      KDL::Frame m_tip_pose_;
      m_fk_solver_->JntToCart(m_joint_position, m_tip_pose_);

      double roll, pitch, yaw;

      m_tip_pose_.M.GetRPY(roll,pitch,yaw);
      ROS_INFO(" ");
      ROS_INFO("Forward Kinematics:");
      ROS_INFO("X: %f, Y: %f, Z: %f",
               m_tip_pose_.p.x(),m_tip_pose_.p.y(),m_tip_pose_.p.z());
      ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f",
              roll,pitch,yaw);

  }

    delete m_fk_solver_;
    delete m_ik_vel_solver_;
    delete m_ik_pos_solver_;

    return 0;
}
