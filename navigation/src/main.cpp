/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.cpp
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/ 
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include files */
#include "ros/ros.h"
#include "navigation/rt_nonfinite.h"
#include "navigation/solveQP.h"
#include "navigation/main.h"
#include "navigation/solveQP_terminate.h"
#include "navigation/solveQP_emxAPI.h"
#include "navigation/solveQP_initialize.h"

/* Function Declarations */
static void argInit_4x1_real_T(double result[4]);
static emxArray_real_T *argInit_Unboundedx1_real_T();
static double argInit_real_T();
static void main_solveQP();

/* Function Definitions */
static void argInit_4x1_real_T(double result[4])
{
  int idx0;

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < 4; idx0++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result[idx0] = 1;
  }
}

static emxArray_real_T *argInit_Unboundedx1_real_T()
{
  emxArray_real_T *result;
  static int iv2[1] = { 48 };

  int idx0;

  /* Set the size of the array.
     Change this size to the value that the application requires. */
  result = emxCreateND_real_T(1, iv2);

  /* Loop over the array to initialize each element. */
  for (idx0 = 0; idx0 < result->size[0U]; idx0 = idx0 + 4) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result->data[idx0] = 100;
    result->data[idx0+1] = 100;
    result->data[idx0+2] = 0;
    result->data[idx0+3] = 0;
  }

  return result;
}

static double argInit_real_T()
{
  return 0.0;
}

static void main_solveQP()
{
  double xk[4];
  //~ double rp[80];
  emxArray_real_T *rp;
  double x_data[20];
  int x_size[1];
  //~ xk = {0,0,0,0};
  /* Initialize function 'solveQP' input arguments. */
  /* Initialize function input argument 'xk'. */
  argInit_4x1_real_T(xk);

  /* Initialize function input argument 'rp'. */
  rp = argInit_Unboundedx1_real_T();
  //~ rp = {100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0,100,100,0,0};
  /* Call the entry-point 'solveQP'. */
  solveQP(xk, rp, x_data, x_size); 
  emxDestroyArray_real_T(rp); 
  std::cout << "xk:" << xk[0]<<","<< xk[1]<<","<< xk[2]<<","<< xk[3]<< std::endl;
  //~ std::cout << "rp:" << *rp->data[0]<<","<<rp->data[1]<<","<< rp->data[2]<<","<< rp->data[3]<< std::endl;
  //~ for (int i = 0; i <= 80; i = i+4 )
  //~ {
    //~ std::cout << "rp:" << rp->data[i]<<","<<rp->data[i+1]<<","<< rp->data[i+2]<<","<< rp->data[i+3]<< std::endl;
   //~ }
   
  for (int i = 0; i < 10; i = i+1 )
  {
    std::cout << "x_data:" << x_data[i]<< std::endl;
   }
}

void timerCallback(const ros::TimerEvent&)
{ 
     main_solveQP();
    }
  
int main(int argc, char **argv)
{
    
  ros::init(argc, argv, "Test_qp_solution_node");
  ros::NodeHandle nh; 
  /* Initialize the application.
     You do not need to do this more than one time. */
  solveQP_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_solveQP();

  /* Terminate the application.
     You do not need to do this more than one time. */
  //~ solveQP_terminate();
  
  double dTimeStepSec = 0.5;
  ros::Timer timer = nh.createTimer(ros::Duration(dTimeStepSec), timerCallback);

  ROS_INFO_STREAM("Main has started.");
  ros::spin();
  return 0;
}

/* End of code generation (main.cpp) */
