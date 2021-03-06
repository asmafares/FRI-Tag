#ifndef LearningController_h__guard
#define LearningController_h__guard

#include "geometry_msgs/Twist.h"

#include <vector>

struct LearningController {
  

 virtual geometry_msgs::Twist computeAction(const std::vector<double>& state)=0;  
 
 virtual void learn(const std::vector<double>& s,const geometry_msgs::Twist& a , double r, 
                                  const std::vector<double>&s_prime,const geometry_msgs::Twist& a_prime )=0;
  
 virtual ~LearningController() {};
};


#endif