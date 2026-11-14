#pragma once
#include <algorithm>
namespace go2_multifloor_exploration {
struct SimulationProgress {
 bool initialized{false},paused{false};
 double last_sim{0},last_wall{0},last_advance_wall{0},extend_deadlines{0};
 bool sample(double sim,double wall) {
  extend_deadlines=0;
  if(!initialized || sim<last_sim){
    initialized=true;paused=false;last_sim=sim;last_wall=last_advance_wall=wall;return true;
  }
  const double dt=std::max(0.,wall-last_wall);
  const bool advance=sim>last_sim+1e-9;
  if(advance) {
    if(paused)extend_deadlines=dt;
    paused=false;last_advance_wall=wall;
  } else if(paused)extend_deadlines=dt;
  else if(wall-last_advance_wall>=.5) {
    paused=true;extend_deadlines=std::max(0.,wall-last_advance_wall);
  }
  last_sim=sim;last_wall=wall;return advance;
 }
};
}
