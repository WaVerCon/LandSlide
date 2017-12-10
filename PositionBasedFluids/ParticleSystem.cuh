#ifndef PARTICLE_SYSTEM_CUH
#define PARTICLE_SYSTEM_CUH

#include "common.h"

void update(solver *s, solverParams *sp);
void setParams(solverParams *tempParams);
void initBoundaryPsi(solver *s, solverParams *sp);
#endif