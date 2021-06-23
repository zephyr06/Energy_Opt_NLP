# Energy_Opt_NLP
This repo implements energy optimization for real-time systems based on nonlinear programming, and explores special structure to achieve efficient optimization.

# RoadMap/TODO
1. warm start on response time during optimization;
can we bound the incremental response time analysis?

2. test with scale
2. incremental optimization
3. DAG optimization
0. What are the unique features of global result, how can we guide the optimization algorithm to find it by changing some settings
0. add weight to barrier function, and see whether it can guide better


# Solved
order of variables in GTSAM, should the Jacobian be a upper or a lower triangular matrix?
-- COLMAD automatically adjusts it

optimize at 'int' precision; if adjustment is smaller than 1, then not necessary to move further
-- bound relative error tolerance
