# Energy_Opt_NLP
This repo implements energy optimization for real-time systems based on nonlinear programming, and explores special structure to achieve efficient optimization.

# RoadMap/TODO


2. test with scale
how to limit the boundary, i.e., the computation time can only increase
LM will probably have a much worse performance than DL?
scale issue, why does it perform worse with larger task sets?

2. n20-v2
float-point has a big influence for large task sets
eliminate during optimization
2. weight issue, summarize why do we need to control weight
- be large, otherwise variables may go backwards when they are near the boundary
- not be too large, otherwise the schedulability constraints lose effects, can be circumvented by increasing punishment coefficient correspondingly
- be small, because we want the barrier gradient to guide the optimization process towards the global optimal more??
3. n20-v3
weight cannot be too large

2. incremental optimization

3. DAG optimization
4. what kind of optimality can we guarantee

0. What are the unique features of global result, how can we guide the optimization algorithm to find it by changing some settings
0. add weight to barrier function, and see whether it can guide better


# Research topic
1. how to round float point to int
- it should be applied every time after elimination
2. can we decide priority assignment from energy otpimization part? i.e., we decide priority assignment that make the task set both schedulable, and more energy efficient by fully using the processor resource

# Solved
order of variables in GTSAM, should the Jacobian be a upper or a lower triangular matrix?
-- COLMAD automatically adjusts it

optimize at 'int' precision; if adjustment is smaller than 1, then not necessary to move further
-- bound relative error tolerance

long long int issue
-- solved

log barrier function may return inf if responseTime equals deadline
-- clamp the minimum value that log can take as input; if it is 0, we'll return log(0 + toleranceBarrier/100);
-- However, the gradient should be very large at this point, to warn the optimizer not to move further

1. warm start on response time during optimization;
can we bound the incremental response time analysis?

3. test-n3-v11: 
- optiaml result is just 1, so make sure that whatever happens, the variables will not decrease during optimization
- the "eliminate" procedure should be able to detect this and freeze variables
- how to evaluate Jacobian matrix appropriately for this test case?

1. how to optimize when it's close to the scheduling boundary
- probably a two-phase method; whenever some tasks reach their boundary, we freeze all the hp variables and remove them from the variable set; however, what kind of optimality can we guarantee in that case? how do we know whether the freezed variables already reach their optimal solution? In essence, the previous optimization iterations update variables following a consistent proportion, while the direction is given by GN method;
- -how to decide when to eliminate these variables? 