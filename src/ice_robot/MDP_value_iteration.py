import numpy as np

############################################################################
def MDP_value_iteration(T, R, discount, epsilon, max_iter):
# MDP_value_iteration   Discounted MDP with value iteration algorithm 
# Let M is the number of states, N is the number of actions
# INPUTS:
#   T(MxMxN):   transition matrix        
#   R(MxMxN):   reward matrix
#   discount:   discount rate in range [0, 1]
#   epsilon:    epsilon-optimal policy search, upper than 0,
#   max_iter:   maximum number of iteration to be done, upper than 0, 

# OUTPUTS:
#   policy(M):    epsilon-optimal policy
# ---------------------------------------------------------------------

    SR = computeSumReward(T, R)
    # initialize value function
    M = T.shape[0]
    V0 = np.zeros((M,1))
    V = V0
    i = 0
    while (True):
        i = i + 1
        Vprev = V
        (V, policy) = bellman_equation(T, SR, discount, V)
        Vdiff = np.abs(V - Vprev)
        if (np.max(Vdiff) < epsilon):
            break
        elif (i == max_iter):
            break

    return policy

############################################################################
def bellman_equation(T, PR, discount, Vprev):
# bellman_equation  Apply the Bellman operation
# Let M is the number of states, N is the number of actions
# INPUTS:
#   T(MxMxN):   transition matrix
#   PR(MxN):    reward matrix
#   discount:   discount rate
#   Vprev(M):      previous value function
# OUTPUTS:
#   V(M):       new value function
#   policy(M):  improving policy
#----------------------------------------------------------------------------

    N = T.shape[2]
    for i in range(0, N):
        A = np.matrix(T[:,:,i])
        B = np.matrix(Vprev)
        Q[:,i] = PR[:,i] + discount*A*B

    V = np.max(Q, axis=1)
    policy = np.argmax(Q, axis=1)
    return (V, policy)

############################################################################
def compute_sum_reward(T, R):
# compute_sum_reward  Computes the reward for the system in one state chosing an action
# Let M is the number of states, N is the number of actions
# INPUTS:
#   T:          transition matrix (MxNxN)
#   R:          reward matrix (MxMxN)
# OUTPUTS:
#   PR(MxN):    reward matrix
#----------------------------------------------------------------------------

    N = T.shape[2]
    for i in range(0, N):
        SR[:,i] = np.sum(np.multiply(T[:,:,i], R[:,:,i]), axis=1)

    return SR
    
    
