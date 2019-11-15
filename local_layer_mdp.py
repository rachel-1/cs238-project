import mdptoolbox, mdptoolbox.example
from config import *
from scipy import stats
import numpy as np
from utils import *

class LocalLayerMDP():
    def __init__(self, start_state):
        # (A, S, S)
        # state: (distance from goal, speed)
        # states[r,c]:
        #  - speed = r * SPEED_STEP
        #  - distance from goal = c * DIST_STEP
        self.num_actions = int(2*DRONE_MAX_ACCEL/ACCEL_STEP + 1)
        self.num_rows = int(DRONE_MAX_SPEED/SPEED_STEP + 1)
        self.num_cols = int(start_state[1]/DIST_STEP + 1)
        self.num_states = self.num_rows * self.num_cols
        self.T = self._calc_T()

        self.start_state = start_state
        self.start_state_idx = self.state_to_idx(start_state)
        
        # penalty for acceleration (S, A)
        self.R = np.zeros((self.num_states, self.num_actions))
        self.R[:] = -abs(np.arange(-DRONE_MAX_ACCEL, DRONE_MAX_ACCEL+ACCEL_STEP, ACCEL_STEP))*ACCEL_PENALTY
        # penalize velocity - TODO
        R = self.R.T
        R = R.reshape((self.num_actions, self.num_rows, self.num_cols))
        R = np.swapaxes(R, 1, 2)
        R[:, :] -= SPEED_PENALTY*np.arange(0, DRONE_MAX_SPEED+SPEED_STEP, SPEED_STEP)
        R = R.reshape(self.num_actions, self.num_states)
        self.R = R.T
        
        self.discount = DISCOUNT

    def _calc_T(self):
        num_states = self.num_rows * self.num_cols
        # transition probability based on delta distance vs speed
        T = np.zeros((self.num_actions,num_states,num_states))
        for action_index, intended_action in enumerate(np.arange(-DRONE_MAX_ACCEL, DRONE_MAX_ACCEL+ACCEL_STEP, ACCEL_STEP)):
            for curr_state_r in range(self.num_rows):
                for curr_state_c in range(self.num_cols):
                    curr_state_index = curr_state_r*self.num_cols + curr_state_c
                    curr_state_speed = curr_state_r * SPEED_STEP
                    curr_state_distance = curr_state_c * DIST_STEP
                    intended_speed = RandVar(curr_state_speed + intended_action, SPEED_VARIANCE)
                    #print('-'*80)
                    # calculate next state
                    for next_state_r in range(self.num_rows):
                        current_speed = next_state_r * SPEED_STEP # can't go backwards
                        # probability of getting speed (as represented by row in states table)
                        speed_prob = intended_speed.probability(current_speed, SPEED_STEP/2)
                        intended_dist = RandVar(abs(curr_state_distance - intended_speed.mean), DIST_VARIANCE)
                        for next_state_c in range(self.num_cols):
                            current_dist = next_state_c * DIST_STEP
                            total_prob = speed_prob*intended_dist.probability(current_dist, DIST_STEP/2) # TODO - independent??
                            #print("a={}: ({}, {}) -> ({}, {}) = {}".format(intended_action, curr_state_speed, curr_state_distance, current_speed, current_dist, total_prob))
                            next_state_index = next_state_r*self.num_cols + next_state_c
                            T[action_index, curr_state_index, next_state_index] = np.around(total_prob, decimals=5)

                    # if a given action is not valid from the given state, force the action to stay in current state
                    # NOTE: this is necessary because the MDP solver requires that every row add up to 1
                    if T[action_index, curr_state_index].sum() == 0:
                        T[action_index, curr_state_index, curr_state_index] = 1
                    # normalize values in case rounding made it such that things don't sum to 1
                    T[action_index, curr_state_index] /= T[action_index, curr_state_index].sum()
        return T

    def visualize_T(self):
        for action_index, intended_action in enumerate(np.arange(-DRONE_MAX_ACCEL, DRONE_MAX_ACCEL+ACCEL_STEP, ACCEL_STEP)):
            print("action",intended_action)
            num_rows = int(DRONE_MAX_SPEED/SPEED_STEP + 1)
            num_cols = int(DRONE_MAX_DIST/DIST_STEP + 1)
            for curr_state_r in range(num_rows):
                for curr_state_c in range(num_cols):
                    curr_state_index = curr_state_r*num_cols + curr_state_c
                    curr_state_speed = curr_state_r * SPEED_STEP
                    curr_state_distance = curr_state_c * DIST_STEP
                    print("({}, {}): ".format(curr_state_speed, curr_state_distance), end='')
                    print(self.T[action_index, curr_state_index])
                    
    '''
    def simulate_full_policy(self, deterministic=False):
        state_idx = self.start_state_idx
    
        if type(self.policy) == tuple:
            while state_idx != 0:
                action_idx = self.policy[state_idx]
                new_state_idx = np.argmax(self.T[action_idx, state_idx])
                reward = self.R[state_idx, action_idx]
                print("from s={}, a={} yields r={} and s'={}".format(
                    self.idx_to_state(state_idx),
                    self.idx_to_action(action_idx),
                    reward,
                    self.idx_to_state(new_state_idx)))
                state_idx = new_state_idx
            return

        timesteps = self.policy.shape[1]
        for i in range(timesteps):
            action_idx = self.policy[state_idx, i]
            new_state_idx = np.argmax(self.T[action_idx, state_idx])
            reward = self.R[state_idx, action_idx]
            print("from s={}, a={} yields r={} and s'={}".format(
                self.idx_to_state(state_idx),
                self.idx_to_action(action_idx),
                reward,
                self.idx_to_state(new_state_idx)))
            state_idx = new_state_idx
    '''
    def idx_to_action(self, action_idx):
        return -DRONE_MAX_ACCEL + action_idx*ACCEL_STEP

    def action_to_idx(self, action):
        return int((action - (-DRONE_MAX_ACCEL))/ACCEL_STEP)

    def idx_to_state(self, state_idx):
        num_cols = int(self.start_state[1]/DIST_STEP + 1)
        curr_state_c = state_idx % num_cols
        curr_state_r = int(state_idx / num_cols)
        return (curr_state_r * SPEED_STEP, curr_state_c * DIST_STEP)

    def state_to_idx(self, state):
        num_cols = int(self.start_state[1]/DIST_STEP + 1)
        speed, distance = state
        curr_state_r = int(speed / SPEED_STEP)
        curr_state_c = int(distance / DIST_STEP)
        return curr_state_r*num_cols + curr_state_c

class UnconstrainedFlightMDP(LocalLayerMDP):
    def __init__(self, start_state):
        super().__init__(start_state)

    def solve(self):
        # penalize distance to goal - TODO
        R = self.R.T
        R = R.reshape((self.num_actions, self.num_rows, self.num_cols))
        R[:, :] -= 5*DIST_STEP*np.arange(0, self.num_cols)#self.start_state[1]+DIST_STEP, DIST_STEP)
        R = R.reshape(self.num_actions, self.num_states)
        R = R.T
        vi = mdptoolbox.mdp.ValueIteration(self.T, R, discount=1)
        vi.run()
        self.policy = vi.policy
        self.value_func = vi.V

    def policy_step(self, curr_state):
        state_idx = self.state_to_idx(curr_state)
        action_idx = self.policy[state_idx]
        new_state_idx = np.random.choice(range(self.num_states), p=self.T[action_idx, state_idx])
        return self.idx_to_state(new_state_idx)

    def get_edge_weight(self):
        return -1*self.value_func[self.start_state_idx]
    
class ConstrainedFlightMDP(LocalLayerMDP):
    def __init__(self, start_state, horizon):
        super().__init__(start_state)
        self.horizon = horizon
        
    def solve(self):
        final_rewards = np.ones(self.num_states)*-10
        final_rewards[0] = 0
        vi = mdptoolbox.mdp.FiniteHorizon(self.T, self.R, self.discount, N=self.horizon, h=final_rewards)
        vi.run()
        self.policy = vi.policy
        self.value_func = vi.V

    def policy_step(self, curr_state, i):
        state_idx = self.state_to_idx(curr_state)
        if i >= self.policy.shape[1]:
            return None
        action_idx = self.policy[state_idx, i]
        new_state_idx = np.random.choice(range(self.num_states), p=self.T[action_idx, state_idx])
        return self.idx_to_state(new_state_idx)

    def get_edge_weight(self):
        return -1*self.value_func[self.start_state_idx, 0]
    
class RidingPolicy():
    def __init__(self, start_state, horizon):
        self.start_state = start_state
        self.horizon = horizon

    def policy_step(self, curr_state, step):
        speed, distance = curr_state
        
        if step == 0 or step == self.horizon - 1:
            new_speed = 0
        elif step == 1 or step == self.horizon - 2:
            new_speed = 1
        else:
            new_speed = BUS_SPEED
            
        return new_speed, distance - new_speed
        

        
