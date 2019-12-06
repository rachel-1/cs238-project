import mdptoolbox, mdptoolbox.example
from config import *
from scipy import stats
import numpy as np
from utils import *

class ConstrainedFlight():
    def __init__(self, max_distance, max_timesteps):
        # (A, S, S)
        # state: (speed, distance from goal, timesteps remaining)
        # states[r,c,t]:
        #  - speed = r * SPEED_STEP
        #  - distance from goal = c * DIST_STEP
        self.num_actions = int(2*DRONE_MAX_ACCEL/ACCEL_STEP + 1)
        self.num_rows = int(DRONE_MAX_SPEED/SPEED_STEP + 1)
        self.num_cols = int(max_distance/DIST_STEP + 1)
        self.num_timesteps = max_timesteps+1 # to account for zero-indexing
        self.state_space = (self.num_rows,self.num_cols,self.num_timesteps)
        print("self.state_space: ", self.state_space) # TODO - remove debug statement
        self.num_states = self.num_rows*self.num_cols*self.num_timesteps

        # penalty for acceleration (S, A)
        self.R = np.zeros((self.num_states, self.num_actions))
        self.R[:] = -abs(np.arange(-DRONE_MAX_ACCEL, DRONE_MAX_ACCEL+ACCEL_STEP, ACCEL_STEP))*ACCEL_PENALTY
        
        # penalize velocity - TODO
        '''
        R = self.R.T
        R = R.reshape((self.num_actions, self.num_rows, self.num_cols, self.num_timesteps))
        R = np.swapaxes(R, 1, 2)
        R[:, :] -= SPEED_PENALTY*np.arange(0, DRONE_MAX_SPEED+SPEED_STEP, SPEED_STEP)
        R = np.swapaxes(R, 2, 1)
        R = R.reshape(self.num_actions, self.num_states)
        self.R = R.T
        '''
        '''
        # more efficient way of masking out invalid actions?? - TODO
        arr = np.ones((self.num_rows, self.num_actions))
        mask_one = np.flip(np.tril(arr,arr.shape[0]-arr.shape[1]+1), axis=0) == 1
        mask_two = np.flip(np.tril(arr,arr.shape[0]-arr.shape[1]+1), axis=1) == 1
        tmp = np.repeat((mask_one | mask_two), self.num_cols, axis=0)
        self.R[tmp] -= INVALID_ACTION_PENALTY
        '''
        self.T = self._calc_T() # modifies R to penalize invalid actions
        self.discount = DISCOUNT

    def _calc_T(self):
        num_states = self.num_rows * self.num_cols * self.num_timesteps
        # transition probability based on delta distance vs speed
        T = np.zeros((self.num_actions,num_states,num_states))
        for action_index, intended_action in enumerate(np.arange(-DRONE_MAX_ACCEL, DRONE_MAX_ACCEL+ACCEL_STEP, ACCEL_STEP)):
            for curr_state_r in range(self.num_rows):
                for curr_state_c in range(self.num_cols):
                    for curr_state_t in range(self.num_timesteps):
                        curr_state_index = self.ravel_idx(curr_state_r, curr_state_c, curr_state_t)
                        curr_state_speed = curr_state_r * SPEED_STEP
                        curr_state_distance = curr_state_c * DIST_STEP
                        intended_speed = RandVar(curr_state_speed + intended_action, SPEED_VARIANCE)
                        # only calculate probability if action is valid
                        if intended_speed.mean <= DRONE_MAX_SPEED and intended_speed.mean >= 0 \
                           and intended_speed.mean <= curr_state_distance: # don't want to overshoot
                            # calculate next state
                            for next_state_r in range(self.num_rows):
                                next_state_speed = next_state_r * SPEED_STEP # can't go backwards
                                # probability of getting speed (as represented by row in states table)
                                speed_prob = intended_speed.probability(next_state_speed, SPEED_STEP/2)
                                intended_dist = RandVar(abs(curr_state_distance - intended_speed.mean), DIST_VARIANCE)
                                for next_state_c in range(self.num_cols):
                                    next_state_distance = next_state_c * DIST_STEP
                                    dist_prob = intended_dist.probability(next_state_distance, DIST_STEP/2) # TODO - independent??
                                    total_prob = speed_prob*dist_prob
                                    for next_state_t in range(self.num_timesteps):
                                        next_state_index = self.ravel_idx(next_state_r, next_state_c, next_state_t)
                                        if next_state_t == max(0, curr_state_t - 1):
                                            T[action_index, curr_state_index, next_state_index] = np.around(total_prob, decimals=5)
                                        else:
                                            T[action_index, curr_state_index, next_state_index] = 0

                        else:
                            # if a given action is not valid from the given state, force the action to stay in current state
                            # NOTE: this is necessary because the MDP solver requires that every row add up to 1
                            self.R[curr_state_index, action_index] -= INVALID_ACTION_PENALTY
                            next_state_index = self.ravel_idx(curr_state_r, curr_state_c, max(0,curr_state_t-1))
                            T[action_index, curr_state_index, next_state_index] = 1
                            
                        # normalize values in case rounding made it such that things don't sum to 1
                        T[action_index, curr_state_index] /= T[action_index, curr_state_index].sum()
        return T

    def idx_to_action(self, action_idx):
        return -DRONE_MAX_ACCEL + action_idx*ACCEL_STEP

    def action_to_idx(self, action):
        return int((action - (-DRONE_MAX_ACCEL))/ACCEL_STEP)

    def idx_to_state(self, state_idx):
        r, c, t = np.unravel_index(state_idx, self.state_space)
        return (r * SPEED_STEP, c * DIST_STEP, t)

    def ravel_idx(self, r, c, t):
        return np.ravel_multi_index([[r], [c], [t]], self.state_space)[0]
    
    def state_to_idx(self, state):
        speed, dist, t = state
        r = int(speed / SPEED_STEP)
        c = int(dist / DIST_STEP)
        return self.ravel_idx(r,c,int(t))
    
    def visualize_T(self): # TODO
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
                    
    def solve(self):
        # penalize unsuccessful states
        R = self.R.T
        R = R.reshape((self.num_actions, self.num_rows, self.num_cols, self.num_timesteps))
        R[:, :, :, 0] += R.min()*self.num_timesteps # over-approximate worst possible reward
        R[0, 0, 0, 0] = 0
        R = R.reshape(self.num_actions, self.num_states)
        R = R.T

        vi = mdptoolbox.mdp.ValueIteration(self.T, self.R, self.discount)
        vi.run()
        self.policy = vi.policy
        self.value_func = vi.V

    def policy_step(self, speed, distance, time):
        try:
            state_idx = self.state_to_idx((speed, distance, time))
        except ValueError: # if outside state space, use deterministic policy
            accel = min(DRONE_MAX_ACCEL, DRONE_MAX_SPEED - speed)
            new_speed = speed + accel
            return new_speed, distance - new_speed
        
        action_idx = self.policy[state_idx]
        new_state_idx = np.random.choice(range(self.num_states), p=self.T[action_idx, state_idx])
        return self.idx_to_state(new_state_idx)[:2]

    def get_edge_weight(self, speed, distance, available_time):
        # how much cost is incurred by executing the deterministic policy above
        deterministic_cost = 0 
        
        # if distance too large
        if distance > (self.num_cols-1)*DIST_STEP:
            # how long to get to distance in state space
            distance_travelled = distance - (self.num_cols-1)*DIST_STEP
            time_spent = distance_travelled/DRONE_MAX_SPEED
            deterministic_cost += TIME_COST*time_spent + DISTANCE_COST*distance_travelled
            
            available_time -= time_spent
            distance = (self.num_cols-1)*DIST_STEP

        # if time remaining is still too large, just hover at goal
        if available_time > (self.num_timesteps-1):
            time_spent = available_time - (self.num_timesteps-1)
            deterministic_cost += TIME_COST*time_spent
            available_time = self.num_timesteps - 1
        
        state_idx = self.state_to_idx((speed, distance, available_time))
        return deterministic_cost + -1*self.value_func[state_idx]

class UnconstrainedFlight():
    def policy_step(self, speed, distance):
        
        new_speed = min(DRONE_MAX_SPEED, distance)
            
        return new_speed, distance - new_speed

    @staticmethod
    def get_edge_weight(start_speed, distance):
        # TODO - use start_speed?
        time = distance / DRONE_MAX_SPEED
        return TIME_COST*time + DISTANCE_COST*distance

    
class Riding():
    def policy_step(self, speed, distance, remaining_time):
        
        if speed == 0 or remaining_time == 1:
            new_speed = 1
        elif remaining_time == 0:
            new_speed = distance = 0
        else:
            new_speed = distance/remaining_time

        return new_speed, distance - new_speed

    @staticmethod
    def get_edge_weight(distance):
        return distance/BUS_SPEED
        
    def get_policy_action(self, state, remaining_time):
        speed, distance = state
        if speed == 0 or remaining_time == 1:
            new_speed = 1
        elif remaining_time == 0:
            new_speed = distance = 0
        else:
            new_speed = distance/remaining_time
        return new_speed - speed


        
