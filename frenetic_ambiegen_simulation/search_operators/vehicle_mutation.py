import copy
from pymoo.core.mutation import Mutation
import numpy as np
import config as cf
import logging as log


class VehicleMutation(Mutation):
    """
    Module to perform the mutation
    """

    def __init__(self, mut_rate):
        super().__init__()
        self.mut_rate = mut_rate

    def _do(self, problem, X, **kwargs):
        for i in range(len(X)):
            r = np.random.random()
            s = X[i, 0]

            # with a given probability - perform the mutation
            if r < self.mut_rate:
                log.debug("Mutation performed on individual %s", s)
                sn = copy.deepcopy(s)

                wr = np.random.random()
                child = [list(state) for state in sn.states]  # Convert tuples to lists

                n = np.random.randint(1, 4)
                if wr < 0.5:  # Exchange mutation
                    log.debug("Exchange mutation performed on individual %s", s)
                    while n > 0:
                        candidates = np.random.randint(0, len(child) - 1, size=2)
                        child[candidates[0]], child[candidates[1]] = child[candidates[1]], child[candidates[0]]
                        n -= 1

                else:  # Change of value mutation
                    log.debug("Change of value mutation performed on individual %s", s)
                    while n > 0:
                        num = np.random.randint(0, len(child) - 1)

                        # Mutate action type
                        old_action = child[num][0]
                        new_action = np.random.choice([a for a in [0, 1, 2] if a != old_action])
                        child[num][0] = new_action

                        # Mutate action value
                        if new_action == 0:  # Go straight, change length
                            value_list = np.arange(cf.vehicle_env["min_len"], cf.vehicle_env["max_len"], 2)
                            child[num][1] = int(np.random.choice(value_list))
                        else:  # Turn left/right, change angle
                            value_list = np.arange(cf.vehicle_env["min_angle"], cf.vehicle_env["max_angle"], 5)
                            child[num][2] = int(np.random.choice(value_list))

                        n -= 1

                # Convert back to tuples if required
                sn.states = [tuple(state) for state in child]
                X[i, 0] = sn

        return X
