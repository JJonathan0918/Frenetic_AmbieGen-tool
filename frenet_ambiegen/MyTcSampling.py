import numpy as np
from pymoo.model.sampling import Sampling
from frenet_ambiegen.Solution import Solution

import frenet_ambiegen.config as cf
from frenet_ambiegen.road_gen import RoadGen


class MyTcSampling(Sampling):

    '''
    Module to generate the initial population
    '''
    def _do(self, problem, n_samples, **kwargs):
        generator = RoadGen(
            cf.model["map_size"],
        )
        X = np.full((n_samples, 1), None, dtype=np.object)

        for i in range(n_samples):
            states = generator.test_case_generate()
            s = Solution()
            s.states = states
            X[i, 0] = s
        return X
