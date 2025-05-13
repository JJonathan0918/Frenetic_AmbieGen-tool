import numpy as np
from pymoo.model.crossover import Crossover
from frenet_ambiegen.Solution import Solution
import random as rm

class MyTcCrossover(Crossover):
    '''
    Module to perform the crossover
    '''
    def __init__(self, cross_rate):
        super().__init__(2, 2)
        self.cross_rate = cross_rate

    def _do(self, problem, X, **kwargs):
        _, n_matings, _ = X.shape
        Y = np.full_like(X, None, dtype=np.object)

        for k in range(n_matings):
            r = np.random.random()
            s_a, s_b = X[0, k, 0], X[1, k, 0]

            if r < self.cross_rate:
                tc_a, tc_b = s_a.states, s_b.states

                if len(tc_a) <= 1 or len(tc_b) <= 1:
                    # print("Not enough states to crossover! Skipping crossover.")
                    Y[0, k, 0], Y[1, k, 0] = s_a, s_b
                    continue  # 直接跳过交叉

                # 确保 crossover_point 在合法范围内
                crossover_point = rm.randint(1, min(len(tc_a), len(tc_b)) - 1)

                if s_a.n_states > 2 and s_b.n_states > 2:
                    offa, offb = {}, {}

                    # One-point crossover
                    for i in range(0, crossover_point):
                        offa["st" + str(i)] = tc_a["st" + str(i)]
                        offb["st" + str(i)] = tc_b["st" + str(i)]
                    for m in range(crossover_point, len(tc_b)):
                        offa["st" + str(m)] = tc_b["st" + str(m)]
                    for n in range(crossover_point, len(tc_a)):
                        offb["st" + str(n)] = tc_a["st" + str(n)]

                    off_a, off_b = Solution(), Solution()
                    off_a.states, off_b.states = offa, offb

                    # 计算新个体的新颖度
                    off_a.novelty = off_a.calc_novelty(tc_a, off_a.states)
                    off_b.novelty = off_b.calc_novelty(tc_b, off_b.states)

                    Y[0, k, 0], Y[1, k, 0] = off_a, off_b
                else:
                    # print("Not enough valid states for crossover! Using original individuals.")
                    Y[0, k, 0], Y[1, k, 0] = s_a, s_b  # 直接保留原始个体

            else:
                Y[0, k, 0], Y[1, k, 0] = s_a, s_b  # 不进行交叉的情况

        return Y
