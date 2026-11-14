"""Wait for a cold estimator's alignment to settle before reusing an old map."""
from collections import deque
import math


class StableAlignment:
    def __init__(self, minimum_age=8.0, window=3.0, distance=.006, angle=.01):
        self.minimum_age, self.window = minimum_age, window
        self.distance, self.angle = distance, angle
        self.first = None
        self.samples = deque()

    def add(self, stamp, transform):
        if self.samples and stamp <= self.samples[-1][0]:
            return False
        if self.first is None:
            self.first = stamp
        self.samples.append((stamp, transform))
        while len(self.samples)>1 and self.samples[1][0] <= stamp-self.window:
            self.samples.popleft()
        if stamp-self.first < self.minimum_age or stamp-self.samples[0][0] < self.window:
            return False
        p, q = transform
        return all(math.dist(p, previous[0]) <= self.distance and
                   2*math.acos(min(1.,abs(sum(a*b for a,b in zip(q,previous[1]))))) <= self.angle
                   for _,previous in self.samples)
