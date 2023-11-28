import random
import math


def random_angle() -> float:
    """Return a random angle between -pi and pi Radians"""
    return random.uniform(-math.pi, math.pi)
