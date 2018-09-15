#!/usr/bin/env python

import rospy

from sympy import symarray
from sympy import sin
from sympy import cos
from sympy import Q
from sympy import pi
from sympy.core.symbol import Symbol
from sympy.core.symbol import symbols
from sympy.solvers.solveset import nonlinsolve
from sympy.assumptions.assume import assuming
from sympy.assumptions.assume import global_assumptions


def test_sympy():
    """
    Using position and velocity data from subscribing to detect_target/target_state topic, this function determines:
    - time of launch
    - time of intercept
    - azimuth and elevation angles for the projectile

    After solving the symbolic system of equations, the projectile is launched.
    :return:
    """
    # constant variables
    # velocity magnitude
    velocity_mag = 20
    # projectile pos
    proj_x = 1
    proj_y = 1
    proj_z = 0
    # gravitational acc
    g = 9.8

    # test variables
    vx = 15
    vy = 10
    vz = 5
    x = 30
    y = 15
    z = 9
    T = 10
    # declare symbols

    tau, phi, theta = symbols("tau, phi, theta", positive=True, real=True)
    # create assumptions

    global_assumptions.add(Q.positive(tau))
    global_assumptions.add(Q.real(tau))
    print global_assumptions

    eq1 = symarray("0, 0, g", 3)*(T-tau)**2/2 + symarray("vx, vy, vz", 3)*T \
        - velocity_mag*symarray("sin(theta)*cos(phi), sin(theta)*sin(phi), cos(theta)", 3)*(T-tau) \
        + symarray("x - proj_x, y - proj_y, z - proj_z", 3)
    system = [eq1]
    eq_symbols = [tau, phi, theta]
    try:
        print "Started solving.."
        print nonlinsolve(system, eq_symbols)
        print "Done."
    except ValueError as ve:
        print ve
    except AttributeError as ae:
        print ae


if __name__ == "__main__":
    test_sympy()