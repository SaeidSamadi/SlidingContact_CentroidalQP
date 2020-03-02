#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from numpy.linalg import inv
import math
from numpy import array, dot, random
from qpsolvers import solve_qp


def solve_qp2(P, q, G, h, lb, ub):
	Gl = -np.eye(np.shape(P)[0])
	hl = -lb
	Gu = np.eye(np.shape(P)[0])
	hu = ub	
	G = np.vstack((G,Gl,Gu)) 
	h = np.hstack((h,hl,hu))
	return solve_qp(P, q, G, h)

def solve_unbounded_qp(P, q):
	return np.matmul(-inv(np.transpose(P)),q)

if __name__ == "__main__":
    M = array([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
    P = dot(M.T, M)  # quick way to build a symmetric matrix
    q = dot(array([3., 2., 3.]), M).reshape((3,))
    G = array([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
    h = array([3., 2., -2.]).reshape((3,))
    lb = array([-1., -1., -1.])
    ub = array([+2.1, +2.1, +2.1])
    print "solve_qp2(P, q, lb, ub) =", solve_qp2(P, q, G, h, lb, ub)
    print "solve_unbounded_qp(P, q) =", solve_unbounded_qp(P, q)
