# -*- coding: utf-8 -*-
"""
Created on Wed May 15 22:15:43 2019

@author: Sarah Li
"""
import numpy as np
import cvxpy as cvx
import scipy.linalg as sla

import subspace

def isa(V0, A, B, eps = 0.0, verbose=False):
    K = V0;
    Vt= 1.0*V0;
    isInvariant = False;
    iteration = 0;
    while not isInvariant:
        iteration += 1
        if verbose:
            print ("--------------in iteration ", iteration);
        Ainved =  subspace.a_inv_v(A, subspace.union(Vt, B));
        Vnext = subspace.intersect(K, Ainved)
        if verbose:
            print ("rank of Vt ", subspace.rank(Vt));
            print ("rank of Vnext ", subspace.rank(Vnext));
            print ("Vnext is contained in Vt ", subspace.contained(Vnext, Vt))
        if subspace.rank(Vnext) == subspace.rank(Vt):
            isInvariant = True
            if verbose:
                print ("ISA returns-----") 
        else:
            Vt = Vnext;
    return Vt

def disturbance_decoupling(H, A, B, return_alpha=False, verbose=False):
    """ Generate a distrubance decoupling controller F. 
    
    Args:
        - H: the observation matrix [ndarray, lxn]. 
        - A: system dynamics matrix [ndarray, nxn].
        - B: control matrix [ndarray, nxm].
        - return_alpha: True if you want the corresponding laypunov pontential.
        - verbose: True if want to see cvx output, False otherwise.
    Returns:
        - V: a matrix whose range is largest (A,B) control invariant set in
             the kernel of H.T [ndarray, nxk].
        - F: a disturbance decoupling optimal controller [ndarray, mxn].
        - alpha: magnitude of Lyapunov potential if needed [float].
        - P: the lyapunov potential matrix [ndarray nxn]. 
    """        
    V = isa(subspace.ker(H), A, B)
    F_array = None
    alpha = None
    P = None
    if V is None:
        print('System has no disturbance decouplable subspace')
    else:
        VB = np.concatenate((V,B), axis=1)
        v_row, v_col = V.shape
        n,m = B.shape
        F = cvx.Variable((m, n))
        XS = cvx.Variable((v_col + m, v_col))
        constraint = [VB@XS == A.dot(V)]
        constraint.append(XS[v_col:, :] == -F@V) 
        dd_opt = cvx.Problem(cvx.Minimize(0), constraint)
        dd_opt.solve(solver=cvx.MOSEK, verbose=verbose) 
        
        # abstract resulting controller and confirm constraints.
        F_array = F.value
        XS_array = XS.value
        X_0 = XS_array[:v_col, :]
        lhs = V.dot(X_0) - B.dot(F_array).dot(V)
        rhs = A.dot(V)
        if verbose:
            print(f'DD condition is satisfied {np.allclose(lhs,rhs)}')
        if return_alpha: # solve Lyapunov euqation: 
            A_cl = A+ B.dot(F_array)
            e, v = sla.eig(A_cl.T + A_cl)
            alpha = max(np.abs(e)) * -0.5
            VB = np.concatenate((V, B),axis=1)
            _,v_col = V.shape
            P = np.eye(n)
            return V, F_array, alpha, P
    return V, F_array