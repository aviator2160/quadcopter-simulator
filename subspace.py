# -*- coding: utf-8 -*-
"""
Created on Thu Apr  8 13:50:01 2021

Subspace linear algebra operations.

@author: Sarah Li
"""
import numpy as np
import numpy.linalg as la

def rank(X, eps = 1e-14):
    """ Return the rank of the input matrix.
    
    Args:
        - X: matrix [ndarray].
    Returns:
        - rank: rank of X [int]. 
    """
    m,n = X.shape
    if n == 0: 
        return 0
    else:
        return la.matrix_rank(X, eps)
    
def ker(A, eps = 1e-14):
    """ Return a matrix whose columns span the kernel of the input matrix.
    
    Args:
        - A: matrix [ndarray].
    Returns:
        - ker: a matrix whose columns form an orthogonal basis for
          matrix A's kernel [ndarray].  
    """
    U,D,V = la.svd(A, full_matrices=True)
    r = rank(A, eps)
    m,n = V.shape
    if r == m:
        return np.zeros(V.T.shape)
    else:
        return V[r:, :].T
    
def image(X):
    """ Return a matrix whose columns span the image of the input matrix.
    
    Args: 
        - X: matrix [ndarray].
    Returns: 
        - image: a matrix whose columns form an orthogonal basis for
          matrix A's rank [ndarray].    
    """
    U,D,V = la.svd(X)
    r = rank(X)
    return U[:,:r]

def control_subspace(A, B):
    """ Return the dimension of the (A, B) controllable subspace.
    
    Args: 
        - A: system dynamics matrix [ndarray].
        - B: control matrix [ndarray].
    Returns: 
        - control_subspace: rank of the controllable subspace [int].    
    """
    n,_ = A.shape
    controllable_list = [la.matrix_power(A, i).dot(B) for i in range(n)]
    controllability_matrix = np.concatenate(controllable_list, axis=1)
    return rank(controllability_matrix)

def union(A, B):
    """ Add two subspaces (A, B) together.
    
    Args: 
        - A: a matrix whose columns span subspace A [ndarray].
        - B: a matrix whose columns span subspace B [ndarray].
    Returns:
        - union: a matrix whose columns form the orthogonal basis for subspace
            addition A+B [ndarray]. 
    """
    m,n = A.shape
    x,y = B.shape    
    if m != x:
        raise Exception('input matrices need to be of same height');
    T = np.hstack((A, B))
    return image(T)

def a_inv_v(A, V):
    """ Return the subspace A^{-1}(im(V)).
    
    Note that (A^{-1}im(V)) = ker((A^Tker(V.T)).T)
    
    Args:
        - A: matrix A [ndarray].
        - V: matrix V whose columns span subspace V [ndarray].
    Returns:
        - A_inv_V: a matrix whose columns span A^{-1}(im(V)) [ndarray].
    """
    kerV = ker(V.T);
    AtV = A.T.dot(kerV)
    return ker(AtV.T)

def intersect(A, B, verbose=False):
    """ Return A intersect B. 
    
    Given matrix A and matrix B  return the intersection of their ranges by
    noting that A intersect B = ker( (ker(A.T) + ker(B.T)).T )

    Args:
        - A: a matrix whose columns span the subspace A [ndarray]. 
        - B: a matrix whose columns span the subspace B [ndarray].
        - verbose: True if print debugging info [bool].
    Returns:
        - intersect: a matrix whose columns span (A intersect B) [ndarray].
    """
    kerA = ker(A.T, 1e-14)
    kerB = ker(B.T, 1e-14)
    if verbose:
        print("In intersect")
        print (kerA.shape)
        print (kerB.shape)
    return ker(np.hstack((kerA, kerB)).T, 1e-14)

def contained(A,B):
    """ Check if subspace A is contained in subspace B.
    
    Args:
        - A: a matrix whose columns span the subspace A [ndarray]. 
        - B: a matrix whose columns span the subspace B [ndarray].
    Returns:
        - contained: True if A subseteq B, False otherwise [bool].
    """
    cap = ker(B.T).T.dot(A)
    if np.allclose(cap, np.zeros(cap.shape)):   
        return True
    else: 
        return False