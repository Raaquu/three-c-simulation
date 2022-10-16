
# -*- coding: utf-8 -*-

import math 
import numpy

from . import dual_functions 
neaqual  = dual_functions.neaqual

# --- 

def vector_uncross ( crossy ) : 
    '''
    a^{(+)}
    '''
    axial = numpy.array( [- crossy[1,2], crossy[ 0,2 ], - crossy[ 0,1 ]] )
    return axial 


def vector_cross( axis ) : 
    '''
    return the skew-symmetric 'cross matrix' a^{(x)}
    '''
    a1, a2, a3 = axis  
    
    C = numpy.array( [  [   0, -a3,  a2], 
                        [  a3,   0, -a1], 
                        [ -a2,  a1,   0]] )                         
    return C


def vector_unit( axis ) : 
    '''
    return the "unit matrix" or the "axial square matrix"
    '''    
    CC = vector_cross( axis ) 
    
    return - numpy.dot( CC, CC ) 


def vector_square( some_vec ) : 
    '''
    '''
    return numpy.outer( some_vec, some_vec ) 
    

    

def signs( some_values ) : 
    '''
    '''    
    from . import tools
    signs = numpy.array( [ tools.sign( val ) for val in some_values.ravel()  ] ) 
    signs = signs.reshape( some_values.shape )
    return signs


def vector_unsquare( Smat )  : 
    '''
    '''
    from . import tools
    
    enn_abs = numpy.sqrt( numpy.diag( Smat)  ) 
    
    S_times_Spinv   = numpy.dot( Smat, numpy.linalg.pinv( vector_square(enn_abs) )  ) 
    Sign_mat        = numpy.diag( signs( numpy.diag( S_times_Spinv ) ) )  
    enn_real        = numpy.dot( Sign_mat, enn_abs ) 
    
    assert neaqual( Smat, numpy.outer( enn_real, enn_real )  ) 
    
    return enn_real
    



def vector_three( some_vec ) : 
    '''
    TODO : take the one from vector_tools !?!
    '''
    UU = vector_unit( some_vec ) 
    CC = vector_cross( some_vec ) 
    QQ = vector_square( some_vec ) 

    return UU, CC, QQ 



    
def rotation_REGG( angle, axis ) : 
    '''
    '''
    cos_angle   = math.cos( angle ) 
    sin_angle   = math.sin( angle ) 
    
    UU, CC, ZZ  = vector_three( axis ) 
    
    RRR         = cos_angle * UU + sin_angle * CC + ZZ 
    
    return RRR  
    
    
# ---      

def screw_cross( enn, emm ) : 
    '''
    L^{[x]}
    '''
    
    NN = cross_mat( enn )
    MM = cross_mat( emm )
    
    ZZ = numpy.zeros( (3,3) ) 
    return numpy.hstack( [ numpy.vstack( [NN, MM] ), numpy.vstack( [ZZ, NN] ) ] ) 


def screw_unit( enn, emm ) : 
    '''
    L^{[o]}
    '''
    CC = screw_cross( enn, emm ) 
    return - numpy.dot( CC, CC )
    

def screw_square( enn, emm ) : 
    '''
    L^{[/]}
    '''
    UU = screw_unit( enn, emm ) 
    
    return numpy.dot( enn, enn ) * numpy.eye( 6 )  - UU
    
    
def screw_three( enn, emm ) : 
    '''
    generate the three matrices 
    
        ( L^{[o]}, L^{[+]}, L^{[/]} ) 
        
        Dgrave  = exp ( phi * L^{[x]} ) 
                = cos PHI * L^{[o]} + sin PHI * L^{[x]} + 1 * L^{[/]} )     
    '''
    UU = screw_unit( enn, emm ) 
    CC = screw_cross( enn, emm ) 
    QQ = screw_square( enn, emm ) 

    return UU, CC, QQ 
    
    
    
def screw_uncross( cross_six ) : 
    '''
    L^{[+]}
    '''
    enn = uncross( cross_six[0:3, 0:3 ]) 
    emm = uncross( cross_six[3:6, 0:3 ]) 
    
    raise NotImplementedError
    
    

    
