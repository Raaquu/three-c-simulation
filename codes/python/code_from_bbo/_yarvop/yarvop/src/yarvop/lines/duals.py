

from __future__ import division, print_function

# --- 

import numpy 

# --- 

import yarvop.lines.dual_functions as dual_functions 

# --- 

class Dual() : 
    '''
    '''
    def __init__( self, prim, dual ):
        '''
        '''
        self.prim = prim
        self.dual = dual 
        
        self.tuple = (self.prim, self.dual)
        
        
    def __add__(self, other):
        '''
        '''
        result_tuple = dual_functions.sum_dual( self.tuple, other.tuple  ) 
        return Dual( *result_tuple ) 
    
    def __mul__(self, other ) : 
        '''
        '''
        if other.__class__ == self.__class__ : 
            result_tuple = dual_functions.mult_dual( self.tuple, other.tuple  ) 
            return Dual( *result_tuple ) 
        
        elif type( other ) == numpy.ndarray :
            assert len( other ) == 6 
            other_primal, other_dual = other[0:3], other[3:6]
            other_tuple = (other_primal, other_dual)
            res_primal, res_dual = dual_functions.dot_dual( self.tuple, other_tuple )
            result = numpy.hstack( (res_primal, res_dual) )    
            assert len( result ) == 6 
            return result 
        
        elif type( other ) in [int, float] : 
            '''
            '''
            return self*Dual( other, 0.0 ) 
            
        else : 
            print( other )
            import ipdb 
            ipdb.set_trace() 
            assert False 
    
    def __getitem__( self, index ) : 
        '''
        '''
        return self.tuple[index]
    
    def __repr__( self ) : 
        return str( self.tuple )
    
    def __sub__(self, other ) : 
        '''
        '''
        result_tuple = dual_functions.dif_dual( self.tuple, other.tuple  ) 
        return Dual( *result_tuple ) 
            
    
    def __truediv__( self, other ) : 
        '''
        '''
        result_tuple = dual_functions.divide_dual( self.tuple, other.tuple  ) 
        return Dual( *result_tuple ) 
    
    
    def __div__( self, other ) : 
        '''
        '''
        return self.__truediv__( other ) 
    

    def __pos__( self ) : 
        '''
        '''
        return self * Dual( 1, 0 )

    def __neg__( self ) : 
        '''
        '''
        return self * Dual( -1, 0 )

    def __pow__( self, other ) : 
        '''
        '''
        if other==2 : 
          # nope return self * self           
          result_tuple = dual_functions.square_dual( self.tuple ) 
          return Dual( *result_tuple ) 
  
        elif other.prim == 0.5 and other.dual == 0.0 : 
            result_tuple = dual_functions.sqrt_dual( self.tuple ) 
            return Dual( *result_tuple ) 
        
        else : 
            raise NotImplementedError()
            
    def sqrt( self ) : 
        '''
        '''
        return self**Dual( 0.5, 0.0 )
    
