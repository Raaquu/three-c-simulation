

# -*- coding: utf-8 -*-






import math 
import numpy 
numpy.set_printoptions( precision = 4, suppress=True ) 





def acos_stable( cos_angle ) : 
    '''
    TODO : the implementation on the private PC is better ... !
    '''
    if cos_angle == None : 
        return numpy.nan

    if cos_angle < 1.0 and cos_angle > -1.0 :  
        return math.acos( cos_angle ) 
   
    else :             
        if neaqual( cos_angle, 1.0 )  : 
            cos_angle = 1.0 
        
        elif neaqual( cos_angle, - 1.0 ) :  
            cos_angle = - 1.0 
        
        elif cos_angle > 1.0 or cos_angle < -1.0 :  
            return None 
        
        else :
            pass 
        
        angle_zero_pi = math.acos( cos_angle ) 
        
        return angle_zero_pi 
        
        
def neaqual( A, B, eps=0.0001 ) : 
    '''
    '''
    return numpy.allclose( A, B, rtol=eps, atol=eps ) 



def normalized( some_vec ) : 
    '''
    '''
    some_vec = numpy.array( some_vec ) 
    return some_vec / numpy.linalg.norm( some_vec ) 
    
# ---

def recipro_dual( aa ) : 
    '''
    '''
    return divide_dual( (1.0, 0.0 ), aa ) 

def negative_dual( aa ) :
    '''
    '''
    return - aa[0], - aa[1]
    
def divide_dual( aa, bb ) : 
    '''
    either aa or n
    '''
    # assert bb[0].is_digit() 
    
    primal  = aa[0] / bb[0] 
    dual    = ( aa[1] * bb[0] - aa[0] * bb[1] ) / bb[0]**2
    
    return primal, dual 
    
def mult_dual( aa, bb )  : 
    '''
    '''
    primal  = aa[0] * bb[0] 
    dual    = aa[1] * bb[0] + aa[0] * bb[1] 
    
    return primal, dual 


def dot_dual( aa, bb )  :     
    '''
    '''
    if type( aa ) != tuple   and type( bb ) != tuple : 
        aa = tuplize( aa ) 
        bb = tuplize( bb ) 
    
    primal  = numpy.dot( aa[0], bb[0] )  
    dual    = numpy.dot( aa[1], bb[0] ) + numpy.dot( aa[0], bb[1] ) 
    
    return primal, dual 


def atan2_dual( YY, XX ) : 
    '''
    '''    
    yyp, yyd = YY
    xxp, xxd = XX
    
    primal      = math.atan2( yyp, xxp ) 
    dual        = (xxp * yyd - xxd * yyp)  / ( xxp**2 + yyp**2 )
    
    return primal, dual 
    
    
    
def square_dual( XX ) : 
    '''
    '''
    xxp, xxd    = XX
    
    primal      = xxp**2 
    dual        = 2 * xxd * abs( xxp ) 
    
    return primal, dual 
    
    
def sqrt_dual( XX ) : 
    '''
    '''
    xxp, xxd    = XX
    
    primal      = math.sqrt( xxp ) 
    dual        = 0.5 * xxd / math.sqrt( xxp ) 
    
    return primal, dual 
    
    
    
def tuplize( something ) : 
    '''
    '''
    # print( something, len( something )/2  )
    assert len( something ) % 2 == 0 
    
    splitind = int( len( something )/2 ) 
    primal = something[0:splitind ]
    dual   = something[splitind:]

    return primal, dual 
    

def dual_norm( something ) : 
    '''
    '''
    if type( something ) != tuple : 
        something = tuplize( something ) 
        
    return sqrt_dual( dot_dual( something, something )  ) 
    
    
def dif_dual( XX, YY ) : 
    '''
    '''
    if type( XX ) != tuple   and type( YY ) != tuple : 
        XX = tuplize( XX ) 
        YY = tuplize( YY ) 
        
    return XX[0] - YY[0], XX[1] - YY[1]


def sum_dual( XX, YY ) : 
    '''
    '''
    if type( XX ) != tuple   and type( YY ) != tuple : 
        XX = tuplize( XX ) 
        YY = tuplize( YY ) 
    
    return XX[0] + YY[0], XX[1] + YY[1]
    
    
def inversion_dual( AA, BB, CC ) : 
    '''
    solve the dual trigonometric equation 
    '''
    AAsquare    = square_dual( AA )
    BBsquare    = square_dual( BB )
    CCsquare    = square_dual( CC )
    
    
    argument    = dif_dual( sum_dual( AAsquare, BBsquare ), CCsquare )  
    
    
    DD          = sqrt_dual( argument  )  
    
    sol_zero    = atan2_dual( BB, AA ) 
    sol_delta   = atan2_dual( DD, CC ) 
    
    sol_plus    = sum_dual( sol_zero, sol_delta ) 
    sol_minus   = dif_dual( sol_zero, sol_delta ) 
    
    return sol_plus, sol_minus, sol_zero, sol_delta
    

def cos_dual( app, add ) : 
    '''
    '''
    primal  = math.cos(app)
    dual    = - add * math.sin( app )
    
    return primal, dual 
    

def sin_dual( app, add ) : 
    '''
    '''
    primal  = math.sin(app)
    dual    = + add * math.cos( app )
    
    return primal, dual 
    
    
def acos_dual( app, add ) : 
    '''
    app : angle primal 
    add : angle dual  
    '''
    acos_primal = acos_stable( app ) 
    acos_dual   = - add / math.sin( acos_primal ) 
    
    return acos_primal, acos_dual 
    

def asin_dual( app, add ) : 
    '''
    app : angle primal 
    add : angle dual  
    '''
    asin_primal = math.asin( app ) 
    asin_dual   = + add / math.cos( asin_primal ) 
    
    return asin_primal, asin_dual 
    

