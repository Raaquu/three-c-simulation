


def matrify_dual( primal, dual ) : 
    '''
    '''
    PRIMAL  = primal * numpy.eye( 3 ) 
    DUAL    = dual   * numpy.eye( 3 )  
    ZZ      = numpy.zeros( (3,3) ) 
    
    return numpy.hstack( [ numpy.vstack( [PRIMAL, DUAL] ), numpy.vstack( [ZZ, PRIMAL] ) ] ) 




def adjoint_trig_rodrigues( angle_dual, plucker_vector ) : 
    '''
    the adjoint rodrigues formula 
    '''
    assert len( plucker_vector ) == 6
    enn, emm  = plucker_vector[0:3], plucker_vector[3:6]
    
    cos_angle = cos_dual( *angle_dual  ) 
    sin_angle = sin_dual( *angle_dual  ) 
    
    UU, CC, ZZ = screw_three_new( enn, emm ) 
    
    COS = matrify_dual( *cos_angle )
    SIN = matrify_dual( *sin_angle )
    
    # print 
    # print "COS TRACE" , numpy.trace( numpy.dot( COS, UU )[3:6,0:3 ]) 
    # print "? COS TRACE" , numpy.trace( numpy.dot( SIN, CC )[3:6,0:3 ]) 
    # print "? COS TRACE" , numpy.trace( ZZ [3:6,0:3 ]) 
    # print 
    
    DD = numpy.dot( COS, UU ) + numpy.dot( SIN, CC ) + ZZ 
    
    return DD 

