# -*- coding: utf-8 -*-


import numpy 

import yarvop.tools.math_tools  as math_tools
# import yarvop.pov_math as pov_math


import yarvop.lines.assembled_tools as atools

import yarvop.lines.plucker as plucker
project_point_on_line = plucker.project_point_on_line

import yarvop.tools.log_tools as log_tools    

# import yarvop.vectors.vector_tools as vector_tools

# --- 


def plucker_line_constellation( lambda_A, lambda_B ) : 
    '''
    '''
    
    cos_dual_angle  = atools.plucker_dot( lambda_A, lambda_B ) 
    cos_dual_angle_obj  = atools.Dual( *cos_dual_angle  ) 
    
    
    A_times_B       = atools.plucker_cross( lambda_A, lambda_B )
    
    '''
    print() 
    print( lambda_A  ) 
    print( lambda_B  )
    print( A_times_B ) 
    print( cos_dual_angle  ) 
    print() 
    '''
    
    case = None 
    if math_tools.numpy_neaqual( cos_dual_angle, (1,0) ) : 
        # coincident or parallel 
        if math_tools.numpy_neaqual( A_times_B, numpy.zeros( 6, ) ) : 
            case  = "coincident" 
        
        else : 
            case  = "parallel" 
            # print( A_times_B  )
            # raise NotImplementedError
    
    elif math_tools.numpy_neaqual( cos_dual_angle_obj.dual, 0.0 ) : 
        case  = "intersecting" 
        
    else : 
        case  = "skew" 
        # print( cos_dual_angle )
        # print() 
        #  raise NotImplementedError
    
    print( log_tools.green( case  ))  
    return case 





def create_linking_strategy_A_A_default() :
    '''
    '''
    mid_anchor      = "of_anchors"
    mid_direction   = "mean_ZZ"
    leaving         = "towards_middle"
    approaching     = "from_middle"
    
    return LinkingStrategy( mid_anchor, mid_direction, leaving, approaching  ) 
    



def create_linking_strategy_ZB_M_anchors() :
    '''
    '''
    mid_anchor      = "of_anchors"
    mid_direction   = "mean_ZZ"
    leaving         = "along_Z_pre_and_back"
    approaching     = "from_middle"
    
    return LinkingStrategy( mid_anchor, mid_direction, leaving, approaching  ) 


def create_linking_strategy_Z_M_anchors() :
    '''
    '''
    mid_anchor      = "of_anchors"
    mid_direction   = "mean_ZZ"
    leaving         = "along_Z_pre"
    approaching     = "from_middle"
    
    return LinkingStrategy( mid_anchor, mid_direction, leaving, approaching  ) 

def create_linking_strategy_ZB_BZ_anchors() : 
    '''
    '''
    mid_anchor      = "of_anchors"
    mid_direction   = "mean_ZZ"
    leaving         = "along_Z_pre_and_back"
    approaching     = "along_Z_back_and_post"
    
    return LinkingStrategy( mid_anchor, mid_direction, leaving, approaching  ) 
    
    
    
    
def create_linking_strategy_ZB_Z_anchors() : 
    '''
    '''
    mid_anchor      = "of_anchors"
    mid_direction   = "mean_ZZ"
    leaving         = "along_Z_pre_and_back"
    approaching     = "along_Z_post"
    
    return LinkingStrategy( mid_anchor, mid_direction, leaving, approaching  ) 
    
    
def create_linking_strategy_Z_Z_anchors( offsets=(4,4) ) :
    '''
    '''
    mid_anchor      = "of_anchors"
    mid_direction   = "mean_ZZ"
    leaving         = "along_Z_pre"
    approaching     = "along_Z_post"
    
    return LinkingStrategy( mid_anchor, mid_direction, leaving, approaching, offset_pre=offsets[0], offset_post=offsets[1]  ) 
    
    
def create_linking_strategy_Z_Z_default() :
    '''
    '''
    mid_anchor      = "of_lines_ZZ"
    mid_direction   = "mean_ZZ"
    leaving         = "along_Z_pre"
    approaching     = "along_Z_post"
    
    return LinkingStrategy( mid_anchor, mid_direction, leaving, approaching  ) 
    
    
class LinkingStrategy() : 
    '''
    strategy to create a link of two joints 
    '''
    def __init__( self, mid_anchor, mid_direction, leaving, approaching, offset_pre=4.0, offset_post=4.0 ) :
        '''
        '''
        assert mid_anchor in [ "of_anchors", "of_lines_ZZ" ] 
        self.mid_anchor     = mid_anchor
        
        assert mid_direction in [ "ident_Z_pre", "ident_Z_pst", "mean_ZZ" ] 
        self.mid_direction  = mid_direction 
        
        assert leaving in [ "along_Z_pre", "along_Z_pre_and_back", "along_X_pre", "towards_middle" ] 
        self.leaving        = leaving
        
        assert approaching in [ "along_Z_post", "along_Z_back_and_post",  "along_X_post", "from_middle" ] 
        self.approaching    = approaching    
        
        self.offset_pre     = offset_pre
        self.offset_post    = offset_post
        
        

def _generate_point_sequence( joint_A, joint_B, strategy, connector_name, verbose=False ) : 
    '''
    '''
    start_point = joint_A.anchor
    stop_point  = joint_B.anchor

    if strategy.mid_anchor == "of_anchors" : 
        anchor_middle = 0.5 * ( joint_A.anchor + joint_B.anchor )
    
    elif strategy.mid_anchor == "of_lines_ZZ" : 
        lambda_A        = atools.direct_anchor_2_plucker( joint_A.direction, joint_A.anchor )        
        lambda_B        = atools.direct_anchor_2_plucker( joint_B.direction, joint_B.anchor )    
        
        cpoint_A_2_B, cpoint_B_2_A  = atools.plucker_closest_points( lambda_A, lambda_B ) 
        anchor_middle = 0.5 * ( cpoint_A_2_B + cpoint_B_2_A )
    
    else : 
        assert False 
    

    if strategy.mid_direction == "ident_Z_pre" : 
        direction_middle = joint_A.direction 
        
    elif strategy.mid_direction == "ident_Z_pre" : 
        direction_middle = joint_B.direction 
        
    elif strategy.mid_direction == "mean_ZZ" : 
        direction_middle = math_tools.normalized( joint_A.direction + joint_B.direction  ) 
        
    else : 
        assert False 
        
    middle_line = atools.direct_anchor_2_plucker( direction_middle, anchor_middle ) 
    
    #_lambda_A        = atools.direct_anchor_2_plucker( joint_A.direction, joint_A.anchor )        
    #_lambda_B        = middle_line  
    #towards_middle  = atools.plucker_cross( _lambda_A, _lambda_B )

    start_interpoint = None 
    skip_mid_point_A = False 
    
        
        
    if strategy.leaving == "along_Z_pre" : 
        start_offset = start_point + strategy.offset_pre * joint_A.direction
        
        lambda_A        = atools.direct_anchor_2_plucker( joint_A.direction, joint_A.anchor )  
        middle_on_A     = project_point_on_line( anchor_middle, lambda_A  )
        
        if numpy.linalg.norm( middle_on_A - start_point ) > numpy.linalg.norm( start_offset - start_point )  :             
            start_offset = middle_on_A 
            skip_mid_point_A = True 
        
    elif strategy.leaving == "along_Z_pre_and_back" :
        start_offset     = start_point + strategy.offset_pre * joint_A.direction
        
        lambda_X        = atools.direct_anchor_2_plucker( joint_A.direction, anchor_middle )  
        
        if strategy.approaching == "along_Z_back_and_post" :
            lambda_X        = middle_line 
        
        start_interpoint =  project_point_on_line( start_offset, lambda_X )
        
    elif strategy.leaving == "along_X_pre" : 
        start_offset = start_point + strategy.offset_pre * joint_A.direction
        
    elif strategy.leaving == "towards_middle" :
        lambda_A        = atools.direct_anchor_2_plucker( joint_A.direction, joint_A.anchor )        
        lambda_B        = middle_line  
        
        # this is not properly directed 
        # towards_middle  = atools.plucker_cross( lambda_A, lambda_B )        
        anchor, direction, cpoint_A_2_B, cpoint_B_2_A  = plucker.aligned_perpendicular( lambda_A, lambda_B, "along_pairing" ) 
        
        start_offset = start_point + strategy.offset_pre * direction
        
        if numpy.linalg.norm( start_offset - start_point ) >= numpy.linalg.norm( anchor_middle - start_point )  : 
            start_offset = None 
           
        if start_offset is not None and math_tools.neaqual( numpy.linalg.norm( start_offset - start_point ), 0.0 ):
            start_offset = None 
        
    else : 
        assert False 
    
    skip_mid_point_B = False 
    stop_interpoint  = None 
    if strategy.approaching == "along_Z_post" : 
        stop_offset = stop_point - strategy.offset_post * joint_B.direction
        
        lambda_B        = atools.direct_anchor_2_plucker( joint_B.direction, joint_B.anchor )  
        middle_on_B     = project_point_on_line( anchor_middle, lambda_B  )
        
        if numpy.linalg.norm( middle_on_B - stop_point ) > numpy.linalg.norm( stop_offset - stop_point )  : 
            stop_offset = middle_on_B 
            skip_mid_point_B  = True 
            
    elif strategy.approaching == "along_Z_back_and_post" : 
        stop_offset = stop_point - strategy.offset_post * joint_B.direction
         
        lambda_X        = atools.direct_anchor_2_plucker( joint_B.direction, anchor_middle )  

        if strategy.leaving == "along_Z_pre_and_back" : 
            lambda_X        = middle_line 
            
        stop_interpoint =  project_point_on_line( stop_offset, lambda_X )
       
        
    elif strategy.approaching == "along_X_post" : 
        stop_offset  = stop_point -  strategy.offset_post * joint_B.direction
        
        
        
    elif strategy.approaching == "from_middle" :
        lambda_A        = middle_line  
        lambda_B        = atools.direct_anchor_2_plucker( joint_B.direction, joint_B.anchor )     
        ## from_middle     = atools.plucker_cross( lambda_A, lambda_B ) 
        
        anchor, direction, cpoint_A_2_B, cpoint_B_2_A  = plucker.aligned_perpendicular( lambda_A, lambda_B, "along_pairing" ) 
        
        stop_offset     = start_point + strategy.offset_pre * direction
        
        if numpy.linalg.norm( stop_offset - stop_point ) >= numpy.linalg.norm( anchor_middle - stop_point )  : 
            stop_offset = None 
        
        if stop_offset is not None and math_tools.neaqual( numpy.linalg.norm( stop_offset - stop_point ), 0.0 ):
            stop_offset = None 
            
    else : 
        assert False 
        
    
    sequence = [ start_point ]
    if start_offset is not None : 
        sequence.append( start_offset  )

    if start_interpoint  is not None :
        sequence.append( start_interpoint  )
        
    if skip_mid_point_A  and skip_mid_point_B : 
        pass 
    else : 
        sequence.append( anchor_middle ) 
    
    if stop_interpoint is not None : 
        sequence.append( stop_interpoint ) 
        
    if stop_offset is not None : 
        sequence.append( stop_offset )
    
    sequence.append( stop_point  ) 
    
    if verbose : 
        for el in sequence : 
            print( el )
        print() 
    
    #if connector_name == "002_003_003_004" : 
    #    import ipdb 
    #    ipdb.set_trace() 
            
    # import sys 
    # sys.exit(0)
    return sequence  

import yarvop.pov_math as pov_math
import yarvop.pov_common as pov_common

class PovConnector( pov_common.PovObject ) : 
    '''
    '''
    povtype = "body"
    povcategory = "Connector" 
    
    def __init__( self, name, link, point_sequence, color=None ) : 
        '''
        '''
        self.name = name 
        
        
        self.link = link
        
        self.pose_label             = "Pose_Connector_"     + self.name
        self.texture_label_major    = "Texture_Major_"      + self.name
        self.texture_label_minor    = "Texture_Minor_"      + self.name
        self.radius_label           = "Connector_Radius_"   + self.name

        self.point_sequence = point_sequence 
        
        self.pose = numpy.eye( 4 )
        
        if color is None : 
            self.color = pov_common.select_color( "random" ) 
        else : 
            self.color = color 
        
        pov_common.PovObject.__init__( self )
        
    
    @classmethod 
    def create_pov_macros_and_declares( cls ) : 
        '''
        no class defines yet 
        '''
        macros = str()
        declares = str( )
        
        # declares = '''
        # #declare ''' + self.texture_name + ''' = texture { pigment { color rgb CHSL2RGB( <244,1,0.5> ) } } 
        # '''
        return macros, declares
    
    
    def update_pose( self, pose, psr ) : 
        '''
        '''
        self.pose          = pose
        psr.set_declare( ( self.__class__.povcategory, self.pose_label ), " transform { " + pov_math.povray_matrix_from_homat( pose )  +  "}" ) 
                        
    
    def generate_pov_declares( self ) : 
        '''
        '''
        pose_matrix_string  = pov_math.povray_matrix_from_homat( self.pose ) 
        declare_label       = (self.pose_label, " transform { " + pose_matrix_string + "}" )

        declare_radius      = (self.radius_label, "0.25" ) 
        
        color_str           = str( self.color )
        
        # HSL = < Hue, Saturation, Lightness, Filter, Transmit > 
        #
        sat  = str( 0.0 ) 
                    
        decl_texture_major  = ( self.texture_label_major, " texture { pigment { color rgb CHSL2RGB( <" + color_str + ", " + sat + ", 0.375> ) } } "  )
        decl_texture_minor  = ( self.texture_label_minor, " texture { pigment { color rgb CHSL2RGB( <" + color_str + ", " + sat + ", 0.5> ) } } "  )
        
        declare_tuples      = [ declare_label, declare_radius, decl_texture_major, decl_texture_minor ]
        
        return declare_tuples 
    
    
    def generate_pov_string( self ) : 
        '''
        pretty pov representation 
        '''
        connection_string = '''
merge
{
'''
        for index, (point_S, point_T) in enumerate( pairwise( self.point_sequence )  ): 
            point_S_string  = "<" + pov_math.formatter.format( point_S[0] ) + "," + pov_math.formatter.format( point_S[1] ) + "," +  pov_math.formatter.format( point_S[2] ) + ">" 
            point_T_string  = "<" + pov_math.formatter.format( point_T[0] ) + "," + pov_math.formatter.format( point_T[1] ) + "," +  pov_math.formatter.format( point_T[2] ) + ">" 
            
            connection_string += '''    cylinder{''' + point_S_string + "," + point_T_string + ", " + self.radius_label + "\n        texture{ " +   self.texture_label_major + "}}\n\n"
            if index < len( self.point_sequence )-2 : 
                connection_string += '''    sphere{ '''+ point_T_string  + ", 1.0*" + self.radius_label + "\n         texture{ " + self.texture_label_major + " }}\n\n" 
    
        connection_string += "    transform " + self.pose_label + "\n\n"
        
        connection_string += "}\n"
    
        return connection_string 
            
        
    
        
        
        
def generate_link_via_strategy( pov_mechanism, link, joint_A, joint_B, color, strategy ) : 
    '''
    '''
    
    connector_name  = joint_A.name[3:] + "_" + joint_B.name[3:] 
    point_sequence  = _generate_point_sequence( joint_A, joint_B, strategy, connector_name  )  
    
    # new implementation, 2020-03-09
    connector       = PovConnector( connector_name, link, point_sequence, color  )
    
    pov_mechanism.register_connector( link, connector )
    
    return connector 
    

    # old implementation     
    pov_string      = _generate_connector_sequence_string( point_sequence , pose_label_connector, "Texture_Body_LightGray_A" ) 
    return pov_string
    
    
def generate_link( joint_A, joint_B, leaving_A="with_Z", approaching_B="with_Z", offset=2 ) : 
    '''
    '''
    anchor_midpoint = 0.5 * ( joint_A.anchor + joint_B.anchor )
    
    if leaving_A == "with_Z" : 
        lambda_A        = atools.direct_anchor_2_plucker( joint_A.direction, joint_A.anchor )        
        point_A         = joint_A.anchor+joint_A.direction*offset 
        direct_A        = joint_A.direction
        
    elif leaving_A == "with_X" :     
        lambda_A        = atools.direct_anchor_2_plucker( joint_A.direction, joint_A.anchor )        
        point_A         = joint_A.anchor+joint_A.xaxis*offset 
        direct_A        = joint_A.direction
        
    else : 
        assert False 
        
    if approaching_B == "with_Z" : 
        lambda_B        = atools.direct_anchor_2_plucker( joint_B.direction, joint_B.anchor )    
        point_B         = joint_B.anchor-joint_B.direction*offset 
        direct_B        = joint_B.direction
        
    elif approaching_B == "with_X" :     
        lambda_B        = atools.direct_anchor_2_plucker( joint_B.direction, joint_B.anchor )
        point_B         = joint_B.anchor-joint_B.xaxis*offset  
        direct_B        = joint_B.direction
        
    else : 
        assert False 
    
    constellation   = plucker_line_constellation( lambda_A, lambda_B ) 
    
    blob = None 
    if constellation == "coincident" : 
        blob            = _generate_connector_sequence_string( ( point_A, point_B ), "Texture_Body_LightGray_A" ) 
        
    elif constellation == "intersecting" : 
        cpoint_A_2_B, cpoint_B_2_A  = atools.plucker_closest_points( lambda_A, lambda_B ) 
        # sequence                    = [ point_A, cpoint_A_2_B, point_B ]
        sequence                    = [ joint_A.anchor, point_A, anchor_midpoint, point_B, joint_B.anchor ]
        
        # aux_A = joint_A.anchor+joint_A.direction*offset
        # aux_B = joint_B.anchor-joint_B.direction*offset
        # sequence                    = [ joint_A.anchor, aux_A, aux_B, joint_B.anchor ]
        
        # print( "sequence" )
        # print( sequence )
        # print()
            

        '''
        if math_tools.numpy_neaqual( point_A, cpoint_A_2_B ) : 
            sequence.pop( 0 )                    
        
        elif math_tools.numpy_neaqual( cpoint_A_2_B, point_B ) : 
            sequence.pop( -1 )                    
        '''
        
        # print( "sequence" )
        # print( sequence )
        # print()
            
        blob            = _generate_connector_sequence_string( sequence , "Texture_Body_LightGray_A" ) 
    

    elif constellation == "parallel" : 
        # sequence  = _four_point_sequence( point_A , direct_A, joint_B.anchor, direct_B, offset )         
        
        sequence        = [  joint_A.anchor, point_A , anchor_midpoint, point_B, joint_B.anchor ]
        blob            = _generate_connector_sequence_string( sequence, "Texture_Body_LightGray_A" )     
        

    elif constellation == "skew" : 
        cpoint_A_2_B, cpoint_B_2_A  = atools.plucker_closest_points( lambda_A, lambda_B ) 
      
        sequence                    = [ point_A, cpoint_A_2_B, cpoint_B_2_A, point_B  ]
        print(  "sequence (skew, post) " )
        print( sequence )
        print()
        if math_tools.numpy_neaqual( point_A, cpoint_A_2_B ) : 
            sequence.pop( 0 )                    
        
        if math_tools.numpy_neaqual( cpoint_B_2_A, point_B ) : 
            sequence.pop( -1 )                    
          
        print( "sequence (skew, pre) " )
        print( sequence )
        print()
        
        
        '''
        aux_A = joint_A.anchor+joint_A.direction*offset
        aux_B = joint_B.anchor-joint_B.direction*offset
        sequence                    = [ joint_A.anchor, aux_A, aux_B, joint_B.anchor ]
        
        print( "sequence (NEW) " )
        print( sequence )
        print()
        '''

        sequence                    = [ joint_A.anchor, point_A, anchor_midpoint, point_B, joint_B.anchor ]        
        
        blob            = _generate_connector_sequence_string( sequence , "Texture_Body_LightGray_A" ) 
        
    else : 
        assert False 
        
    assert blob 
    return blob 



def generate_connector( psr, terminal_A, terminal_B, pose_label_connector, texture_ident ) : 
    '''
    '''
    if len( terminal_A ) == 3 and len( terminal_B ) == 3 : 
        return _generate_connector_two_points( psr, terminal_A, terminal_B, pose_label_connector, texture_ident ) 

    else : 
        raise NotImplementedError 

        
def _four_point_sequence( anchor_A, direction_A, anchor_B, direction_B, offset )   : 
    '''
    '''
    point_1 = anchor_A 
    point_2 = anchor_A + offset * math_tools.normalized( direction_A )
    point_3 = anchor_B + offset * math_tools.normalized( direction_B )
    point_4 = anchor_B   
    
    return point_1, point_2, point_3, point_4 




from itertools import tee

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)

def _generate_connector_sequence_string( list_of_points, pose_label_connector, texture_name ) : 
    '''
    '''
    connection_string = '''
#declare Radius = 0.25;
merge
{
'''
    for index, (point_S, point_T) in enumerate( pairwise( list_of_points )  ): 
        point_S_string  = "<" + str( point_S[0] ) + "," + str( point_S[1] ) + "," +  str( point_S[2] ) + ">" 
        point_T_string  = "<" + str( point_T[0] ) + "," + str( point_T[1] ) + "," +  str( point_T[2] ) + ">" 
        
        connection_string += '''    cylinder{''' + point_S_string + "," + point_T_string + "," + " Radius     texture{" +   texture_name + "}}\n"
        if index < len( list_of_points )-2 : 
            connection_string += '''    sphere{ '''+ point_T_string  + ", 1.0*Radius                texture{ " + texture_name + " }}\n" 

    # to be added 
    # 
    
    connection_string += "    transform " + pose_label_connector + "\n\n"
    
    connection_string += "}\n"
    
    
    return connection_string    



    
            
def _generate_connector_two_points( psr, point_A, point_B, pose_label_connector, texture_ident ) : 
    '''
    '''
    list_of_points  = point_A, point_B 
    
    blob            = _generate_connector_sequence_string( list_of_points, pose_label_connector, texture_ident  )
    psr.add_object_simple_tmp( "Connector (two points)", blob ) 
    
    return blob 

        

