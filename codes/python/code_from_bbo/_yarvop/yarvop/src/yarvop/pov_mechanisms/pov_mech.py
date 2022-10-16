# -*- coding: utf-8 -*-


import yarvop.pov_mechanisms.pov_joints as pov_joints 
import yarvop.pov_mechanisms.pov_links as pov_links 

# ---

import numpy 
    
import yarvop.tools.math_tools  as math_tools

    
class PovMechanism( ) : 
    '''
    '''
    def __init__( self, mechanism, linking_strategies, plot_specs=None  ) : 
        '''
        '''
        self.mechanism              = mechanism
        self.linking_strategies     = linking_strategies 
        self.plot_specs             = plot_specs
        
        self.links_2_pov_connectors = dict()
        self.joints = dict() 
        
        
        
    @classmethod 
    def create_pov_macros_and_declares( self  ) : 
        '''
        '''
        raise NotImplementedError 

    
    def register_connector( self, link, connector ) : 
        '''
        '''
        # assert False 
        if link in self.links_2_pov_connectors : 
            self.links_2_pov_connectors[link.name].append( connector  )
        else : 
            self.links_2_pov_connectors[link.name] = [ connector ] 
        
        

    def configuation_named_2_dictionary( self, config_name ) : 
        '''
        '''
        config = self.mechanism.configurations.get( config_name, None ) 
        
        configuration = dict() 
        for diction in config.jointvalues :             
            configuration[ diction.name ] =  numpy.array( [ math_tools.interpret( val ) for val in diction.values ]  )
        
        
        for aux_joint in self.mechanism.get_aux_joints() :
            configuration[ aux_joint.name ] = None 
        
        return configuration 
            
            
    def select_configuration( self, config_info, psr, external=False ) : 
        '''
        is this really for the pov class ?!
        
        external == use a plain bare dictionary 
        '''
        
        if external is True : 
            configuration   = config_info
        
        elif external is False :             
            config_name = config_info
            assert type( config_name ) == str

            configuration   = self.configuation_named_2_dictionary( config_name ) 
            assert configuration 
            
            if config_name  == "config_zero" : 
                for jname, joint in self.mechanism.joints.items() : 
                    configuration[jname ] = numpy.array( joint.zero_config() )
                
        else : 
            assert False            
                
        products = self.mechanism.compute_link_poses_for_configuration_FID( configuration ) 
        
        sorted_link_keys = sorted( self.mechanism.links.keys() )
        
        # .pose_zero_body
        for index, sk in enumerate( sorted_link_keys  ):        
            link = self.mechanism.links[sk]
            if index*2-1 < 0 : 
                link .pose_curr_proximal = products[ index*2-1 ]
            else : 
                link.pose_curr_proximal= products[ index*2-1 ]
                
            link.pose_curr_distal = products[ index*2 ]
            
        
        import yarvop.tools.math_tools as math_tools
        import yarvop.displacements.tools as disp_tools
        
        for index, sk in enumerate( sorted_link_keys  ):             
            link = self.mechanism.links[sk]
            
            
            assert link.pose_zero_proximal is not None 
            assert link.pose_curr_proximal is not None 

            pose__zero__prox      = link.pose_zero_proximal 
            pose__zero__prox__inv = numpy.linalg.inv( pose__zero__prox )
            pose__curr__prox      = link.pose_curr_proximal
            disp__zero_t__prox    = numpy.dot( pose__zero__prox__inv, pose__curr__prox  )                    
            link_displacement     = math_tools.dots( pose__zero__prox, disp__zero_t__prox, pose__zero__prox__inv )
            
            
            link_connectors = self.links_2_pov_connectors.get( sk, None ) 
            if link_connectors : 
                for lconn in link_connectors : 
                    
                    assert lconn.link ==  self.mechanism.links[sk]  
                    link = lconn.link
                    lconn.update_pose( link_displacement, psr ) 
            else : 
                pass 
                # print( "no connector for ", sk)
                

            # update joints 
            #       
            ljoiints_in, ljoints_out = self.mechanism.get_adjacent_joints( link )
            
            for joint_in in ljoiints_in : 
                pov_joint = self.joints[ joint_in.name  ] 
                pov_joint.update_pose_post( link_displacement, psr ) 
                
            for joint_out in ljoints_out : 
                pov_joint = self.joints[ joint_out.name  ] 
                pov_joint.update_pose_pre( link_displacement, psr ) 
                
            # import ipdb 
            # ipdb.set_trace() 
            
            # def update_pose_pre( self, pose_pre, psr ) : 

def add_pov_mechanism_to_psr( psr, pov_mechanism ) : 
    '''
    '''
    joints = pov_mechanism.mechanism.joints  
    links = pov_mechanism.mechanism.links
   
     
    for joint_name, joint in joints.items() : 
   
        color = pov_mechanism.plot_specs[ joint_name ]["color"]
        
        if joint.typus == "terminal_ground" :             
            pov_joint  = pov_joints.PovTerminalGround( joint, color ) 
        
        elif joint.typus  == "terminal_effector" : 
            pov_joint = pov_joints.PovTerminalEfffector( joint, color )  
            
        elif joint.typus  == "revolute" : 
            pov_joint = pov_joints.PovJointRevolute( joint, color )  
            
        elif joint.typus == "prismatic" : 
            pov_joint= pov_joints.PovJointPrismatic( joint, color )  
            
        elif joint.typus == "cylindric" : 
            pov_joint= pov_joints.PovJointCylindric( joint, color ) 
                    
        else : 
            raise NotImplementedError 
    
        pov_mechanism.joints[ joint_name ] = pov_joint             
        pov_joint.add_to_renderer( psr )    
            
    
   
    
    for lname, link in links.items() : 
        
        # for all adjacent joint paris ..
        #
        adj_joints_in, adj_joints_out = pov_mechanism.mechanism.get_adjacent_joints(  link )
                
        for joint_in in adj_joints_in : 
            for joint_out in adj_joints_out  : 
                # joint_in = pov_mechanism.mechanism.joints[ joint_name_in ]
                # joint_out = pov_mechanism.mechanism.joints[ joint_name_out ]
                
                
                
                ## TODO : make this an object like spear or point ... 
                ## 
                ##def generate_pov_declares( self ) : 
                ##    pose_matrix_string  = pov_math.povray_matrix_from_homat( self.pose ) 
                ##    declare_label       = (self.pose_label, " transform { " + pose_matrix_string + "}" )                    
                #                     assert False 
                
                strategy_name   = pov_mechanism.linking_strategies.get( (joint_in.name, joint_out.name ), None ) 
                
                import yarvop.pov_graphics.pov_colors    as pov_colors
                
                if strategy_name : 
                    strategy        = eval( "pov_links." + strategy_name + "()" )     
                    
                    color           = pov_colors.mean_color_int( pov_mechanism.joints[ joint_in.name ].color, pov_mechanism.joints[ joint_out.name ].color ) 
                    
                    connector       = pov_links.generate_link_via_strategy( pov_mechanism, link, joint_in, joint_out, color, strategy ) 
                    connector.add_to_renderer( psr )    
                # import ipdb 
                # ipdb.set_trace() 

                # old implementation
                # link = pov_links.generate_link_via_strategy( joint_in, joint_out,  pose_label_connector, strategy ) 
                # psr.add_object_simple_tmp( "Link 001", link  ) 
    
    return 
    
