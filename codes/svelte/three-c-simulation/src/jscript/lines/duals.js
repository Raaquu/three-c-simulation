
import * as dual_functions from "./dual_functions.js" ; 
import * as THREE from 'three';

// import * as math from "https://cdnjs.cloudflare.com/ajax/libs/mathjs/11.3.0/math.js"
import nj from "numjs"

export class Dual { 

    constructor(prim,dual){
        this.prim = prim;
        this.dual = dual;
        this.tuple = [this.prim, this.dual];
    }
        
    __add__(other){
        var result_tuple = dual_functions.sum_dual( this.tuple, other.tuple  ) 
        return new Dual( result_tuple[0], result_tuple[1] ) 
    }

     __mul__(other ) { 
        
       if (other instanceof Dual) { 
     
            var result_tuple = dual_functions.mult_dual( this.tuple, other.tuple  ) 
            return new Dual( result_tuple[0], result_tuple[1] )
        }

        else if (other instanceof ndarray) {
            //  originally checks whether other is ndarray instance
            // TODO check if other is ndarray not array 
            // (maybe there is no difference though in variable type)
            dual_functions.assert( other.length  == 6, "Length not 6");  
            [other_primal, other_dual] = [other.slice(0,3), other.slice(3,6)]
            var other_tuple = [other_primal, other_dual]

            var res = dual_functions.dot_dual( this.tuple, other_tuple )
            var result = np.hstack( (res[0], res[1]) )    
            dual_functions.assert( result.length  == 6, "Length not 6");  
            return result 
        }

        else if (other instanceof Number){ 
            
            
            return this*Dual( other, 0.0 ) 
        }    
        else { 
            // print( other )
            // import ipdb 
            // ipdb.set_trace() 
            // assert False 
        }
    }   
     __getitem__(index ) {       
        return this.tuple[index]
    }

     __repr__() { 
        return ( this.tuple )
    }

     __sub__(other ) {     
        var result_tuple = dual_functions.dif_dual( this.tuple, other.tuple  ) 
        return new Dual( result_tuple[0], result_tuple[1] ) 
    }        
    
     __truediv__(other ) {   
        var result_tuple = dual_functions.divide_dual( this.tuple, other.tuple  ) 
        return new Dual( result_tuple[0], result_tuple[1] ) 
    }
    
     __div__( other ) { 
        return this.__truediv__( other ) 
    }

     __pos__() {     
        return this.__mul__( new Dual (1, 0) )
    }

     __neg__() {     
        return this.__mul__( new Dual (-1, 0) )
    }

     __pow__(other ) { 
        
        
        if (other==2) { 
        //   nope return this * this           
          var result_tuple = dual_functions.square_dual( this.tuple ) 
          return new Dual( result_tuple[0], result_tuple[1] )  
        }    
        else if (other.prim == 0.5 && other.dual == 0.0) { 
            result_tuple = dual_functions.sqrt_dual( this.tuple ) 
            return new Dual( result_tuple[0], result_tuple[1] )  
        }

        else { 
            //  raise NotImplementedError()
        }
    }      

     sqrt() {        
        return this** new Dual( 0.5, 0.0 )
    }
}


// var test2  = new Dual(22,6)
// var xx = [11,2]
// var yy = [3,4]

// console.log(test1.__pos__())
// console.log(test1.__neg__())
// console.log(test1.__repr__())
// console.log(test1.__div__(test2))


// console.log(test1)
// var test3 = (test1.__add__(test2))
// test1.__add__(test2);
// console.log(test1)