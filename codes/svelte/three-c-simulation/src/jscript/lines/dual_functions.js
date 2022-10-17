/**
 * Trying out two numpy like functions
 * @param ndarray Numpy like library in JS, with "modified" arrays
 * Checkout http://scijs.net/packages/#scijs/ndarray 
 * @param jsnumpy Numpy like library in JS
 * @param three Three.js 
 */

//  var nj = globalThis.nj
import * as THREE from 'three';

//import * as math from "https://cdnjs.cloudflare.com/ajax/libs/mathjs/11.3.0/math.js"
// import { onMount } from "svelte";
// let nj;

// onMount(async () => {
	
//     nj = await (await import ("numjs")).default


//     })

import nj from "numjs"
// import {createRequire} from "module";

export function acos_stable (cos_angle) {

    if (cos_angle == null) {
        return NaN;
    }

    if (cos_angle < 1.0 && cos_angle > -1.0){
        return math.acos( cos_angle ) ;
    }

    else  {
        if (neaqual( cos_angle, 1.0 )) {
            cos_angle = 1.0; 
        }           
        else if (neaqual( cos_angle, -1.0 )) {
                cos_angle = -1.0; 
        }
        else if (cos_angle > 1.0 ||  cos_angle <-1.0 ) {
            return null; 
        }
        else pass;           
               
        var angle_zero_pi = math.acos( cos_angle );
    
        return angle_zero_pi; 
    }
}

// If the following equation is element-wise True, then 
// neaqual returns True.

//  absolute(a - b) <= (atol + rtol * absolute(b))



export function neaqual (A, B, eps=0.001) {

    if (A instanceof nj.NdArray && B instanceof nj.NdArray) {
        var AA = A.tolist()
        var BB = B.tolist()

        for(var i in range(A.length)) {

            if ((math.abs(AA[i] - BB[i])) > (eps + eps * math.abs(BB[i]))){
         
                return false;
            }
        }
        return true;
    }

    else if (math.abs( A - B )> (eps + eps *math.abs(B))){
            
        return false;
    }  
    return true;
}

export function range(size, startAt = 0) {
    return [...Array(size).keys()].map(i => i + startAt);
}

export function normalized (some_vec) {

    if(some_vec instanceof nj.NdArray){
        some_vec = some_vec.tolist()
    }

    var copy_some_vec = some_vec

    if(math.norm(some_vec) == 0){
        return nj.zeros(some_vec.length)
    }

    return nj.array(copy_some_vec.map(function(x) { return x * (1/math.norm(some_vec)); }));


}

export function recipro_dual( aa ) { 
     
    return divide_dual( [1.0, 0.0 ], aa );
    
}


/**
 * JavaScript does not support multiple return like
 * "return primal, dual"
 * Has to be returned as an array like
 * "return [primal, dual]"
 * or object literal
 * return{primal, dual}
 */

export function negative_dual( aa ) {
       
    var first = - aa[0];
    var second =  - aa[1];

    return [first, second];
}
export function assert(condition, message) {
    if (!condition) {
        throw new Error(message || "Assertion failed");
    }
}

export function divide_dual( aa, bb ) {
     
    // assert(bb[0].is_digit() );
    
    var primal  = aa[0] / bb[0] ;
    var dual    = ( aa[1] * bb[0] - aa[0] * bb[1] ) / bb[0]**2;
    
    return [primal, dual] ;
}

export function mult_dual( aa, bb )  { 
     
     
    var primal  = aa[0] * bb[0]; 
    var dual    = aa[1] * bb[0] + aa[0] * bb[1]; 
    
    return [primal, dual] 
}


export function dot_dual( aa, bb )  {    
     
     
    if ((aa.length > 2 && aa.length % 2 == 0) &&
        (bb.length > 2 && bb.length % 2 == 0)) {
        var aa = tuplize( aa ); 
        var bb = tuplize( bb ); 
    
    }
    // math.dot takes two arrays as arguments !

    var primal  = math.dot( [aa[0]], [bb[0] ]);      
    var dual    = math.dot([ aa[1]], [bb[0] ]) + math.dot([ aa[0]], [bb[1] ]); 

    return [primal, dual]; 
    
}


export function atan2_dual( YY, XX ) {

    // formula 36
    // phi +, phi - = atan2_dual(b,a) +- atan2(d,c) formula 35


    var yyp    = YY[0]
    var yyd    = YY[1];     
    var xxp    = XX[0]
    var xxd    = XX[1];
    
    var primal      = math.atan2( yyp, xxp ) ;
    var dual        = math.subtract(math.multiply(xxp , yyd), math.multiply(xxd , yyp)) 
                     / (math.add(math.multiply(xxp,xxp) , math.multiply(yyp,yyp)));
    
    if(isNaN(dual)){
        dual = 0;
    }
    return [primal, dual] ;
}
    
export function square_dual( XX ) {
    
    var xxp    = XX[0]
    var xxd    = XX[1]
    
    var primal      = math.multiply(xxp,xxp);
    var dual        = math.multiply(math.multiply(2 , xxd ), math.abs( xxp ))
    
    return [primal, dual] ;
}
    
    
export function sqrt_dual( XX ) {
     
     
    var xxp    = XX[0]
    var xxd    = XX[1];
    
    var primal      = math.sqrt( xxp ) ;
    var dual        =  xxd / (2* math.sqrt( xxp ) );

    if(isNaN(dual) || dual == Infinity){
        dual = 0;
    };
    
    return [primal, dual] ;
}
    
export function tuplize( something ) {
     

    let splitind = ( something.length /2 ); 


    let primal = something.slice(0,splitind);
    let dual   = something.slice(splitind);

    return [primal, dual]; 

}

// console.log(tuplize([22,12],[44,55]));

export function dual_norm( something ) {
     
     
    if (something.length > 2){
        something = tuplize( something ) ;
    }  
    return sqrt_dual( dot_dual( something, something )  ); 
    
}  
export function dif_dual( XX, YY ) {
     
     
    if ((XX.length > 2 && XX.length % 2 == 0) &&
    (YY.length > 2 && YY.length % 2 == 0)) {
        XX = tuplize( XX ) ;
        YY = tuplize( YY ) ;
    }    
    return [XX[0] - YY[0], XX[1] - YY[1]];
}

export function sum_dual( XX, YY ) {
     
    if ((XX.length > 2 && XX.length % 2 == 0) &&
    (YY.length > 2 && YY.length % 2 == 0)) {
        XX = tuplize( XX ) ;
        YY = tuplize( YY ) ;
    }    
    return [XX[0] + YY[0], XX[1] + YY[1]]
}   
  
// console.log(sum_dual([12,5],[22,6]))

export function inversion_dual( AA, BB, CC ) {

    // solve the dual trigonometric equation 

    var AAsquare    = square_dual( AA );
    var BBsquare    = square_dual( BB );
    var CCsquare    = square_dual( CC );

    var argument    = dif_dual( sum_dual( AAsquare, BBsquare ), CCsquare );  
    
    // console.log(argument)
    if(argument[0] < 0){
        throw alert("no solution possible, real part < 0")
    }

    // if(argument < [0]){
    //     return["x","x"]
    // }

    // console.log(argument)
    var DD          = sqrt_dual( argument  )  ;

// TODO: try out acos for c from bbo paper
    var sol_zero    = atan2_dual( BB, AA ) ;
    var sol_delta   = atan2_dual( DD, CC ) ;

    var sol_plus    = sum_dual( sol_zero, sol_delta ) ;
    var sol_minus   = dif_dual( sol_zero, sol_delta ) ;

    return [sol_plus, sol_minus];
}

export function cos_dual( app, add ) {
     
     
    var primal  = math.cos(app);
    var dual    = math.multiply(-add, math.sin( app ));
    
    return [primal, dual] ;
    
}
export function sin_dual( app, add ) { 
     
     
    var primal  = math.sin(app);
    var dual    = math.multiply( +add,  math.cos( app ));
    
    return [primal, dual] ;
}  
    
export function acos_dual( app, add ) {
     
    var acos_primal = acos_stable( app ); 

    var acos_dual   = - add / math.sin( acos_primal ) ;
    
    return [acos_primal, acos_dual] 
    
}
export function asin_dual( app, add ) {
     
     
    var asin_primal = math.asin( app ) ;
    var asin_dual   = + add / math.cos( asin_primal ) ;
    
    return [asin_primal, asin_dual]; 
    
}




/**
 * Testing the Functions
 * !! Three.js seems to overwrite the Vectors if a operation is used.
 * You first have to copy the Vector and then use the operation !!
 * (Or functionine global Vector3s)
 * First three functions work otherwise as supposed.
 */

    // var Vector1 = new THREE.Vector3(10,-7,8);
    // console.log(Vector1);
    // var Vector1N = normalized(Vector1);
  
    // console.log(Vector1);

    // console.log(neaqual([4,2],[1,3]));

    // console.log(neaqual([1,2,3],[1,2,3]));


    // console.log(neaqual([1e10,1e8], [1.e9,1e-8]));
    
    // console.log(neaqual([1e10,1e-8], [1e9,1e-9]));
    
    // console.log(neaqual([1e10,1e-8], [1.e10,1e-10]));
    

    // console.log(acos_stable(null));
    // console.log(acos_stable(0.5));
    // console.log(acos_stable(2));
