import * as dual_functions from "./dual_functions.js" ; 
import { getDualPart } from "../displacements/tools.js";
//import * as math from "https://cdnjs.cloudflare.com/ajax/libs/mathjs/11.3.0/math.js";
import nj from "numjs";
import { sqrt_dual, square_dual, sum_dual, dif_dual } from "./dual_functions.js";

export function vector_cross( axis ) { 
    
    // return the skew-symmetric 'cross matrix' a^{(x)}
    
    if(axis instanceof nj.NdArray){
        var a1 = axis.get(0)
        var a2 = axis.get(1) 
        var a3 = axis.get(2)
    }
    else{
        var a1 = axis[0]
        var a2 = axis[1] 
        var a3 = axis[2]
    }
    
    var C = nj.array( [  [   0, -a3,  a2], 
                        [  a3,   0, -a1], 
                        [ -a2,  a1,   0]] )   

    return C
}

function vector_unit( axis ) { 
    
    // return the "unit matrix" or the "axial square matrix"

    var CC = vector_cross( axis ) 

    return nj.dot(CC, CC )
}


const cartesianProduct = (a, b) =>
  a.reduce((p, x) => [...p, ...b.map(y => [x * y])], []);

function vector_square(axis){

    if(axis instanceof nj.NdArray){
        var a1 = axis.get(0)
        var a2 = axis.get(1) 
        var a3 = axis.get(2)
    }
    else{
        var a1 = axis[0]
        var a2 = axis[1] 
        var a3 = axis[2]
    }

   

    
    var C = nj.array( [  [   a1*a1, a1*a2,  a1*a3], 
                        [  a2*a1,   a2*a2, a2*a3], 
                        [ a3*a1,  a3*a2,   a3*a3]] )   

    return C
    
}

function vector_three( some_vec ) { 
    
    var UU = vector_unit( some_vec ) 
    var CC = vector_cross( some_vec )
    var QQ = vector_square( some_vec )

    return [UU, CC, QQ] 

}
function toRadians (angle) {
    return angle * (Math.PI / 180);
  }
    
export function rotation_REGG( angle, axis ) { 
    
    // math.cos takes RADIAN !!!
    // formula 15 in atrd
    // var angleRadian = toRadians(angle)
    var cos_angle   = math.cos( angle ) 
    var sin_angle   = math.sin( angle ) 

    var normAxis = dual_functions.normalized(axis)

    var UU  = vector_three( normAxis )[0].tolist()
    var CC  = vector_three( normAxis )[1].tolist()
    var ZZ  = vector_three( normAxis )[2].tolist()

    var COS = math.dotMultiply(cos_angle,UU)
    var SIN = math.dotMultiply(sin_angle,CC)

    var RR  = math.add(math.add( COS, SIN), ZZ)
    
    return nj.array(RR)
    
    
}   


// vector enn, emm has to be in transposed(right) form
/*** 
 * @input Sliced plucker vector in n and m
 * @returns Cross matrix L^{[x]} for a plucker_vector
 * */ 
export function screw_cross( enn, emm ) { 
    
    // L^{[x]}
    var NN = vector_cross(enn);
    var MM = vector_cross(emm);
    return blowUpMatrix(NN, MM);
    
}

export function blowUpMatrix(NN, MM) {
    var ZZ = nj.zeros([3, 3]);

    var stackedN = nj.stack([NN, MM]);
    var stackedM = nj.stack([ZZ, NN]);

    var x = stackedN.transpose(2, 0, 1).reshape(3, -1);
    var y = stackedM.transpose(2, 0, 1).reshape(3, -1);
    var stackedZ = nj.stack([x, y]);

    var z = stackedZ.transpose(2, 0, 1).reshape(6, -1);

    // (NN ZZ  )
    // ( MM NN )
    return z;
}

/*** 
 * @input Sliced plucker vector in n and m
 * @returns unit matrix L^{[o]} for a plucker_vector
 * */ 

export function screw_unit( enn, emm ) { 
    
    // L^{[o]}
    
    var CC = screw_cross( enn, emm ) 
    // console.log(CC)
    return nj.dot(CC, CC )
}

// TODO: use full plucker vector instead of sliced
/*** 
 * @input Sliced plucker vector in n and m
 * @returns square matrix L^{[/]} for a plucker_vector
 * */ 

// trying out why some tests fail
// var x=   [ -0.6804138174397718, -8.044732423601474 ]
// var y =[ 0.4132869465853779, 0.5663802654757472 ]
// var z =[ -0.7600019137428282, -7.7293711388426205 ]

// var xx = square_dual(x)

// var yy = square_dual(y)
// var zz = square_dual(z)

// console.log(sqrt_dual(dif_dual( sum_dual( xx, yy ), zz )));

export function screw_square( enn, emm ) { 
    
    // L^{[/]}
    
    var UU = screw_unit( enn, emm )
    var primalPart = vector_square(enn)

    var dualPart = nj.negative(getDualPart(UU))
    return blowUpMatrix(primalPart,dualPart)
}
 
/*** 
 * @input Sliced plucker vector in n and m
 * @returns unit[0], cross[1] and square[2] matrices, 
 *  L^{[o]}, L^{[+]}, L^{[/]} )for a plucker_vector
 * */ 
export function screw_three( enn, emm ) { 
    
    // generate the three matrices 
    
    // ( L^{[o]}, L^{[+]}, L^{[/]} ) 
    // prerequisit for formula 15 in atrd
    // Dgrave  = exp ( phi * L^{[x]} ) 
    // = cos PHI * L^{[o]} + sin PHI * L^{[x]} + 1 * L^{[/]} )     
    

    var UU = screw_unit( enn, emm ) 
    var CC = screw_cross( enn, emm ) 
    var QQ = screw_square( enn, emm ) 


    return [UU, CC, QQ] 
    
}  