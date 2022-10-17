import * as dual_functions from "../lines/dual_functions.js" ;
import * as st from "../lines/screw_three.js";
//import * as math from "https://cdnjs.cloudflare.com/ajax/libs/mathjs/11.3.0/math.js"
//import nj from "numjs"
//import * as math from "numjs";
// Class for pluecker vectors, its first part being the direction and the second part being the momentum
class p_vec {
    
    constructor(p_vec){
        this.p_vec = p_vec
        this.n = p_vec.slice([0,3]);
        this.m = p_vec.slice([3,6]);
        this.T = p_vec.T
        }
}

/***
 * @params  a1,a2,a3 anchor points, n1,n2,n3 direction, zero ref = position of first frame ,
 *          targe_pos and target_rot = Target pose to acquire
 * @returns Six Displacement Matrices M1_pos, M2_pos, M3_pos, M1_neg, M2_neg, M3_neg
 */
export function inverseKinematics(a1,a2,a3,n1,n2,n3,zero_ref,target_pos,target_rot){

    // The unit lines from the given input anchors and directions
    var A_12 = new p_vec(make_unit_line(n1,a1))
    var A_23 = new p_vec(make_unit_line(n2,a2))
    var A_34 = new p_vec(make_unit_line(n3,a3))

    // zero reference posture in 6x6 frame
    // Assuming rotation is 0
    var zero_ref = make_adjoint_matrix(zero_ref,0)

    // D14
    var target_pose = make_adjoint_matrix(target_pos,target_rot)

    // reference displacements
    var zero_ref_inv = getDualMatrixInverse(zero_ref)
    var s = nj.dot(target_pose, zero_ref_inv)

    // Getting the trigonometric variables to solve equation 37
    var trig_var = get_trig_var(A_12,A_23,A_34,s)

    // TODO: test the trig var
    // Calculation phi2 via equation 37
    // console.log(trig_var)
    var phi2 = dual_functions.inversion_dual(trig_var[0],trig_var[1],trig_var[2])


    var phi2Pos = phi2[0]
    var phi2Neg = phi2[1]

    // Therefore two matrices are defined for positive and negative angle
    var M_23_pos = adjoint_trig_rodrigues(phi2Pos,A_23)
    var M_23_neg = adjoint_trig_rodrigues(phi2Neg,A_23)

    // Now we can compute the other dual angles as well
    var phi1Pos = acos_3(nj.dot(s,(A_34.p_vec).T),
        nj.dot(M_23_pos,(A_34.p_vec).T),nj.negative((A_12.p_vec).T))
    var phi1Neg = acos_3(nj.dot(s,(A_34.p_vec).T),
        nj.dot(M_23_neg,(A_34.p_vec).T),nj.negative((A_12.p_vec).T))

    // // And its displacements matrices
    var M_12_pos = adjoint_trig_rodrigues(phi1Pos,A_12)
    var M_12_neg = adjoint_trig_rodrigues(phi1Neg,A_12)

    // var A_34_pos = getPositive(A_34.p_vec)
    
    // Same for phi3
    var phi3Pos = acos_3(nj.dot(getDualMatrixInverse(s),(A_12.p_vec).T) ,
        nj.dot(getDualMatrixInverse(M_23_pos), (A_12.p_vec).T), (A_34.p_vec).T)
    var phi3Neg = acos_3(nj.dot(getDualMatrixInverse(s),(A_12.p_vec).T) ,
        nj.dot(getDualMatrixInverse(M_23_neg), (A_12.p_vec).T), (A_34.p_vec).T)
    
    var M_34_pos = adjoint_trig_rodrigues(phi3Pos,A_34)
    var M_34_neg = adjoint_trig_rodrigues(phi3Neg,A_34)


    // Checkup if multiplication matches Displacements

    var D_expected = nj.dot(nj.dot(nj.dot(M_12_pos,M_23_pos) ,M_34_pos), zero_ref) 
    // var D_neg = nj.dot(nj.dot(nj.dot(M_12_neg,M_23_neg),M_34_neg),zero_ref)

    return[D_expected,M_12_pos,M_12_neg,M_23_pos,M_23_neg,M_34_pos,M_34_neg,phi1Neg,phi1Pos,phi2Neg,phi2Pos,phi3Neg,phi3Pos]

}


export function getTranslationAndRotation(DispMatrix){

    var rotation = getPrimalPart(DispMatrix)

    // Inverse of a rotation matrix is the transpose, using here because inverse is not working properly
    var rotation_T = math.transpose(rotation)
    
    // console.log(getDualPart(DispMatrix))
    var Skew = nj.dot(getDualPart(DispMatrix),rotation_T)


    var translation = [Skew.get(2,1),Skew.get(0,2),Skew.get(1,0)]

    return [rotation,translation]
}
// var z = (getTranslationAndRotation(D_14))
// console.log(z)

export function getAngle(rotMat){
    return math.acos(math.divide(math.trace(rotMat)-1,2))
}

// console.log(getAngle(z[0]))
// check if the given Matrix is a Rotation matrix
function isRotationMatrix(R){
    var R_t = math.transpose(R)
    var shouldBeI = nj.dot(R_t,R).tolist()
    var I = nj.identity(3).tolist()
    var n = math.norm(math.subtract(I, shouldBeI))

    return n < 1e-6

}

// convert rotation matrix to euler angles in order ZYX
export function rotMatrixtoEulerAngles (R){
    if(!isRotationMatrix(R)){
        throw console.error("Matrix is not a Rotation Matrix");
    }
    var R_nj = nj.array(R)

    var sy = math.sqrt((R_nj.get(0,0) * R_nj.get(0,0)) + (R_nj.get(1,0) * R_nj.get(1,0) ))
    var singular = sy < 1e-6
    if(!singular){
        var x = math.atan2(R_nj.get(2,1), R_nj.get(2,2))
        var y = math.atan2(- R_nj.get(2,0), sy)
        var z = math.atan2(R_nj.get(1,0) , R_nj.get(0,0))

    }else{
        var x = math.atan2(- R_nj.get(1,2) , R_nj.get(1,1))
        var y = math.atan2(- R_nj.get(2,0) , sy)
        var z = 0
     }
     return [z,y,x]
}

// get inverse for dual matrix inverse
export function getDualMatrixInverse(DualMat){


    var transVec = getTranslationAndRotation(DualMat)

    var rot = transVec[0]
    var trans = transVec[1]

    // console.log(rot)
    var primalInverse = math.transpose(rot)

    // console.log(primalInverse)
    var dualInverse = nj.dot(nj.negative(primalInverse), st.vector_cross(trans))

    var inverseMat = st.blowUpMatrix(primalInverse, dualInverse)
    return inverseMat
}

/**
 * @input 6x6 Adjoint Matrix
* @returns Dual Part of a 6x6 Matrix
*/
export function getDualPart(adjMat){

    adjMat = adjMat.tolist()

    var Pre = adjMat.slice([3])

    // reduce takes the columns of a 2d array, so the result has to be transposed
    return math.transpose(Pre.reduce(function(acc,x){
        acc[0].push(x[0])
        acc[1].push(x[1])
        acc[2].push(x[2])
        // console.log(acc)
        return acc
    },[[],[],[]]))
}

// console.log(getDualPart(x))

/**
 * @input 6x6 Adjoint Matrix
* @returns Primal Part of a 6x6 Matrix
*/
export function getPrimalPart(adjMat){

    adjMat = adjMat.tolist()

    var Pre = adjMat.slice([3])

    // result has to be transposed, because reduce takes the columns of a 2darray
    // reduce is normally used in 1darrays so its a bit confusing
    return math.transpose(Pre.reduce(function(acc,x){
        acc[0].push(x[3])
        acc[1].push(x[4])
        acc[2].push(x[5])
        return acc
    },[[],[],[]]))
}
    

/*** 
 * @input primal and dual part of a number/angle
 * @returns Blown up 6x6 matrix for dual numer/angle
 * */ 
function matrify_dual( primal, dual ) {

    var PRIMAL  = nj.diag([primal,primal,primal])
    var DUAL = nj.diag([dual,dual,dual])

    return st.blowUpMatrix(PRIMAL,DUAL)
}

/*** 
 * @input translation vector and angle of rotation
 * @returns D^Adl , leftadjoint displacement matrix R 0
 *                                                  txR R
 * */ 
// TODO: Use rotation Matrix instead of angle
export function make_adjoint_matrix (translation, angle){

    // rotation around its own axis
    var rotMat = st.rotation_REGG(angle, translation)
    // console.log(st.vector_cross(translation))
    var txR = nj.dot(st.vector_cross(translation),rotMat)

    return st.blowUpMatrix(rotMat,txR)
}

/*** 
 * @input dual_angle and plucker vector of direction
 * @returns D^Adl , leftadjoint displacement matrix
 * */ 
export function get_trig_var(A_12,A_23,A_34,s){

    // if s is identity, a and c should be the same
    // Thre screw 6x6 matrices are needed for computation, Square, cross and unit
    var A_23_screw = st.screw_three(A_23.n,A_23.m)
    var z = new p_vec(nj.dot(A_23_screw[0],A_34.T))

    // Inner Product of pluecker vectors, treated as dual numbers
    // Looks ugly
    // TODO: Use dual_ mult instead, make arrays in duals work
    // TODO: Use duals instead of array, might have to refractor whole code for this
    var primal_a = nj.dot((A_12.n).T , (z.n).T)
    var dual_a   = nj.add(nj.dot((A_12.m).T , (z.n).T) ,
                   nj.dot((A_12.n).T , (z.m).T))    

    var a = [primal_a.get(0),dual_a.get(0)]

    var zz = new p_vec(nj.dot(A_23_screw[1] , A_34.T))

    var primal_b = nj.dot((A_12.n).T ,(zz.n).T)
    var dual_b   = nj.add(nj.dot((A_12.m).T , (zz.n).T) ,
                   nj.dot((A_12.n).T , (zz.m).T))

    var b = [primal_b.get(0),dual_b.get(0)]

    // calculating c for the trig equation
    var zy = new p_vec(nj.dot(nj.subtract(s,  A_23_screw[2]) , A_34.T))
    // console.log(zy.p_vec)

    var primal_c = nj.dot((A_12.n).T , (zy.n).T)
    var dual_c   = nj.add(nj.dot((A_12.n).T , (zy.m).T) ,
                   nj.dot((A_12.m).T , (zy.n).T))

    var c = [primal_c.get(0),dual_c.get(0)]

    return [a,b,c]
}

/*** 
 * @input dual_angle and plucker vector
 * @returns D^Adl , leftadjoint displacement matrix
 * */ 

export function adjoint_trig_rodrigues( angle_dual, plucker_vector ) {

    var enn =  plucker_vector.n
    var emm  = plucker_vector.m

    // angle dual[0] is primal part, [1] is dual part
    var cos_angle = dual_functions.cos_dual( angle_dual[0], angle_dual[1] ) 
    var sin_angle = dual_functions.sin_dual( angle_dual[0], angle_dual[1] ) 

    // The three screw matrices needed for computation
    var UU = st.screw_three( enn, emm )[0] 
    var CC = st.screw_three( enn, emm )[1] 
    var ZZ = st.screw_three( enn, emm )[2] 

    // console.log(UU,CC,ZZ)

    // Blowing the angle up to a 6x6 matrix
    var COS = matrify_dual( cos_angle[0], cos_angle[1] )
    var SIN = matrify_dual( sin_angle[0], sin_angle[1] )

    // Equation 33 adjoint trigonometric rodriguez formula
    var pre = nj.add(nj.dot(COS,UU), nj.dot(SIN,CC))

    var DD = nj.add(pre, ZZ ) 
    return DD 

}

/***
 * @returns Dual angle from a to b wrt to n;
 *          angle_dual[0] is primal part, [1] is dual
 */
export function acos_3(a,b,n){

    // The function follows the algorithm of sinan barut acos3
    var vec_na = norm_plucker_vector(ort_spear(n,a))
    var vec_nb = norm_plucker_vector(ort_spear(n,b))

    var n_na = vec_na.slice([0,3]).tolist()
    var n_nb = vec_nb.slice([0,3]).tolist()

    var m_na = vec_na.slice([3,6]).tolist()
    var m_nb = vec_nb.slice([3,6]).tolist()


    var n_a = a.slice([0,3]).tolist()
    var n_b = b.slice([0,3]).tolist()
    var n_n = n.slice([0,3]).tolist()


    if(math.norm(n_na) == 0 || math.norm(n_nb) == 0 ){
        return [0,0]
    }
    // if det is 0, sgn is 0 too! 
    var sigma_abn = ornt3(n_na,n_nb,n_n)
    if(sigma_abn == 0){
        sigma_abn = 1
    }

    var c_abn = math.dot(n_na, n_nb)
    var primal_part = sigma_abn * dual_functions.acos_stable(c_abn)


    if(!is_Parallel(n_n,n_a) && !is_Parallel(n_n,n_b)){

        var a_na = math.cross(n_na, m_na)   
        var a_nb = math.cross(n_nb, m_nb)
 
        var dual_part = math.dot((math.subtract(a_nb, a_na)), n_n)

    }
    else dual_part = 0
    return [primal_part,dual_part]
}



 /**
  * @input Pluecker vector
*  @returns A normalized plucker vector with ||n|| = 1 or ||m|| = 1 if ||n|| = 0
*/
function norm_plucker_vector(pvec){

    // TODO: Use plucker class instead
    // Retrieving full vector, direction and moment
    var p = pvec.tolist()
    var n = pvec.slice([0,3]).tolist()
    var m = pvec.slice([3,6]).tolist()

    // Plucker length is 1/||n|| if ||n|| != 0
    if(math.norm(n) != 0){
        return nj.transpose(p.map(function(x) { return x * (1/math.norm(n)); }));
    }
    if(math.norm(n) == 0 && math.norm(m) == 0){
        return nj.transpose([0,0,1,0,0,0])
    }
    // else length is ||m||
    if (math.norm(n) == 0 && math.norm(m) != 0){
        return nj.transpose(p.map(function(x) { return x * (1/math.norm(m)); }));
    }
}

/**
 * @input n = direction and anchor a
*  @returns unit line 𝚲ij with vector n and anchor A
*/
export function make_unit_line(n,anchor){

    // First Making the standard pvec
    // console.log(n)
    var x = dual_functions.normalized(n).tolist()
    // console.log(x)
    var pvec = nj.concatenate(nj.transpose(x),nj.transpose(math.cross(anchor,x)))


    return pvec

}

/***
@returns mxn of input spears m,n
*/
export function ort_spear(a,b){

    //page 12 of baruts presentation

    var n_a = a.slice([0,3]).tolist()
    var m_a = a.slice([3,6]).tolist()

    var n_b = b.slice([0,3]).tolist()
    var m_b = b.slice([3,6]).tolist()

    //If m and n are parallel, n and m are swapped
    if(is_Parallel(n_a, n_b)){
        // console.log("parallel")
        // console.log(a,b)
        var m_axb = math.cross (m_a,m_b)
        var n_axb = math.add(math.cross (m_a, n_b) , math.cross(n_a,m_b))
    }
    // Normal calc
    else if (!is_Parallel(n_a,n_b)){
        // console.log("notParallel")
        var n_axb = nj.array(math.cross(n_a,n_b))
        var m_axb = nj.array(math.add(math.cross(n_a,m_b), math.cross(m_a,n_b)))
    }

    var result_spear = nj.concatenate(n_axb,m_axb)
    // console.log(result_spear)
    return result_spear
}


/**
@param m Vector
@param n Vector
@returns True or false whether m,n are parallel.
*/
export function is_Parallel(m,n){

    // Normalizing input vectors and calculating dot product
    var x = (dual_functions.normalized(m))
    var y = (dual_functions.normalized(n))
    var z = (nj.dot(x,y))

    // Due to rounding errors neqaul is need
    // If dot product = 1, vectors are parallel elsewise not
    if(dual_functions.neaqual (nj.dot(z,z), nj.array([1]) )){
            return true
    }
    else{
        return false
    }
}

/***
@input  vector a and b and relative vector n
@returns Orientation of a,b in relation to n
*/
function ornt3(a,b,n){
    // var dot_prod = n.slice[0,3] * math.cross(a.slice[0,3], b.slice[0,3]);

    var mat = math.transpose([a,b,n])

    // with plucker vectors as input has to be sliced in first and second vector
    // first vector being the direction and relevant for orientation
    var mat_det = (math.det(mat))
    // console.log(mat_det)
    var ornt = math.sign(mat_det)
    return ornt
}
