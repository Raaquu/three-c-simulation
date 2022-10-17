// Logging some tests to see if the results are valid
//import * as math from "mathjs";
import nj from "numjs"
import {

    MeshStandardMaterial,
    CylinderBufferGeometry,
    Vector3,
    Mesh,
    CylinderGeometry,
    Line,
    Quaternion,
    Object3D,
    BufferGeometry,
    LineBasicMaterial,
    Matrix4
  } from "three";
import * as dual_functions from "../lines/dual_functions.js" ;
import { make_adjoint_matrix ,acos_3 ,getTranslationAndRotation, inverseKinematics, getAngle,
     adjoint_trig_rodrigues, make_unit_line, ort_spear, get_trig_var, getDualMatrixInverse, rotMatrixtoEulerAngles, is_Parallel} from '../displacements/tools.js';
import { blowUpMatrix, rotation_REGG, screw_three, vector_cross } from "../lines/screw_three.js";




class p_vec {
    
    constructor(p_vec){
        this.p_vec = p_vec
        this.n = p_vec.slice([0,3]);
        this.m = p_vec.slice([3,6]);
        this.T = p_vec.T
        }
}

// generate a1,n2,a2,n2,a3,n3 and zero ref, target_pos and target_rot in this order
// TODO: make generation random, for now testing with static variables
function generateRandom(){

    var a1 = [0,0,0]
    var a2 = [6,0,8]
    var a3 = [4,4,8]

    var n1 = [1,0,0]
    var n2 = [-1,4,2]
    var n3 = [0,0,1]

    var zero_ref = [4,4,10]
    var target_pos = [4,4,12]
    var target_rot = 30

    return [a1,a2,a3,n1,n2,n3,zero_ref, target_pos, target_rot]
}

function testGenerate(){

        var genVar = generateRandom()
        var inverse_solution = inverseKinematics(genVar[0],genVar[1],
                genVar[2],genVar[3],genVar[4],
                genVar[5],genVar[6],genVar[7],genVar[8])
        return inverse_solution
}
var x = testGenerate()


// var terminal_ground = new Object3D()
// terminal_ground.up = new Vector3(0,0,1)


var a1 = [0,0,0]
var a2 = [0,5,8]
var a3 = [5,2,5]
var a4 = [5,36,0]

var z =  [0,0,1]
var n1 = [0,1,1]
var n2 = [0,0,1]
var n3 = [4,8,0]
var n4 = [0,0,1]

var zero01 = getZeroRef(a1,n1,z)
var zero12 = getZeroRef(a2,n2,z)
var zero23 = getZeroRef(a3,n3,z)
var zero34 = getZeroRef(a4,n4,z)

// console.log(getTranslationAndRotation(zero34))
// console.log(getTranslationAndRotation(zero23))
// console.log(getTranslationAndRotation(zero12))
// console.log(getTranslationAndRotation(zero01))

var l1 = make_unit_line(n1,a1)
var l2 = make_unit_line(n2,a2)
var l3 = make_unit_line(n3,a3)
var l4 = make_unit_line(n4,a4)

// console.log(generateLink(a1,a2,n1,n2,2))

var mid1 = (getNearestPoints(l1,l2))
var mid2 =(getNearestPoints(l2,l3))
var mid3 = (getNearestPoints(l3,l4))

var dir1 = dual_functions.normalized(math.add(n1,n2))
var dir2 = dual_functions.normalized(math.add(n2,n3))
var dir3 = dual_functions.normalized(math.add(n2,n3))



export function getNewAnchor(relativeAnchor, matrix){
    var rot = getTranslationAndRotation(matrix)[0]
    var trans = getTranslationAndRotation(matrix)[1]    
    var transformMat = new Matrix4()
    transformMat.set(   rot[0][0], rot[1][0], rot[2][0], trans[0],
                        rot[0][1], rot[1][1], rot[2][1], trans[1],
                        rot[0][2], rot[2][0], rot[2][2], trans[2],
                        0,         0,         0,         1         )

    var newAnchor = new Object3D()
    newAnchor = relativeAnchor

    console.log(newAnchor.position)
    newAnchor.position.setFromMatrixPosition(transformMat)
    console.log(newAnchor.position)
    // console.log(newAnchor.rotation)
    newAnchor.setRotationFromMatrix(transformMat)
    console.log(newAnchor.rotation)
  return newAnchor

}

var counterPositive = 0;

function testInverse(){
    var counter = 0;
    for(let i = -1; i < 1; i+= 0.1){
        for(let j = -1; j < 1; j+= 0.1){
            for(let k = -1; k < 1; k+= 0.1){

                
            var A_12 = new p_vec(make_unit_line([i,0,0],
                [math.random(-10,10),math.random(-10,10),math.random(-10,10)]))
            
            var A_23 = new p_vec(make_unit_line([i,-j,k],
                    [math.random(-10,10),math.random(-10,10),math.random(-10,10)]))

            var A_34 = new p_vec(make_unit_line([i-2,j+1,-2],
                [math.random(-10,10),math.random(-10,10),math.random(-10,10)]))

            var target_rot = 40
            var zero_ref = [5,5,0]
            var target_pos = [-4,4,12]
            counter += 1;
                solveInverse(A_12, A_23, A_34,zero_ref,target_pos,target_rot)
            }
        }
    }
    return [counter, counterPositive];
}

console.log(testInverse())

// TODO: calculate no kinematics if no yaml file is given
// TODO: Update the joints that are being ed
export function solveInverse(A_12,A_23,A_34, zero_ref, target_pos, target_rot){


    // zero reference posture in 6x6 frame
    // Assuming rotation is 0
    var zero_ref = make_adjoint_matrix(zero_ref,0)

    // D14
    // console.log(zero_ref)
    var target_pose = make_adjoint_matrix(target_pos,target_rot)

    // reference displacements
    var zero_ref_inv = getDualMatrixInverse(zero_ref)
    var s = nj.dot(target_pose, zero_ref_inv)

    // Getting the trigonometric variables to solve equation 37
    var trig_var = get_trig_var(A_12,A_23,A_34,s)

    var phi2 = dual_functions.inversion_dual(trig_var[0],trig_var[1],trig_var[2])

    var phi2Pos = phi2[0]
    var phi2Neg = phi2[1]

    if(phi2Pos == "x" && phi2Neg == "x"){
        // console.log("no solution")
        return;
    }

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
    
    // Same for phi3
    var phi3Pos = acos_3(nj.dot(getDualMatrixInverse(s),(A_12.p_vec).T) ,
        nj.dot(getDualMatrixInverse(M_23_pos), (A_12.p_vec).T), (A_34.p_vec).T)
    var phi3Neg = acos_3(nj.dot(getDualMatrixInverse(s),(A_12.p_vec).T) ,
        nj.dot(getDualMatrixInverse(M_23_neg), (A_12.p_vec).T), (A_34.p_vec).T)

    var M_34_pos = adjoint_trig_rodrigues(phi3Pos,A_34)
    var M_34_neg = adjoint_trig_rodrigues(phi3Neg,A_34)


    var D_neg = nj.dot(nj.dot(nj.dot(M_12_neg,M_23_neg),M_34_neg),zero_ref)
    var D_pos = nj.dot(nj.dot(nj.dot(M_12_pos,M_23_pos) ,M_34_pos), zero_ref)
    var D_expected = make_adjoint_matrix(target_pos,target_rot)

    if (math.subtract (math.norm(D_expected.tolist()), math.norm(D_pos.tolist())) > 1e-3
    && math.subtract (math.norm(D_expected.tolist()), math.norm(D_neg.tolist())) > 1e-3)
    {
        console.log("false" ,
        trig_var, phi1Neg, phi2Neg, phi3Neg)
        counterPositive +=1;
        
    }else{
    }
 } 
// solveInverse()

function distArr(a,b){ 

    return Number(math.sqrt( math.pow((a[0]-b[0]), 2) + math.pow((a[1]-b[1]), 2) + math.pow((a[2]-b[2]), 2)).toFixed(3));
  }


function getNearestPoints(line1, line2){
    var n_a = line1.slice([0,3]).tolist()
    var m_a = line1.slice([3,6]).tolist()
    var n_b = line2.slice([0,3]).tolist()
    var m_b = line2.slice([3,6]).tolist()
  
    var a_a = math.cross(n_a, m_a)
    var a_b = math.cross(n_b, m_b)
  
    var d_ab = math.subtract(a_b,a_a)
  
    var upTerm1 = math.dot(math.cross(math.cross(n_b,n_a), n_b), d_ab)
    var upTerm2 = math.dot(math.cross(math.cross(n_a,n_b), n_a), d_ab)
    var downTerm = math.norm(math.cross(n_a, n_b)) * math.norm(math.cross(n_a, n_b))
  
    var line1_p = math.add(a_a, math.dotMultiply(math.divide(upTerm1, downTerm) , n_a))
    var line2_p = math.subtract(a_b, math.dotMultiply(math.divide(upTerm2, downTerm), n_b))
  
    // divide by zero
    if(downTerm == 0){
        var line1_p = a_a
        var line2_p = a_b
    }
    // like strategy of lines ZZ in BBO
  
    var anchor_middle = math.dotMultiply( 0.5, math.add(line1_p,line2_p))
    return anchor_middle
  }
  

  // ungenauigkeiten sind in der berechnung, rotation REGG oder quaternion
function getZeroRef(anchor, jointAxis, standardUp) {
  
  
    var quaternion = new Quaternion();
    var up = new Vector3(standardUp[0],standardUp[1],standardUp[2]).normalize()
    var a2 = new Vector3(jointAxis[0], jointAxis[1], jointAxis[2]).normalize();
    quaternion.setFromUnitVectors(up, a2);
  
    var axis = [quaternion.x, quaternion.y, quaternion.z];
    if(axis[0] == 0 && axis[1] == 0 && axis[2] == 0){
        axis = [0,0,1]
    }
    var angle = math.multiply(2, math.acos(quaternion.w));
    var rotation = rotation_REGG(angle, axis);
  
    var txR = nj.dot(vector_cross(anchor), rotation);
    return blowUpMatrix(rotation, txR);
  }

function testTrigVar (){
    var s = nj.identity(6)
    for(let i = 1; i < 2; i+= 0.1){
        for(let j = 0; j < 2; j+= 0.1){
            for(let k = 0; k < 2; k+= 0.1){
                // parallelcase
            var A12 = new p_vec(make_unit_line([i,j,k],
                [math.random(-10,10),math.random(-10,10),math.random(-10,10)]))
            // var A23 = make_unit_line([i,j,k],[math.random(-10,10),math.random(-10,10),math.random(-10,10)])
            
            var A23 = new p_vec(make_unit_line([j-1,i,k],
                [math.random(-10,10),math.random(-10,10),math.random(-10,10)]))
                // not parallel
            var A34 = new p_vec(make_unit_line([j,-i,k],
                [math.random(-10,10),math.random(-10,10),math.random(-10,10)]))

            var trigVar = get_trig_var(A12,A23,A34,s)
                if(math.subtract(math.abs(trigVar[0]), math.abs(trigVar[2])) > 1e-6){
                    console.log("false", trigVar)
                }
            }
        }
    }
}
// testTrigVar()

function testScrews(){
    for(let i = 0; i < 2; i+= 0.1){
        for(let j = 0; j < 2; j+= 0.1){
            for(let k = 1; k < 2; k+= 0.1){
            var unitline = make_unit_line([i,j,k],[math.random(-10,10),math.random(-10,10),math.random(-10,10)])
            var enn =  unitline.slice([0,3])
            var emm  = unitline.slice([3,6])
            var S = screw_three(enn,emm)
            var CC = S[1]
            var ZZ = S[2]
            if(math.subtract(math.abs(nj.dot(CC,unitline).tolist()) ,nj.zeros([6]).tolist()) > 1e-6 ){
                    console.log(nj.dot(CC,unitline),"false1")
                }
                if(math.subtract(math.abs(nj.dot(ZZ,unitline).tolist()) , math.abs(unitline.tolist())) > 1e-6){
                    console.log("false2")
                }
                else{
                    console.log("true")
                }
            }
        }
    }
}
// testScrews()

function testRodrigues(angle_dual){
    for(let i = 1; i < 2; i+= 0.1){
        for(let j = 0; j < 2; j+= 0.1){
            for(let k = 0; k < 2; k+= 0.1){
        
            var unitline = make_unit_line([i,j,k],[math.random(-10,10),math.random(-10,10),math.random(-10,10)])
            var trig = adjoint_trig_rodrigues(angle_dual,unitline)
            if(math.subtract(math.abs(nj.dot(trig, unitline).tolist()), 
                math.abs(unitline.tolist())) > 1e-6 ){
                console.log(unitline, nj.dot(trig,unitline), "false")
                }
                else{
                    console.log("true")
                    // console.log(math.subtract(math.abs(nj.dot(trig, unitline).tolist()), 
                    // math.abs(unitline.tolist())))
                }
            }
        }
    }
}
function testDualAnglesInRodrig(){
    for (let i = - math.pi; i <= math.pi; i += math.pi/4 ){
        testRodrigues([i,math.random(-10,10)])
    }
}

function testOrtSpear(){
    for(let i = 1; i < 2; i+= 0.1){
        for(let j = 0; j < 2; j+= 0.1){
            for(let k = 0; k < 2; k+= 0.1){
                // parallelcase
            var unitline = make_unit_line([i,j,k],[math.random(-10,10),math.random(-10,10),math.random(-10,10)])
            var parallel = make_unit_line([i,j,k],[math.random(-10,10),math.random(-10,10),math.random(-10,10)])
            // not parallel
            var otherLine = make_unit_line([j,-i,k],[math.random(-10,10),math.random(-10,10),math.random(-10,10)])
            var orth = ort_spear(unitline, parallel)
            var orth2 = ort_spear(unitline,otherLine)
                if(nj.dot(unitline.slice([0,3]), orth.slice([0,3])) > 1e-6|| nj.dot(parallel.slice([0,3]),orth.slice([0,3])) > 1e-6){
                    console.log("false", orth, unitline)
                }
                if(nj.dot(unitline.slice([0,3]), orth2.slice([0,3])) > 1e-6|| nj.dot(otherLine.slice([0,3]),orth2.slice([0,3])) > 1e-6){
                    console.log("false", orth2, unitline)
                }
                else{
                    console.log("true")
                }
            }
        }
    }
}
// testOrtSpear()
// testDualAnglesInRodrig()

// Trash from the numjs import, keeping for better track of mistakes while importing
// var nj = require('numjs');
// var ndarray = require("ndarray");
// var nj = globalThis.nj		
// nj = (await import ("numjs")).default
// import {createRequire} from "module";
// const require = createRequire( import.meta.url);
// const { Matrix } = require('ml-matrix')