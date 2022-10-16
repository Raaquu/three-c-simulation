import { correct,dataPos, dataNeg, dataZero } from '../yamlUpload/stores.js';
import {generate} from "../yamlUpload/yamlConfig.js"
import { inverseKinematics ,getTranslationAndRotation, rotMatrixtoEulerAngles} from '../displacements/tools.js';
import { getJoint} from './rotation.js';
import { generateLink, getZeroRef , getEffector} from "./rotation.js"
import nj from "numjs"
import { normalized } from '../lines/dual_functions.js';
import * as math from "mathjs"

let configDataPos = [];
let configDataNeg = [];
let configDataZero = [];
let cor;

// for adding the joints in the config store
function addToData( smth,x ) {
    x.push(smth)
  }
  
correct.subscribe(value => {
    cor = value;
  });

// TODO: calculate no kinematics if no yaml file is given
// TODO: Update the joints that are being ed
export function solveInverse(){

    // generating the data from yaml
    var genVar = generate()
    console.log(genVar)
    // solving the inverse Kinematics with the yaml data
    var inverse_solution = inverseKinematics(genVar[0],genVar[1],
            genVar[2],genVar[3],genVar[4],
            genVar[5],genVar[6],genVar[7],genVar[8])

    // passing the data to local variables, unneccessary but good for code understanding
    var a_12 = genVar[0]
    var a_23 = genVar[1]
    var a_34 = genVar[2]

    var jointAxis_12 = genVar[3]
    var jointAxis_23 = genVar[4]
    var jointAxis_34 = genVar[5]

    // all of the calculated displacements matrices for the joints
    var M_12_pos = inverse_solution[1]
    var M_12_neg = inverse_solution[2]
    var M_23_pos = inverse_solution[3]
    var M_23_neg = inverse_solution[4]
    var M_34_pos = inverse_solution[5]
    var M_34_neg = inverse_solution[6]

    // all of the dual angles
    var phi1Neg = inverse_solution[7]
    var phi1Pos = inverse_solution[8]
    var phi2Neg = inverse_solution[9]
    var phi2Pos = inverse_solution[10]
    var phi3Neg = inverse_solution[11]
    var phi3Pos = inverse_solution[12]

    // standardup
    var target_pos = genVar[7]
    // console.log(target_pos)
    var base = genVar[9]
    var z = genVar[10]
    var term_effector = genVar[11]
    var term_axis = genVar[12]
   
    // for positive solution
    var zero01 = getZeroRef(a_12,jointAxis_12,z)
    var zero12 = getZeroRef(a_23,jointAxis_23,z)
    var zero23 = getZeroRef(a_34,jointAxis_34,z)
    var zero34 = getZeroRef(term_effector, term_axis ,z)

    console.log("ssssssssssssssssssssssss")
      console.log(getTranslationAndRotation( zero34))
    console.log("ssssssssssssssssssssssss")
    // -----------------------------------------------------------------------
    // -----------------------------------------------------------------------
    // -----------------------------------------------------------------------
    // negative stuff
    var a1_endN = math.add(a_12, math.dotMultiply(phi1Neg[1], normalized(jointAxis_12).tolist()))

    // new anchor point of joint 2, and endposition with translational offset
    var pointAnchor2N = nj.dot(M_12_neg, zero12)
    var a2_newN = getTranslationAndRotation(pointAnchor2N)[1]
    var a2_endN = math.add(a2_newN, math.dotMultiply(phi2Neg[1], normalized(jointAxis_23).tolist()))

    // new anchor point of joint 3, and endposition with translational offset
    var pointAnchor3N = nj.dot(nj.dot(M_12_neg,M_23_neg), zero23)
    var a3_newN = getTranslationAndRotation( pointAnchor3N)[1]
    var a3_endN = math.add(a3_newN, math.dotMultiply(phi3Neg[1], normalized(jointAxis_34).tolist()))

    // the endeffector
    var endpointN = nj.dot(nj.dot(nj.dot(M_12_neg,M_23_neg),M_34_neg),zero34)
    var a_endpointN = getTranslationAndRotation(endpointN)[1]
    // add the joint values to the Data store, negative Solution
    addToData(getJoint(a_12,jointAxis_12, phi1Neg[0], phi1Neg[1]), configDataNeg)
    addToData(getJoint(a2_newN,jointAxis_23, phi2Neg[0], phi2Neg[1]), configDataNeg)
    addToData(getJoint(a3_newN,jointAxis_34, phi3Neg[0], phi3Neg[1]), configDataNeg)

    addToData(generateLink(base,a_12,z, jointAxis_12,0), configDataNeg)
    addToData(generateLink(a1_endN,a2_newN, jointAxis_12, jointAxis_23, 2), configDataNeg)
    addToData(generateLink(a2_endN,a3_newN, jointAxis_23, jointAxis_34, 2), configDataNeg)
    addToData(generateLink(a3_endN, a_endpointN, jointAxis_34, term_axis, 2), configDataNeg)

    // the negative dual angles to display
    addToData(round(phi1Neg), configDataNeg)
    addToData(round(phi2Neg), configDataNeg)
    addToData(round(phi3Neg), configDataNeg)

    addToData(getEffector(target_pos, z),configDataNeg)

    // -----------------------------------------------------------------------
    // -----------------------------------------------------------------------
    // -----------------------------------------------------------------------

    // positive stuff
    // endposition of trans offset of joint 1
    var a1_end = math.add(a_12, math.dotMultiply(phi1Pos[1], normalized(jointAxis_12).tolist()))

    // new anchor point of joint 2, and endposition with translational offset
    var pointAnchor2 = nj.dot(M_12_pos, zero12)
    var a2_new = getTranslationAndRotation(pointAnchor2)[1]
    var a2_end = math.add(a2_new, math.dotMultiply(phi2Pos[1], normalized(jointAxis_23).tolist()))

    // new anchor point of joint 3, and endposition with translational offset
    var pointAnchor3 = nj.dot(nj.dot(M_12_pos,M_23_pos), zero23)
    var a3_new = getTranslationAndRotation( pointAnchor3)[1]
    var a3_end = math.add(a3_new, math.dotMultiply(phi3Pos[1], normalized(jointAxis_34).tolist()))

    // the endeffector
    var endpoint = nj.dot(nj.dot(nj.dot(M_12_pos,M_23_pos),M_34_pos),zero34)
    var a_endpoint = getTranslationAndRotation(endpoint)[1]

    // add the joint values to the Data store, positive Solution
    addToData(getJoint(a_12,jointAxis_12, phi1Pos[0], phi1Pos[1]), configDataPos)
    addToData(getJoint(a2_new,jointAxis_23, phi2Pos[0], phi2Pos[1]), configDataPos)
    addToData(getJoint(a3_new,jointAxis_34, phi3Pos[0], phi3Pos[1]), configDataPos)

    // links between joints,base and terminal for positive solution
    addToData(generateLink(base,a_12,z, jointAxis_12,0), configDataPos)
    addToData(generateLink(a1_end,a2_new, jointAxis_12, jointAxis_23, 2), configDataPos)
    addToData(generateLink(a2_end,a3_new, jointAxis_23, jointAxis_34, 2), configDataPos)
    addToData(generateLink(a3_end, a_endpoint, jointAxis_34, term_axis, 2), configDataPos)

    // the positive dual angles to display
    addToData(round(phi1Pos),configDataPos)
    addToData(round(phi2Pos),configDataPos)
    addToData(round(phi3Pos),configDataPos)

    // TODO: get axis from endeffector with rotation
    addToData(getEffector(target_pos, z),configDataPos)

    // -----------------------------------------------------------------------
    // -----------------------------------------------------------------------
    // -----------------------------------------------------------------------

    // zero reference posture
    addToData(getJoint(a_12,jointAxis_12, 0, 1),configDataZero)
    addToData(getJoint(a_23,jointAxis_23, 0, 1),configDataZero)
    addToData(getJoint(a_34,jointAxis_34, 0, 1),configDataZero)

    // zero reference links
    addToData(generateLink(base,a_12, z, jointAxis_12, 2),configDataZero)
    addToData(generateLink(a_12,a_23, jointAxis_12, jointAxis_23, 2),configDataZero)
    addToData(generateLink(a_23,a_34, jointAxis_23, jointAxis_34, 2),configDataZero)
    addToData(generateLink(a_34,term_effector, jointAxis_34, term_axis, 2),configDataZero)
    console.log("xxxxxxxxxxxxxxxxxxx")
    console.log(getTranslationAndRotation(inverse_solution[0]))
    console.log(a_endpoint)
    console.log(a_endpointN)
    console.log("xxxxxxxxxxxxxxxxxxx")


    addToData(getEffector(base, z), configDataZero)
    addToData(getEffector(term_effector, term_axis), configDataZero)

    dataPos.set(configDataPos)
    dataNeg.set(configDataNeg)
    dataZero.set(configDataZero)

    cor = true;
    correct.set(cor)

    } 
// rounding an array of numbers, i.e a vector
function round(arrIn){
    arrIn = arrIn.map(function(each_element){
        return Number(each_element.toFixed(3))
    })
    return arrIn
}

// get rounded distance of two points
function dist(a,b){ 
  return Number(Math.sqrt( Math.pow((a.x-b.x), 2) + Math.pow((a.y-b.y), 2) + Math.pow((a.z-b.z), 2)).toFixed(3));
 
 }
 