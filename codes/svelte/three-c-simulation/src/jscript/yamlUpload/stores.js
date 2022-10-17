//import { writable } from 'svelte/store';

export const submitted = writable(false)
export const fileText = writable([])
export const dataPos = writable([])
export const dataNeg = writable([])
export const dataZero = writable([])
export const yamlData = writable([])
export const correct = writable(false)
export const dispZero = writable(true)
export const showOtherForm = writable(true)

    /**

    < !-- import {
        AxesHelper,
        BoxBufferGeometry,
        GridHelper,
        MeshStandardMaterial,
    } from "three";
import {
    Canvas,
    DirectionalLight,
    HemisphereLight,
    OrbitControls,
    PerspectiveCamera,
    Mesh,
    Object3DInstance,
} from "@threlte/core"; -->

    	< !--import {
    Mesh,
    Object3DInstance,
} from "@threlte/core";
import { correct, dataPos, dispZero } from './jscript/yamlUpload/stores.js' -->
    	< !--import {
    Mesh,
    Object3DInstance,
} from "@threlte/core";
import { correct, dataNeg, dispZero } from './jscript/yamlUpload/stores.js' -->
    	< !--import {
    Mesh,
    Object3DInstance,
} from "@threlte/core";
import { correct, dataZero, dispZero } from './jscript/yamlUpload/stores.js' --> */