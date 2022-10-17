<script>
  import {
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
  } from "@threlte/core";
  import Joint1 from "../Joint1.svelte";
  import Joint2 from "../Joint2.svelte";
  import Joint3 from "../Joint3.svelte";
  import {fileText, submitted,correct, dataZero, dataPos, dataNeg, dispZero, showOtherForm} from '../jscript/yamlUpload/stores.js'
  import {solveInverse} from "../jscript/displacements/cylinder.js"


  let files;
  let zero_ref;
  let text = [];
  let show;

  // loading data from a yaml file which the user can select
  function dataLoad(files){	
    if (window.FileReader) {
      let reader = new FileReader();
      reader.readAsText(files[0]);
      const fileName = files[0].name;
            
      if (fileName.search("yaml") > -1) {
          reader.onload = function (event) {
          let yaml = event.target.result;
          text = yaml
          fileText.set(text)
          };
      } else {
        alert(
          "Uploaded file is not a Yaml. Please, make sure to import Yaml, and include .yaml at the end of the file."
        );
      document.getElementById("yaml_file").value = null;
    }
    } else {
    alert("FileReader are not supported in this browser.");
    }
  };

  // setting submit and showZero config to true if button is clicked
  function isSubmitted(){
    submitted.set(true)
  }

  function showForm(){
    showOtherForm.update(n => !n)
  }
  
  function showZero(){
    dispZero.update(n => !n)
  }

  // values for different stores and subscriptions
  let submit;
  let cor;
  let configDataNeg = [];
  let configDataPos = [];
  let configDataZero = [];

  dispZero.subscribe(value => {
    zero_ref = value;
  });

  correct.subscribe(value => {
    cor = value;
  });

  submitted.subscribe(value =>{
    submit=value
  })

  dataNeg.subscribe(value => {
    configDataNeg = value; 
  });
  dataPos.subscribe(value => {
    configDataPos = value; 
  });

  dataZero.subscribe(value => {
    configDataZero = value; 
  });

  showOtherForm.subscribe(value => {
    show = value; 
  });


    // execute loading of data if a file is selected
  $: if (files) {
    dataLoad(files);
  }

</script>
<!-- TODO: ask BBO about orientation of camera -->
<div class = "canvas">
<Canvas>

  <PerspectiveCamera position={{ x: 0, y: 5, z: 100 }} lookAt={{x: 0, y: 0, z: 0}}>
      <OrbitControls 
      enableZoom={true}/>
      defaultUp
  </PerspectiveCamera>

  <DirectionalLight
      shadow
      color={'#EDBD9C'}
      position={{ x: -15, y: 45, z: 20 }}
  />

  <HemisphereLight
      skyColor={0x4c8eac}
      groundColor={0xac844c}
      intensity={0.6}
  />

  <Object3DInstance object = {new AxesHelper(100)} />

  <Object3DInstance  
  object={new GridHelper(100, 100, 0x444444, 0x555555)}
  />

  <!-- {#if cor}
  
  <Object3DInstance
  object = {configDataZero[7]}
  />
 {/if} -->
  {#if show}
    <Joint1></Joint1>
  {/if}

  {#if !show}
    <Joint2></Joint2>
  {/if}

  <Joint3></Joint3>

</Canvas>
</div>



<!-- TODO: display phi in degrees instead of radian -->
<div class="controls">

  {#if !cor}
  <h1> Please upload a vaild yaml file </h1>
  {/if}
  <!-- displaying the dual angles -->
  {#if cor}
    {#if !show}
    <input Phi1: 
      bind:value={configDataNeg[7]} >

    <input Phi2: 
    bind:value={configDataNeg[8]} >

    <input Phi3: 
      bind:value={configDataNeg[9]} >
    {/if}
    {#if show}

    <input Phi1: 
    bind:value={configDataPos[7]} >

    <input Phi2: 
    bind:value={configDataPos[8]} >

    <input Phi3: 
    bind:value={configDataPos[9]} >

    {/if}
  {/if}
  <!-- submit the files -->
  <label
    > <input type="submit" bind:value={submit} on:click={isSubmitted} /> </label
  >
  <!-- generate the Joints -->
  <label 
  ><input type="submit" on:click={solveInverse} /> </label
  >
  <!-- load data from yaml -->
  <input type='file' class="opacity-0" id="yamlFile" bind:files />
  <!-- show zero posture with joint configs = I -->
  <label
  > <input type="submit" bind:value={zero_ref} on:click={showZero} /> </label
>
<label
  > <input type="submit" bind:value={show} on:click={showForm} /> </label
>
</div>


<style>
  div {
    position: fixed;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
  }
  .controls {
    position: absolute;
    width: 100%;
    height: 10%
  }
  .canvas {
    width : 100%;
    height: 100%;
  }
</style>
  