<script>
	<!--import {
    Mesh,
    Object3DInstance,
  } from "@threlte/core";
    import {correct, dataNeg, dispZero} from './jscript/yamlUpload/stores.js' -->

  // subscribing to different variables that are changed due runtime
    let configData = [];
    let cor
    let zero_ref;

    dispZero.subscribe(value =>{
      zero_ref = value;
    });
    dataNeg.subscribe(value => {
      configData = value; 
    });

    correct.subscribe(value => {
      cor = value;
    });

</script>

<!-- Meshes of the First Joint
only gets displayed if a correct inverse solution can be found -->
{#if cor}
<!-- display the zero reference posture if button is clicked -->
  {#if !zero_ref}
            <!-- Joint1 -->
            <Object3DInstance
            object={configData[0]}
            />
    
            <!-- Joint2 -->
            <Object3DInstance
            object={configData[1]}
            />
          
            <!-- Joint3 -->
            <Object3DInstance
            object={configData[2]}
            />
            
            <!-- base link -->
            <Object3DInstance
            object={configData[3]}
            />
    
            <!-- link 1 -->
            <Object3DInstance
            object={configData[4]}
            />
    
            <!-- link 2 -->
            <Object3DInstance
            object={configData[5]}
            />
    
            <!-- link 3 -->
            <Object3DInstance
            object={configData[6]}
            />

          <!-- effector -->
          <Object3DInstance
          object={configData[10]}
          />
  {/if}
{/if}