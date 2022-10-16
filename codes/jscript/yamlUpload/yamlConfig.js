import {fileText, submitted} from './stores.js'
import yaml from 'js-yaml'

let data;
let submit;

fileText.subscribe(value => {
  data = value;
});


submitted.subscribe(value => {
  submit = value;
});

// generating Data from the yaml file
export function generate(){

  if(submit == true){
    let config = yaml.load(data)

    var terminal_ground  = config.joints[0].anchor
    var a_12 = config.joints[1].anchor
    var a_23 = config.joints[2].anchor
    var a_34 = config.joints[3].anchor
    var terminal_effector = config.joints[4].anchor

    var ground_axis = config.joints[0].direction
    var jointAxis_12 = config.joints[1].direction
    var jointAxis_23 = config.joints[2].direction
    var jointAxis_34 = config.joints[3].direction
    var effector_axis = config.joints[4].direction


    var zero_ref = config.configurations[0].position
    var target_pos = config.configurations[1].position
    var target_rot = config.configurations[1].angle[0]

    return [a_12,a_23,a_34,jointAxis_12,jointAxis_23,jointAxis_34,zero_ref,
      target_pos,target_rot, terminal_ground, ground_axis, terminal_effector, effector_axis]
  }
  else {
    alert("Please submit your data first")
  }
}
