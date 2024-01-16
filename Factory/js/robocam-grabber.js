import {Component, Type} from '@wonderlandengine/api';
import {Emitter} from '@wonderlandengine/api';
import { Switch } from './switch';


/**
 * robocam-grabber
 */
export class RobocamGrabber extends Component {
    static TypeName = 'robocam-grabber';
    /* Properties that are configurable in the editor */
    static Properties = {
        /** Robot1 switch object */
        robot1Switch: {type: Type.Object},
        /** Robot2 switch object */
        robot2Switch: {type: Type.Object},
        /** Robot3 switch object */
        robot3Switch: {type: Type.Object},
    };

    // Add an emitter class attribute for the class
    emitter = new Emitter();

    init() {
        
    }

    onActivate(){
        console.log("Activating the cam grabber...");
    }

    onDeactivate(){
        console.log("Deactivating the cam grabber...");
    }

    start() {
        const switch1Component = this.robot1Switch.getComponent(Switch);
        switch1Component.emitter.add((data)=>{
            this.selectRobot(data['id']);
            
        });

        const switch2Component = this.robot2Switch.getComponent(Switch);
        switch2Component.emitter.add((data)=>{
            this.selectRobot(data['id']);
        });

        const switch3Component = this.robot3Switch.getComponent(Switch);
        switch3Component.emitter.add((data)=>{
            this.selectRobot(data['id'])
        });

        this.currentRobotNamespace = ""
        this.robotNamespaces = ['waffle1', 'waffle2', 'waffle3']


    }


    update(dt) {
        
    }
    takeSnapshot(){
        //console.log("Taking snapshot from " + this.currentRobotNamespace );
        this.emitter.notify({command:"snapshot", namespace:this.currentRobotNamespace}); // Emit snapshot request for the screen component.
    }

    selectRobot(robotId){
        if(robotId <= 3){
            //console.log("Switched cam robot to " + robotId)
            this.currentRobotNamespace = this.robotNamespaces[robotId-1];
            this.takeSnapshot();
        }
    }

    
}
