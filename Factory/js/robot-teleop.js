import {Component, Type, Property} from '@wonderlandengine/api';
import { Switch } from './switch';


/**
 * robot-teleop
 */
export class RobotTeleop extends Component {
    static TypeName = 'robot-teleop';
    /* Properties that are configurable in the editor */
    static Properties = {
        
        /** Handedness for VR cursors to accept input only from respective controller */
        handedness: {type: Type.Enum, values: ['left', 'right'], default: 'left'},
        
        /** Teleop Type (Linear/Angular) */
        teleopType: {type: Type.Enum, values: ['linear', 'angular'], default: 'linear'},

        /** Cruise Linear velocity */
        linearVelocity: {type: Type.Float, default: 0.1},
        
        /** Cruise Angular velocity */
        angularVelocity: {type: Type.Float, default: 0.3},

        /** Robot1 switch object */
        robot1Switch: {type: Type.Object},
        /** Robot2 switch object */
        robot2Switch: {type: Type.Object},
        /** Robot3 switch object */
        robot3Switch: {type: Type.Object},
        /** Thumbstick deadzone */
        stickDeadzone: {type: Type.Float, default:0.0},
    };

    static onRegister(engine) {
        /* Triggered when this component class is registered.
         * You can for instance register extra component types here
         * that your component may create. */
    }

    init() {
        /** Deadzone for thumbstick lateral movements */
        this.lateralDeadZone = 0.8;
    }

    start() {
        //Stat
        /** Initiating the WebSocket address */
        //this.socket = new WebSocket("ws://localhost:3000");
        //this.socket = new WebSocket("ws://192.168.1.71:3000");
        this.socket = new WebSocket("ws://192.168.165.185:3000");

        this.socket.addEventListener("open", (event) => {
        });

        const switch1Component = this.robot1Switch.getComponent(Switch);
        switch1Component.emitter.add((data)=>{
            this.selectRobot(data['id'])
        });

        const switch2Component = this.robot2Switch.getComponent(Switch);
        switch2Component.emitter.add((data)=>{
            this.selectRobot(data['id'])
        });

        const switch3Component = this.robot3Switch.getComponent(Switch);
        switch3Component.emitter.add((data)=>{
            this.selectRobot(data['id'])
        });

        this.currentRobotNamespace = ""
        this.robotNamespaces = ['waffle1', 'waffle2', 'waffle3'] 
    }

    update(dt) {
        let s = this.engine.xrSession;
        if (!s) return;

        for (let i = 0; i < s.inputSources.length; i++){
            let input = s.inputSources[i];
            if (input.handedness == ['left', 'right'][this.handedness]){
                let gamepad = input.gamepad;
                if(!gamepad) continue;

                //Gather input from controller
                let xAxis =
                    this.controlSource == 'thumbstick' ? gamepad.axes[0] : gamepad.axes[2];
                let yAxis =
                    this.controlSource == 'thumbstick' ? gamepad.axes[1] : gamepad.axes[3];
                /*
                if(this.teleopType == 0){
                    this.moveLinear(yAxis, dt); // yAxis on the thumbstick is vertical
                } 
                
                else if (this.teleopType == 1){
                    this.moveAngular(xAxis, dt); // xAxis on the thumbstick is horizontal
                }
                */
                this.moveLinear(yAxis, dt); // yAxis on the thumbstick is vertical
                this.moveAngular(xAxis, dt); // xAxis on the thumbstick is horizontal
            }
        }
    }

    moveLinear(yAxis, dt){
        
        if(yAxis < -this.stickDeadzone && this.socket.readyState){
            var msg = {vel: this.linearVelocity, identifier : this.currentRobotNamespace + '/lin_vel'};
            msg = JSON.stringify(msg);
            this.socket.send(msg);
            
        }

        else if(yAxis > this.stickDeadzone && this.socket.readyState){
            var msg = {vel: -this.linearVelocity, identifier : this.currentRobotNamespace + '/lin_vel'};
            msg = JSON.stringify(msg);
            this.socket.send(msg);
        }

        else if(yAxis > -this.stickDeadzone && yAxis < this.stickDeadzone && this.socket.readyState){
            var msg = {vel: 0.0, identifier : this.currentRobotNamespace + '/lin_vel'};
            msg = JSON.stringify(msg);
            this.socket.send(msg);
        }
            
    }

    moveAngular(xAxis, dt){
        
        if(xAxis < -this.stickDeadzone && this.socket.readyState){
            var msg = {vel: this.angularVelocity, identifier : this.currentRobotNamespace + '/ang_vel'};
            msg = JSON.stringify(msg);
            this.socket.send(msg);
        }

        else if(xAxis > this.stickDeadzone && this.socket.readyState){
            var msg = {vel: -this.angularVelocity, identifier : this.currentRobotNamespace + '/ang_vel'};
            msg = JSON.stringify(msg);
            this.socket.send(msg);
        }

        else if(xAxis > -this.stickDeadzone && xAxis < this.stickDeadzone && this.socket.readyState){
            var msg = {vel: 0.0, identifier : this.currentRobotNamespace + '/ang_vel'};
            msg = JSON.stringify(msg);
            this.socket.send(msg);
        }
        
            
    }

    selectRobot(robotId){
        if(robotId <= 3){
            //console.log("Switched teleop robot to " + robotId)
            this.currentRobotNamespace = this.robotNamespaces[robotId-1];
        }
    }

}
