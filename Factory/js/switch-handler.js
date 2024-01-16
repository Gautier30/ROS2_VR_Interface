import {Component, Type, Property} from '@wonderlandengine/api';
import { Switch } from './switch';
import {RobotTeleop} from './robot-teleop';
import {GoalPoseComponent} from './goal-pose';
import {GoalPoseFinger} from './goal_pose_finger';
import {TeleportComponent} from '../node_modules/@wonderlandengine/components/dist/teleport';
import {Emitter} from '@wonderlandengine/api';
import {SnapTurn} from './snap-turn'



/**
 * switch-handler
 */
export class SwitchHandler extends Component {
    static TypeName = 'switch-handler';
    /* Properties that are configurable in the editor */
    static Properties = {
        /** Robot1 switch object */
        robot1Switch: {type: Type.Object},
        /** Robot2 switch object */
        robot2Switch: {type: Type.Object},
        /** Robot3 switch object */
        robot3Switch: {type: Type.Object},
        /** Teleop switch object */
        teleopSwitch: {type: Type.Object},
        /** Teleport witch object */
        teleportSwitch: {type: Type.Object},
        /** cursor left object */
        cursorLeftObject: {type: Type.Object},
        /** Cursor right object */
        cursorRightObject: {type: Type.Object},
    };

    // Add an emitter class attribute for the class
    emitter = new Emitter();


    start() {
        
        this.switch1ComponentToggle = this.robot1Switch.getComponent(Switch);
        this.switch2ComponentToggle = this.robot2Switch.getComponent(Switch);
        this.switch3ComponentToggle = this.robot3Switch.getComponent(Switch);
        this.switchTeleopToggle = this.teleopSwitch.getComponent(Switch);
        this.switchTeleportToggle = this.teleportSwitch.getComponent(Switch);


        const switch1Component = this.robot1Switch.getComponent(Switch);
        switch1Component.emitter.add((data)=>{
            this.blockSwitches(data['id'])
        });

        const switch2Component = this.robot2Switch.getComponent(Switch);
        switch2Component.emitter.add((data)=>{
            this.blockSwitches(data['id'])
        });

        const switch3Component = this.robot3Switch.getComponent(Switch);
        switch3Component.emitter.add((data)=>{
            this.blockSwitches(data['id'])
        });

        const switchTeleop = this.teleopSwitch.getComponent(Switch);
        switchTeleop.emitter.add((data)=>{
            this.blockSwitches(data['id'])
        });

        const switchTeleport = this.teleportSwitch.getComponent(Switch);
        switchTeleport.emitter.add((data)=>{
            this.blockSwitches(data['id'])
        });

        /* Left cursor components */
        // Goal pose component
        this.goalPoseComponent = this.cursorLeftObject.getComponent(GoalPoseComponent);
        // this.goalPoseFingerComponent = this.cursorLeftObject.getComponent(GoalPoseFinger);
        // Teleop component
        this.teleopComponentLeft = this.cursorLeftObject.getComponent(RobotTeleop);

        /* Right cursor components */
        // Teleport component
        this.teleportComponent = this.cursorRightObject.getComponent(TeleportComponent);
        // Teleop component
        this.teleopComponentRight = this.cursorRightObject.getComponent(RobotTeleop);
        // Snap Turn component
        this.snapTurnComponent = this.cursorRightObject.getComponent(SnapTurn);
    }
    deactivateTeleopAndTeleport(){
        // Deactivate teleport and goal pose and snap turn
        this.teleportComponent.active = false;
        this.goalPoseComponent.active = false;
        this.snapTurnComponent.active = false;
        // Deactivate teleop
        this.teleopComponentRight.active = false;
        this.teleopComponentLeft.active = false;
        // this.goalPoseFingerComponent.active = false;

    }

    activateTeleop(){
        /* This switch activates robot teleoperation */
            
            // Activate teleop for left and right controllers
            this.teleopComponentLeft.active = true;
            this.teleopComponentRight.active = true;
            // Deactivate teleport and goal pose and snap turn
            this.teleportComponent.active = false;
            this.goalPoseComponent.active = false;
            this.snapTurnComponent.active = false;
            // this.goalPoseFingerComponent.active = false;
    }

    activateTeleport(){
        /* This switch activates teleportation */
            
            // Activate teleport and goal pose and snap turn
            this.teleportComponent.active = true;
            this.goalPoseComponent.active = true;
            this.snapTurnComponent.active = true;
            // this.goalPoseFingerComponent.active = true;
            // Deactivate teleop
            this.teleopComponentRight.active = false;
            this.teleopComponentLeft.active = false;
    }

    blockSwitches(switchId){
        //Reset robot switches
        if(switchId===999){ 
            this.switch1ComponentToggle.active = true;
            this.switch2ComponentToggle.active = true;
            this.switch3ComponentToggle.active = true;
        }
        //Reset mode switches
        if(switchId===888){
            this.switchTeleportToggle.active = true;
            this.switchTeleopToggle.active = true;
            this.deactivateTeleopAndTeleport();
        }
        //Block other switches
        else if(switchId === 1){
            this.emitter.notify({'id':switchId}); // Broadcast robot index
            // this.switch2ComponentToggle.active = false;
            // this.switch3ComponentToggle.active = false;
            this.switch2ComponentToggle.reset();
            this.switch3ComponentToggle.reset();
        }
        else if(switchId === 2){
            this.emitter.notify({'id':switchId});
            // this.switch1ComponentToggle.active = false;
            // this.switch3ComponentToggle.active = false;
            this.switch1ComponentToggle.reset();
            this.switch3ComponentToggle.reset();
        }
        else if(switchId === 3){
            this.emitter.notify({'id':switchId});
            // this.switch1ComponentToggle.active = false;
            // this.switch2ComponentToggle.active = false;
            this.switch1ComponentToggle.reset();
            this.switch2ComponentToggle.reset();
        }
        else if(switchId === 4){
            //this.switchTeleportToggle.active = false;
            this.switchTeleportToggle.reset();
            this.activateTeleop();
        }
        else if(switchId === 5){
            //this.switchTeleopToggle.active = false;
            this.switchTeleopToggle.reset();
            this.activateTeleport();
        }
    }

}
