import {Component, Type, Property, Material, TextComponent} from '@wonderlandengine/api';
import {RobotTeleop} from './robot-teleop';
import {GoalPoseComponent} from './goal-pose';
import {TeleportComponent} from '../node_modules/@wonderlandengine/components/dist/teleport';
import { Switch } from './switch';
import {ButtonComponent} from './button';
import { vec3, quat } from 'gl-matrix';
import {RobocamGrabber} from './robocam-grabber';
import {Screen2} from './screen2';
import {SnapTurn} from './snap-turn';
/**
 * button-handler
 */
export class ButtonHandler extends Component {
    static TypeName = 'button-handler';
    /* Properties that are configurable in the editor */
    static Properties = {
        /** CursorLeft object */
        cursorLeftObject: Property.object(),
        /** CursorRight object */
        cursorRightObject: Property.object(),
        /** Root of the player's camera */
        camRoot: { type: Type.Object },
        /** Left eye object */
        eyeLeft: {type: Type.Object},
        /** Teleport button object */
        teleportSwitch: {type: Type.Object},
        /** Teleop button object */
        teleopSwitch: {type: Type.Object},
        /** Robot POV button object */
        botPOVButton: {type: Type.Object},
        /** Free Roam button object */
        freeRoamButton: {type: Type.Object},
        /** Robot Camera Display Object */
        robotCamDispObject: {type: Type.Object},
        /** Screen object */
        screenObject: {type: Type.Object},
        /** Robot Switch 1 object */
        robot1Switch: {type: Type.Object},
        /** Robot Switch 2 object */
        robot2Switch: {type: Type.Object},
        /** Robot Switch 3 object */
        robot3Switch: {type: Type.Object},
        /**Teleop Menu Object */
        teleopMenuObj: {type: Type.Object},
        /**Teleport Menu Object */
        teleportMenuObj: {type: Type.Object},
        /**Robot POV Menu Object */
        povMenuObj: {type: Type.Object},
        /**Free Roam Menu Object */
        roamMenuObj: {type: Type.Object},
        /** Grey Text Material */
        greyTextMaterial: {type: Type.Material},
        /** Default Text Material */
        defaultTextMaterial: {type: Type.Material},

    };

    start() {
        /* Left cursor components */
        // Goal pose component
        this.goalPoseComponent = this.cursorLeftObject.getComponent(GoalPoseComponent);
        // Teleop component
        this.teleopComponentLeft = this.cursorLeftObject.getComponent(RobotTeleop);

        /* Right cursor components */
        // Teleport component
        this.teleportComponent = this.cursorRightObject.getComponent(TeleportComponent);
        // Teleop component
        this.teleopComponentRight = this.cursorRightObject.getComponent(RobotTeleop);
        // Snap Turn component
        this.snapTurnComponent = this.cursorRightObject.getComponent(SnapTurn);

        /** Text Menu Components */
        this.teleopMenuText = this.teleopMenuObj.getComponent(TextComponent);
        this.teleportMenuText = this.teleportMenuObj.getComponent(TextComponent);
        this.povMenuText = this.povMenuObj.getComponent(TextComponent);
        this.roamMenuText = this.roamMenuObj.getComponent(TextComponent);

        /* Robot Camera Display components*/
        this.robocamGrabberComponent = this.robotCamDispObject.getComponent(RobocamGrabber);

        /* Screen Object components */
        this.screenComponent = this.screenObject.getComponent(Screen2);

        /* Free roam button  */
        this.freeRoamButtonToggle = this.freeRoamButton.getComponent(ButtonComponent);
        this.freeRoamSpawn = true;
        
        /* Empty vector used to calculate camera rotation*/
        this._tempVec = new Float32Array(3);
        /* Empty vector used to retrieve camera position*/
        this._tempVec0 = new Float32Array(3);

        /* Arrays for player position and rotation */
        this.lastPlayerPosition = new Float32Array(3);
        this.lastPlayerRotation = new Float32Array(4);

        this.teleportSwitchComponent = this.teleportSwitch.getComponent(Switch);
        
        this.teleopSwitchComponent = this.teleopSwitch.getComponent(Switch);        

        const button1Component = this.teleopSwitch.getComponent(Switch);
        button1Component.emitter.add((data)=>{
            this.menuBehavior(data['id'])
        });

    
        const button2Component = this.teleportSwitch.getComponent(Switch);
        button2Component.emitter.add((data)=>{
            this.menuBehavior(data['id'])
        });

        const button3Component = this.botPOVButton.getComponent(ButtonComponent);
        button3Component.emitter.add((data)=>{
            this.menuBehavior(data['id'])
        });

        const button4Component = this.freeRoamButton.getComponent(ButtonComponent);
        button4Component.emitter.add((data)=>{
            this.menuBehavior(data['id'])
        });

        this.isPOV = false; // Track if the user is already in Robot POV mode

        this.switch1ComponentToggle = this.robot1Switch.getComponent(Switch);
        this.switch2ComponentToggle = this.robot2Switch.getComponent(Switch);
        this.switch3ComponentToggle = this.robot3Switch.getComponent(Switch);
    }

    update(dt) {
    }

    _getCamRotation() {
        this.eyeLeft.getForwardWorld(this._tempVec);
        this._tempVec[1] = 0;
        vec3.normalize(this._tempVec, this._tempVec);
        return Math.atan2(this._tempVec[0], this._tempVec[2]);
    }

    _getPlayerPosition(){
        this.camRoot.getPositionWorld(this._tempVec0);
        return this._tempVec0;
    }

    _getPlayerRotation(){
        
        // Calculate the rotation in quaternion
        const rotationQuaternion = quat.create();
        quat.fromEuler(rotationQuaternion, 0, this._getCamRotation() * (180 / Math.PI), 0);
        return rotationQuaternion;
    }

    menuBehavior(buttonId){

        if(buttonId == 1){
            /* This button activates robot POV */
            if(this.isPOV === false){
                this.isPOV = true;

                // Reset robot switches
                this.switch1ComponentToggle.reset();
                this.switch2ComponentToggle.reset();
                this.switch3ComponentToggle.reset();

                // Store player's last position and orientation
                this.lastPlayerPosition = this._getPlayerPosition();
                this.lastPlayerRotation = this._getPlayerRotation();

                // Teleport player to skydome
                this.camRoot.setPositionWorld([-8,0,0]);
                this.camRoot.setRotationWorld([0, -0.707, 0, 0.707]); // Face the screen when teleported [0, -90, 0] in euler.

                // Deactivate teleop, teleport and goal pose, snap turn
                this.teleopComponentRight.active = false;
                this.teleopComponentLeft.active = false;
                this.teleportComponent.active = false;
                this.goalPoseComponent.active = false;
                this.snapTurnComponent.active = false;

                // Deactivate buttons for teleop & teleport + Reset switches state
                // (Prevents the user from escaping the sphere / Driving the robot blindly)
                this.teleportSwitchComponent.active = false;
                this.teleopSwitchComponent.active = false;
                this.teleportSwitchComponent.reset();
                this.teleopSwitchComponent.reset();

                // Change text colors and text for menu
                this.teleopMenuText.material = this.greyTextMaterial;
                this.teleportMenuText.material = this.greyTextMaterial;
                this.roamMenuText.material = this.defaultTextMaterial;
                this.povMenuText.text = "Refresh frame................................";

                // Activate free roam button
                this.freeRoamButtonToggle.active = true;
                this.freeRoamSpawn = false;
                console.log(this.freeRoamSpawn)

            }
            else if(this.isPOV === true){
                // Deactivate Screen for refresh
                //this.screenComponent.active = false;
                //Deactivate robot cam grabber for refresh
                this.robocamGrabberComponent.active = false;
            }

            // Activate Screen
            //this.screenComponent.active = true;

            // Activate robot cam grabber
            this.robocamGrabberComponent.active = true; 

            


        }

        if(buttonId == 2){
            /* This button activates free roam in the scene */
            console.log("Already in free roam");
            if(this.freeRoamSpawn === false){
                console.log("Going to free roam");
                this.isPOV = false;

                // Deactivate RobocamGrabber
                this.robocamGrabberComponent.active = false;

                // Deactivate Screen
                //this.screenComponent.active = false;
                
                // // Activate teleport and goal pose and snap turn
                // this.teleportComponent.active = true;
                // this.goalPoseComponent.active = true;
                // this.snapTurnComponent.active = true;

                // Activate buttons for teleop & teleport
                this.teleportSwitchComponent.active = true;
                this.teleopSwitchComponent.active = true;

                // Change text colors and text for menu
                this.teleopMenuText.material = this.defaultTextMaterial;
                this.teleportMenuText.material = this.defaultTextMaterial;
                this.roamMenuText.material = this.greyTextMaterial;
                this.povMenuText.text = "Visualize robot's POV......................";


                // Teleport player to last position in scene
                this.camRoot.setPositionWorld(this.lastPlayerPosition);
                this.camRoot.setRotationWorld(this.lastPlayerRotation);
                this.camRoot.rotateAxisAngleDegObject([0,1,0], 180);

                // Deactivate free roam button
                this.freeRoamButtonToggle.active = false;

                // Reset robot switches
                this.switch1ComponentToggle.reset();
                this.switch2ComponentToggle.reset();
                this.switch3ComponentToggle.reset();
            }
            
        }     
    }
}