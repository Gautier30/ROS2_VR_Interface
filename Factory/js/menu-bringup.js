import {Component, Property, Type} from '@wonderlandengine/api';
import { vec3, quat } from 'gl-matrix';

/**
 * menu-bringup
 */
export class MenuBringup extends Component {
    static TypeName = 'menu-bringup';
    /* Properties that are configurable in the editor */
    static Properties = {
        /* Menu Object */
        menuObject: { type: Type.Object },
        /* Camera root object */
        camRootObj: { type: Type.Object},
        /* Handedness for VR cursors to accept input only from respective controller */
        handedness: {type: Type.Enum, values: ['left', 'right'], default: 'left'},
        /* Left eye object */
        eyeLeft: {type: Type.Object},
        /* Right eye object */
        eyeRight: {type: Type.Object},
        /* Distance from player */
        distancePlayer: {type: Type.Float, default:1},
    };

    init() {
        console.log('init() with param', this.param);
    }

    start() {
        console.log('start() with param', this.param);
        // Close menu by default at startup
        this.menuObject.scaleLocal([0,0,0]);
        // Flag for menu already open
        this.isOpen = false;
        // Position array of Player
        this.playerPos = [0, 0, 0];
        // Position array of Menu
        this.menuPos = [0, 0, 0];
        // Vector used to calculate head rotation
        this._tempVec = new Float32Array(3);
        this._tempVec0 = new Float32Array(3);
    }

    _getCamRotation() {
        this.eyeLeft.getForwardWorld(this._tempVec);
        this._tempVec[1] = 0;
        vec3.normalize(this._tempVec, this._tempVec);
        return Math.atan2(this._tempVec[0], this._tempVec[2]);
    }

    _getPlayerPosition(){
        this.camRootObj.getPositionWorld(this.playerPos);
        return(this.playerPos);
    }

    _setMenuPose(){
        
        const offsetX = Math.sin(this._getCamRotation()) * this.distancePlayer;
        const offsetY = 1;
        const offsetZ = Math.cos(this._getCamRotation()) * this.distancePlayer;

        const menuPose = [
            this._getPlayerPosition()[0] + offsetX,
            this._getPlayerPosition()[1] + offsetY,
            this._getPlayerPosition()[2] + offsetZ,
        ];

        this.menuObject.setPositionWorld(menuPose);

        // Calculate the rotation in quaternion
        const rotationQuaternion = quat.create();
        quat.fromEuler(rotationQuaternion, 0, this._getCamRotation() * (180 / Math.PI), 0);
        this.menuObject.setRotationWorld(rotationQuaternion);
        this.menuObject.rotateAxisAngleDegObject([0,1,0], 180);

    }

    update(dt) {
        /* Called every frame. */
        let s = this.engine.xrSession;
        if (!s) return;

        //Handle input
        for (let i = 0; i < s.inputSources.length; ++i) {
            let input = s.inputSources[i];
            if (input.handedness == ['left', 'right'][this.handedness]) {
                let gamepad = input.gamepad;
                if (!gamepad) continue;

                //Gather input from controller
                let buttonPressed = gamepad.buttons[4].pressed;

                //Handle button presses
                if (buttonPressed) {

                    if(this.isOpen == false){
                        /* Open the menu */
                        // setTimeout of 200ms to debounce the button
                        setTimeout(()=>{

                            // Place the menu in front of the player
                            this._setMenuPose();
                            // Reset scaling to identity
                            this.menuObject.resetScaling();
                            // Scale down to desired size (editor size)
                            this.menuObject.scaleLocal([0.3,0.3,0.3]);
                            this.isOpen = true;
                        },200);
                        
                    }
                    else if(this.isOpen == true){
                        // Close the menu
                        
                        setTimeout(()=> {
                            this.menuObject.scaleLocal([0,0,0]);
                            this.isOpen = false;
                         },200);

                    }
                    
                }
            }
        }
    }

}
