import {Component, Type, Property} from '@wonderlandengine/api';

/**
 * snap-turn
 */
export class SnapTurn extends Component {
    static TypeName = 'snap-turn';
    /* Properties that are configurable in the editor */
    static Properties = {
        /** Handedness for VR cursors to accept input only from respective controller */
        handedness: {type: Type.Enum, values: ['left', 'right'], default: 'left'},
        /** Root of the player's camera */
        camRoot: {type: Type.Object},
        /** Snap turn angle */
        snapAngle: {type: Type.Int, default:30},
        /** Thumbstick deadzone */
        stickDeadzone: {type: Type.Float, default:0.0},
    };


    start() {
        /* Turning boolean */
        this.turning = false;
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

                this.snapTurn(xAxis, dt); // xAxis on the thumbstick is horizontal

            }
        }
    }

    snapTurn(xAxis, dt){
        if(xAxis < -this.stickDeadzone && this.turning === false){
            this.turning = true;
            this.camRoot.rotateAxisAngleDegObject([0,1,0], this.snapAngle);
            console.log("Turning left.")

        }

        else if(xAxis > this.stickDeadzone && this.turning === false){
            this.turning = true;
            this.camRoot.rotateAxisAngleDegObject([0,1,0], -this.snapAngle);
            console.log("Turning right.")
        }

        else if(xAxis === 0){
            this.turning = false;
        }
    }

}
