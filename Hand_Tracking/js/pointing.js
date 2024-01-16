import {Component, Type, Property} from '@wonderlandengine/api';
import { HandTracking } from '@wonderlandengine/components';

/**
 * pointing
 */
export class Pointing extends Component {
    static TypeName = 'pointing';
    /* Properties that are configurable in the editor */
    static Properties = {
        leftHand: {type: Type.Object},

    };


    start() {
        this.handTrackerL = this.leftHand.getComponent(HandTracking);
    }

    update(dt) {
        if(this.handTrackerL.isPointing()){
            console.log("Left hand is pointing.")
        }
    }
}
