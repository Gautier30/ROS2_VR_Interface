import {Component, InputComponent, Type, MeshComponent, Property} from '@wonderlandengine/api';
import {Emitter} from '@wonderlandengine/api';
import {CursorTarget, HowlerAudioSource} from '@wonderlandengine/components';


/**
 * Helper function to trigger haptic feedback pulse.
 *
 * @param {Object} object An object with 'input' component attached
 * @param {number} strength Strength from 0.0 - 1.0
 * @param {number} duration Duration in milliseconds
 */
export function hapticFeedback(object, strength, duration) {
    const input = object.getComponent(InputComponent);
    if (input && input.xrInputSource) {
        const gamepad = input.xrInputSource.gamepad;
        if (gamepad && gamepad.hapticActuators)
            gamepad.hapticActuators[0].pulse(strength, duration);
    }
}

/**
 * switch
 */
export class Switch extends Component {
    static TypeName = 'switch';
    /* Properties that are configurable in the editor */
    static Properties = {
        /** Object that has the button's mesh attached */
        switchMeshObject: Property.object(),
        /** Material to apply when the user hovers the button */
        hoverMaterial: Property.material(),
        /** Button ID */
        switchId: {type: Type.Int},
        /** Switch type */
        switchType: {type: Type.Enum, values:['robot', 'mode'], default: 'robot'}
    };

    /* Declare an instance of emitter as attribute of Switch */
    emitter = new Emitter();

    static onRegister(engine) {
        /* Triggered when this component class is registered.
         * You can for instance register extra component types here
         * that your component may create. */
    }


    start() {
        this.mesh = this.switchMeshObject.getComponent(MeshComponent);
        this.defaultMaterial = this.mesh.material;
        this.switchMeshObject.getTranslationLocal(this.returnPos);
        this.toggled = false;

        this.target =
            this.object.getComponent(CursorTarget) ||
            this.object.addComponent(CursorTarget);

        this.soundClick = this.object.addComponent(HowlerAudioSource, {
            src: 'sfx/click.wav',
            spatial: true,
        });
        this.soundUnClick = this.object.addComponent(HowlerAudioSource, {
            src: 'sfx/unclick.wav',
            spatial: true,
        });
    }

    onActivate(){
        this.target.onHover.add(this.onHover);
        this.target.onUnhover.add(this.onUnhover);
        this.target.onDown.add(this.onDown);
        //this.target.onUp.add(this.onUp);
    }

    onDeactivate(){
        this.target.onHover.remove(this.onHover);
        this.target.onUnhover.remove(this.onUnhover);
        this.target.onDown.remove(this.onDown);
        //this.target.onUp.remove(this.onUp);
    }

    /* Called by 'cursor-target' */
    onHover = (_, cursor) => {
        this.mesh.material = this.hoverMaterial;
        if (cursor.type === 'finger-cursor') {
            this.onDown(_, cursor);
        }

        hapticFeedback(cursor.object, 0.5, 50);
    }

    /* Called by 'cursor-target' */
    onDown = (_, cursor) => {
        if(this.toggled === false){
            this.soundClick.play();
            this.mesh.material = this.hoverMaterial;
            hapticFeedback(cursor.object, 1.0, 20);
            this.emitter.notify({'id':this.switchId});
            this.toggled = true;
        }
        /*
        else{
            console.log(this.switchType)
            this.soundUnClick.play();
            this.mesh.material = this.defaultMaterial;
            hapticFeedback(cursor.object, 1.0, 20);
            if(this.switchType === 0){
                this.emitter.notify({'id':999});
                console.log("reset")
            }
            else if(this.switchType === 1){
                this.emitter.notify({'id':888});
            }
            this.toggled = false;
        }
        */
    }

    /* Called by 'cursor-target' */
    onUnhover = (_, cursor) => {
        if(this.toggled===false){
            this.mesh.material = this.defaultMaterial;
        }
        if (cursor.type === 'finger-cursor') {
            this.onUp(_, cursor);
        }

        hapticFeedback(cursor.object, 0.3, 50);
    }

    reset(){
        this.mesh.material = this.defaultMaterial;
        this.toggled = false;
    }
}
