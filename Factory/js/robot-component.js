import {Component, Property} from '@wonderlandengine/api';



/**
 * robot-component
 */
export class RobotComponent extends Component {
    static TypeName = 'robot-component';
    /* Properties that are configurable in the editor */
    static Properties = {
        namespace: Property.string(""),
    };

    static onRegister(engine) {
        /* Triggered when this component class is registered.
         * You can for instance register extra component types here
         * that your component may create. */
    }

    init() {
        console.log('init() with param', this.param);
    }

    start() {
        //this.socket = new WebSocket("ws://localhost:3000");
        //this.socket = new WebSocket("ws://192.168.1.71:3000");
        this.socket = new WebSocket("ws://192.168.165.185:3000");
        // Connection opened
        this.socket.addEventListener("open", (event) => {
            //socket.send("Hello Server!");
        });

        // Listen for messages
        this.socket.addEventListener("message", (event) => {
            //console.log("Message from server ", event.data);
            if(!(event.data instanceof Blob) && typeof event.data != "object"){
                var msgParsed = JSON.parse(event.data);
                if(msgParsed.identifier === 'robot_pose'){
                    for(var transform of msgParsed.transforms){
                        //console.log(transform['child_frame_id']);
                        if(transform['child_frame_id'] === this.namespace+'/base_footprint' && transform['header']['frame_id'] === this.namespace+'/odom'){
                            var xp = transform['transform']['translation']['x'];
                            var yp = transform['transform']['translation']['y'];
                            var zp = transform['transform']['translation']['z'];

                            var xr = transform['transform']['rotation']['x'];
                            var yr = transform['transform']['rotation']['y'];
                            var zr = transform['transform']['rotation']['z'];
                            var wr = transform['transform']['rotation']['w'];

                            //console.log (xr, yr, zr, wr)

                            this.object.setPositionWorld([xp, 0.125, -yp]); //Negating z coordinate to match Rviz frame
                            this.object.setRotationWorld([0, zr, 0, wr]);

                        }
                    } 
                }
            }
        });

    }
    
    
    update(dt) {
        /* Called every frame. */
    }
}
