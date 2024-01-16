import { Component, Property, Type, Texture } from '@wonderlandengine/api';
import {RobocamGrabber} from './robocam-grabber';


export class Screen2 extends Component {
    static TypeName = 'screen2';
    static Properties = {
        /** Screen material to which the texture/frame should be attached */
        material: Property.material(),
        /** Object to which the robot cam grabber is attached */
        robotCamGrabber: Property.object(),
        
    };

    // Define rgb8Array and canvasTexture as class properties
    //rgb8Array = null;
    //canvasTexture = null;

    onActivate() {
        console.log("Screen is active!");

        this.canvas = document.createElement("canvas");
        this.ctx = this.canvas.getContext("2d");
        let rgb8Array; // Declare rgb8Array here to avoid undeclared variable error

        //const socket = new WebSocket("ws://192.168.1.58:3000");
        //const socket = new WebSocket("ws://192.168.1.71:3000");
        const socket = new WebSocket("ws://192.168.165.185:3000");
        

        socket.addEventListener("open", (event) => {
            // console.log("WebSocket connection established.");
        });

        

        const robotCamGrabberComponent = this.robotCamGrabber.getComponent(RobocamGrabber);
        robotCamGrabberComponent.emitter.add((data)=>{
            if(data["command"] === "snapshot"){
                const messageTrigger = JSON.stringify({"identifier":data["namespace"]+"/camera_trigger", "command":"capture_ON"})
                socket.send(messageTrigger);
            }
        });

        const handleMessage = (event) => {
            if (typeof event.data === "object") {
                if (event.data instanceof Blob) {
                    const fileReader = new FileReader();
                    fileReader.onload = () => {
                        const arrayBuffer = fileReader.result;
                        rgb8Array = new Uint8Array(arrayBuffer);
                        const imageData = this._convertRGB8ToImageData(rgb8Array, 1280, 720);
                        this.canvas.width = imageData.width;
                        this.canvas.height = imageData.height;
                        this.ctx.putImageData(imageData, 0, 0);
                        console.log("Updating texture...");
                        this.canvasTexture = new Texture(this.engine, this.canvas);
                        this.material.flatTexture = this.canvasTexture;
                        this.canvasTexture.update();
                    };
                    fileReader.readAsArrayBuffer(event.data);
                }
                //socket.removeEventListener("message", handleMessage);
            }
        };

        socket.addEventListener("message", handleMessage);
    }

    onDeactivate() {
        console.log("Screen is inactive!");
    }

    start() {
        
    }

    update(dt) {
        // Called every frame.
        

    }

    _convertRGB8ToImageData(rgb8Array, imageWidth, imageHeight) {
        const pixelCount = rgb8Array.length / 3;
        const width = imageWidth;
        const height = imageHeight;
        const imageData = new ImageData(width, height);

        for (let i = 0; i < pixelCount; i++) {
            const r = rgb8Array[i * 3];
            const g = rgb8Array[i * 3 + 1];
            const b = rgb8Array[i * 3 + 2];
            const a = 255; // Alpha channel (fully opaque)
            const x = i % width;
            const y = Math.floor(i / width);
            const index = (y * width + x) * 4;

            imageData.data[index] = r;
            imageData.data[index + 1] = g;
            imageData.data[index + 2] = b;
            imageData.data[index + 3] = a;
        }

        return imageData;
    }
}
