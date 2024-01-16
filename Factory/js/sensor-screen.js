import {Component, Property, Type, TextComponent} from '@wonderlandengine/api';
import {SwitchHandler} from './switch-handler'

/**
 * sensor-screen
 */
export class SensorScreen extends Component {
    static TypeName = 'sensor-screen';
    /* Properties that are configurable in the editor */
    static Properties = {
        /** Robot label object for the sensor screen */
        robotValue: {type: Type.Object},
        /** Sensor label object for the sensor screen */
        sensorValue: {type: Type.Object},
        /** Value label object for the sensor screen */
        valueValue: {type: Type.Object},
        /** Warning label object for the sensor screen */
        warningObj: {type: Type.Object},
        /** Menu object for Switch handler component */
        menuObject: {type: Type.Object},
    };


    start() {
        this.sensors = ["Gas", "Temperature", "Humidity"];
        this.units = ["%", "°C", "%"]
        this.values = [100, 27, 80]
        this.robotValueText = this.robotValue.getComponent(TextComponent);
        this.sensorValueText = this.sensorValue.getComponent(TextComponent);
        this.valueValueText = this.valueValue.getComponent(TextComponent);
        this.warningText = this.warningObj.getComponent(TextComponent);
        this.warningText.text = "";

        const switchHandlerComponent = this.menuObject.getComponent(SwitchHandler);
        switchHandlerComponent.emitter.add((data)=>{
            this.updateLabels(data['id'])
        })
    }

    updateLabels(robotId){
        this.robotValueText.text = String(robotId);
        this.sensorValueText.text = this.sensors[robotId - 1];
        this.valueValueText.text = String(this.values[robotId - 1])+ this.units[robotId - 1];
        if(this.values[robotId - 1] > 50){
            this.warningText.text = "WARNING!";
        }
        else{
            this.warningText.text = "";
        }
    }
}
