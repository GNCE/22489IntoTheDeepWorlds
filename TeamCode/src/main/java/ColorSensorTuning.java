


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;



@TeleOp(name = "Tuning Color Sensor", group = "Tuning")
public class ColorSensorTuning extends OpMode {
    ColorSensor colorSensor;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
    }

    @Override
    public void loop() {
        telemetry.addData("Red",colorSensor.red());
        telemetry.addData("Blue",colorSensor.blue());
        telemetry.addData("Green (for Yellow)",colorSensor.green());
        telemetry.update();
    }

}