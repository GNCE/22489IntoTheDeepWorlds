import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Deprecated
@Disabled
@TeleOp (name = "Intake Test")
@Config
public class IntakeTest extends OpMode {
    public static float COLOR_SENSOR_GAIN = 10;
    public static double DOOR_OPEN_POS = 0.1, DOOR_REST_POS = 0.3, DOOR_CLOSE_POS = 0.5;
    public static double intakePower = 0;
    private static double ip = 0;
    public static boolean amIRed = true;
    public static float[] hsvValues = new float[3];
    public static double distance;
    public static NormalizedRGBA colors;
    public static double flipPosition = 0.3;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet;
    CRServo rintake;
    CRServo lintake;
    Servo intakePivot;
    private Servo reintake;
    private Servo leintake;
    public static double extendoPos = 0;

    Servo door;
    NormalizedColorSensor colorSensor;

    @Override
    public void init(){
        packet = new TelemetryPacket();

        rintake = hardwareMap.get(CRServo.class, "rintake");
        lintake = hardwareMap.get(CRServo.class, "lintake");
        rintake.setDirection(CRServo.Direction.REVERSE);
        lintake.setDirection(CRServo.Direction.FORWARD);
        intakePivot = hardwareMap.get(Servo.class, "fintake");
        reintake = hardwareMap.get(Servo.class,"reintake");
        leintake = hardwareMap.get(Servo.class, "leintake");
        reintake.setDirection(Servo.Direction.REVERSE);

        door = hardwareMap.get(Servo.class, "intake_door");
        door.setDirection(Servo.Direction.FORWARD);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
    }
    private boolean isRed(){
        return hsvValues[0] <= 62;
    }

    private boolean isBlue(){
        return hsvValues[0] >= 200 && hsvValues[0] <= 250;
    }

    private boolean isYellow(){
        return hsvValues[0] >= 70 && hsvValues[0] <= 120;
    }

    public boolean isCorrectColor(){
        return distance < 5 && hsvValues[1] > 0.5 && (isYellow() || (Storage.isRed && isRed()) || (!Storage.isRed && isBlue()));
    }

    public boolean isWrongColor(){
        return distance < 5 && hsvValues[1] > 0.5 && (!Storage.isRed && isRed()) || (Storage.isRed && isBlue());
    }

    public enum DoorState{
        OPEN,
        CLOSE,
        REST,
    }

    public void setDoorState(DoorState doorState){
        switch (doorState){
            case OPEN:
                door.setPosition(DOOR_OPEN_POS);
                break;
            case REST:
                door.setPosition(DOOR_REST_POS);
                break;
            case CLOSE:
                door.setPosition(DOOR_CLOSE_POS);
                break;
        }
    }

    @Override
    public void loop(){
        Storage.isRed = amIRed;

        rintake.setPower(ip);
        lintake.setPower(ip);
        intakePivot.setPosition(flipPosition);
        reintake.setPosition(extendoPos);
        leintake.setPosition(extendoPos);


        colorSensor.setGain(COLOR_SENSOR_GAIN);

        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        if(colorSensor instanceof  DistanceSensor){
            distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        }

        if(isWrongColor()){
            ip=intakePower;
            setDoorState(DoorState.OPEN);
        } else if(isCorrectColor()){
            ip=0;
            setDoorState(DoorState.CLOSE);
        } else {
            ip = intakePower;
            setDoorState(DoorState.REST);
        }

        packet.put("TeamColorIsRed", Storage.isRed);
        packet.put("Red", colors.red);
        packet.put("Green", colors.green);
        packet.put("Blue", colors.blue);
        packet.put("Alpha", colors.alpha);
        packet.put("Hue", hsvValues[0]);
        packet.put("Saturation", hsvValues[1]);
        packet.put("Value", hsvValues[2]);
        if(colorSensor instanceof  DistanceSensor){
            packet.put("Distance", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }
        packet.put("ExtensionPosition", extendoPos);

        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);
        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }
        telemetry.update();
    }
}
