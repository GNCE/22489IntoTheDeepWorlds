

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake{
    // Parts
    private Servo intakePivot;
    private TouchSensor touchSensor;
    public CRServo lintake;
    public CRServo rintake;
    public Servo leintake;
    public Servo reintake;

    public Servo door;
    NormalizedColorSensor colorSensor;

    // Constants
    private final double INTAKE_POWER = 1;

    // Values

    public NormalizedRGBA sensedColor;
    public float[] hsvValues = new float[3];
    OpMode opMode;


    double fin = 0;
    double intakePower = 0;


    /** LINKAGE EXTENSION VARIABLES */
    double extPos = 0;
    // Length of first linkage (Linkage that connects to servo) (mm)
    final double LINK1 = 200;
    // Length of second linkage (Linkage that connects to the slide) (mm)
    final double LINK2 = 300;
    // Offset X axis (CURRENT VALUE IS CORRECT)
    final double XOFFSET = 107;
    // Offset Y axis (CURRENT VALUE IS CORRECT)
    final double YOFFSET = 8.25;
    // Length of the slides when fully extended (mm)
    final double FULL_EXTENSION = 1000;
    // Default servo angle


    Telemetry telemetry;

    public Intake(HardwareMap hardwareMap, OpMode opMode) {
        rintake = hardwareMap.get(CRServo.class, "rintake");
        lintake = hardwareMap.get(CRServo.class, "lintake");
        intakePivot = hardwareMap.get(Servo.class, "fintake");
        rintake.setDirection(CRServo.Direction.FORWARD);
        lintake.setDirection(CRServo.Direction.REVERSE);
        intakePivot.setDirection(Servo.Direction.REVERSE);

        door = hardwareMap.get(Servo.class, "intake_door");
        door.setDirection(Servo.Direction.FORWARD);

        leintake = hardwareMap.get(Servo.class, "leintake");
        reintake = hardwareMap.get(Servo.class, "reintake");
        reintake.setDirection(Servo.Direction.FORWARD);
        leintake.setDirection(Servo.Direction.REVERSE);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch_intake");

        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    // Intake Init
    public void initiate(){
        intakeExtendTo(0);
        rintake.setPower(0);
        lintake.setPower(0);
        intakePivot.setPosition(0);
        door.setPosition(0);
    }

    boolean isIntakeDown = false;
    // Intake Loop
    public void intakeLoop(){
        sensedColor = colorSensor.getNormalizedColors();
        Color.colorToHSV(sensedColor.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Red", "%.3f", sensedColor.red)
                .addData("Green", "%.3f", sensedColor.green)
                .addData("Blue", "%.3f", sensedColor.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", sensedColor.alpha);
        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }
        telemetry.update();

        double INTAKE_TRANSFER_POS = 0.5;

        switch(intakeState){
            case FLIP_UP:
                isIntakeDown = false;
                fin = 0;
                break;
            case INTAKE:
                fin = INTAKE_DOWN_POS;
                intakePower = INTAKE_POWER;
                if(intakeTime.time() > 0.5){
                    isIntakeDown = true;
                    if(isCorrectColor()){
                        intakePower = 0;
                        setIntakeState(IntakeState.TRANSFER);
                    }
                }
                break;
            case DEPOSIT:
                intakePower = -1;
                if(!isCorrectColor()){
                    intakePower = 0;
                    setIntakeState(IntakeState.FLIP_UP);
                }
                break;
            case SHOOT:
                fin = INTAKE_DOWN_POS;
                if(intakeTime.time() > 0.5){
                    isIntakeDown = true;
                    intakePower = -1;
                    if(!isCorrectColor()){
                        intakePower = 0;
                        setIntakeState(IntakeState.TRANSFER);
                    }
                }
            case TRANSFER:
                fin = INTAKE_TRANSFER_POS;
                if(intakeTime.time() > 1){
                    isIntakeDown = false;
                }
            default:
                break;
        }
    }

    // Color Sensor
    private boolean isRed(){
        return hsvValues[0] <= 65;
    }

    private boolean isBlue(){
        return hsvValues[0] >= 200 && hsvValues[0] <= 250;
    }

    private boolean isYellow(){
        return hsvValues[0] >= 80 && hsvValues[0] <= 120;
    }

    public boolean isCorrectColor(){
        return hsvValues[1] > 0.5 && (isYellow() || (Storage.isRed && isRed()) || (!Storage.isRed && isBlue()));
    }

    // Intake Extension
    private double getServoAngleWithLength(double l1, double l2, double l3, double xo, double yo, int servoRange){
        // All units are mm and degrees.
        double beta = Math.toDegrees(Math.acos((Math.pow(l1, 2) - Math.pow(l2, 2) + Math.pow(xo + l3, 2) + Math.pow(yo, 2))/(2*l1*Math.sqrt(Math.pow(xo+l3, 2) + Math.pow(yo, 2)))));
        double gamma = Math.toDegrees(Math.atan((xo+l3)/yo));
        return (180 - beta - gamma)/servoRange;
    }
    private void intakeExtendTo(double length){
        double targetPos = getServoAngleWithLength(LINK1, LINK2, length, XOFFSET, YOFFSET, 360*5);
        leintake.setPosition(targetPos);
        reintake.setPosition(targetPos);
    }
    public void setIntakeExtensionTarget(double target){
        if(target > FULL_EXTENSION) target = FULL_EXTENSION;
        else if(isIntakeDown && target < 300) target = 300;
        else if(target < 0) target = 0;
        extPos = target;
    }
    public void extendoLoop(){
        if (intakePivot.getPosition()!=fin) {
            intakePivot.setPosition(fin);
        }
        if (rintake.getPower()!= intakePower){
            rintake.setPower(intakePower);
            lintake.setPower(intakePower);
        }
        intakeExtendTo(extPos);
    }
    public void ManualExtend(){
        setIntakeExtensionTarget(FULL_EXTENSION);
    }
    public void ManualRetract(){
        setIntakeExtensionTarget(0);
    }
    public void TeleopExtend(double valueFromZeroToOne){
        if(valueFromZeroToOne < 0) valueFromZeroToOne = 0;
        else if(valueFromZeroToOne > 1) valueFromZeroToOne = 1;
        setIntakeExtensionTarget(valueFromZeroToOne * FULL_EXTENSION);
    }
    public boolean isRetracted(){
        return touchSensor.isPressed();
    }

    // Intake States
    private final double INTAKE_DOWN_POS = 0.919;

    public enum IntakeState {
        FLIP_UP,
        INTAKE,
        DEPOSIT,
        SHOOT,
        TRANSFER,
    }

    IntakeState intakeState = IntakeState.FLIP_UP;
    ElapsedTime intakeTime;

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
        intakeTime.reset();
    }

    // Door States
    public enum DoorState{
        OPEN,
        CLOSE,
        REST,
    }

    static double DOOR_OPEN_POS = 0.5, DOOR_REST_POS = 0.3, DOOR_CLOSE_POS = 0.1;
    public void setDoorState(IntakeTest.DoorState doorState){
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

    public void flipUp(){
        intakeState = IntakeState.FLIP_UP;
    }
    public void deposit(){
        intakeState = IntakeState.DEPOSIT;
    }
    public void startIntake(){
        intakeState = IntakeState.INTAKE;
    }
    public void shootOut(){
        intakeState = IntakeState.SHOOT;
    }
    public void flipToTransfer(){
        intakeState = IntakeState.TRANSFER;
    }
}
