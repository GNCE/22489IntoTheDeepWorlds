

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake{
    // Parts
    public static float COLOR_SENSOR_GAIN = 10;
    public static double DOOR_OPEN_POS = 0.1, DOOR_REST_POS = 0.3, DOOR_CLOSE_POS = 0.5;
    public static double intakePower = 0;
    private static double ip = 0;
    public static boolean amIRed = true;
    public static float[] hsvValues = new float[3];
    public static double distance;
    public static NormalizedRGBA colors;
    public static double flipPosition = 0.3;
    private OpMode opMode;
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


    static double INTAKE_DOWN_EXTENSION_LIMIT = 300;

    /** LINKAGE EXTENSION VARIABLES */
    double extPos = 280;
    // Length of first linkage (Linkage that connects to servo) (mm) (correct)
    final double LINK1 = 310;
    // Length of second linkage (Linkage that connects to the slide) (mm) (correct)
    final double LINK2 = 324;
    // Offset X axis (CURRENT VALUE IS CORRECT)
    final double XOFFSET = 107;
    // Offset Y axis (CURRENT VALUE IS CORRECT)
    final double YOFFSET = 8.25;
    // Length of the slides when fully extended (mm)
    final double FULL_EXTENSION = 0.3;
    // Default servo angle (correct)


    Telemetry telemetry;

    public Intake(HardwareMap hardwareMap, OpMode opMode) {
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
    private final double INTAKE_TRANSFER_POS = 0.8;
    private final double INTAKE_DOWN_POS = 0.13;

    // Intake Loop
    public void intakeLoop(){
        switch(intakeState){
            case FLIP_UP:
                isIntakeDown = false;
                flipPosition = 1;
                break;
            case INTAKE:
                if(extPos < INTAKE_DOWN_EXTENSION_LIMIT) setIntakeState(IntakeState.TRANSFER);
                flipPosition = INTAKE_DOWN_POS;
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
                flipPosition = INTAKE_DOWN_POS;
                if(intakeTime.time() > 0.5){
                    isIntakeDown = true;
                    intakePower = -1;
                    if(!isCorrectColor()){
                        intakePower = 0;
                        setIntakeState(IntakeState.TRANSFER);
                    }
                }
            case TRANSFER:
                flipPosition = INTAKE_TRANSFER_POS;
                if(intakeTime.time() > 1){
                    isIntakeDown = false;
                }
            default:
                break;
        }
    }

    // Color Sensor
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
        else if(isIntakeDown && target < INTAKE_DOWN_EXTENSION_LIMIT) target = INTAKE_DOWN_EXTENSION_LIMIT;
        else if(target < 0) target = 0;
        extPos = target;
    }
    public void extendoLoop(){
        if (intakePivot.getPosition()!= flipPosition) {
            intakePivot.setPosition(flipPosition);
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
        return false;
    }

    // Intake States


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


}
