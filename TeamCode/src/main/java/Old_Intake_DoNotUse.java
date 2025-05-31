

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import utils.Storage;

@Deprecated
@Config
public class Old_Intake_DoNotUse {
    public static float COLOR_SENSOR_GAIN = 10;
    public static double DOOR_OPEN_POS = 0.1, DOOR_REST_POS = 0.4, DOOR_CLOSE_POS = 0.6;
    public static double intakePower = 0;
    public static float[] hsvValues = new float[3];
    public static double distance;
    public static NormalizedRGBA colors;
    private OpMode opMode;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet;
    CRServo lintake, rintake;
    public Servo intakePivot, reintake, leintake, door;
    NormalizedColorSensor colorSensor;

    ElapsedTime intakeTime, pivotTime, extensionTime;
    public ElapsedTime intakeDelay;
    public static double INTAKE_DOWN_EXTENSION_LIMIT = 200;
    public static double TRANSFER_EXTENSION_POS = 0;

    /** LINKAGE EXTENSION VARIABLES */
    public static double extPos = 0;
    final double LINK1 = Math.sqrt(97408); // Length of first linkage (Linkage that connects to servo) (mm) (correct)
    final double LINK2 = 320; // Length of second linkage (Linkage that connects to the slide) (mm) (correct)
    final double XOFFSET = 97; // Offset X axis (CURRENT VALUE IS CORRECT)
    final double YOFFSET = 8.25; // Offset Y axis (CURRENT VALUE IS CORRECT)
    public static double FULL_EXTENSION = 530; // Length of the slides when fully extended (mm)
    public static double EXTENSION_ZERO_OFFSET = -0.02; // Servo Zero Offset
    final int SERVO_RANGE = 300; // Servo Range in degrees

    Telemetry telemetry;

    public Old_Intake_DoNotUse(HardwareMap hardwareMap, OpMode opMode) {
        packet = new TelemetryPacket();

        rintake = hardwareMap.get(CRServo.class, "rintake");
        lintake = hardwareMap.get(CRServo.class, "lintake");
        rintake.setDirection(CRServo.Direction.FORWARD);
        lintake.setDirection(CRServo.Direction.REVERSE);
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

        intakeTime = new ElapsedTime();
        intakeTime.startTime();

        intakeDelay = new ElapsedTime();
        intakeDelay.startTime();

        pivotTime = new ElapsedTime();
        pivotTime.startTime();

        extensionTime = new ElapsedTime();
        extensionTime.startTime();

        this.opMode = opMode;
        this.telemetry = opMode.telemetry;
    }

    // Intake Init
    public void initiate(){
        extendTo(extPos);
        rintake.setPower(0);
        lintake.setPower(0);
        intakePivot.setPosition(0.8);
        door.setPosition(0);
    }

    private final double INTAKE_TRANSFER_POS = 0.8;
    private final double INTAKE_DOWN_POS = 0.12;

    // Intake Loop
    public static double INTAKE_POWER = 1.0;
    private double pivotWaitTime = 0, extensionWaitTime = 0;
    private boolean reverseIntake = false;
    public void intakeLoop(){
        // Color Sensor Control
        switch (intakeState){
            case INTAKE:
            case SHOOT:
                useColorSensor = true;
                break;
            default:
                useColorSensor = reverseIntake;
                break;
        }

        if(useColorSensor) updateColorSensorReading();

        // Intake Wheel, Door, and Intake State Control
        switch(intakeState){
            case INTAKE:
                if(extPos < INTAKE_DOWN_EXTENSION_LIMIT) extPos = INTAKE_DOWN_EXTENSION_LIMIT + 100;
                if (extPos < 80) intakeDelay.reset();
                intakePower = INTAKE_POWER;
                if(!isPivotBusy()){
                    switch (getCurrentSampleState(false)){
                        case INCORRECT:
                            intakePower = INTAKE_POWER;
                            setDoorState(DoorState.OPEN);
                            break;
                        case CORRECT:
                            intakePower = 0;
                            setDoorState(DoorState.CLOSE);
                            setIntakeState(IntakeState.TRANSFER);
                            break;
                        case NOTHING:
                            intakePower = INTAKE_POWER;
                            setDoorState(DoorState.REST);
                            break;
                    }
                }
                break;
            case FLIP_UP:
            case TRANSFER:
                intakePower = 0;
                break;
            case SHOOT:
                if(extPos < INTAKE_DOWN_EXTENSION_LIMIT) extPos = INTAKE_DOWN_EXTENSION_LIMIT;
                startReverseIntake();
                if(!reverseIntake) setIntakeState(IntakeState.TRANSFER);
                break;
            default:
                break;
        }

        if(reverseIntake){
            if(!isPivotBusy()){
                intakePower = -1;
                if(getCurrentSampleState(false) == Old_Intake_DoNotUse.SENSOR_READING.NOTHING){
                    intakePower = 0;
                    reverseIntake = false;
                }
            }
        }

        // Intake Pivot Control
        switch (intakeState){
            case FLIP_UP:
                updateIntakePivot(1);
                break;
            case SHOOT:
            case INTAKE:
                if (!(extPos < INTAKE_DOWN_EXTENSION_LIMIT) && intakeDelay.time() > 0.39){
                updateIntakePivot(INTAKE_DOWN_POS);}
                break;
            case TRANSFER:
                updateIntakePivot(INTAKE_TRANSFER_POS);
                break;
        }

        // Intake Wheels
        if (rintake.getPower()!= intakePower){
            rintake.setPower(intakePower);
            lintake.setPower(intakePower);
        }

        // Intake Extension
        extendTo(extPos);
    }

    // Intake Reverse
    public void startReverseIntake(){
        reverseIntake = true;
    }

    // Intake Pivot
    private void updateIntakePivot(double targetPosition){
        if (intakePivot.getPosition()!= targetPosition) {
            pivotTime.reset();
            pivotWaitTime = Math.abs(intakePivot.getPosition() - targetPosition);
            intakePivot.setPosition(targetPosition);
        }
    }

    public boolean isPivotBusy(){
        return pivotTime.time() <= pivotWaitTime;
    }

    public boolean isIntakeDown(){
        return intakePivot.getPosition() == INTAKE_DOWN_POS;
    }
    public boolean isIntakeTransfer(){
        return intakePivot.getPosition() == INTAKE_TRANSFER_POS;
    }

    // Color Sensor
    public enum SENSOR_READING {
        RED, BLUE, YELLOW, NOTHING, CORRECT, INCORRECT
    }
    SENSOR_READING currentSensorReading;
    static boolean useColorSensor = false;
    private void updateColorSensorReading(){
        colorSensor.setGain(COLOR_SENSOR_GAIN);

        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        if(colorSensor instanceof DistanceSensor)
            distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        if(distance > 5 || hsvValues[1] < 0.5) currentSensorReading = SENSOR_READING.NOTHING;
        else if(hsvValues[0] <= 62) currentSensorReading = SENSOR_READING.RED;
        else if(hsvValues[0] >= 200 && hsvValues[0] <= 250) currentSensorReading = SENSOR_READING.BLUE;
        else if(hsvValues[0] >= 70 && hsvValues[0] <= 120) currentSensorReading = SENSOR_READING.YELLOW;
        else currentSensorReading = SENSOR_READING.NOTHING;
    }

    public SENSOR_READING getCurrentSensorReading(){
        return currentSensorReading;
    }

    public SENSOR_READING getCurrentSampleState(boolean allianceSpecific){
        SENSOR_READING cur = getCurrentSensorReading();
        boolean correct;
        switch (cur){
            case YELLOW:
                correct = !allianceSpecific;
                break;
            case RED:
                correct = Storage.isRed;
                break;
            case BLUE:
                correct = !Storage.isRed;
                break;
            default:
                return SENSOR_READING.NOTHING;
        }
        return correct ? SENSOR_READING.CORRECT : SENSOR_READING.INCORRECT;
    }

    // Intake Door States
    public enum DoorState{
        OPEN, CLOSE, REST
    }

    public void setDoorState(Old_Intake_DoNotUse.DoorState doorState){
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
        double beta = Math.toDegrees(Math.acos((Math.pow(l1, 2) - Math.pow(l2, 2) + Math.pow(xo + l3, 2) + Math.pow(yo, 2))/(2.0*l1*Math.sqrt(Math.pow(xo+l3, 2) + Math.pow(yo, 2)))));
        double gamma = Math.toDegrees(Math.atan((xo+l3)/yo));
        return (180.0 - beta - gamma)/servoRange;
    }
    private void extendTo(double length){
        double targetPos = EXTENSION_ZERO_OFFSET + getServoAngleWithLength(LINK1, LINK2, length, XOFFSET, YOFFSET, SERVO_RANGE);
        if(leintake.getPosition() != targetPos){
            extensionWaitTime = Math.abs(targetPos - leintake.getPosition());
            extensionTime.reset();

            leintake.setPosition(targetPos);
            reintake.setPosition(targetPos);
        }
    }
    public boolean isExtensionBusy(){
        return extensionTime.time() <= extensionWaitTime;
    }
    public void setExtensionTarget(double target){
        if(target > FULL_EXTENSION) target = FULL_EXTENSION;
        else if(isIntakeDown() && target < INTAKE_DOWN_EXTENSION_LIMIT) target = INTAKE_DOWN_EXTENSION_LIMIT;
        else if(isIntakeTransfer() && target < TRANSFER_EXTENSION_POS) target = TRANSFER_EXTENSION_POS;
        else if(target < 0) target = 0;
        extPos = target;
    }
    public void TeleopExtend(double valueFromZeroToOne){
        if(valueFromZeroToOne < 0) valueFromZeroToOne = 0;
        else if(valueFromZeroToOne > 1) valueFromZeroToOne = 1;
        setExtensionTarget(valueFromZeroToOne * FULL_EXTENSION);
    }

    // Intake States
    public enum IntakeState {
        FLIP_UP, INTAKE, SHOOT, TRANSFER
    }
    IntakeState intakeState = IntakeState.TRANSFER;
    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
        intakeTime.reset();
    }
}
