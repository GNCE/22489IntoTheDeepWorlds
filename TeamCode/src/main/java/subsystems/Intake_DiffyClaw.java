package subsystems;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import utils.Motor;

import utils.Storage;

@Config
public class Intake_DiffyClaw extends SubsysCore {
    private Servo IntakeClamp;
    private Servo IntakeRDiffy;
    private Servo IntakeLDiffy;
    private Servo RightArmPivot;
    private Servo LeftArmPivot;

    // Color Sensor
    private NormalizedColorSensor intakeCSensor;
    private static float COLOR_SENSOR_GAIN = 10;
    public static float[] hsvValues = new float[3];
    public static NormalizedRGBA colors;
    public enum SENSOR_READING {
        RED, BLUE, YELLOW, NOTHING, CORRECT, INCORRECT, OFF
    }
    SENSOR_READING currentSensorReading;
    static boolean useColorSensor = false;
    public static double distance;


    private IntakeLimelightSubsys ll;
    public Motor IntakeExtend;

    public static int pipelineNumber = 4;

    public static double DEFAULT_LDIFFY_POS = 0.505;
    public static double DEFAULT_RDIFFY_POS = 0.495;
    public static double LdiffyPos = DEFAULT_LDIFFY_POS;
    public static double RdiffyPos = DEFAULT_RDIFFY_POS;

    public static double ArmPosition = 0.285;
    public static double clawPos = 0.93;
    public static double CLAW_CLOSED = 0.64;
    public static double CLAW_LOOSE = 0.58;
    public static double CLAW_OPENED = 0.2;
    public static double CLAW_SPIKE_OPEN = 0.1;
    //tune these values vvvvv
    public static double ARM_REST = 0.08;
    public static double ARM_TRANSFER_POS = 0.4;
    public static double ARM_TRANSFER_WAIT_POS = 0.44;
    public static double ARM_RETRACTED_HOLD = 0.45;
    public static double ARM_PICKUP_READY = 0.52;
    public static double ARM_PICKUP_DOWN = 0.6;
    public static double ARM_DEPOSIT_BACK = 0.08;
    public static double ARM_HANG = 0.25;
    public static double ARM_VISION = 0.41;


    //EXTENSION CONTROLS
    private PIDController controller, visionPID, hangPID;
    public static double p = 0.015, i = 0, d = 0.0004;
    public static double vp = 0.023, vi = 0, vd = 0.00027;
    public static double hp = 0.03, hi=0, hd = 0.00027, hf = -0.0001;
    public int target = 0;
    private UnifiedTelemetry tel = new UnifiedTelemetry();
    ElapsedTime extensionTime;


    public enum IntakeState {
        TRANSFER,
        TRANSFER_WAIT,
        INTAKE_ARM_READY,
        INTAKE_ARM_PICKUP,
        INTAKE_RETRACT_HOLD,
        INTAKE_REST,
        DEPOSIT,
        VISION,
        HANG

    }
    public Intake_DiffyClaw() {
        IntakeClamp = hardwareMap.get(Servo.class, "intakeClamp");
        IntakeClamp.setDirection(Servo.Direction.FORWARD);
        RightArmPivot = hardwareMap.get(Servo.class, "rightArmPivot");
        LeftArmPivot = hardwareMap.get(Servo.class, "leftArmPivot");
        RightArmPivot.setDirection(Servo.Direction.FORWARD);
        LeftArmPivot.setDirection(Servo.Direction.REVERSE);
        IntakeRDiffy = hardwareMap.get(Servo.class,"IntakeRDiffy");
        IntakeLDiffy = hardwareMap.get(Servo.class,"IntakeLDiffy");
        IntakeRDiffy.setDirection(Servo.Direction.REVERSE);
        IntakeLDiffy.setDirection(Servo.Direction.FORWARD);

        intakeCSensor = hardwareMap.get(NormalizedColorSensor.class, "ISC");
        if (intakeCSensor instanceof SwitchableLight) {
            ((SwitchableLight)intakeCSensor).enableLight(true);
        }

        IntakeExtend = new Motor(hardwareMap.get(DcMotorEx.class, "horizExtend"), 0.005);
        IntakeExtend.setDirection(DcMotor.Direction.REVERSE);
        IntakeExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeExtend.stopAndResetEncoder();
        ll = new IntakeLimelightSubsys();

        extensionTime = new ElapsedTime();
        extensionTime.startTime();

        timer = new ElapsedTime();
        timer.startTime();

        controller = new PIDController(p, i, d);
        visionPID = new PIDController(vp, vi, vd);
        hangPID = new PIDController(hp, hi, hd);
        hanging = false;
        usingLL = false;
        dontReset = false;
    }

    private void updateColorSensorReading(){
        if (useColorSensor) {
            intakeCSensor.setGain(COLOR_SENSOR_GAIN);

            colors = intakeCSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            if (intakeCSensor instanceof DistanceSensor)
                distance = ((DistanceSensor) intakeCSensor).getDistance(DistanceUnit.MM);

            if (hsvValues[2] < 0.35 || hsvValues[1] < 0.4) currentSensorReading = SENSOR_READING.NOTHING;
            else if (hsvValues[0] <= 35) currentSensorReading = SENSOR_READING.RED;
            else if (hsvValues[0] >= 200 && hsvValues[0] <= 250)
                currentSensorReading = SENSOR_READING.BLUE;
            else if (hsvValues[0] >= 50 && hsvValues[0] <= 120)
                currentSensorReading = SENSOR_READING.YELLOW;
            else currentSensorReading = SENSOR_READING.NOTHING;
        } else {
            currentSensorReading = SENSOR_READING.OFF;
        }
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
            case OFF:
                correct = false;
            default:
                return SENSOR_READING.NOTHING;
        }
        return correct ? SENSOR_READING.CORRECT : SENSOR_READING.INCORRECT;
    }
    public void setUseColorSensor(boolean use){
        useColorSensor = use;
    }

    @Config
    public static class INTAKE_DIFFY_POSITIONS {
        public static double TRANSFER_POS = 65;
        public static double INTAKE_POS = -115;
        public static double INTAKE_FINAL_POS = -95;
        public static double REST_POS = 10;
        public static double DEPOSIT_POS = -10;
        public static double VISION_POS = -50;
        public static double ORIENTATION_UP = 0;
        public static double ORIENTATION_DOWN = 220;
        public static double ORIENTATION_ALIGNED = 0;
        public static double INTAKE_RETRACT_HOLD=-80;
        public static double HANG = 0;

    }

    private void setPivotPosition(double UpDownAngle, double Orientation){
        double ServoRange = 355;
        LdiffyPos = DEFAULT_LDIFFY_POS + UpDownAngle/ServoRange + Orientation*((double) 18/52)/ServoRange;
        RdiffyPos = DEFAULT_RDIFFY_POS + UpDownAngle/ServoRange - Orientation*((double) 18/52)/ServoRange;
    }

    private void updatePivotPosition(){
        if(IntakeLDiffy.getPosition() != LdiffyPos || IntakeRDiffy.getPosition() != RdiffyPos){
            IntakeLDiffy.setPosition(LdiffyPos);
            IntakeRDiffy.setPosition(RdiffyPos);
        }
    }

    public void changePipeline(int pipelineNum) {
        pipelineNumber = pipelineNum;
    }

    public IntakeState intakeState = IntakeState.TRANSFER;

    @Override
    public void init(){
        setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_REST);
        encoderUpdated = 0;
        usingLL = false;
        hanging = false;
        dontReset = false;
        useColorSensor = false;
        powerScale = 1;
        target=IntakeExtensionPositions.RETRACTED_POS;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop(){
        //extendTo(extPos);
        updateColorSensorReading();

        switch(intakeState){
            case TRANSFER:
                ArmPosition = ARM_TRANSFER_POS;
                setPivotPosition(INTAKE_DIFFY_POSITIONS.TRANSFER_POS, INTAKE_DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case TRANSFER_WAIT:
                ArmPosition = ARM_TRANSFER_WAIT_POS;
                setPivotPosition(INTAKE_DIFFY_POSITIONS.TRANSFER_POS, INTAKE_DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case INTAKE_ARM_READY:
                ArmPosition = ARM_PICKUP_READY;
                setPivotPosition(INTAKE_DIFFY_POSITIONS.INTAKE_POS, INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED);
                break;
            case INTAKE_ARM_PICKUP:
                ArmPosition = ARM_PICKUP_DOWN;
                setPivotPosition(INTAKE_DIFFY_POSITIONS.INTAKE_FINAL_POS, INTAKE_DIFFY_POSITIONS.ORIENTATION_ALIGNED);
                break;
            case INTAKE_REST:
                ArmPosition = ARM_REST;
                setPivotPosition(INTAKE_DIFFY_POSITIONS.REST_POS, INTAKE_DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case DEPOSIT:
                ArmPosition = ARM_DEPOSIT_BACK;
                setPivotPosition(INTAKE_DIFFY_POSITIONS.DEPOSIT_POS, INTAKE_DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case HANG:
                ArmPosition = ARM_HANG;
                setPivotPosition(INTAKE_DIFFY_POSITIONS.HANG, INTAKE_DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case VISION:
                ArmPosition = ARM_VISION;
                setPivotPosition(INTAKE_DIFFY_POSITIONS.VISION_POS, INTAKE_DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case INTAKE_RETRACT_HOLD:
                ArmPosition = ARM_RETRACTED_HOLD;
                setPivotPosition(INTAKE_DIFFY_POSITIONS.INTAKE_RETRACT_HOLD, INTAKE_DIFFY_POSITIONS.ORIENTATION_UP);
                break;
        }

        updatePivotPosition();

        if (ArmPosition != RightArmPivot.getPosition()){
            RightArmPivot.setPosition(ArmPosition);
            LeftArmPivot.setPosition(ArmPosition);
        }
        if (clawPos != IntakeClamp.getPosition()){
            IntakeClamp.setPosition(clawPos);
        }

        tel.addData("Horizontal Extension Target Position", target);
        tel.addData("Horizontal Extension Current Position", IntakeExtend.getPosition());
        tel.addData("Horizontal Extension Amps", IntakeExtend.getCurrent());
        tel.addData("Horizontal Extension Velocity", IntakeExtend.getVelocity());
        tel.addData("Horizontal Extension Motor Encoder Reset?", encoderReset);
        tel.addData("Horizontal Extension Encoder Updated", encoderUpdated);
        tel.addData("Detected Color", getCurrentSensorReading());
        tel.addData("Intake Hue", hsvValues[0]);
        tel.addData("Intake Saturation", hsvValues[1]);
        tel.addData("Intake Value", hsvValues[2]);
        tel.addData("Distance", distance);
    }

    public void setIntakeState(IntakeState intakeState){
        this.intakeState = intakeState;
    }
    public enum CLAW_STATE {
        OPEN,
        CLOSED,
        LOOSE,
        SPIKE
    }

    public void setClawState(CLAW_STATE state){
        switch (state){
            case OPEN:
                clawPos = CLAW_OPENED;
                break;
            case CLOSED:
                clawPos = CLAW_CLOSED;
                break;
            case LOOSE:
                clawPos = CLAW_LOOSE;
                break;
            case SPIKE:
                clawPos = CLAW_SPIKE_OPEN;
                break;
        }
    }



    //EXTENSION:
    private static int prevTarget = 0;
    private static boolean encoderReset = true;
    private static int encoderUpdated = 0;
    private int encoderResetVar = -1;

    ElapsedTime timer;

    public int getCurrentPosition(){
        return IntakeExtend.getPosition();
    }
    public int getTargetPosition(){
        return target;
    }
    // 11.5 is full

    private double powerScale = 1;
    public void setPowerScale(double newPowerScale){
        newPowerScale = Math.min(newPowerScale, 1);
        newPowerScale = Math.max(0, newPowerScale);
        powerScale = newPowerScale;
    }

    private boolean usingLL;

    public void useVision(){
        usingLL = true;
    }
    public void stopVision(){
        usingLL = false;
    }

    private boolean hanging = false;
    private boolean dontReset = false;

    public void useHang(){ hanging = true; }
    public void stopHang(){ hanging = false; }

    public void setDontReset(boolean dontReset) {
        this.dontReset = dontReset;
    }

    public void HoldExtension(){ //TODO: Call this in the main loop
        if(!dontReset && encoderResetVar >= 0 && encoderResetVar < 30){
            IntakeExtend.resetEncoder();
            encoderResetVar++;
            if(getCurrentPosition() <=1 && Math.abs(IntakeExtend.getVelocity()) == 0 && IntakeExtend.getCurrent() < 0.05){
                encoderResetVar = -1;
            }
        }

        double power;
        if (Math.abs(opMode.gamepad2.left_trigger) > 0.1){
            power = opMode.gamepad2.left_trigger;
            target = getCurrentPosition();
        } else if(Math.abs(opMode.gamepad2.right_trigger) > 0.1) {
            power = -opMode.gamepad2.right_trigger;
            target = getCurrentPosition();
        } else if(usingLL && ll.isResultValid()) {
            double error = 13 - ll.getTx();
            visionPID.setPID(vp, vi, vd);
            power = visionPID.calculate(error);
        } else if(hanging){
            hangPID.setPIDF(hp, hi, hd, hf);
            power = hangPID.calculate(getCurrentPosition(), target);
        } else if (target == IntakeExtensionPositions.RETRACTED_POS && (prevTarget != target || !encoderReset)) {
            // If target is zero and either the target was just set to zero or the encoder is not reset yet
            if (prevTarget != target) {
                encoderReset = false;
                timer.reset();
            }
            power = -1;
            if (IntakeExtend.getCurrent() > 5 && Math.abs(IntakeExtend.getVelocity()) <= 20  && timer.seconds() > 0.5) {
                encoderResetVar = 0;
                encoderReset = true;
            }
        } else {
            // PIDF Controller
            controller.setPID(p, i, d);
            double pid = controller.calculate(getCurrentPosition(), target);
            if(Math.abs(getCurrentPosition() - target) > (target == 0 ? 10 : 5)) power = pid;
            else power = 0;
        }

        if (getCurrentPosition() >= IntakeExtensionPositions.FULL_EXTENSION_POS-5 && power > 0) power = 0;

        power*=powerScale;
        IntakeExtend.setPower(power);
        prevTarget = target;
    }

    public enum IntakeExtensionStates {
        FULL_EXTENSION, RETRACTED, AUTO_POS;
    }
    public enum ExtensionUnits {
        inches, ticks;
    }

    @Config
    public static class IntakeExtensionPositions{
        public static int FULL_EXTENSION_POS = 380;

        public static int RETRACTED_POS = 0;
        public static int AUTO_POS = 225;
    }
    public static double inchesToTicks = 380/14.0;

    public void ExtendTo(double input, ExtensionUnits unit){
        switch (unit){
            case inches:
                input *= inchesToTicks;
            case ticks:
                break;
        }
        input = Math.min(input, IntakeExtensionPositions.FULL_EXTENSION_POS);
        input = Math.max(input, IntakeExtensionPositions.RETRACTED_POS);
        target = (int) Math.round(input);
    }

    public void ExtendTo(IntakeExtensionStates input){
        switch(input){
            case FULL_EXTENSION:
                target = IntakeExtensionPositions.FULL_EXTENSION_POS;
                break;
            case RETRACTED:
                target = IntakeExtensionPositions.RETRACTED_POS;
                break;
            case AUTO_POS:
                target = IntakeExtensionPositions.AUTO_POS;
            default:
                break;
        }
    }
    public boolean extensionReachedTarget(){
        return Math.abs(IntakeExtend.getPosition() - target) <= 20;
    }
}