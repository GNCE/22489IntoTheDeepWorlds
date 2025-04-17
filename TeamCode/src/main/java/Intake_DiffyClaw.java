
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import subsystems.IntakeLimelightSubsys;
import subsystems.SubsysCore;
import subsystems.UnifiedTelemetry;

@Config
public class Intake_DiffyClaw extends SubsysCore {
    private Servo IntakeClamp;
    private Servo IntakeRDiffy;
    private Servo IntakeLDiffy;
    private Servo RightArmPivot;
    private Servo LeftArmPivot;
    private IntakeLimelightSubsys ll;
    public DcMotorEx IntakeExtend;

    public static int pipelineNumber = 4;

    public static double DEFAULT_LDIFFY_POS = 0.440;
    public static double DEFAULT_RDIFFY_POS = 0.540;
    public static double LdiffyPos = DEFAULT_LDIFFY_POS;
    public static double RdiffyPos = DEFAULT_RDIFFY_POS;

    public static double ArmPosition = 0.285;
    public static double clawPos = 0.93;
    public static double CLAW_CLOSED = 0.35;
    public static double CLAW_LOOSE = 0.42;
    public static double CLAW_OPENED = 0.93;
    //tune these values vvvvv
    public static double ARM_REST = 0.05;
    public static double ARM_TRANSFER_POS = 0.43;
    public static double ARM_TRANSFER_WAIT_POS = 0.43;
    public static double ARM_PICKUP_READY = 0.52;
    public static double ARM_PICKUP_DOWN = 0.6;
    public static double ARM_DEPOSIT_BACK = 0.05;
    public static double ARM_HANG = 0.25;


    //EXTENSION CONTROLS
    private PIDController controller, visionPID, hangPID;
    public static double p = 0.02, i = 0, d = 0.00048;
    public static double vp = 0.03, vi = 0, vd = 0.00027;
    public static double hp = 0.03, hi=0, hd = 0.00027, hf = -0.0001;
    public int target = 0;
    private UnifiedTelemetry tel = new UnifiedTelemetry();
    ElapsedTime extensionTime;


    public enum IntakeState {
        TRANSFER,
        TRANSFER_WAIT,
        INTAKE_ARM_READY,
        INTAKE_ARM_PICKUP,
        INTAKE_REST,
        DEPOSIT,
        HANG

    }
    public Intake_DiffyClaw() {
        IntakeClamp = hardwareMap.get(Servo.class, "intakeClamp");
        IntakeClamp.setDirection(Servo.Direction.REVERSE);
        RightArmPivot = hardwareMap.get(Servo.class, "rightArmPivot");
        LeftArmPivot = hardwareMap.get(Servo.class, "leftArmPivot");
        RightArmPivot.setDirection(Servo.Direction.FORWARD);
        LeftArmPivot.setDirection(Servo.Direction.REVERSE);
        IntakeRDiffy = hardwareMap.get(Servo.class,"IntakeRDiffy");
        IntakeLDiffy = hardwareMap.get(Servo.class,"IntakeLDiffy");
        IntakeRDiffy.setDirection(Servo.Direction.FORWARD);
        IntakeLDiffy.setDirection(Servo.Direction.REVERSE);

        IntakeExtend = hardwareMap.get(DcMotorEx.class, "horizExtend");
        IntakeExtend.setDirection(DcMotor.Direction.REVERSE);
        IntakeExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        IntakeExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    }
    @Config
    public static class INTAKE_DIFFY_POSITIONS {
        public static double TRANSFER_POS = 90;
        public static double INTAKE_POS = -115;
        public static double INTAKE_FINAL_POS = -80;
        public static double REST_POS = -40;
        public static double DEPOSIT_POS = -50;
        public static double ORIENTATION_UP = 0;
        public static double ORIENTATION_DOWN = 220;
        public static double ORIENTATION_ALIGNED = 0;
        public static double HANG = 0;

    }

    private void setPivotPosition(double UpDownAngle, double Orientation){
        double ServoRange = 360*5;
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

    IntakeState intakeState = IntakeState.TRANSFER;

    @Override
    public void init(){
        setIntakeState(Intake_DiffyClaw.IntakeState.INTAKE_REST);
        encoderUpdated = 0;
        usingLL = false;
        hanging = false;
        powerScale = 1;
        target=IntakeExtensionPositions.RETRACTED_POS;
    }

    @Override
    public void loop(){
        //extendTo(extPos);
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
        tel.addData("Horizontal Extension Current Position", IntakeExtend.getCurrentPosition());
        tel.addData("Horizontal Extension Amps", IntakeExtend.getCurrent(CurrentUnit.AMPS));
        tel.addData("Horizontal Extension Velocity", IntakeExtend.getVelocity());
        tel.addData("Horizontal Extension Motor Encoder Reset?", encoderReset);
        tel.addData("Horizontal Extension Encoder Updated", encoderUpdated);
    }

    public void setIntakeState(IntakeState intakeState){
        this.intakeState = intakeState;
    }
    enum CLAW_STATE {
        OPEN,
        CLOSED,
        LOOSE
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
        }
    }



    //EXTENSION:
    private static int prevTarget = 0;
    private static boolean encoderReset = true;
    private static int encoderUpdated = 0;

    ElapsedTime timer;

    public int getCurrentPosition(){
        return IntakeExtend.getCurrentPosition();
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

    private boolean usingLL = false;

    public void useVision(){
        usingLL = true;
    }
    public void stopVision(){
        usingLL = false;
    }

    private boolean hanging = false;
    public void useHang(){ hanging = true; }
    public void stopHang(){ hanging = false; }

    public void HoldExtension(){ //TODO: Call this in the main loop
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
            if (IntakeExtend.getCurrent(CurrentUnit.AMPS) > 3 && IntakeExtend.getVelocity() == 0 && timer.seconds() > 0.5) {
                IntakeExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                IntakeExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                encoderReset = true;
                encoderUpdated++;
            }
        } else {
            // PIDF Controller
            controller.setPID(p, i, d);
            double pid = controller.calculate(getCurrentPosition(), target);
            if(Math.abs(getCurrentPosition() - target) > 5) power = pid;
            else power = 0;
        }

        if (getCurrentPosition() >= IntakeExtensionPositions.FULL_EXTENSION_POS-5 && power > 0) power = 0;

        power*=powerScale;
        power = Math.max(power, -1);
        power = Math.min(power, 1);
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
        public static int FULL_EXTENSION_POS = 335;

        public static int RETRACTED_POS = 0;
        public static int AUTO_POS = 225;
    }
    public void ExtendTo(double input, ExtensionUnits unit){
        switch (unit){
            case inches:
                input *= 330/11.5;
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
        return Math.abs(IntakeExtend.getCurrentPosition() - target) <= 20;
    }
}