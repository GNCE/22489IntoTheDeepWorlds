
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake_DiffyClaw {
    private Servo IntakeClamp;
    private Servo IntakeRDiffy;
    private Servo IntakeLDiffy;
    private Servo RightArmPivot;
    private Servo LeftArmPivot;
    public Servo leintake, reintake;

    public static int pipelineNumber = 4;

    public static double DEFAULT_LDIFFY_POS = 0.527;
    public static double DEFAULT_RDIFFY_POS = 0.503;
    public static double LdiffyPos = DEFAULT_LDIFFY_POS;
    public static double RdiffyPos = DEFAULT_RDIFFY_POS;

    public static double ArmPosition = 0.285;
    public static boolean clawOpen = false;
    public static double CLAW_CLOSED = 0.39;
    public static double CLAW_OPENED = 0.18;
    //tune these values vvvvv
    public static double ARM_REST = 0;
    public static double ARM_TRANSFER_POS = 0.3;
    public static double ARM_TRANSFER_WAIT_POS = 0.3;
    public static double ARM_PICKUP_READY = 0.5;
    public static double ARM_PICKUP_DOWN = 0.57;

    /** LINKAGE EXTENSION VARIABLES */
    public static double extPos = 0;
    final double LINK1 = Math.sqrt(97408); // Length of first linkage (Linkage that connects to servo) (mm) (correct)
    final double LINK2 = 320; // Length of second linkage (Linkage that connects to the slide) (mm) (correct)
    final double XOFFSET = 97; // Offset X axis (CURRENT VALUE IS CORRECT)
    final double YOFFSET = 8.25; // Offset Y axis (CURRENT VALUE IS CORRECT)
    public static double FULL_EXTENSION = 400; // Length of the slides when fully extended (mm)
    public static double EXTENSION_ZERO_OFFSET = -0.02; // Servo Zero Offset
    final int SERVO_RANGE = 300; // Servo Range in degrees

    //other extendo variables
    double extensionWaitTime = 0;
    public static double distance;

    public static double INTAKE_DOWN_EXTENSION_LIMIT = 0;
    public static double TRANSFER_EXTENSION_POS = 0;
    ElapsedTime extensionTime;


    public enum IntakeState {
        TRANSFER,
        TRANSFER_WAIT,
        INTAKE_ARM_READY,
        INTAKE_ARM_PICKUP,
        INTAKE_REST,

    }
    public Intake_DiffyClaw(HardwareMap hardwareMap) {
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
        reintake = hardwareMap.get(Servo.class,"reintake");
        leintake = hardwareMap.get(Servo.class, "leintake");
        reintake.setDirection(Servo.Direction.REVERSE);

        extensionTime = new ElapsedTime();
        extensionTime.startTime();
    }
    @Config
    public static class INTAKE_DIFFY_POSITIONS {
        public static double TRANSFER_POS = 60;
        public static double INTAKE_POS = -115;
        public static double INTAKE_FINAL_POS = -100;
        public static double REST_POS = 20;
        public static double ORIENTATION_UP = 0;
        public static double ORIENTATION_DOWN = 200;
        public static double ORIENTATION_ALIGNED = 0;

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
            extensionWaitTime = Math.abs((targetPos - leintake.getPosition() )* 14);
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
        else if(target < 0) target = 0;
        extPos = target;
    }
    public void TeleopExtend(double valueFromZeroToOne){
        if(valueFromZeroToOne < 0) valueFromZeroToOne = 0;
        else if(valueFromZeroToOne > 1) valueFromZeroToOne = 1;
        setExtensionTarget(valueFromZeroToOne * FULL_EXTENSION);
    }
    public void changePipeline(int pipelineNum) {
        pipelineNumber = pipelineNum;
    }

    IntakeState intakeState = IntakeState.TRANSFER;
    public void intakeLoop(){
        extendTo(extPos);
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
                setPivotPosition(INTAKE_DIFFY_POSITIONS.REST_POS, INTAKE_DIFFY_POSITIONS.ORIENTATION_DOWN);
                break;
        }

        updatePivotPosition();

        if (ArmPosition != RightArmPivot.getPosition()){
            RightArmPivot.setPosition(ArmPosition);
            LeftArmPivot.setPosition(ArmPosition);
        }
        if ((IntakeClamp.getPosition()!=CLAW_CLOSED) && !clawOpen){
            IntakeClamp.setPosition(CLAW_CLOSED);
        } else if ((IntakeClamp.getPosition()!=CLAW_OPENED) && clawOpen){
            IntakeClamp.setPosition(CLAW_OPENED);
        }
    }


    public void setIntakeState(IntakeState intakeState){
        this.intakeState = intakeState;
    }

    public void setClawOpen(boolean state){
        clawOpen = state;
    }
}