package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake extends SubsysCore {
    private Servo clamp;
    private Servo Rdiffy;
    private Servo Ldiffy;
    private Servo rpivhigh;
    private Servo lpivhigh;

    public static double DEFAULT_LDIFFY_POS = 0.52;
    public static double DEFAULT_RDIFFY_POS = 0.52;
    public static double LdiffyPos = DEFAULT_LDIFFY_POS;
    public static double RdiffyPos = DEFAULT_RDIFFY_POS;

    public static double ArmPosition = 0.2;
    public static double leftArmOffset = (double) 0.036;
    public static boolean clawOpen = false;
    public static double CLAW_CLOSED = 0.415;
    public static double CLAW_OPENED = 0.13;
    public static double CLAW_LOOSE_CLOSED = 0.355;
    public static double zeroOffset = 0.013;
    //tune these values vvvvv
    public static double ARM_SAMPSCORE_POS = 0.7;
    public static double ARM_TRANSFER_POS = 0.38;
    public static double ARM_TRANSFER_WAIT = 0.38;
    public static double ARM_FRONTPICKUP_POS = 0.38;
    public static double ARM_BACKSCORE_POS = 0.88;
    public static double AUTO_ARM_BACKSCORE_POS = 0.87;
    public static double ARM_BACKSCORE_OUT = 0.91;
    public static double OLD_ARM_FRONTSCORE_POS = 0.37;
    public static double ARM_FRONTSCORE_WAIT_POS = 0.58;
    public static double ARM_FRONTSCORE_DONE_POS = 0.44;
    public static double ARM_BACKPICKUP_POS = 0.9;
    public static double ARM_PARK_POS = 1.055 / 2;
    public static double ARM_DEPOSIT_POS = 0.9;
    public static double ARM_SAMPLE_SCORE_WAIT = 0.65;
    public enum OuttakeState {
        SPECFRONTPICKUP,
        SPECFRONTSCOREWAIT,
        SPECFRONTSCOREDONE,
        SPECFRONTSCOREOLD,
        TRANSFER,
        TRANSFER_WAIT,
        SAMPLESCORE,
        SPECBACKSCORE,
        AUTO_BACK,
        SPECBACKSCOREOUT,
        SPECBACKPICKUP,
        PARK,
        RESET_ENCODER,
        Auto_Wait,
        SAMPLE_SCORE_WAIT,
        AUTO_SAMPLE_DEPOSIT,

    }

    @Override
    public void init() {
        clamp = hardwareMap.get(Servo.class, "outtakeClamp");
        clamp.setDirection(Servo.Direction.REVERSE);
        rpivhigh = hardwareMap.get(Servo.class, "outtakeRightArm");
        lpivhigh = hardwareMap.get(Servo.class, "outtakeLeftArm");
        rpivhigh.setDirection(Servo.Direction.REVERSE);
        lpivhigh.setDirection(Servo.Direction.FORWARD);
        Rdiffy = hardwareMap.get(Servo.class,"outtakeRDiffy");
        Ldiffy = hardwareMap.get(Servo.class,"outtakeLDiffy");
        Rdiffy.setDirection(Servo.Direction.REVERSE);
        Ldiffy.setDirection(Servo.Direction.FORWARD);

        setClawState(ClawStates.CLOSED);
        DIFFY_POSITIONS.ORIENTATION_ALIGNED = 0;
    }

    @Config
    public static class DIFFY_POSITIONS {
        public static double AUTO_INIT = 100;
        public static double SAMPLE_SCORE = 40;
        public static double TRANSFER = 0;
        public static double SPECIMEN_FRONT_PICKUP = -10;
        public static double SPECIMEN_BACK_SCORE = -60;
        public static double AUTO_SPECIMEN_BACK_SCORE = -60;
        public static double SPECIMEN_BACK_SCORE_OUT = 0;
        public static double SPECIMEN_BACK_PICKUP = 10;
        public static double SPECIMEN_FRONT_SCORE_OLD = 10;
        public static double SPECIMEN_FRONT_SCORE_WAIT = -35;
        public static double SPECIMEN_FRONT_SCORE_DONE = -35;
        public static double SAMPLE_DEPOSIT = 90;
        public static double ORIENTATION_UP = 0;
        public static double ORIENTATION_DOWN = 200;
        public static double ORIENTATION_ALIGNED = 0;
        public static double SAMPLE_SCORE_WAIT = 0;
    }

    private void setPivotPosition(double UpDownAngle, double Orientation){
        double ServoRange = 355;
        LdiffyPos = DEFAULT_LDIFFY_POS + UpDownAngle/ServoRange + Orientation*((double) 18/52)/ServoRange;
        RdiffyPos = DEFAULT_RDIFFY_POS + UpDownAngle/ServoRange - Orientation*((double) 18/52)/ServoRange;
    }

    private void updatePivotPosition(){
        if(Ldiffy.getPosition() != LdiffyPos || Rdiffy.getPosition() != RdiffyPos){
            Ldiffy.setPosition(LdiffyPos);
            Rdiffy.setPosition(RdiffyPos);
        }
    }

    OuttakeState outtakeState = OuttakeState.TRANSFER;

    public void setAlignedTo(double newAligned){
        DIFFY_POSITIONS.ORIENTATION_ALIGNED = MathUtils.clamp(newAligned, -DIFFY_POSITIONS.ORIENTATION_DOWN, DIFFY_POSITIONS.ORIENTATION_DOWN);
    }

    @Override
    public void loop(){
        switch(outtakeState){
            case TRANSFER:
                ArmPosition = ARM_TRANSFER_POS;
                setPivotPosition(DIFFY_POSITIONS.TRANSFER, DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case SAMPLESCORE:
                ArmPosition = ARM_SAMPSCORE_POS;
                setPivotPosition(DIFFY_POSITIONS.SAMPLE_SCORE, DIFFY_POSITIONS.ORIENTATION_DOWN);
                break;
            case SPECFRONTPICKUP:
                ArmPosition = ARM_FRONTPICKUP_POS;
                setPivotPosition(DIFFY_POSITIONS.SPECIMEN_FRONT_PICKUP, DIFFY_POSITIONS.ORIENTATION_DOWN);
                break;
            case PARK:
                ArmPosition = ARM_PARK_POS;
                setPivotPosition(DIFFY_POSITIONS.TRANSFER, DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case SPECBACKSCORE:
                ArmPosition = ARM_BACKSCORE_POS;
                setPivotPosition(DIFFY_POSITIONS.SPECIMEN_BACK_SCORE, DIFFY_POSITIONS.ORIENTATION_ALIGNED);
                break;
            case SPECBACKSCOREOUT:
                ArmPosition = ARM_BACKSCORE_OUT;
                setPivotPosition(DIFFY_POSITIONS.SPECIMEN_BACK_SCORE_OUT, DIFFY_POSITIONS.ORIENTATION_ALIGNED);
                break;
            case RESET_ENCODER:
                ArmPosition = 0.55;
                setPivotPosition(DIFFY_POSITIONS.SPECIMEN_FRONT_PICKUP, DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case SPECBACKPICKUP:
                ArmPosition = ARM_BACKPICKUP_POS;
                setPivotPosition(DIFFY_POSITIONS.SPECIMEN_BACK_PICKUP, DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case SPECFRONTSCOREOLD:
                ArmPosition = OLD_ARM_FRONTSCORE_POS;
                setPivotPosition(DIFFY_POSITIONS.SPECIMEN_FRONT_SCORE_OLD, DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case Auto_Wait:
                ArmPosition = 0.4;
                setPivotPosition(DIFFY_POSITIONS.AUTO_INIT, DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case TRANSFER_WAIT:
                ArmPosition = ARM_TRANSFER_WAIT;
                setPivotPosition(DIFFY_POSITIONS.TRANSFER, DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case SAMPLE_SCORE_WAIT:
                ArmPosition = ARM_SAMPLE_SCORE_WAIT;
                setPivotPosition(DIFFY_POSITIONS.SAMPLE_SCORE_WAIT, DIFFY_POSITIONS.ORIENTATION_DOWN);
                break;
            case SPECFRONTSCOREDONE:
                ArmPosition = ARM_FRONTSCORE_DONE_POS;
                setPivotPosition(DIFFY_POSITIONS.SPECIMEN_FRONT_SCORE_DONE, DIFFY_POSITIONS.ORIENTATION_DOWN);
                break;
            case SPECFRONTSCOREWAIT:
                ArmPosition = ARM_FRONTSCORE_WAIT_POS;
                setPivotPosition(DIFFY_POSITIONS.SPECIMEN_FRONT_SCORE_WAIT, DIFFY_POSITIONS.ORIENTATION_DOWN);
                break;
            case AUTO_SAMPLE_DEPOSIT:
                ArmPosition = ARM_DEPOSIT_POS;
                setPivotPosition(DIFFY_POSITIONS.SAMPLE_DEPOSIT, DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case AUTO_BACK:
                ArmPosition = AUTO_ARM_BACKSCORE_POS;
                setPivotPosition(DIFFY_POSITIONS.AUTO_SPECIMEN_BACK_SCORE, DIFFY_POSITIONS.ORIENTATION_ALIGNED);

        }
        ArmPosition += zeroOffset;

        updatePivotPosition();

        rpivhigh.setPosition(ArmPosition);
        lpivhigh.setPosition(ArmPosition + leftArmOffset);

        switch(clawState){
            case OPEN:
                clamp.setPosition(CLAW_OPENED);
                break;
            case LOOSE_CLOSED:
                clamp.setPosition(CLAW_LOOSE_CLOSED);
                break;
            case CLOSED:
                clamp.setPosition(CLAW_CLOSED);
                break;
        }
    }

    public void setOuttakeState(OuttakeState outtakeState){
        this.outtakeState = outtakeState;
    }

    public enum ClawStates{
        OPEN, CLOSED, LOOSE_CLOSED;
    }
    private ClawStates clawState;

    public void setClawState(ClawStates clawState){
        this.clawState = clawState;
    }
}