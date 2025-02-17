
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake {
    private Servo clamp;
    private Servo Rdiffy;
    private Servo Ldiffy;
    private Servo rpivhigh;
    private Servo lpivhigh;

    public static double DEFAULT_LDIFFY_POS = 0.28;
    public static double DEFAULT_RDIFFY_POS = 0.13;
    public static double LdiffyPos = DEFAULT_LDIFFY_POS;
    public static double RdiffyPos = DEFAULT_RDIFFY_POS;

    public static double ArmPosition = 0;
    public static boolean clawOpen = false;
    public static double CLAW_CLOSED = 0.655;
    public static double CLAW_OPENED = 0.627;
    //tune these values vvvvv
    public static double ARM_SAMPSCORE_POS = 1;
    public static double ARM_TRANSFER_POS = 0;
    public static double ARM_FRONTSPEC_POS = 0;
    public static double ARM_BACKSPEC_POS = 1;
    public enum OuttakeState {
        SPECFRONTPICKUP,
        SPECFRONTSCORE,
        TRANSFER,
        SAMPLESCORE,
        SPECBACKSCORE,
        SPECBACKPICKUP,

    }
    public Outtake(HardwareMap hardwareMap) {
        clamp = hardwareMap.get(Servo.class, "clamp");
        rpivhigh = hardwareMap.get(Servo.class, "rpivhigh");
        lpivhigh = hardwareMap.get(Servo.class, "lpivhigh");
        rpivhigh.setDirection(Servo.Direction.FORWARD);
        lpivhigh.setDirection(Servo.Direction.REVERSE);
        Rdiffy = hardwareMap.get(Servo.class,"Rdiffy");
        Ldiffy = hardwareMap.get(Servo.class,"Ldiffy");
        Rdiffy.setDirection(Servo.Direction.FORWARD);
        Ldiffy.setDirection(Servo.Direction.REVERSE);
    }

    @Config
    public static class DIFFY_POSITIONS {
        static double SAMPLE_SCORE = 0;
        static double TRANSFER = 0;
        static double SPECIMEN_FRONT_PICKUP = 0;
        static double SPECIMEN_BACK_SCORE = 0;
        static double ORIENTATION_UP = 0;
        static double ORIENTATION_DOWN = 200;
    }

    private void setPivotPosition(double UpDownAngle, double Orientation){
        double ServoRange = 300;
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
                ArmPosition = ARM_FRONTSPEC_POS;
                setPivotPosition(DIFFY_POSITIONS.SPECIMEN_FRONT_PICKUP, DIFFY_POSITIONS.ORIENTATION_UP);
                break;
            case SPECBACKSCORE:
                ArmPosition = ARM_BACKSPEC_POS;
                setPivotPosition(DIFFY_POSITIONS.SPECIMEN_BACK_SCORE, DIFFY_POSITIONS.ORIENTATION_DOWN);
                break;
        }

        updatePivotPosition();

        if (ArmPosition != rpivhigh.getPosition()){
            rpivhigh.setPosition(ArmPosition);
            lpivhigh.setPosition(ArmPosition);
        }
        if ((clamp.getPosition()!=CLAW_CLOSED )&& !clawOpen){
            clamp.setPosition(CLAW_CLOSED);
        } else if ((clamp.getPosition()!=CLAW_OPENED) && clawOpen){
            clamp.setPosition(CLAW_OPENED);
        }
    }

    public void setOuttakeState(OuttakeState outtakeState){
        this.outtakeState = outtakeState;
    }

    public void setClawOpen(boolean state){
        clawOpen = state;
    }
}