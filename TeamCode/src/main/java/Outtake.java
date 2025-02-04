
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
    double ArmPosition = 0;
    boolean clawOpen = false;
    final double CLAW_CLOSED = 0.655;
    final double CLAW_OPENED = 0.627;
    //tune these values vvvvv
    static double ARM_SAMPSCORE_POS = 1;
    static double ARM_TRANSFER_POS = 0;
    static double ARM_FRONTSPEC_POS = 0;
    static double ARM_BACKSPEC_POS = 1;
    public enum OuttakeState {
        SPECFRONT,
        TRANSFER,
        SAMPLESCORE,
        SPECBACKSCORE

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
    public enum DiffyState {
        ORIENTATION_UP,
        ORIENTATION_DOWN,
        UPDOWN_TRANSFER,
        UPDOWN_SPECBACKSCORE,
        UPDOWN_SPECFRONT,
        UPDOWN_SAMPLESCORE,

    }
    DiffyState UPDOWNdiffyState = DiffyState.UPDOWN_TRANSFER;
    DiffyState ORIENTATIONdiffyState = DiffyState.ORIENTATION_UP;
    double rightDiffyPos = 0;
    double leftDiffyPos = 0;
    double upDownPos = 0;
    double orientationPos = 0;

    //tune these values vvvvv
    static double DIFFY_SAMPLESCOREPOS = 0;
    static double DIFFY_TRANSFERPOS = 0;
    static double DIFFY_SPECFRONTPOS = 0;
    static double DIFFY_SPECBACKPOS = 0;
    static double DIFFY_ORIENTATION_UP = 0;
    static double DIFFY_ORIENTATION_DOWN = 0;
    public void DiffyControl(DiffyState UPDOWNdiffyState, DiffyState ORIENTATIONdiffyState){
        switch (UPDOWNdiffyState){
            case UPDOWN_SAMPLESCORE:
                upDownPos = DIFFY_SAMPLESCOREPOS;
                break;
            case UPDOWN_SPECBACKSCORE:
                upDownPos = DIFFY_SPECBACKPOS;
                break;
            case UPDOWN_SPECFRONT:
                upDownPos = DIFFY_SPECFRONTPOS;
                break;
            case UPDOWN_TRANSFER:
                upDownPos = DIFFY_TRANSFERPOS;
                break;
        }
        switch (ORIENTATIONdiffyState){
            case ORIENTATION_DOWN:
                orientationPos = DIFFY_ORIENTATION_UP;
                break;
            case ORIENTATION_UP:
                orientationPos = DIFFY_ORIENTATION_DOWN;
                break;
        }
        rightDiffyPos = upDownPos + orientationPos;
        leftDiffyPos = upDownPos - orientationPos;
    }
    OuttakeState outtakeState = OuttakeState.TRANSFER;
    public void loop(){
        switch(outtakeState){
            case TRANSFER:
                ArmPosition = ARM_TRANSFER_POS;
                clawOpen = true;
                DiffyControl(DiffyState.UPDOWN_TRANSFER,DiffyState.ORIENTATION_UP);
                break;
            case SAMPLESCORE:
                ArmPosition = ARM_SAMPSCORE_POS;
                DiffyControl(DiffyState.UPDOWN_SAMPLESCORE,DiffyState.ORIENTATION_DOWN);
                break;
            case SPECFRONT:
                ArmPosition = ARM_FRONTSPEC_POS;
                DiffyControl(DiffyState.UPDOWN_SPECFRONT, DiffyState.ORIENTATION_UP);
                break;
            case SPECBACKSCORE:
                ArmPosition = ARM_BACKSPEC_POS;
                DiffyControl(DiffyState.UPDOWN_SPECBACKSCORE, DiffyState.ORIENTATION_DOWN);
                break;
        }
        if (ArmPosition != rpivhigh.getPosition()){
            rpivhigh.setPosition(ArmPosition);
            lpivhigh.setPosition(ArmPosition);
        }
        if ((clamp.getPosition()!=CLAW_CLOSED )&& !clawOpen){
            clamp.setPosition(CLAW_CLOSED);
        } else if ((clamp.getPosition()!=CLAW_OPENED) && clawOpen){
            clamp.setPosition(CLAW_OPENED);
        }
        if((Rdiffy.getPosition() != rightDiffyPos)||(Ldiffy.getPosition() != leftDiffyPos)){
            Rdiffy.setPosition(rightDiffyPos);
            Ldiffy.setPosition(leftDiffyPos);
        }
    }

    public void setOuttakeState(OuttakeState outtakeState){
        this.outtakeState = outtakeState;
    }

    public void setClawOpen(boolean state){
        clawOpen = state;
    }
}