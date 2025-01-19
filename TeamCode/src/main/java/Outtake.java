
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Outtake {
    private Servo clamp;
    private Servo pivlow;
    private Servo rpivhigh;
    private Servo lpivhigh;
    double pivpos = 0;

    public Outtake(HardwareMap hardwareMap) {
        clamp = hardwareMap.get(Servo.class, "clamp");
        rpivhigh = hardwareMap.get(Servo.class, "rpivhigh");
        pivlow = hardwareMap.get(Servo.class, "pivlow");
        lpivhigh = hardwareMap.get(Servo.class, "lpivhigh");
        rpivhigh.setDirection(Servo.Direction.FORWARD);
        lpivhigh.setDirection(Servo.Direction.REVERSE);
        pivlow.setDirection(Servo.Direction.FORWARD);
        clamp.setDirection(Servo.Direction.REVERSE);
    }
    public void updatePivPosition(){
        if (pivpos != rpivhigh.getPosition()){
        rpivhigh.setPosition(pivpos);
        lpivhigh.setPosition(pivpos);
        }
    }
    public void pivotToScoreSpecFront(){
        pivpos = 0.6;
        pivlow.setPosition(0.25);
    }
    public void pivotToScoreSpecBack(){
        pivpos = 1.6;
        pivlow.setPosition(0.25);
    }
    public void pivotToScoreSamp(){
        pivpos = 1.2;
        pivlow.setPosition(.5);
    }
    public void pivotToPickup (){
        pivpos = 0;
        pivlow.setPosition(0);
    }
    public void pivotToTransfer (){
        pivpos = 0;
        pivlow.setPosition(0.3);
    }
    public void openClaw(){
        clamp.setPosition(.05);
    }
    public void closeClaw(){
        clamp.setPosition(0);
    }


}