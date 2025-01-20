
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Outtake {
    private Servo clamp;
    private Servo updownpiv;
    private Servo spinpiv;
    private Servo rpivhigh;
    private Servo lpivhigh;
    double pivpos = 0;

    public Outtake(HardwareMap hardwareMap) {
        clamp = hardwareMap.get(Servo.class, "clamp");
        rpivhigh = hardwareMap.get(Servo.class, "rpivhigh");
        updownpiv = hardwareMap.get(Servo.class, "updownpiv");
        lpivhigh = hardwareMap.get(Servo.class, "lpivhigh");
        spinpiv = hardwareMap.get(Servo.class, "spinpiv");
        rpivhigh.setDirection(Servo.Direction.FORWARD);
        lpivhigh.setDirection(Servo.Direction.REVERSE);
        updownpiv.setDirection(Servo.Direction.FORWARD);
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
        updownpiv.setPosition(0.25);
    }
    public void pivotToScoreSpecBack(){
        pivpos = 1.6;
        updownpiv.setPosition(0.25);
    }
    public void pivotToScoreSamp(){
        pivpos = 1.2;
        updownpiv.setPosition(.5);
    }
    public void pivotToPickup (){
        pivpos = 0;
        updownpiv.setPosition(0);
    }
    public void pivotToTransfer (){
        pivpos = 0;
        updownpiv.setPosition(0.3);
    }
    public void openClaw(){
        clamp.setPosition(.05);
    }
    public void closeClaw(){
        clamp.setPosition(0);
    }
}