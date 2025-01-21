
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
    public void pivotToScoreorpickupSpecFront(){
        pivpos = 0.433;
        updownpiv.setPosition(0.163);
        spinpiv.setPosition(0.889);
    }
    public void pivotToScoreSampandBackSpec(){
        pivpos = .82;
        updownpiv.setPosition(.147);
        spinpiv.setPosition(0.99);
    }
    public void pivotToPickupBack(){
        pivpos = 0;
        updownpiv.setPosition(0.235);
        spinpiv.setPosition(0.99);
    }
    public void pivotToTransfer (){
        pivpos = 0.02;
        updownpiv.setPosition(0.163);
        spinpiv.setPosition(0.889);
    }
    public void openClaw(){
        clamp.setPosition(.615);
    }
    public void closeClaw(){
        clamp.setPosition(0.655);
    }
}