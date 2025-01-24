
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
    public void pivotToFront(){
        pivpos = 0.38;
        updownpiv.setPosition(0.1605);
        spinpiv.setPosition(0.883);
    }
    public void pivotToScoreSamp(){
        pivpos = .82;
        updownpiv.setPosition(.147);
        spinpiv.setPosition(0.883);
    }
    public void pivotToPickupBack(){
        pivpos = 0;
        updownpiv.setPosition(0.235);
        spinpiv.setPosition(0.985); // spins it around
    }
    public void pivotToTransfer (){
        pivpos = 0;
        updownpiv.setPosition(0.1550);
        spinpiv.setPosition(0.985);
    }

    public void openClaw(){
        clamp.setPosition(0.620);
    }
    public void closeClaw(){
        clamp.setPosition(0.655);
    }
}