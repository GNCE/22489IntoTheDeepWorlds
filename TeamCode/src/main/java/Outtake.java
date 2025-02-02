
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Outtake {
    private Servo clamp;
    private Servo updownpiv;
    private Servo spinpiv;
    private Servo rpivhigh;
    private Servo lpivhigh;
    double pivpos = 0;

    boolean clawOpen = false;
    final double CLAW_CLOSED = 0.655;
    final double CLAW_OPENED = 0.627;

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
    public void loop(){
        if (pivpos != rpivhigh.getPosition()){
            rpivhigh.setPosition(pivpos);
            lpivhigh.setPosition(pivpos);
        }
        if(clawOpen) clamp.setPosition(CLAW_OPENED);
        else clamp.setPosition(CLAW_CLOSED);
    }
    public void pivotToFront(){
        pivpos = 0.45;
        updownpiv.setPosition(0.154);
        spinpiv.setPosition(0.985);
    }
    public void pivotToScoreSamp(){
        pivpos = .82;
        updownpiv.setPosition(.127);
        spinpiv.setPosition(0.883);
    }
    // 0.242
    public void pivotToScoreSpecBack(){
        pivpos =1;
        updownpiv.setPosition(0.127);
        spinpiv.setPosition(0.985);
    }
    public void pivotToPickupBack(){
        pivpos = 0;
        updownpiv.setPosition(0.205);
        spinpiv.setPosition(0.985); // spins it around
    }
    public void pivotToTransfer (){
        pivpos = 0.0;
        updownpiv.setPosition(0.119);
        spinpiv.setPosition(0.985);
    }

    public void setClaw(boolean state){
        clawOpen = state;
    }

    public boolean isClawBusy(){
        return (clawOpen && Math.abs(clamp.getPosition() - CLAW_OPENED) < 0.005) || (!clawOpen && Math.abs(clamp.getPosition() - CLAW_CLOSED) < 0.005);
    }
    public boolean isArmBusy(){
        return pivpos != rpivhigh.getPosition();
    }
}