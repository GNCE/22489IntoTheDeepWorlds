


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Outtake {
    private Servo clamp;
    private Servo pivlow;
    private Servo rpivhigh;
    private Servo lpivhigh;
    float pivpos = 0;

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
        rpivhigh.setPosition(pivpos);
        lpivhigh.setPosition(pivpos);
    }
    public void pivotToScore (){
        pivpos = 0;
        pivlow.setPosition(0);
    }
    public void pivotToPickup (){
        pivpos = 1;
        pivlow.setPosition(1);
    }
    public void openClaw(){
        clamp.setPosition(1);
    }

}