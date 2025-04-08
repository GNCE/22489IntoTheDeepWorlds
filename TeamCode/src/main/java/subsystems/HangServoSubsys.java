package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HangServoSubsys extends SubsysCore {
    private Servo rpivhigh;
    private Servo lpivhigh;
    public static double armpos = 0.0;
    public static double loffsetPos = 0.0;
    public static double roffsetPos = 0.0;

    public static double lHangPos = 0.06;
    public static double rHangPos = 0.80;
    public static double lRestPos = 0.39;
    public static double rRestPos = 0.43;

    @Override
    public void init(){
        rpivhigh = hardwareMap.get(Servo.class, "rhang");
        lpivhigh = hardwareMap.get(Servo.class, "lhang");
        rpivhigh.setDirection(Servo.Direction.FORWARD);
        lpivhigh.setDirection(Servo.Direction.FORWARD);
    }

    public void rest(){
        lpivhigh.setPosition(lRestPos);
        rpivhigh.setPosition(rRestPos);
    }

    public void hang(){
        lpivhigh.setPosition(lHangPos);
        rpivhigh.setPosition(rHangPos);
    }

    @Override
    public void loop(){

    }
}
