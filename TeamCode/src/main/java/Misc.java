

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Misc{
    private Servo sweeper;
    private Servo door;
    double dr = 0.08;
    double swp = 0;

    public Misc(HardwareMap hardwareMap) {
        sweeper = hardwareMap.get(Servo.class, "sweeper");
        door = hardwareMap.get(Servo.class, "door");
        sweeper.setDirection(Servo.Direction.FORWARD);
        door.setDirection(Servo.Direction.FORWARD);
    }
    public void initiate(){
        sweeper.setPosition(0);
        door.setPosition(dr);
    }
    public void loop(){
        if (door.getPosition()!=dr) {
            door.setPosition(dr);
        }
        if (sweeper.getPosition() != swp){
            sweeper.setPosition(swp);
        }
    }

    public void setSweep(boolean state){
        if(state) swp = 0.5;
        else swp = 0;
    }
    public boolean reachedSweepTarget(){
        return Math.abs(sweeper.getPosition() - swp) < 0.005;
    }
    public void door(){
        dr = .127;
    }
    public void undoor(){
        dr = 0.08;
    }

}

