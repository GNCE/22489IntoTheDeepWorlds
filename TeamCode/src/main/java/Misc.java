

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Misc{
    private Servo sweeper;
    private Servo door;
    double dr = 0;
    double swp = 0;

    public Misc(HardwareMap hardwareMap) {
        sweeper = hardwareMap.get(Servo.class, "sweeper");
        door = hardwareMap.get(Servo.class, "door");
        sweeper.setDirection(Servo.Direction.FORWARD);
    }
    public void initiate(){
        sweeper.setPosition(0);
        door.setPosition(0);
    }
    public void moveThings(){
        if (door.getPosition()!=dr) {
            door.setPosition(dr);
        }
        if (sweeper.getPosition()!=swp){
            sweeper.setPosition(swp);
        }

    }
    public void sweep(){
        swp = 1;
    }
    public void unsweep(){
        swp =0;
    }
    public void door(){
        dr = 1;
    }
    public void undoor(){
        dr =0;
    }

}

