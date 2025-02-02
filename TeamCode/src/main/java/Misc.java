

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    enum SweepStates{
        START_SWEEP,
        REACHED_TARGET,
        END_SWEEP,
    }
    SweepStates sweepState;

    ElapsedTime sweepTime;
    private void setSweepState(SweepStates newState){
        sweepState = newState;
        sweepTime.reset();
    }
    public void startSweep(){
        setSweepState(SweepStates.START_SWEEP);
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

    public void loop(){
        if (door.getPosition()!=dr) {
            door.setPosition(dr);
        }

        switch(sweepState){
            case START_SWEEP:
                sweeper.setPosition(0.5);
                setSweepState(SweepStates.REACHED_TARGET);
                break;
            case REACHED_TARGET:
                if(sweepTime.time() > 2){
                    sweeper.setPosition(0);
                    setSweepState(SweepStates.END_SWEEP);
                }
                break;
            case END_SWEEP:
                break;
            default:
                break;
        }
    }

}

