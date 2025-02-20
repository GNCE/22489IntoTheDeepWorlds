

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Misc{
    private Servo sweeper;

    public static double OPEN_POS = 0.07, CLOSE_POS = 0;

    public Misc(HardwareMap hardwareMap) {
        sweeper = hardwareMap.get(Servo.class, "sweeper");
        sweeper.setDirection(Servo.Direction.FORWARD);
    }
    public void initiate(){
        sweeper.setPosition(CLOSE_POS);
    }

    public enum SweepStates{
        START_SWEEP,
        REACHED_TARGET,
        END_SWEEP,
    }
    SweepStates sweepState = SweepStates.END_SWEEP;


    private void setSweepState(SweepStates newState){
        sweepState = newState;
    }
    public void startSweep(){
        setSweepState(SweepStates.START_SWEEP);
    }
    public void endSweep(){
        setSweepState(SweepStates.REACHED_TARGET);
    }

    public void loop(){
        switch(sweepState){
            case START_SWEEP:
                sweeper.setPosition(OPEN_POS);
                break;
            case REACHED_TARGET:
                sweeper.setPosition(CLOSE_POS);
                setSweepState(SweepStates.END_SWEEP);
                break;
            case END_SWEEP:
                break;
            default:
                break;
        }
    }

}

