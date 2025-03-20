package subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import commands.CommandCore;

public class OuttakeLiftSubsys extends SubsysCore{
    DcMotorEx llift, rlift, clift;
    private static int target = 0;
    private static UnifiedTelemetry tel = new UnifiedTelemetry();

    @Override
    void init(){
        llift = hardwareMap.get(DcMotorEx.class, "llift");
        rlift = hardwareMap.get(DcMotorEx.class, "rlift");
        clift = hardwareMap.get(DcMotorEx.class, "clift");
        llift.setDirection(DcMotorSimple.Direction.REVERSE);
        rlift.setDirection(DcMotorSimple.Direction.FORWARD);
        clift.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public class HoldLift extends CommandCore {
        @Override
        public void initialize() {

        }
        @Override
        public void loop(){

        }
    }

    public class runToZero extends CommandCore {
        @Override public void initialize(){}
        @Override
        public void loop(){

        }
    }

    public int getPosition(){
        return (llift.getCurrentPosition() + rlift.getCurrentPosition() + clift.getCurrentPosition())/3;
    }

    @Override
    void loop(){
        tel.addData("Target", target);
        tel.addData("Current Lift Position", getPosition());
    }
}
