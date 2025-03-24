package subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SubsysCore {
    public static HardwareMap hardwareMap;
    public static OpMode opMode;
    public static void setGlobalParameters(HardwareMap hardwareMap, OpMode opMode){
        SubsysCore.hardwareMap = hardwareMap;
        SubsysCore.opMode = opMode;
    }

    public abstract void init();

    public abstract void loop();
}
