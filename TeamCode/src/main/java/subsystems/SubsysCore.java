package subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SubsysCore {
    public static HardwareMap hardwareMap;
    public void setGlobalParameters(HardwareMap hardwareMap){
        SubsysCore.hardwareMap = hardwareMap;
    }

    public abstract void init();

    public abstract void loop();
}
