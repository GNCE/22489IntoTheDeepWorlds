package config.core.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class SubsystemCore extends SubsystemBase {
    protected static HardwareMap hardwareMap;
    protected static MultipleTelemetry tel;

    public static void setGlobalParameters(HardwareMap hardwareMap, OpMode opMode, Telemetry tel){
        SubsystemCore.hardwareMap = hardwareMap;
        SubsystemCore.tel = new MultipleTelemetry(tel, FtcDashboard.getInstance().getTelemetry());
    }

    public SubsystemCore(){
        super();
        init();
    }

    public abstract void init();
}
