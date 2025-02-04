import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp (name = "Intake Test")
@Config
public class IntakeTest extends OpMode {
    public static double intakePower = 0;
    private static double ip = 0;
    public static boolean amIRed = true;
    public static float[] hsvValues = new float[3];
    public static NormalizedRGBA colors;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet;
    CRServo rintake;
    CRServo lintake;

    Servo door;
    NormalizedColorSensor colorSensor;


    @Override
    public void init(){
        packet = new TelemetryPacket();

        rintake = hardwareMap.get(CRServo.class, "rintake");
        lintake = hardwareMap.get(CRServo.class, "lintake");
        rintake.setDirection(CRServo.Direction.FORWARD);
        lintake.setDirection(CRServo.Direction.REVERSE);

        door = hardwareMap.get(Servo.class, "door");
        door.setDirection(Servo.Direction.FORWARD);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
    }
    private boolean isRed(){
        return hsvValues[0] <= 10;
    }

    private boolean isBlue(){
        return hsvValues[0] >= 200 && hsvValues[0] <= 250;
    }

    private boolean isYellow(){
        return hsvValues[0] >= 60 && hsvValues[0] <= 120;
    }

    public boolean isCorrectColor(){
        return hsvValues[1] > 0.5 && (isYellow() || (Storage.isRed && isRed()) || (!Storage.isRed && isBlue()));
    }

    public void setOpen(boolean open){
        if(open) door.setPosition(0.5);
        else door.setPosition(0);
    }

    @Override
    public void loop(){
        Storage.isRed = amIRed;

        rintake.setPower(ip);
        lintake.setPower(ip);

        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        if(isCorrectColor()){
            ip=0;
            setOpen(false);
        } else {
            ip= intakePower;
            setOpen(true);
        }


        packet.put("TeamColorIsRed", Storage.isRed);
        packet.put("Red", colors.red);
        packet.put("Green", colors.green);
        packet.put("Blue", colors.blue);
        packet.put("Alpha", colors.alpha);
        packet.put("Hue", hsvValues[0]);
        packet.put("Saturation", hsvValues[1]);
        packet.put("Value", hsvValues[2]);
        if(colorSensor instanceof  DistanceSensor){
            packet.put("Distance", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }

        dashboard.sendTelemetryPacket(packet);

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);
        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
        }
        telemetry.update();
    }
}
