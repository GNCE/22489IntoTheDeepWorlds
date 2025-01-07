
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake{
    private Servo frintake;
    private Servo flintake;
    private CRServo lintake;
    private CRServo rintake;
    ColorSensor colorSensor;
    OpMode opMode;
    double fr = 0;
    public Intake(HardwareMap hardwareMap, OpMode opMode) {
        rintake = hardwareMap.get(CRServo.class, "rintake");
        lintake = hardwareMap.get(CRServo.class, "lintake");
        frintake = hardwareMap.get(Servo.class, "frintake");
        flintake = hardwareMap.get(Servo.class, "flintake");
        rintake.setDirection(CRServo.Direction.FORWARD);
        lintake.setDirection(CRServo.Direction.REVERSE);
        frintake.setDirection(Servo.Direction.REVERSE);
        flintake.setDirection(Servo.Direction.FORWARD);
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
        this.opMode = opMode;
    }
    public void moveThings(){
        if (frintake.getPosition()!=fr) {
            frintake.setPosition(fr);
            flintake.setPosition(fr);
        }

    }
    public void flipDown(){
        fr = .7075;
        rintake.setPower(.6);
        lintake.setPower(.6);
        while (colorSensor.red()>300 && !opMode.gamepad1.right_bumper){

        }
    }
    public void flipUp(){
        frintake.setPosition(0);
        flintake.setPosition(0);
        rintake.setPower(0);
        lintake.setPower(0);
    }
}
