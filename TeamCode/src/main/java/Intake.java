

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Intake{
    private Servo finintake;
    private CRServo lintake;
    private CRServo rintake;
    private DcMotor extendo;
    ColorSensor colorSensor;

    OpMode opMode;
    double fin = 0;
    double ip = 0;
    double ex = 0;
    public Intake(HardwareMap hardwareMap, OpMode opMode) {
        rintake = hardwareMap.get(CRServo.class, "rintake");
        lintake = hardwareMap.get(CRServo.class, "lintake");
        finintake = hardwareMap.get(Servo.class, "fintake");
        rintake.setDirection(CRServo.Direction.FORWARD);
        lintake.setDirection(CRServo.Direction.REVERSE);
        finintake.setDirection(Servo.Direction.REVERSE);
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
        this.opMode = opMode;
        extendo = hardwareMap.get(DcMotor.class, "extendo");
        extendo.setDirection(DcMotor.Direction.REVERSE);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public boolean isRed(){
        if (colorSensor.red()>500){
            return true;
        } else return false;
    }
    public boolean isBlue(){
        if (colorSensor.blue()>500){
            return true;
        } else return false;
    }
    public void initiate(){
        extendo.setTargetPosition(0);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(1);
        rintake.setPower(0);
        lintake.setPower(0);
        finintake.setPosition(0);
    }
    public void moveThings(){
        if (finintake.getPosition()!=fin) {
            finintake.setPosition(fin);
        }
        if (rintake.getPower()!=ip){
            rintake.setPower(ip);
            lintake.setPower(ip);
        }
        if (extendo.getTargetPosition()!=ex*250){
            extendo.setTargetPosition((int)Math.round(ex*250));
        }

    }
    public void ManualExtend(){
        ex = 250;
    }
    public void ManualRetract(){
        ex = 0;
    }
    public void TeleopExtend(){
        ex = opMode.gamepad1.left_trigger;
    }
    public void deposit(){
        while ((isRed())){
            ip = -1;
        }
        ip = 0;
    }
    public void flipDown(){
            fin = .7075;
            ip = .7;
    }
    public void flipUp(){
        fin = 0;
        ip = 0;
    }
}
