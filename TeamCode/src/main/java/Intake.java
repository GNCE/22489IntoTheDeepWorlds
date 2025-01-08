

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Intake{
    private Servo frintake;
    private Servo flintake;
    private CRServo lintake;
    private CRServo rintake;
    private DcMotor extendo;
    ColorSensor colorSensor;

    OpMode opMode;
    double fr = 0;
    double ip = 0;
    double ex = 0;
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
        extendo = hardwareMap.get(DcMotor.class, "extendo");
        extendo.setDirection(DcMotor.Direction.FORWARD);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public boolean isRed(){
        if (colorSensor.red()>300){
            return true;
        } else return false;
    }
    public boolean isBlue(){
        if (colorSensor.blue()>300){
            return true;
        } else return false;
    }
    public void initiate(){
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(.8);
    }
    public void moveThings(){
        if (frintake.getPosition()!=fr) {
            frintake.setPosition(fr);
            flintake.setPosition(fr);
        }
        if (rintake.getPower()!=ip){
            rintake.setPower(ip);
            lintake.setPower(ip);
        }
        if (extendo.getTargetPosition()!=ex){
            extendo.setTargetPosition((int)Math.round(ex));
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
        while (!isRed() && !opMode.gamepad1.right_bumper){
            fr = .7075;
            ip = .7;
        }
        flipUp();
    }
    public void flipUp(){
        fr = 0;
        ip = 0;
    }
}
