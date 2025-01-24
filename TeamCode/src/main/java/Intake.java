

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Intake{
    private Servo finintake;
    public CRServo lintake;
    public CRServo rintake;
    public DcMotor extendo;
    ColorSensor colorSensor;

    OpMode opMode;
    double fin = 0;
    double intakePower = 0;
    double ex = 0;
    int depo =0;
    boolean useRTP = true;
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
        extendo.setDirection(DcMotor.Direction.FORWARD);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public boolean isRed(){
        return colorSensor.red() > 250;
    }
    public boolean isYellow(){
        return colorSensor.green() > 500;
    }
    public boolean isBlue(){
        return colorSensor.blue() > 500;
    }
    public void initiate(){
        extendo.setTargetPosition(0);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(.5);
        rintake.setPower(0);
        lintake.setPower(0);
        finintake.setPosition(0);
    }
    public void moveThings(){
        if (finintake.getPosition()!=fin) {
            finintake.setPosition(fin);
        }
        if (rintake.getPower()!= intakePower){
            rintake.setPower(intakePower);
            lintake.setPower(intakePower);
        }
        if (extendo.getTargetPosition()!=ex*450){
            extendo.setTargetPosition((int)Math.round(ex*450));
        }

    }
    public void ManualExtend(){
        ex = 1; extendo.setPower(.3);
    }
    public void ManualRetract(){
        ex = 0; extendo.setPower(1);
    }
    public void TeleopExtend(){
            ex = opMode.gamepad1.left_trigger;
            if (opMode.gamepad1.left_trigger > 0.3) {
                extendo.setPower(.3);
            } else {
                extendo.setPower(1);
            }
    }
    public void deposit(){
        intakePower =-1;
        depo = 1;
    }
    public void flipDown(){
        fin = .919;
        intakePower = .22;
    }
    public void check(){
        if ((isRed() || isYellow()) && depo ==0){
            flipUp();
            intakePower = 0;
        } else if (!isRed() && !isYellow() &&depo==1){
            intakePower = 0;
            depo = 0;
        }
    }
    public void flipUp(){
        fin = 0;
        intakePower = -0.2;
    }
}
