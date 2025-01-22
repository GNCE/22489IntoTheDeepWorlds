

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
    int depo =0;
    boolean useRTP = false;
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
        if (colorSensor.red()>250){
            return true;
        } else return false;
    }
    public boolean isYellow(){
        if (colorSensor.green()>500){
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
        extendo.setPower(.5);
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
        if (extendo.getTargetPosition()!=ex*450){
            extendo.setTargetPosition((int)Math.round(ex*450));
        }

    }
    public void ManualExtend(){
        ex = 1;
    }
    public void ManualRetract(){
        ex = 0;
    }
    public void TeleopExtend(){
        useRTP = !(opMode.gamepad2.right_stick_y <= 0.1) || !(opMode.gamepad2.right_stick_y >= -0.1);
        if (!useRTP){
            if (extendo.getMode()==(DcMotor.RunMode.RUN_TO_POSITION) ){
                if (extendo.getCurrentPosition() <= 25 && extendo.getPower() < -0.5){
                    extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                } else extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            extendo.setPower(-opMode.gamepad2.right_stick_y);
        } else {
            if (extendo.getMode()==(DcMotor.RunMode.RUN_USING_ENCODER) ){
                extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            ex = opMode.gamepad1.left_trigger;
            if (opMode.gamepad1.left_trigger > 0.3) {
                extendo.setPower(.3);
            } else {
                extendo.setPower(1);
            }
        }
    }
    public void deposit(){
        ip=-1;
        depo = 1;
    }
    public void flipDown(){
            fin = .92;
            ip = .22;
    }
    public void check(){
        if (isRed() || isYellow()){
            flipUp();
            ip = 0;
        } else if (!isRed() && !isYellow() &&depo==1){
            ip = 0;
            depo = 0;
        }
    }
    public void flipUp(){
        fin = 0;
        ip = -0.2;
    }
}
