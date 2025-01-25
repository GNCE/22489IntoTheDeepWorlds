

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

    public enum IntakeState {
        FLIP_UP,
        INTAKE,
        DEPOSIT,

    }

    IntakeState intakeState = IntakeState.FLIP_UP;
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
    private boolean isRed(){
        return colorSensor.red() > 250;
    }
    private boolean isYellow(){
        return colorSensor.green() > 500;
    }
    private boolean isBlue(){
        return colorSensor.blue() > 250;
    }

    public boolean isCorrectColor(){
        return isYellow() || (PoseStorage.isRed && isRed()) || (!PoseStorage.isRed && isBlue());
    }
    public void initiate(){
        extendo.setTargetPosition(0);
        extendo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extendo.setPower(.5);
        rintake.setPower(0);
        lintake.setPower(0);
        finintake.setPosition(0);
    }
    public void extendoLoop(){
        if (finintake.getPosition()!=fin) {
            finintake.setPosition(fin);
        }
        if (rintake.getPower()!= intakePower){
            rintake.setPower(intakePower);
            lintake.setPower(intakePower);
        }
        if (extendo.getTargetPosition()!=(int)Math.round(ex*450)){
            extendo.setTargetPosition((int)Math.round(ex*450));
        }
    }
    public boolean ManualExtend(){
        ex = 1; extendo.setPower(.3);
        return Math.abs(extendo.getTargetPosition() - ex*450) < 2;
    }
    public boolean ManualRetract(){
        ex = 0; extendo.setPower(1);
        return Math.abs(extendo.getTargetPosition() - ex*450) < 2;
    }
    public void TeleopExtend(){
        ex = opMode.gamepad1.left_trigger;
        if (opMode.gamepad1.left_trigger > 0.3) {
            extendo.setPower(.3);
        } else {
            extendo.setPower(1);
        }
    }
    public void intakeLoop(){
        switch(intakeState){
            case FLIP_UP:
                fin = 0;
                break;
            case INTAKE:
                fin = 0.919;
                intakePower = 0.22;
                if(isCorrectColor()){
                    intakePower = 0;
                    intakeState = IntakeState.FLIP_UP;
                }
                break;
            case DEPOSIT:
                intakePower = -1;
                if(!isCorrectColor()){
                    intakePower = 0;
                    intakeState = IntakeState.FLIP_UP;
                }
                break;
            default:
                break;
        }
    }
    public void flipUp(){
        intakeState = IntakeState.FLIP_UP;
    }
    public void deposit(){
        intakeState = IntakeState.DEPOSIT;
    }
    public void flipDown(){
        intakeState = IntakeState.INTAKE;
    }
}
