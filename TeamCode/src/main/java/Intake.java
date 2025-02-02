

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake{
    public Servo finintake;
    public CRServo lintake;
    public CRServo rintake;
    public DcMotor extendo;
    public NormalizedColorSensor colorSensor;
    public float[] hsvValues = new float[3];

    OpMode opMode;

    public enum IntakeState {
        FLIP_UP,
        INTAKE,
        DEPOSIT_AND_FLIP,
        DEPOSIT_ONLY,
        SHOOT,
    }

    IntakeState intakeState = IntakeState.FLIP_UP;
    double fin = 0;
    double intakePower = 0;
    double ex = 0;
    int depo =0;
    boolean useRTP = true;

    public NormalizedRGBA sensedColor;

    public Intake(HardwareMap hardwareMap, OpMode opMode) {
        rintake = hardwareMap.get(CRServo.class, "rintake");
        lintake = hardwareMap.get(CRServo.class, "lintake");
        finintake = hardwareMap.get(Servo.class, "fintake");
        rintake.setDirection(CRServo.Direction.FORWARD);
        lintake.setDirection(CRServo.Direction.REVERSE);
        finintake.setDirection(Servo.Direction.REVERSE);
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        if(colorSensor instanceof SwitchableLight){
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        this.opMode = opMode;
        extendo = hardwareMap.get(DcMotor.class, "extendo");
        extendo.setDirection(DcMotor.Direction.FORWARD);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // when up, 300 280 400
    private boolean isRed(){
        return hsvValues[1] > 0.2 && hsvValues[0] < 34;
    }
    private boolean isYellow(){
        return hsvValues[1] > 0.2 && hsvValues[0] > 75 && hsvValues[0] <=95;
    }
    private boolean isBlue(){
        return hsvValues[1] > 0.2 && hsvValues[0] > 200 && hsvValues[0] < 245;
    }
    private boolean isClose(){
        return distance < 10;
    }

    public boolean isCorrectColor(){
        return (isYellow() || (Storage.isRed && isRed()) || (!Storage.isRed && isBlue()));
    }
    public String getSensedColorName(){
        //if(!isClose()) return "NONE";
        if(isYellow()) return "YELLOW";
        if(isRed()) return "RED";
        if(isBlue()) return "BLUE";
        return "NONE";
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

    private final double INTAKE_DOWN_POS = 0.919;
    public double distance;
    public void intakeLoop(){
        sensedColor = colorSensor.getNormalizedColors();
        Color.colorToHSV(sensedColor.toColor(), hsvValues);
        distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        switch(intakeState){
            case FLIP_UP:
                intakePower = 0;
                fin = 0;
                break;
            case INTAKE:
                fin = INTAKE_DOWN_POS;
                intakePower = 0.22;
                if(isCorrectColor()){
                    intakeState = IntakeState.FLIP_UP;
                }
                break;
            case DEPOSIT_AND_FLIP:
                intakePower = -1;
                if(!isCorrectColor()){
                    intakeState = IntakeState.FLIP_UP;
                }
                break;
            case DEPOSIT_ONLY:
                intakePower = -1;
                break;
            case SHOOT:
                fin = INTAKE_DOWN_POS;
                if(Math.abs(finintake.getPosition() - INTAKE_DOWN_POS) < 0.005){
                    intakePower = -1;
                    if(!isCorrectColor()){
                        intakeState = IntakeState.FLIP_UP;
                    }
                }
            default:
                break;
        }
    }
    public void flipUp(){
        intakeState = IntakeState.FLIP_UP;
    }
    public void depositandflip(){
        intakeState = IntakeState.DEPOSIT_AND_FLIP;
    }
    public void depositOnly(){
        intakeState = IntakeState.DEPOSIT_ONLY;
    }
    public void startIntake(){
        intakeState = IntakeState.INTAKE;
    }
    public void shootOut(){
        intakeState = IntakeState.SHOOT;
    }
    public IntakeState getIntakeState(){
        return intakeState;
    }
}
