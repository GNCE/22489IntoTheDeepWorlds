

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
    public Servo leintake;
    public Servo reintake;
    ColorSensor colorSensor;

    OpMode opMode;

    public enum IntakeState {
        FLIP_UP,
        INTAKE,
        DEPOSIT,
        SHOOT,
        TRANSFER,
    }

    IntakeState intakeState = IntakeState.FLIP_UP;
    double fin = 0;
    double intakePower = 0;


    /** LINKAGE EXTENSION VARIABLES */
    double extPos = 0;
    // Length of first linkage (Linkage that connects to servo) (mm)
    final double LINK1 = 200;
    // Length of second linkage (Linkage that connects to the slide) (mm)
    final double LINK2 = 300;
    // Offset X axis (CURRENT VALUE IS CORRECT)
    final double XOFFSET = 98;
    // Offset Y axis (CURRENT VALUE IS CORRECT)
    final double YOFFSET = 17.25;
    // Length of the slides when fully extended (mm)
    final double FULL_EXTENSION = 1000;
    // Default servo angle

    int depo =0;
    boolean useRTP = true;
    public Intake(HardwareMap hardwareMap, OpMode opMode) {
        rintake = hardwareMap.get(CRServo.class, "rintake");
        lintake = hardwareMap.get(CRServo.class, "lintake");
        finintake = hardwareMap.get(Servo.class, "fintake");
        rintake.setDirection(CRServo.Direction.FORWARD);
        lintake.setDirection(CRServo.Direction.REVERSE);
        finintake.setDirection(Servo.Direction.REVERSE);

        leintake = hardwareMap.get(Servo.class, "leintake");
        reintake = hardwareMap.get(Servo.class, "reintake");
        reintake.setDirection(Servo.Direction.FORWARD);
        leintake.setDirection(Servo.Direction.REVERSE);

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        colorSensor.enableLed(true);
        this.opMode = opMode;
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
        return isYellow() || (Storage.isRed && isRed()) || (!Storage.isRed && isBlue());
    }
    private double getServoAngleWithLength(double l1, double l2, double l3, double xo, double yo, int servoRange){
        // All units are mm and degrees.
        double beta = Math.toDegrees(Math.acos((Math.pow(l1, 2) - Math.pow(l2, 2) + Math.pow(xo + l3, 2) + Math.pow(yo, 2))/(2*l1*Math.sqrt(Math.pow(xo+l3, 2) + Math.pow(yo, 2)))));
        double gamma = Math.toDegrees(Math.atan((xo+l3)/yo));
        return (180 - beta - gamma)/servoRange;
    }
    private void intakeExtendTo(double length){
        double targetPos = getServoAngleWithLength(LINK1, LINK2, length, XOFFSET, YOFFSET, 360*5);
        leintake.setPosition(targetPos);
        reintake.setPosition(targetPos);
    }
    public void setIntakeExtensionTarget(double target){
        if(target > FULL_EXTENSION) target = FULL_EXTENSION;
        else if(target < 0) target = 0;
        extPos = target;
    }
    public void initiate(){
        intakeExtendTo(0);
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
        intakeExtendTo(extPos);
    }
    public void ManualExtend(){
        setIntakeExtensionTarget(FULL_EXTENSION);
    }
    public void ManualRetract(){
        setIntakeExtensionTarget(0);
    }
    public void TeleopExtend(double valueFromZeroToOne){
        if(valueFromZeroToOne < 0) valueFromZeroToOne = 0;
        else if(valueFromZeroToOne > 1) valueFromZeroToOne = 1;
        setIntakeExtensionTarget(valueFromZeroToOne * FULL_EXTENSION);
    }
    private final double INTAKE_DOWN_POS = 0.919;
    private final double INTAKE_TRANSFER_POS = 0.5;
    public void intakeLoop(){
        switch(intakeState){
            case FLIP_UP:
                fin = 0;
                break;
            case INTAKE:
                fin = INTAKE_DOWN_POS;
                intakePower = 0.22;
                if((Math.abs(finintake.getPosition() - INTAKE_DOWN_POS) < 0.005) && isCorrectColor()){
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
            case SHOOT:
                fin = INTAKE_DOWN_POS;
                if(Math.abs(finintake.getPosition() - INTAKE_DOWN_POS) < 0.005){
                    intakePower = -1;
                    if(!isCorrectColor()){
                        intakePower = 0;
                        intakeState = IntakeState.FLIP_UP;
                    }
                }
            case TRANSFER:
                fin = INTAKE_TRANSFER_POS;
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
    public void startIntake(){
        intakeState = IntakeState.INTAKE;
    }
    public void shootOut(){
        intakeState = IntakeState.SHOOT;
    }
    public void flipToTransfer(){
        intakeState = IntakeState.TRANSFER;
    }
}
