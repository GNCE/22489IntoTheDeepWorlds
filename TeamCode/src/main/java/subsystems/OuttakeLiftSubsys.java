package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class OuttakeLiftSubsys extends SubsysCore{
    DcMotorEx llift, rlift, clift;
    DigitalChannel touchSensor;

    private PIDController controller;
    public static double p = 0.012, i = 0, d = 0.00023, f = 0.05;

    public static int target = 0;
    private static UnifiedTelemetry tel = new UnifiedTelemetry();
    private static double internalPrevPower = 0;
    private static DcMotor.RunMode internalPrevRunMode = null;
    private static DcMotor.ZeroPowerBehavior internalPrevZPB = null;

    @Override
    public void init(){
        llift = hardwareMap.get(DcMotorEx.class, "llift");
        rlift = hardwareMap.get(DcMotorEx.class, "rlift");
        clift = hardwareMap.get(DcMotorEx.class, "clift");
        llift.setDirection(DcMotorSimple.Direction.FORWARD);
        rlift.setDirection(DcMotorSimple.Direction.REVERSE);
        clift.setDirection(DcMotorSimple.Direction.REVERSE);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        touchSensor = hardwareMap.get(DigitalChannel.class, "sensor_touch");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        controller = new PIDController(p, i, d);
    }
    public int getCurrentPosition(){
        return (llift.getCurrentPosition() + clift.getCurrentPosition())/2;
    }

    public void setPower(double power){
        if(internalPrevPower == power) return;
        internalPrevPower = power;
        llift.setPower(power);
        rlift.setPower(power);
        clift.setPower(power);
    }

    public void setMode(DcMotor.RunMode runMode){
        if(internalPrevRunMode == runMode) return;
        internalPrevRunMode = runMode;
        llift.setMode(runMode);
        rlift.setMode(runMode);
        clift.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        if(internalPrevZPB == zeroPowerBehavior) return;
        internalPrevZPB = zeroPowerBehavior;
        llift.setZeroPowerBehavior(zeroPowerBehavior);
        rlift.setZeroPowerBehavior(zeroPowerBehavior);
        clift.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public enum OuttakeLiftPositions {
        TRANSFER, LIFT_BUCKET, FRONT_SCORE_WAIT, FRONT_SCORE_DONE, FRONT_PICKUP, BACK_SCORE, BACK_PICKUP, RESET_ENCODER, AVOID_INTAKE
    }
    @Config
    public static class OuttakeLiftPositionsCONFIG {
        public static int TRANSFER_POS = 0;
        public static int BUCKET_POS = 2000;
        public static int FRONT_SCORE_WAIT_POS = 0;
        public static int FRONT_SCORE_DONE_POS = 0;
        public static int FRONT_PICKUP_POS = 0;
        public static int BACK_SCORE_POS = 540;
        public static int BACK_PICKUP_POS = 0;
    }
    public void LiftTo(OuttakeLiftPositions input){
        switch(input){
            case TRANSFER:
                target = OuttakeLiftPositionsCONFIG.TRANSFER_POS;
                break;
            case LIFT_BUCKET:
                target = OuttakeLiftPositionsCONFIG.BUCKET_POS;
                break;
            case FRONT_SCORE_WAIT:
                target = OuttakeLiftPositionsCONFIG.FRONT_SCORE_WAIT_POS;
                break;
            case FRONT_SCORE_DONE:
                target = OuttakeLiftPositionsCONFIG.FRONT_SCORE_DONE_POS;
                break;
            case FRONT_PICKUP:
                target = OuttakeLiftPositionsCONFIG.FRONT_PICKUP_POS;
                break;
            case BACK_SCORE:
                target = OuttakeLiftPositionsCONFIG.BACK_SCORE_POS;
                break;
            case BACK_PICKUP:
                target = OuttakeLiftPositionsCONFIG.BACK_PICKUP_POS;
                break;
            case RESET_ENCODER:
                target = 0;
                break;
            case AVOID_INTAKE:
                target = 500;
                break;
            default:
                break;
        }
    }
    public boolean isBusy(){
        return Math.abs(target - getCurrentPosition()) <= 4;
    }

    private static int prevTarget = 0;
    private boolean encoderReset = true;

    public void holdLift(){
        double power;
        if (Math.abs(opMode.gamepad2.left_stick_y)>0.1) {
            // Manual Takeover. Disable PID or limits
            power = -opMode.gamepad2.left_stick_y;
            target = getCurrentPosition();
        } else if(target == 0 && (target != prevTarget || !encoderReset)) {
            if(target != prevTarget) encoderReset = false;
            power = -1;
            if(touchSensor.getState() || clift.getCurrent(CurrentUnit.AMPS) > 10){
                setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                encoderReset = true;
            }
        } else {
            // PIDF Controller
            controller.setPID(p, i, d);
            double pid = controller.calculate(getCurrentPosition(), target);
            double ticks_in_degree = 145.1 / 360.0;
            double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
            power = pid + ff;
        }

        setPower(power);
    }

    @Override
    public void loop(){
        tel.addData("Target", target);
        tel.addData("Current Lift Position", getCurrentPosition());
        tel.addData("Left Lift Position", llift.getCurrentPosition());
        tel.addData("Right Lift Position", rlift.getCurrentPosition());
        tel.addData("Center Lift Position", clift.getCurrentPosition());
        tel.addData("Clift Current:", clift.getCurrent(CurrentUnit.AMPS));
        tel.addData("Limit Switch Pressed?", touchSensor.getState());
    }
}
