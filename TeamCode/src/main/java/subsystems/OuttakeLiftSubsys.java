package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class OuttakeLiftSubsys extends SubsysCore{
    DcMotorEx llift, rlift, clift;
    DigitalChannel touchSensor;

    private PIDController controller;
    public static double p = 0.0087, i = 0, d = 0.00015, f = 0.05;

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
        hanging = false;
    }
    public int getCurrentPosition(){
        return (llift.getCurrentPosition() + rlift.getCurrentPosition())/2;
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
        BACK_PICKUP_WAIT, TRANSFER, LIFT_BUCKET, FRONT_SCORE, FRONT_SCORE_WAIT_OLD, FRONT_SCORE_DONE_OLD, FRONT_PICKUP, BACK_SCORE, BACK_PICKUP, RESET_ENCODER, AVOID_INTAKE,
        LOW_BAR_WAIT, LOW_BAR_DONE,
        HIGH_BAR_WAIT, HIGH_BAR_DONE
    }
    @Config
    public static class OuttakeLiftPositionsCONFIG {
        public static int TRANSFER_POS = 0;
        public static int BUCKET_POS = 2900;
        public static int FRONT_SCORE_WAIT_POS = 1720;
        public static int FRONT_SCORE_DONE_POS = 2320;
        public static int FRONT_PICKUP_POS = 0;
        public static int BACK_SCORE_POS = 780;
        public static int BACK_PICKUP_POS = 0;
        public static int BACK_PICKUP_WAIT_POS = 600;

        public static int FRONT_SCORE = 400;
        public static int LOW_BAR_WAIT = 1750;
        public static int LOW_BAR_DONE = 1166;
        public static int HIGH_BAR_WAIT = 3350;
        public static int HIGH_BAR_DONE = 0;
    }
    public void LiftTo(OuttakeLiftPositions input){
        switch(input){
            case TRANSFER:
                target = OuttakeLiftPositionsCONFIG.TRANSFER_POS;
                break;
            case LIFT_BUCKET:
                target = OuttakeLiftPositionsCONFIG.BUCKET_POS;
                break;
            case FRONT_SCORE_WAIT_OLD:
                target = OuttakeLiftPositionsCONFIG.FRONT_SCORE_WAIT_POS;
                break;
            case FRONT_SCORE_DONE_OLD:
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
            case BACK_PICKUP_WAIT:
                target = OuttakeLiftPositionsCONFIG.BACK_PICKUP_WAIT_POS;
                break;
            case LOW_BAR_WAIT:
                target = OuttakeLiftPositionsCONFIG.LOW_BAR_WAIT;
                break;
            case LOW_BAR_DONE:
                target = OuttakeLiftPositionsCONFIG.LOW_BAR_DONE;
                break;
            case HIGH_BAR_WAIT:
                target = OuttakeLiftPositionsCONFIG.HIGH_BAR_WAIT;
                break;
            case HIGH_BAR_DONE:
                target = OuttakeLiftPositionsCONFIG.HIGH_BAR_DONE;
                break;
            case FRONT_SCORE:
                target = OuttakeLiftPositionsCONFIG.FRONT_SCORE;
                break;
            default:
                break;
        }
    }
    public boolean isBusy(){
        return Math.abs(target - getCurrentPosition()) <= 12;
    }

    private static int prevTarget = 0;
    private boolean encoderReset = true;

    public boolean hanging = false;
    public void useHang(){
        hanging = true;
    }
    public void stopHang(){
        hanging = false;
    }
    public static double hp = 0.009, hi = 0, hd = 0, hf = -0.000055;
    public boolean doneHanging = false;
    public static double doneHangingPower = -0.5;

    public void holdLift(){
        double power;
        if(!touchSensor.getState()){
            if(getCurrentPosition() != 0){
                setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        if (Math.abs(opMode.gamepad2.left_stick_y)>0.1) {
            // Manual Takeover. Disable PID or limits
            power = -opMode.gamepad2.left_stick_y;
            target = getCurrentPosition();
        } else if(target == 0 && (target != prevTarget || !encoderReset)) {
            if (target != prevTarget) encoderReset = false;
            power = -1;
            if (!touchSensor.getState()) {
                encoderReset = true;
                if(hanging) doneHanging = true;
            }
        } else if(doneHanging){
            power = doneHangingPower;
        }else if(hanging){
            controller.setPIDF(hp, hi, hd, hf);
            power = controller.calculate(getCurrentPosition(), target);
        } else {
            // PIDF Controller
            controller.setPIDF(p, i, d, 0);
            double pid = controller.calculate(getCurrentPosition(), target);
            double ticks_in_degree = 145.1 / 360.0;
            double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
            power = pid + ff;
        }

        prevTarget = target;
        setPower(power);
    }

    @Override
    public void loop(){
        tel.addData("Target", target);
        tel.addData("Current Lift Position", getCurrentPosition());
        tel.addData("Left Lift Position", llift.getCurrentPosition());
        tel.addData("Right Lift Position", rlift.getCurrentPosition());
        tel.addData("Center Lift Position", clift.getCurrentPosition());
        tel.addData("Llift Power", llift.getPower());
        tel.addData("Rlift Power", rlift.getPower());
        tel.addData("Clift Power", clift.getPower());
        tel.addData("Clift Current:", clift.getCurrent(CurrentUnit.AMPS));
        tel.addData("Limit Switch Pressed?", !touchSensor.getState());
    }
}
