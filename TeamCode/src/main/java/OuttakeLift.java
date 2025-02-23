import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class OuttakeLift {
    public DcMotorEx llift1, llift2, rlift1, rlift2;
    public TouchSensor touchSensor;
    private PIDController controller;

    public static boolean FOUR_MOTOR_LIFT = true;

    public static double p = 0.013, i = 0, d = 0.00023, f = 0.05;
    public int target = 0;

    OpMode lopMode;
    public OuttakeLift(HardwareMap hardwareMap, OpMode opMode) {
        llift1 = hardwareMap.get(DcMotorEx.class, "llift1");
        rlift1 = hardwareMap.get(DcMotorEx.class, "rlift1");
        llift2 = hardwareMap.get(DcMotorEx.class, "llift2");
        rlift2 = hardwareMap.get(DcMotorEx.class, "rlift2");

        llift1.setDirection(DcMotor.Direction.FORWARD);
        rlift1.setDirection(DcMotor.Direction.REVERSE);
        llift2.setDirection(DcMotor.Direction.FORWARD);
        rlift2.setDirection(DcMotor.Direction.REVERSE);

        llift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        llift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        llift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        llift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        llift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        llift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rlift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller = new PIDController(p, i, d);
        touchSensor = hardwareMap.get(TouchSensor.class, "sensor_touch");

        this.lopMode = opMode;
    }
    public int getCurrentPosition(){
        return (llift1.getCurrentPosition() + llift2.getCurrentPosition() + rlift1.getCurrentPosition()) / 3;
    }
    public int getTargetPosition(){
        return target;
    }
    public void HoldLift(){
        if(touchSensor.isPressed()){
            llift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rlift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            llift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rlift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            llift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rlift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            llift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rlift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(target < 50) target = 30;
        }

        double power;
        if (Math.abs(lopMode.gamepad2.left_stick_y)>0.1){
            // Manual Takeover. Disable PID or limits
            power = -lopMode.gamepad2.left_stick_y;
            target = getCurrentPosition();
        } else {
            // PIDF Controller
            controller.setPID(p, i, d);
            double pid = controller.calculate(getCurrentPosition(), target);
            double ticks_in_degree = 145.1 / 360.0;
            double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
            power = pid + ff;
        }

        llift1.setPower(power);
        rlift1.setPower(power);
        if(FOUR_MOTOR_LIFT){
            llift2.setPower(power);
            rlift2.setPower(power);
            llift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rlift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            llift2.setPower(0);
            rlift2.setPower(0);
            llift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rlift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    public void LiftTarget(int input){
        target = input;
    }
    public enum OuttakeLiftPositions {
        TRANSFER, LIFT_BUCKET, FRONT_SCORE_WAIT, FRONT_SCORE_DONE, FRONT_PICKUP, BACK_SCORE, BACK_PICKUP, RESET_ENCODER
    }

    public static int TRANSFER_POS = 480;
    public static int BUCKET_POS = 1550;
    public static int FRONT_SCORE_WAIT_POS = 0;
    public static int FRONT_SCORE_DONE_POS = 0;
    public static int FRONT_PICKUP_POS = 0;
    public static int BACK_SCORE_POS = 480;
    public static int BACK_PICKUP_POS = 0;


    public void LiftTo(OuttakeLiftPositions input){
        switch(input){
            case TRANSFER:
                target = TRANSFER_POS;
                break;
            case LIFT_BUCKET:
                target = BUCKET_POS;
                break;
            case FRONT_SCORE_WAIT:
                target = FRONT_SCORE_WAIT_POS;
                break;
            case FRONT_SCORE_DONE:
                target = FRONT_SCORE_DONE_POS;
                break;
            case FRONT_PICKUP:
                target = FRONT_PICKUP_POS;
                break;
            case BACK_SCORE:
                target = BACK_SCORE_POS;
                break;
            case BACK_PICKUP:
                target = BACK_PICKUP_POS;
                break;
            case RESET_ENCODER:
                target = -120;
            default:
                break;
        }
    }
    public boolean isBusy(){
        return Math.abs(target - getCurrentPosition()) <= 4;
    }
    public void setFourMotorLift(boolean newVal){
        FOUR_MOTOR_LIFT = newVal;
    }
}
