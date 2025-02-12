import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.commands.core.LynxResetMotorEncoderCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
public class OuttakeLift {
    public DcMotorEx llift1, llift2, rlift1, rlift2;
    public TouchSensor touchSensor;
    private PIDController controller;

    private static boolean FOUR_MOTOR_LIFT = false;

    public static double p = 0.045, i = 0, d = 0.000, f = 0.1;
    public static int target = 250;

    OpMode lopMode;
    public OuttakeLift(HardwareMap hardwareMap, OpMode opMode) {
        llift1 = hardwareMap.get(DcMotorEx.class, "llift1");
        rlift1 = hardwareMap.get(DcMotorEx.class, "rlift1");
        llift2 = hardwareMap.get(DcMotorEx.class, "llift2");
        rlift2 = hardwareMap.get(DcMotorEx.class, "rlift2");

        llift1.setDirection(DcMotor.Direction.REVERSE);
        rlift1.setDirection(DcMotor.Direction.FORWARD);
        llift2.setDirection(DcMotor.Direction.REVERSE);
        rlift2.setDirection(DcMotor.Direction.FORWARD);

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
        return (llift1.getCurrentPosition() + llift2.getCurrentPosition() + rlift1.getCurrentPosition() + rlift2.getCurrentPosition()) / 4;
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
            target = 30;
        }

        double power;
        if (Math.abs(lopMode.gamepad2.left_stick_y)>0.1){
            // Manual Takeover. Disable PID or limits
            power = -lopMode.gamepad2.left_stick_y;
            target = getCurrentPosition();
        } else {
            // PIDF Controller
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
        TRANSFER_WAIT,
        TRANSFER_GRAB,
        LIFT_BUCKET,
        FRONT_SCORE_WAIT,
        FRONT_SCORE_DONE,
        FRONT_PICKUP,
        BACK_SCORE,
        BACK_PICKUP
    }
    public void LiftTo(OuttakeLiftPositions input){
        switch(input){
            case TRANSFER_WAIT:
                target = 0;
                break;
            case TRANSFER_GRAB:
                target = 0;
                break;
            case LIFT_BUCKET:
                target = 1200;
                break;
            case FRONT_SCORE_WAIT:
                target = 0;
                break;
            case FRONT_SCORE_DONE:
                target = 450;
                break;
            case FRONT_PICKUP:
                target = 200;
                break;
            case BACK_SCORE:
                target = 450;
                break;
            case BACK_PICKUP: //doesnt exist
                target = 200;
                break;
            default:
                break;
        }
    }
    public boolean isBusy(){
        return Math.abs(target - getCurrentPosition()) <= 4;
    }
}
