import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
public class OuttakeLift {
    public DcMotorEx rlift;
    public DcMotorEx llift;
    private PIDController controller;
    private static double p = 0.045, i = 0, d = 0.000, f = 0.1;
    private int target = 250;

    public static int TRANSFER_WAIT = 500;
    public static int TRANSFER_GRAB = 290;
    public static int LIFT_BUCKET = 1200;


    OpMode lopMode;
    public OuttakeLift(HardwareMap hardwareMap, OpMode opMode) {
        rlift = hardwareMap.get(DcMotorEx.class, "rlift");
        llift = hardwareMap.get(DcMotorEx.class, "llift");
        llift.setDirection(DcMotor.Direction.REVERSE);
        rlift.setDirection(DcMotor.Direction.FORWARD);
        llift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        llift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        llift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDController(p, i, d);
        this.lopMode = opMode;
    }
    public void HoldLift(){
        if (Math.abs(lopMode.gamepad2.left_stick_y)>0.3){
            // Manual Takeover. Disable PID or limits
            rlift.setPower(-lopMode.gamepad2.left_stick_y);
            llift.setPower(-lopMode.gamepad2.left_stick_y);
            target = rlift.getCurrentPosition();
        } else {
            // PIDF Controller
            controller.setPID(p, i, d);
            if (target > 1125) target = 1125;
            else if (target < 25) target = 25;
            int liftPos = rlift.getCurrentPosition();
            double pid = controller.calculate(liftPos, target);
            double ticks_in_degree = 145.1 / 360.0;
            double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
            double power = pid + ff;
            rlift.setPower(power);
            llift.setPower(power);
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
                target = 500;
                break;
            case TRANSFER_GRAB:
                target = 290;
                break;
            case LIFT_BUCKET:
                target = 1200;
                break;
            case FRONT_SCORE_WAIT:
                target = 450;
                break;
            case FRONT_SCORE_DONE:
                target = 700;
                break;
            case FRONT_PICKUP:
                target = 200;
                break;
            case BACK_SCORE:
                target = 900;
                break;
            case BACK_PICKUP:
                target = 600;
                break;
            default:
                break;
        }
    }
    public boolean isBusy(){
        return Math.abs(target - getCurrentPosition()) <= 4;
    }
    public void LiftToTransferWait(){
        target = TRANSFER_WAIT;
    }
    public void LiftToTransferGrab(){
        target = TRANSFER_GRAB;
    }
    public void LiftToBucket(){
        target = LIFT_BUCKET;
    }
    public int getCurrentPosition(){
        return rlift.getCurrentPosition();
    }
}
