import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
public class OuttakeLift {
    public DcMotorEx rlift;
    public DcMotorEx llift;
    private PIDController controller;
    public static double p = 0.05, i = 0, d = 0.000, f = 0.1;
    public static int target = 250;
    public final double ticks_in_degree = 145.1 / 360.0;
    public boolean usePID = true;
    public int GotLiftPos = 0;
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
        if (!usePID){
            rlift.setPower(-lopMode.gamepad2.left_stick_y/1.2);
            llift.setPower(-lopMode.gamepad2.left_stick_y/1.2);
            if (Math.abs(lopMode.gamepad2.left_stick_y) < 0.4){
                target = rlift.getCurrentPosition();
            }
        } else {
            controller.setPID(p, i, d);
            if (target > 1120){
                target=1120;
            } else if (target < 25){
                target = 25;
            }
            int liftPos = rlift.getCurrentPosition();
            double pid = controller.calculate(liftPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;
            double power = pid + ff;
            rlift.setPower(power);
            llift.setPower(power);
        }
        usePID = !(Math.abs(lopMode.gamepad2.left_stick_y)>0.3);
    }
    public void LiftTarget(int input){
        target = input;
    }
    public void GetLiftPos(){
        GotLiftPos = rlift.getCurrentPosition();
    }
    public void Auton(){
        usePID = true;
    }

}
