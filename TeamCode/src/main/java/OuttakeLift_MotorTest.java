import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = " outtake motor test")
public class OuttakeLift_MotorTest extends OpMode {
    private DcMotor rlift1;
    private DcMotor llift1;
    private DcMotor llift2;
    public void init(){
        llift1 = hardwareMap.get(DcMotorEx.class, "llift");
        rlift1 = hardwareMap.get(DcMotorEx.class, "rlift");
        llift2 = hardwareMap.get(DcMotorEx.class, "clift");

        llift1.setDirection(DcMotor.Direction.FORWARD);
        rlift1.setDirection(DcMotor.Direction.REVERSE);
        llift2.setDirection(DcMotor.Direction.REVERSE);

        llift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        llift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        llift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        llift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        llift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rlift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        llift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void loop(){
        rlift1.setPower(-gamepad2.right_stick_y);
        llift1.setPower(-gamepad2.right_stick_y);
        llift2.setPower(-gamepad2.right_stick_y);
    }
}
