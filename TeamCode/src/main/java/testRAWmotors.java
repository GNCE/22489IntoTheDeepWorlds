import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@TeleOp (name = "motor test")
public class testRAWmotors extends OpMode {
    private DcMotor rlift1;
    private DcMotor llift1;
    private DcMotor rlift2;
    private DcMotor llift2;
    public void init(){
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
    }
    public void loop(){
        rlift1.setPower(-gamepad2.left_stick_y);
        llift1.setPower(-gamepad2.left_stick_y);
        if (gamepad1.right_bumper) {
            rlift2.setPower(-gamepad2.left_stick_y);
            llift2.setPower(-gamepad2.left_stick_y);
        }
    }
}
