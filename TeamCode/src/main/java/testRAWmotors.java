import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "motor test")
public class testRAWmotors extends OpMode {
    private DcMotor rlift;
    private DcMotor llift;
    private DcMotor extendo;
    public void init(){
        rlift = hardwareMap.get(DcMotorEx.class, "rlift");
        llift = hardwareMap.get(DcMotorEx.class, "llift");
        llift.setDirection(DcMotor.Direction.FORWARD);
        rlift.setDirection(DcMotor.Direction.REVERSE);
        extendo = hardwareMap.get(DcMotor.class, "extendo");
        extendo.setDirection(DcMotor.Direction.REVERSE);
    }
    public void loop(){
        rlift.setPower(-gamepad2.left_stick_y);
        llift.setPower(-gamepad2.left_stick_y);
        extendo.setPower(-gamepad2.right_stick_y);
    }
}
