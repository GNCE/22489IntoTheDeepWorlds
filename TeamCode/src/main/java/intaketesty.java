import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp (name = "intakey")
public class intaketesty extends OpMode {
    private Intake intake;

    public void init() {
        intake = new Intake(hardwareMap,this);
    }
    public void start(){
        intake.initiate();
    }

    public void loop() {
        if (gamepad1.left_bumper){
            intake.flipDown();
        }
        intake.check();
        intake.TeleopExtend(); //left trigger
        if (gamepad1.right_trigger > 0.2){
            intake.flipUp();
            intake.deposit();
        }
        intake.moveThings();
        telemetry.addData("red color", intake.colorSensor.red());
        telemetry.addData("blue color", intake.colorSensor.blue());
        telemetry.addData("green (yellow) color", intake.colorSensor.green());
        telemetry.addData("ip",intake.intakePower);
        telemetry.addData("fin",intake.fin);
        telemetry.addData("ex",intake.ex);
        telemetry.update();
    }
}
