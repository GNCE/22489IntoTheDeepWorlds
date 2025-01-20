import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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
        intake.TeleopExtend(); //left trigger
        //right bumper cancels flipdown
        if (gamepad1.right_trigger > 0.2){
            intake.deposit();
        }
        intake.moveThings();
        telemetry.addData("color", intake.colorSensor.red());
        telemetry.addData("ip",intake.ip);
        telemetry.addData("fin",intake.fin);
        telemetry.addData("ex",intake.ex);
        telemetry.update();
    }
}
