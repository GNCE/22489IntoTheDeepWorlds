import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "outakey")
public class outtaketesty extends OpMode {
    private Outtake outtake;

    public void init() {
        outtake = new Outtake(hardwareMap);
    }

    public void loop() {
        if (gamepad1.y){
            outtake.POS_SpecimanFront();
        }else if (gamepad1.x){
            outtake.POS_scoreSample();
        }else if (gamepad1.a){
            outtake.POS_Transfering();
        }else if (gamepad1.b){
            outtake.POS_scoreSpecimanBack();
        }
        outtake.setClawOpen(gamepad1.left_bumper);
        outtake.loop();
        telemetry.update();
    }
}
