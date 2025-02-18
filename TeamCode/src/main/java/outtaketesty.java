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
            outtake.setOuttakeState(Outtake.OuttakeState.SPECFRONTPICKUP);
        }else if (gamepad1.x){
            outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
        }else if (gamepad1.a){
            outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
        }else if (gamepad1.b){
            outtake.setOuttakeState(Outtake.OuttakeState.SPECBACKSCORE);
        }
        outtake.setClawOpen(gamepad1.left_bumper);
        outtake.outtakeLoop();
        telemetry.update();
    }
}
