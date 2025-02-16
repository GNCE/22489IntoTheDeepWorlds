import com.qualcomm.robotcore.util.ElapsedTime;

public class Macros {
    private Intake intake;
    private OuttakeLift outtakeLift;
    private Outtake outtake;


    private enum TransferState {
        NOTHING,
        READY_OUTTAKE_LIFT,
        READY_OUTTAKE_ARM,
        OPEN_CLAW,
        CLOSE_CLAW,
        SCORE_BUCKET,
    };

    private ElapsedTime transferTime;
    private TransferState transferState = TransferState.NOTHING;
    public void initTransfer(){
        transferState = TransferState.READY_OUTTAKE_LIFT;
    }
    private void setTransferState(TransferState newState){
        transferState = newState;
        transferTime.reset();
    }
    public void sampleMacro(){
        switch (transferState){
            case READY_OUTTAKE_LIFT:
                if(outtakeLift.getCurrentPosition() < 300) outtakeLift.LiftTarget(300);
                if(!outtakeLift.isBusy()){
                    outtake.setOuttakeState(Outtake.OuttakeState.TRANSFER);
                    setTransferState(TransferState.READY_OUTTAKE_ARM);
                }
                break;
            case READY_OUTTAKE_ARM:
                if(transferTime.time() > 2){ // Time it takes for arm to pivot
                    outtake.setClawOpen(true);
                    setTransferState(TransferState.OPEN_CLAW);
                }
                break;
            case OPEN_CLAW:
                if(intake.getCurrentSampleState(false) == Intake.SENSOR_READING.CORRECT){
                    intake.setExtensionTarget(0);
//                    if(intake.isRetracted()){
//                        outtake.setClawOpen(false);
//                        setTransferState(TransferState.NOTHING);
//                    }
                }
                break;
            case CLOSE_CLAW:
                if(transferTime.time() > 0.7){
                    outtake.setOuttakeState(Outtake.OuttakeState.SAMPLESCORE);
                    outtakeLift.LiftTo(OuttakeLift.OuttakeLiftPositions.LIFT_BUCKET);
                    if(outtakeLift.isBusy()){
                        setTransferState(TransferState.SCORE_BUCKET);
                    }
                }
                break;
            case SCORE_BUCKET:
                if(transferTime.time() > 0.4){
                    outtake.setClawOpen(true);
                    if(transferTime.time() > 1){

                    }
                }
            default:
                break;
        }
    }
}
