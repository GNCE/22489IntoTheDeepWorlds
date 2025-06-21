package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.List;

import java.util.Arrays;

@Config
public class IntakeLimelightSubsys extends SubsysCore {
    public static Limelight3A ll;
    public static Servo light;
    public static UnifiedTelemetry tel;
    public static LLResult llResult;
    private static double tx, ty, ta, angle;
    private static double[] pythonOutput;
    private static int pipelineNumber=4;
    private double[] pythonInputs = {0, 0, 0, 0, 0, 0, 0, 0};
    private boolean prevllon = false, llon = false;
    public boolean isDataFresh(){ return llResult != null && llResult.getStaleness() < 400; }

    public boolean isRunning(){ return llon; }
    public void turnOn(){ llon = true; }
    public void turnOff(){ llon = false; }
    public static double horizScale = 0.8;
    public static double horizOffset = 0.3;
    public static double vertOffset = 0.2;
    public static double vertScale = 27;

    // getTx() * 12.0285 + 12.3725
    // getTy() * -0.3546 -0.321

    public boolean isResultValid(){
        if(!llon) return false;
        if(!isDataValid()) return false;
        return isDataFresh() && isDataValid();
    }

    public enum Alliance{
        RED, BLUE
    }

    public enum SampleType{
        ALLIANCE, BOTH, NEUTRAL
    }


    public void setAlliance(Alliance a){
        pythonInputs[0] = a.ordinal() + 1;
    }

    public void setSampleType(SampleType s){
        pythonInputs[1] = s.ordinal() + 1;
    }


    public double getTa() { return ta; }
    public double getTx(){ return tx; }
    public double getTy(){ return ty; }
    public double getAngle(){ return angle; }
    private boolean isDataValid(){
        return !(getTx() == 0 && getTy() == 0);
    }
    public double getVert(){ return ((getTx()*0.198 + 5.41)*2.25)*vertScale + vertOffset; } // inches
    public double getHoriz(){ return ((getTy()*(-0.197) -0.345)*2.25)*horizScale + horizOffset; } // inches
    // getTx() * 12.0285 + 12.3725
    // getTy() * -0.3546 -0.321


    public void setPipelineNumber(int pipelineNum){
        if(pipelineNum > 6 || pipelineNum < 4) return;
        pipelineNumber = pipelineNum;
    }
    public void captureSnapshot(String key){
        ll.captureSnapshot(key);
    }

    public int getPipelineNumber(){ return pipelineNumber; }

    @Override
    public void init(){
        ll = hardwareMap.get(Limelight3A.class, "limelight");
        light = hardwareMap.get(Servo.class, "light");
        tel = new UnifiedTelemetry();
        ll.setPollRateHz(70);
        light.setPosition(0);
        turnOff();
    }

    private boolean connected = false;
    public boolean isConnected(){ return this.connected; }

    private boolean dataNull = false;
    @Override
    public void loop(){
        if(prevllon != llon){
            if(llon) ll.start();
            else ll.pause();
        }

        ll.updatePythonInputs(pythonInputs);

        if(llon){
            // light.setPosition(1);
            ll.pipelineSwitch(pipelineNumber);
            llResult = ll.getLatestResult();
            if(llResult != null){
                tx = llResult.getTx();
                ty = llResult.getTy();
                ta = llResult.getTa();

                pythonOutput = llResult.getPythonOutput();
                if(pythonOutput != null){
                    if(pythonOutput.length > 0) angle = -pythonOutput[0];
                    dataNull = false;
                } else {
                    dataNull = true;
                }
            }
        } else {
            // light.setPosition(0);
        }

        prevllon = llon;
        connected = ll.isConnected();

        tel.addLine("Limelight Data");
        tel.addData("Connected?", this.isConnected());
        tel.addData("Running", this.isRunning());
        if(this.isRunning() && this.isConnected()){
            tel.addData("Raw Tx", getTx());
            tel.addData("Raw Ty", getTy());
            tel.addData("Raw Ta", getTa());
            tel.addData("Raw Angle", getAngle());
            tel.addData("Pipeline:", getPipelineNumber());
            tel.addData("Data Valid", isResultValid());
            tel.addData("Data Fresh", isDataFresh());
            tel.addData("Detected X", getTx());
            tel.addData("Detected Y", getTy());
            tel.addData("Detected Area", getTa());
            tel.addData("Detected Angle", getAngle());
        }
    }
}