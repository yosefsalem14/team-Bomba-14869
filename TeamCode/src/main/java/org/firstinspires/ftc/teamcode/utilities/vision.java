
package org.firstinspires.ftc.teamcode.utilities;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;


public class vision {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "Ae31TOj/////AAABmU6qla4qL0GxgNm3Sj+5YWQXbuPYfw459YaMe+A/w13jJTx4ZE0ArONrBmezY2yfFQB4wsBQzmBif8AFghBNyvrIFTCjzGUePnyNDwghP5UZt1ed/yUkkK3PmGpxHQ8VLJvLogJ/2/MvG6g+H9mrvIWSnurJSnkNbr96s4umRABLLkhBSCd/mN2HwbUkXL9xNpipvdeNqWIrpH4Mt04aa81wgFQBxhCbqFZYwEfM+VnGnttvCIx1tcI1YI0ktylCCL3lwXPPCNNdbs89xrJamIMB7zrKNEJ/S08mKCCDKJO2W2o2yK3JzbdWNXh//fsEm5N+OfZKgBjGz8IM082RLQT7RZPVhKGJyl6D2AniDyVF";


    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;
    private HardwareMap hw;
    private int runs;
    public vision(HardwareMap hw){
        this.runs = 0;
        this.hw = hw;
    }

    public void init() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
    }

    public int sign(double num){
        if(num>0){
            return 1;
        }else if(num<0){
            return -1;
        }
        return 0;
    }
    public int getIter(){
        return this.runs;
    }
    public void activate(){
        if (tfod != null) {
            tfod.activate();
        }
    }
    public int getPos() {
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (int x = 0; x < updatedRecognitions.size(); x++) {
                    Recognition recognition = updatedRecognitions.get(x);
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        return this.getSign(this.estimateAngle(recognition,
                                AngleUnit.DEGREES));
                    }
                }
            }
        }
        return -1;
    }
    public void shutdown(){
        if(tfod!=null){
            tfod.shutdown();
        }
    }
    public int getSign(double num){
        if(num<10 && num > -10){
            return 0;
        }
        if(num>10){
            return 1;
        }
        return -1;
    }
    public double estimateAngle(Recognition R,AngleUnit angleUnit) {
        float focalLength = vuforia.getCameraCalibration().getFocalLength().getData()[0];
        double adjacentSideLength = focalLength;

        double oppositeSideLength = ((R.getTop()+R.getBottom()) * 0.5f) - (0.5f * R.getImageHeight());

        double tangent = oppositeSideLength / adjacentSideLength;
        double angle = angleUnit.fromRadians(Math.atan(tangent));
        return angle;
    }





    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = this.hw.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", this.hw.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();// tfodMonitorViewId;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
