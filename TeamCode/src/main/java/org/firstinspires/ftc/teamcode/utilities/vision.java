
package org.firstinspires.ftc.teamcode.utilities;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.CameraMode;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.FrameFormat;
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
                if (updatedRecognitions.size() == 3) {
                    this.runs++;
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            return 1;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            return 3;
                        } else {
                            return 2;
                        }
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






    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = this.hw.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", this.hw.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
