package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="testMovement")
public class TestMovement extends LinearOpMode {
    private DcMotor backLeft= null;
    private DcMotor backRight= null;
    private DcMotor frontLeft= null;
    private DcMotor frontRight= null;
    @Override
    public void runOpMode(){
        backLeft     = hardwareMap.get(DcMotor.class,"backLeft");
        backRight   = hardwareMap.get(DcMotor.class,"backRight");
        frontLeft   = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        waitForStart();
        while(opModeIsActive()){
            double move = -gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;
            double strafe = (gamepad1.right_trigger - gamepad1.left_trigger)*-1;

            backLeft.setPower(Range.clip(move+turn+strafe,-1,1));
            backRight.setPower(Range.clip(move-turn-strafe,-1,1));
            frontLeft.setPower(Range.clip(move+turn-strafe,-1,1));
            frontRight.setPower(Range.clip(move-turn+strafe ,-1,1));



        }
    }
}
