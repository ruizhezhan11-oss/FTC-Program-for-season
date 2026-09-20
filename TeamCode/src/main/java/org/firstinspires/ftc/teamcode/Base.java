package org.firstinspires.ftc.teamcode;
import android.graphics.Canvas;
import android.util.Size;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.ArrayList;
import java.util.List;


@TeleOp
public class Base extends LinearOpMode {
    private DcMotor FL,FR,BL,BR;
    private AprilTagProcessor aprilTag;
    private VisionPortal camera;
    private IMU imu;
    private GoBildaPinpointDriver pinpoint;
    private void init_(){
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IMU imu=hardwareMap.get(IMU.class,"imu");//取代
        // 請根據 Control Hub 鎖在車上的實際方向調整 Logo 與 USB 轉向：
        // 範例：REV Logo 朝上 (UP)，USB 接口朝前 (FORWARD)
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
    }
    private void init_VI(){
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        camera = new VisionPortal.Builder()
                .setCameraResolution(new Size(720,480))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

    }
    @Override
    public  void runOpMode(){ //主迴圈
        init_VI();
        init_();
        waitForStart();
            while (opModeIsActive()){
            aprilTagTag();
                if (gamepad1.options) {
                    imu.resetYaw();
                }
            run_code();
            word_screen();
            telemetry.update();
            }
    }
    private void run_code() {

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double y = -applyDeadzone(gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x) * 1.1; // 1.1 補償橫移摩擦力
        double rx = applyDeadzone(gamepad1.right_stick_x);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double flPower = rotY + rotX + rx;
        double frPower = rotY - rotX - rx;
        double blPower = rotY - rotX + rx;
        double brPower = rotY + rotX - rx;

        // 馬達功率歸一化（避免超過 1.0）
        double max = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        FL.setPower(flPower);
        FR.setPower(frPower);
        BL.setPower(blPower);
        BR.setPower(brPower);
    }

    private void word_screen(){
        telemetry.addData("Heading",pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("X",pinpoint.getEncoderX());
        telemetry.addData("Y",pinpoint.getEncoderY());
    }
    private double applyDeadzone(double input) {
        return Math.abs(input) > 0.05 ? input : 0.0;
    }
    private void aprilTagTag(){  //辨識
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# Detected Tags", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // 已註冊於 TagLibrary 的官方標籤
                telemetry.addLine(String.format("\n==== Target ID %d (%s) ====", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ Pos : %6.1f %6.1f %6.1f (Inches)",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("RPY Deg : %6.1f %6.1f %6.1f (Degrees)",
                        detection.ftcPose.roll, detection.ftcPose.pitch, detection.ftcPose.yaw));
                telemetry.addLine(String.format("Rng/Brg/Yaw: %6.1f %6.1f %6.1f",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw));
            } else {
                // 未註冊的自訂標籤
                telemetry.addLine(String.format("\nUnknown Tag ID %d detected", detection.id));
            }
            telemetry.update();
        }
    }
}
