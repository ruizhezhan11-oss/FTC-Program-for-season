package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.Collections;
import java.util.List;



@TeleOp
public class VisionTest extends LinearOpMode {
    private DcMotor FL, FR, BL, BR = null;
    private DcMotorEx PIDControl=null;
    private IMU imu = null;
    private AprilTagProcessor AprilTagProcessor1 = null;
    private ColorBlobLocatorProcessor ColorSensor = null;
    private ColorBlobLocatorProcessor ColorSensor2 = null;
    private ColorBlobLocatorProcessor ColorSensor3 = null;
    private PredominantColorProcessor ColorProcessor = null;
    private VisionPortal Camera = null;
    private final double NEW_P =2.5;
    private final double NEW_I = 0.1;
    private final double NEW_D =0.2;
    private int idCode ;
    private boolean Red_Side = false;
    private boolean Blue_Side= false;
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
    private void initMotors() {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        PIDControl=hardwareMap.get(DcMotorEx.class,"PIDControl");
        // 設置馬達方向
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        //*字串的威力*//
        for(DcMotor m :new DcMotor[]{BL,BR,FL,FR}){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        };
        imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters=new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
        double heading = imu.getRobotYawPitchRollAngles().getYaw();
        imu.resetYaw();

    }
    private void init_Vision (){
        AprilTagProcessor1= new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()) // 載入官方賽事 Tag 庫
                .setDrawTagID(true)               // 顯示 ID
                .setCameraPose(cameraPosition, cameraOrientation) // 設定相機姿態
                .setDrawTagOutline(true)          // 繪製外框線
                .setDrawAxes(true)                // 繪製坐標軸
                .setDrawCubeProjection(true)      // 繪製 3D 虛擬立方體
                .build();
        ColorSensor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)   // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .build();
        ColorSensor2 = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)   // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .build();
        ColorSensor3 = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)   // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBlurSize(5)          // Smooth the transitions between different colors in image
                .build();
        ColorProcessor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();
        Camera= new VisionPortal.Builder()
                .addProcessors(ColorSensor,ColorSensor2,ColorSensor3,AprilTagProcessor1,ColorProcessor)
                .setCameraResolution(new Size(1280, 720)) // 標準高清解析度
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

    }
    //*Color Locate
    private void Color_Locate() {
        List<ColorBlobLocatorProcessor.Blob> BB_Blobs = ColorSensor.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> BR_Blobs = ColorSensor2.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> SY_Blobs = ColorSensor3.getBlobs();

        // 過濾雜訊，範圍要用實測值調整
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 300, 50000, BB_Blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 300, 50000, BR_Blobs);
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 150, 30000, SY_Blobs);

        reportBlob("Blue Nectar", BB_Blobs);
        reportBlob("Red Nectar", BR_Blobs);
        reportBlob("Yellow Pollen", SY_Blobs);
    }

    private void reportBlob(String name, List<ColorBlobLocatorProcessor.Blob> blobs) {
        if (blobs.isEmpty()) {
            telemetry.addData(name, "none");
            return;
        }
        ColorBlobLocatorProcessor.Blob b = blobs.get(0);   // 面積最大的
        RotatedRect box = b.getBoxFit();
        telemetry.addData(name, "x=%.0f y=%.0f area=%d circ=%.2f",
                box.center.x, box.center.y, b.getContourArea(), b.getCircularity());
    }
    //*AprilTag
    private void displayAprilTagDetections() {
        idCode = -1; // 每次重設

        List<AprilTagDetection> currentDetections = AprilTagProcessor1.getDetections();
        telemetry.addData("# Tags", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.metadata.name.contains("Obelisk")) {
                    idCode = detection.id;
                } else if (detection.robotPose != null) {
                    telemetry.addLine(String.format("XYZ(in) %6.1f %6.1f %6.1f",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("PRY(deg) %6.1f %6.1f %6.1f",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("Unknown ID %d  Center %6.0f %6.0f",
                        detection.id, detection.center.x, detection.center.y));
            }
        }
        telemetry.addData("Obelisk Id", idCode);
    }
    //*handle
    private void handle(){
        double y = -gamepad1.left_stick_y; // 前後運動 (搖桿推前為負值，故加負號)
        double x = gamepad1.left_stick_x * 1.1; // 左右平移 (乘以1.1修正麥克納姆輪側滑損耗)
        double rx = gamepad1.right_stick_x; // 原地旋轉

        // 麥克納姆輪全向運動運動學算式
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        FL.setPower(frontLeftPower);
        BL.setPower(backLeftPower);
        FL.setPower(frontRightPower);
        BR.setPower(backRightPower);

    }
    //*Color Sensor
    private void Color_v(){
        PredominantColorProcessor.Result result = ColorProcessor.getAnalysis();
        telemetry.addLine(String.format("RGB %3.1f %3.1f %3.1f",
                result.RGB[0],result.RGB[1],result.RGB[2]));
        telemetry.addLine(String.format("HSV %3.1f %3.1f %3.1f",
                result.HSV[0],result.HSV[1],result.HSV[2]));
        telemetry.update();
    }
    @Override //*主迴圈
    public void runOpMode(){
        initMotors(); //馬達、imu
        init_Vision(); //視覺
        waitForStart();
        //*
        PIDCoefficients pidOrig = PIDControl.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        PIDCoefficients pidModified = PIDControl.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                                                                                                       //*
        while (opModeIsActive()){
            displayAprilTagDetections(); //AprilTag顯示
            handle(); //手把
            Color_v();//color sensor
            Color_Locate();//定位
            telemetry.update();
        }
        if(Camera !=null){
            Camera.close();
        }
    }

}
