package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//Road Runner
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d; //座標
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder; //軌跡
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

@Autonomous(name="Maybe_Auto")
public class Maybe_Auto extends LinearOpMode {

    // 馬達與雲台 Servo
    private DcMotor FL, FR, BL, BR;
    private Servo cameraServo;

    // 視覺系統
    private AprilTagProcessor myAprilTagProcessor;
    private VisionPortal myVisionPortal;

    // 雲台 PID 控制參數
    private double kP = 0.005;
    private double kI = 0.000;
    private double kD = 0.001;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double currentServoPos = 0.5; // 預設雲台正中央

    // 定位系統 (goBILDA Pinpoint Odometry)
    public static class AutoLocalizer {
        public GoBildaPinpointDriver smallbox;
        public double x=0,y=0,heading=0;

        public AutoLocalizer(HardwareMap hardwareMap) {
            smallbox = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            smallbox.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            smallbox.setOffsets(-84.0, -168.0, DistanceUnit.MM);
            smallbox.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            reset();
        }

        public void reset() {
            smallbox.resetPosAndIMU();
        }

        public void update() {
            smallbox.update();
            Pose2D pose = smallbox.getPosition();
             x = pose.getX(DistanceUnit.INCH);
             y = pose.getY(DistanceUnit.INCH);
             heading = pose.getHeading(AngleUnit.RADIANS);
        }

        // 強制寫入絕對座標 (單位：Inches, Radians)
        public void setAbsolutePosition(double absX, double absY, double absHeading) {
            smallbox.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    absX,
                    absY,
                    AngleUnit.RADIANS,
                    absHeading
            ));
        }
    }

    public void init_hardware() {
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

        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(currentServoPos);
    }


    @Override
    public void runOpMode() {
        init_hardware();
        AutoLocalizer localizer = new AutoLocalizer(hardwareMap);
        localizer.reset();

        // 建立 AprilTagProcessor
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // 建立 VisionPortal
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(myAprilTagProcessor)
                .build();

        telemetry.addData("狀態", "自動模式初始化完成，擺正車頭後按 Play");
        telemetry.update();
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        //軌跡
        Trajectory TRA = drive.trajectoryBuilder(startPose)
        .forward(24)
        .build();


        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // 1. 更新 Pinpoint 定位數據
            localizer.update();

            // 2. 獲取 AprilTag 偵測結果
            List<AprilTagDetection> currentDetections = myAprilTagProcessor.getDetections();

            if (!currentDetections.isEmpty()) {
                AprilTagDetection targetTag = currentDetections.get(0);

                if (targetTag.metadata != null) {
                    // --- A. PID 雲台鏡頭動態追蹤 ---
                    double error = targetTag.ftcPose.x; // 水平偏差 (Inches)
                    double correction = calculatePID(0, error);

                    currentServoPos = Math.max(0.0, Math.min(1.0, currentServoPos - correction));
                    cameraServo.setPosition(currentServoPos);

                    // --- B. 絕對座標逆推與 Pinpoint 校正 ---
                    // 假設 Target ID 11 在賽場上的固定座標為 (72.0", 144.0")
                    if (targetTag.id == 11) {
                        double tagKnownX = 72.0;  // 賽季官方 Tag X 絕對座標 (吋)
                        double tagKnownY = 144.0; // 賽季官方 Tag Y 絕對座標 (吋)

                        // 算出身體與雲台疊加後的相機場地總角度 (弧度)
                        double servoAngleRad = (currentServoPos - 0.5) * Math.PI;
                        double totalCamAngle = localizer.heading + servoAngleRad;

                        // 讀取相對距離並透過旋轉矩陣轉換至場地座標系
                        double relX = targetTag.ftcPose.x;
                        double relY = targetTag.ftcPose.y;

                        double deltaX = relX * Math.cos(totalCamAngle) - relY * Math.sin(totalCamAngle);
                        double deltaY = relX * Math.sin(totalCamAngle) + relY * Math.cos(totalCamAngle);

                        // 計算車體場地絕對座標
                        double absX = tagKnownX - deltaX;
                        double absY = tagKnownY - deltaY;

                        // 重置 Pinpoint 的數據至絕對座標
                        localizer.setAbsolutePosition(absX, absY, localizer.heading);
                    }

                    telemetry.addData("Target Tag ID", targetTag.id);
                    telemetry.addData("Tag X Error (in)", "%.2f", error);
                    telemetry.addData("Servo Position", "%.3f", currentServoPos);
                }
            } else {
                telemetry.addData("Tag Tracking", "未偵測到 AprilTag");
            }

            // 3. 輸出座標資訊至螢幕
            telemetry.addData("Auto X (cm)", "%.2f", localizer.x * 2.54);
            telemetry.addData("Auto Y (cm)", "%.2f", localizer.y * 2.54);
            telemetry.addData("Auto Heading (Deg)", "%.2f", Math.toDegrees(localizer.heading));

            telemetry.update();
        }
    }

    /**
     * PID 控制器運算
     */
    private double calculatePID(double target, double current) {
        double error = target - current;
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0) dt = 0.001;

        double pOutput = kP * error;

        integralSum += error * dt;
        double iOutput = kI * integralSum;

        double derivative = (error - lastError) / dt;
        double dOutput = kD * derivative;

        lastError = error;

        return pOutput + iOutput + dOutput;
    }
}
