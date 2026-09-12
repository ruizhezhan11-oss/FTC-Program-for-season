package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="AutoTurretWithTriggerOverride")
public class BIOBUZZ_SEASON extends LinearOpMode {

    // 底盤與砲台轉向馬達
    private DcMotor BL, BR, FL, FR;
    private DcMotor camera_angle; // 砲台 DC 馬達

    // 定位與視覺
    private GoBildaPinpointDriver pinpoint;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // 砲台 PID 控制參數
    private double kP = 0.03;
    private double kI = 0.000;
    private double kD = 0.003;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        initHardware();
        initVision();

        telemetry.addData("狀態", "系統就緒，請擺正車頭後按 Play");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // =========================================================
            // 1. 底盤 Field-Centric 全向駕馭
            // =========================================================
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double botHeading = pose.getHeading(AngleUnit.RADIANS);

            if (gamepad1.start) {
                pinpoint.resetPosAndIMU();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

            BL.setPower((rotY + rotX + rx) / denominator);
            BR.setPower((rotY - rotX - rx) / denominator);
            FL.setPower((rotY - rotX + rx) / denominator);
            FR.setPower((rotY + rotX - rx) / denominator);


            // =========================================================
            // 2. 砲台轉向控制：自動 PID 瞄準 vs. RT/LT 手動 Trigger 迴圈
            // =========================================================

            // 計算操控手按下 Trigger 的差值 (RT 順時針, LT 逆時針)
            double manualPower = gamepad1.right_trigger - gamepad1.left_trigger;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            boolean tagFound = false;

            // (A) 優先條件：如果操控手有壓下 RT 或 LT (超過死區 0.05)，強制切換為手動控制
            if (Math.abs(manualPower) > 0.05) {
                camera_angle.setPower(manualPower);
                integralSum = 0; // 重置 PID 積分防止突衝
                telemetry.addData("砲台控制模式", "🎮 駕駛員手動控制中 (RT/LT)");
            }
            // (B) 操控手沒按 Trigger，且有看到 AprilTag：執行 PID 自動鎖定
            else if (!currentDetections.isEmpty()) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        double error = detection.ftcPose.x; // 畫面水平誤差 (inches)

                        // PID 計算
                        double motorPower = calculatePID(0, error);
                        motorPower = Math.max(-1.0, Math.min(1.0, motorPower));

                        camera_angle.setPower(motorPower);

                        telemetry.addData("砲台控制模式", "🎯 自動 PID 瞄準鎖定中");
                        telemetry.addData("鎖定 Tag ID", detection.id);
                        telemetry.addData("水平偏差 (in)", "%.2f", error);
                        telemetry.addData("馬達 Power", "%.2f", motorPower);
                        tagFound = true;
                        break;
                    }
                }
            }

            // (C) 沒按 Trigger，也沒看到 Tag：馬達動力歸零，靠 BRAKE 模式自動煞車鎖定
            if (Math.abs(manualPower) <= 0.05 && !tagFound) {
                camera_angle.setPower(0);
                integralSum = 0; // 重置 PID 積分
                telemetry.addData("砲台控制模式", "⏸️ 待命/自動煞車鎖定中");
            }

            telemetry.addData("車頭角度 (Deg)", "%.2f", Math.toDegrees(botHeading));
            telemetry.update();
        }
    }

    private void initHardware() {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        camera_angle = hardwareMap.get(DcMotor.class, "camera_angle");

        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        camera_angle.setDirection(DcMotorSimple.Direction.FORWARD);

        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 砲台馬達設定為 BRAKE 模式，Power 設為 0 時會鎖死角度
        camera_angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        camera_angle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Odometry 定位板
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

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