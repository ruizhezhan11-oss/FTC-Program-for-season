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

    private DcMotor BL, BR, FL, FR;
    private DcMotor camera_angle;

    private GoBildaPinpointDriver pinpoint;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // PID 參數
    private double kP = 0.03;
    private double kI = 0.000;
    private double kD = 0.003;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime pidTimer = new ElapsedTime();
    private boolean isPidActive = false;

    @Override
    public void runOpMode() {
        initHardware();
        initVision();

        telemetry.addData("狀態", "系統就緒，請擺正車頭後按 Play");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {
            // =========================================================
            // 1. 底盤 Field-Centric 全向駕馭 (加入死區過濾與歸一化)
            // =========================================================
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double botHeading = pose.getHeading(AngleUnit.RADIANS);

            if (gamepad1.start) {
                pinpoint.resetPosAndIMU();
            }

            double y = applyDeadzone(-gamepad1.left_stick_y);
            double x = applyDeadzone(gamepad1.left_stick_x);
            double rx = applyDeadzone(gamepad1.right_stick_x);

            // Field-Centric 座標轉換
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // 標準 Mecanum 矩陣（FL/BR 與 FR/BL 對角同號）
            double flPower = rotY + rotX + rx;
            double frPower = rotY - rotX - rx;
            double blPower = rotY - rotX + rx;
            double brPower = rotY + rotX - rx;

            double maxPower = Math.max(1.0, Math.max(
                    Math.max(Math.abs(flPower), Math.abs(frPower)),
                    Math.max(Math.abs(blPower), Math.abs(brPower))
            ));

            FL.setPower(flPower / maxPower);
            FR.setPower(frPower / maxPower);
            BL.setPower(blPower / maxPower);
            BR.setPower(brPower / maxPower);

            // =========================================================
            // 2. 砲台轉向控制：優先權邏輯與 PID 瞄準
            // =========================================================
            double manualPower = gamepad1.right_trigger - gamepad1.left_trigger;

            if (Math.abs(manualPower) > 0.05) {
                // (A) 駕駛手動 Override 優先
                resetPID();
                camera_angle.setPower(manualPower);
                telemetry.addData("砲台控制模式", "🎮 駕駛員手動控制 (RT/LT)");
            } else {
                AprilTagDetection targetTag = getPrimaryDetection();
                if (targetTag != null) {
                    // (B) 自動 PID 鎖定 AprilTag
                    double error = targetTag.ftcPose.x; // 水平位置偏差 (inches)
                    double pidOutput = updatePID(0, error);
                    double power = Math.max(-1.0, Math.min(1.0, pidOutput));

                    camera_angle.setPower(power);
                    telemetry.addData("砲台控制模式", "🎯 PID 鎖定 Tag ID: %d", targetTag.id);
                    telemetry.addData("水平偏差 (in)", "%.2f", error);
                    telemetry.addData("馬達 Power", "%.2f", power);
                } else {
                    // (C) 無目標且無手動操作：煞車待命
                    resetPID();
                    camera_angle.setPower(0);
                    telemetry.addData("砲台控制模式", "⏸️ 待命 / 自動煞車鎖定");
                }
            }

            telemetry.addData("車頭角度 (Deg)", "%.2f", Math.toDegrees(botHeading));
            telemetry.update();
        }
    }

    private double applyDeadzone(double input) {
        return Math.abs(input) > 0.05 ? input : 0.0;
    }

    private AprilTagDetection getPrimaryDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                return detection;
            }
        }
        return null;
    }

    private double updatePID(double target, double current) {
        double error = target - current;
        double dt = pidTimer.seconds();
        pidTimer.reset();

        // 避免切換鎖定或丟失目標再重新啟用時，dt 過大引發微分衝擊
        if (!isPidActive || dt > 0.2) {
            dt = 0.02; // 設定預估單週期時間 (50Hz)
            isPidActive = true;
        }

        double pOutput = kP * error;
        integralSum += error * dt;
        double iOutput = kI * integralSum;
        double derivative = (error - lastError) / dt;
        double dOutput = kD * derivative;

        lastError = error;
        return pOutput + iOutput + dOutput;
    }

    private void resetPID() {
        integralSum = 0;
        lastError = 0;
        isPidActive = false;
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
        camera_angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        camera_angle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
}