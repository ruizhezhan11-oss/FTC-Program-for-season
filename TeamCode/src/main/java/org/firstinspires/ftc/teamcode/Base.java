package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * FTC TeleOp 模式
 * 功能：
 * - 麥克納姆輪驅動控制
 * - Pinpoint 本體定位與陀螺儀輔助（場景中心驅動）
 * - AprilTag 視覺識別
 */
@TeleOp
public class Base extends LinearOpMode {
    // 馬達與定位模組
    private DcMotor FL, FR, BL, BR;
    private GoBildaPinpointDriver pinpoint1;

    // 視覺系統
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal camera;
    private int idCode;

    // 相機相對於機器人中心的安裝位置與角度偏移
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    /**
     * 初始化馬達
     */
    private void initMotors() {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        // 設置馬達方向
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);

        // 設置馬達制動模式
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * 初始化 Pinpoint 定位模組
     */
    private void initLocate() {
        pinpoint1 = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint1");
        pinpoint1.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint1.setOffsets(111, 111, DistanceUnit.MM);
        pinpoint1.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint1.resetPosAndIMU();
    }

    /**
     * 初始化視覺系統（AprilTag 檢測）
     */
    private void initVision() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()) // 載入官方賽事 Tag 庫
                .setDrawTagID(true)               // 顯示 ID
                .setCameraPose(cameraPosition, cameraOrientation) // 設定相機姿態
                .setDrawTagOutline(true)          // 繪製外框線
                .setDrawAxes(true)                // 繪製坐標軸
                .setDrawCubeProjection(true)      // 繪製 3D 虛擬立方體
                .build();

        camera = new VisionPortal.Builder()
                .setCameraResolution(new Size(1280, 720)) // 標準高清解析度
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }

    @Override
    public void runOpMode() {
        // 初始化所有系統
        initVision();
        initMotors();
        initLocate();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // 主迴圈
        while (opModeIsActive()) {
            // 更新 Pinpoint 座標與角度
            pinpoint1.update();

            // 按下 gamepad1 options 鍵重置 IMU 角度與座標
            if (gamepad1.options) {
                pinpoint1.resetPosAndIMU();
            }

            // 處理底盤控制與資訊更新
            handleInput();
            updateTelemetry();
            telemetry.update();
        }

        // 清理相機資源
        if (camera != null) {
            camera.close();
        }
    }

    /**
     * 處理手柄輸入
     */
    private void handleInput() {
        controlDrivetrain();
    }

    /**
     * 麥克納姆輪 Field-Centric（場景中心）驅動控制
     */
    private void controlDrivetrain() {
        Pose2D pose = pinpoint1.getPosition();
        double headingInRad = pose.getHeading(AngleUnit.RADIANS);

        // 獲取手柄輸入並套用死區
        double y = -applyDeadzone(gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x) * 1.1;  // 1.1 補償橫移摩擦力
        double rx = applyDeadzone(gamepad1.right_stick_x);

        // 陀螺儀角度補償 (Field-Centric)
        double rotX = x * Math.cos(-headingInRad) - y * Math.sin(-headingInRad);
        double rotY = x * Math.sin(-headingInRad) + y * Math.cos(-headingInRad);

        // 計算馬達輸出功率
        double flPower = rotY + rotX + rx;
        double frPower = rotY - rotX - rx;
        double blPower = rotY - rotX + rx;
        double brPower = rotY + rotX - rx;

        // 功率歸一化（避免輸出 > 1.0）
        double max = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower),
                Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        // 輸出至馬達
        FL.setPower(flPower);
        FR.setPower(frPower);
        BL.setPower(blPower);
        BR.setPower(brPower);
    }

    /**
     * 更新所有遙測資料
     */
    private void updateTelemetry() {
        Pose2D pose = pinpoint1.getPosition();
        double yaw = pose.getHeading(AngleUnit.DEGREES);

        telemetry.addLine("========== IMU 資料 ==========");
        telemetry.addData("Yaw (陀螺儀)", String.format("%.1f°", yaw));

        telemetry.addLine("\n========== Pinpoint 定位 ==========");
        telemetry.addData("X 座標", String.format("%.1f mm", pose.getX(DistanceUnit.MM)));
        telemetry.addData("Y 座標", String.format("%.1f mm", pose.getY(DistanceUnit.MM)));

        telemetry.addLine("\n========== AprilTag 檢測 ==========");
        displayAprilTagDetections();

        telemetry.addLine("\n========== 手柄輸入 ==========");
        telemetry.addData("左搖桿", String.format("X:%.2f Y:%.2f", gamepad1.left_stick_x, gamepad1.left_stick_y));
        telemetry.addData("右搖桿 X", String.format("%.2f", gamepad1.right_stick_x));
    }

    /**
     * 顯示 AprilTag 檢測結果
     */
    private void displayAprilTagDetections() {
        // 使用已掛載至 VisionPortal 的 Processor 取得結果
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                idCode = detection.id;

                if (!detection.metadata.name.contains("Obelisk")) {
                    // 加上 robotPose 防空檢查，防止崩潰
                    if (detection.robotPose != null) {
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f",
                                detection.robotPose.getPosition().x,
                                detection.robotPose.getPosition().y,
                                detection.robotPose.getPosition().z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f",
                                detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                                detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                                detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    } else if (detection.ftcPose != null) {
                        // 若無全域姿態，改顯示相對相機的距離
                        telemetry.addLine(String.format("Tag Range %6.1f in", detection.ftcPose.range));
                    }
                }
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));
            }
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }

    /**
     * 死區過濾器
     */
    private double applyDeadzone(double input) {
        return Math.abs(input) > 0.05 ? input : 0.0;
    }
}