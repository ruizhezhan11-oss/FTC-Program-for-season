package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/*
 * FTC TeleOp 模式
 * 功能：
 * - 麥克納姆輪驅動控制
 * - Pinpoint 本體定位與陀螺儀輔助（場景中心驅動）
 * - AprilTag 視覺識別
 */
@TeleOp
public class Base extends LinearOpMode {
    // 馬達
    private DcMotor FL, FR, BL, BR;
    private GoBildaPinpointDriver pinpoint1;

    // 視覺系統
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal camera; //視覺
    int IdCode;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    /**
     * 初始化馬達和 IMU
     */
    private void initMotors() {
        // 獲取硬體
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

        // 初始化 Pinpoint 定位

    }
    private void init_locate(){
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
                .setDrawTagID(true) //ID
                .setCameraPose(cameraPosition, cameraOrientation)
                .setDrawTagOutline(true) //繪製外框線
                .setDrawAxes(true) //繪製坐標軸
                .setDrawCubeProjection(true) //在標籤上方繪製 3D 虛擬立方體，便於可視化確認距離與姿態
                .build();

        camera = new VisionPortal.Builder()
                .setCameraResolution(new Size(1280, 720))  // 標準高清解析度
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2) //省流
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true) //live review
                .setAutoStopLiveView(true)
                .build();
    }
    private void get_library(){
        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder();
        builder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
        AprilTagProcessor processor = builder.build();
        //依照官方的東西 Builder是另外加東西才用的到
    }

    @Override
    public void runOpMode() {
        // 初始化所有系統
        initVision();
        initMotors();
        init_locate();
        get_library();
        waitForStart();
        // 主迴圈
        while (opModeIsActive()) {
            displayAprilTagDetections();
            // 必須在迴圈內更新 Pinpoint 驅動器，才能獲取最新座標與角度
            pinpoint1.update();
            if (gamepad1.options) {
                pinpoint1.resetPosAndIMU();
            }

            // 更新所有感測器和顯示
            handleInput();
            updateTelemetry();
            telemetry.update();
        }

        // 清理資源
        if (camera != null) {
            camera.close();
        }
    }

    /**
     * 處理手柄輸入並控制馬達
     */
    private void handleInput() {
        controlDrivetrain();
    }

    /**
     * 麥克納姆輪驅動控制
     */
    private void controlDrivetrain() {
        // 獲取定位姿態資料
        Pose2D pose = pinpoint1.getPosition();
        double headingInRad = pose.getHeading(AngleUnit.RADIANS);

        // 獲取手柄輸入並應用死區
        double y = -applyDeadzone(gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x) * 1.1;  // 1.1 補償橫移摩擦力
        double rx = applyDeadzone(gamepad1.right_stick_x);

        // 應用陀螺儀補償（場景中心驅動）
        double rotX = x * Math.cos(-headingInRad) - y * Math.sin(-headingInRad);
        double rotY = x * Math.sin(-headingInRad) + y * Math.cos(-headingInRad);

        // 計算每個馬達的功率
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

        // 應用馬達功率
        FL.setPower(flPower);
        FR.setPower(frPower);
        BL.setPower(blPower);
        BR.setPower(brPower);
    }

    /**
     * 更新所有遙測資料
     */
    private void updateTelemetry() {
        // 取得當前姿態
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
        List<AprilTagDetection> List = AprilTagProcessor.easyCreateWithDefaults().getDetections();
        AprilTagDetection AprilTagDetection;
        for(AprilTagDetection detection : List){
            if (detection.metadata !=null) {
                IdCode= detection.id;
                if(!detection.metadata.name.contains("Obelisk")){
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f",
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    detection.robotPose.getPosition().z));//顯示XYZ
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    //PRY
                }

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                // 辨識到未註冊詳細姿態參數的 Tag，仍可取得 ID

            }
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }

    /**
     * 應用死區過濾器（消除搖桿漂移）
     * @param input 原始輸入值（-1.0 到 1.0）
     * @return 過濾後的值
     */
    private double applyDeadzone(double input) {
        return Math.abs(input) > 0.05 ? input : 0.0;
    }
}