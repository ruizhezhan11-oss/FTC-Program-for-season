package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="Maybe_Auto")
public class Maybe_Auto extends LinearOpMode {

    private DcMotor FL, FR, BL, BR;
    private AprilTagProcessor myAprilTagProcessor;
    private VisionPortal myVisionPortal;

    // 全向
    public static class AutoLocalizer {
        public GoBildaPinpointDriver smallbox;
        public double x = 0, y = 0, heading = 0;

        // 正確的建構子格式：名稱與類別一致，且沒有 void
        public AutoLocalizer(HardwareMap hardwareMap) {
            smallbox = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            // 設定 4Bar Pod 型號與實體裝設距離
            smallbox.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            smallbox.setOffsets(-84.0, -168.0, DistanceUnit.MM);
            smallbox.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            reset();
        }

        public void reset() {
            smallbox.resetPosAndIMU(); // 歸零 Pinpoint
        }

        public void update() {
            smallbox.update();
            Pose2D pose = smallbox.getPosition();
            x = pose.getX(DistanceUnit.INCH);      // 吋
            y = pose.getY(DistanceUnit.INCH);      // 吋
            heading = pose.getHeading(AngleUnit.RADIANS); // 弧度
        }
    }

    //TODO 馬達
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
    }

    @Override
    public void runOpMode() {
        init_hardware();
        AutoLocalizer localizer = new AutoLocalizer(hardwareMap);
        localizer.reset();

        // AprilTagProcessor
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        // VisionPortal(鏡頭)
        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(myAprilTagProcessor)
                .build();

        telemetry.addData("狀態", "自動模式初始化完成，擺正車頭後按 Play");
        telemetry.update();



        // 等待 Driver Station 按下 Play 鍵
        waitForStart();

        // 自動模式主迴圈
        while (opModeIsActive()) {
            localizer.update();

            // 印出當前座標到螢幕上
            telemetry.addData("Auto X (cm)", "%.2f", localizer.x * 2.54);
            telemetry.addData("Auto Y (cm)", "%.2f", localizer.y * 2.54);
            telemetry.addData("Auto Heading (Deg)", "%.2f", Math.toDegrees(localizer.heading));


            // TODO: 在這裡撰寫自動走路與 AprilTag 識別的判斷邏輯
            telemetry.update();
        }
    }

    private void setCamera(WebcamName webcamName) {
    }
}
