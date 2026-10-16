package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class VisionTest extends LinearOpMode {
    private DcMotor FL, FR, BL, BR;
    private AprilTagProcessor AprilTagProcessor1;
    private VisionPortal Camera;
    private int idCode;
    private boolean Red_Side = false;
    private boolean Blue_Side= false;
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
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
        //*字串的威力*//
        for(DcMotor m :new DcMotor[]{BL,BR,FL,FR}){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        };
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
        Camera= new VisionPortal.Builder()
                .setCameraResolution(new Size(1280, 720)) // 標準高清解析度
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(AprilTagProcessor1)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }
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
    private void loop_(){
        while(idCode !=-1){
            if(idCode>=30&& idCode<=37){  //紅方
            Red_Side=true;
            }
            if(idCode>=38 && idCode<=45){ //藍方
            Blue_Side=true;
            }

        }
    }
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
    @Override //*主迴圈
    public void runOpMode(){
        initMotors();
        init_Vision();
        waitForStart();
        while (opModeIsActive()){
        displayAprilTagDetections();
        handle();
        loop_();
        telemetry.update();
        }
        if(Camera !=null){
            Camera.close();
        }
    }

}
