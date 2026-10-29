package org.firstinspires.ftc.teamcode.Locate;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
import org.opencv.core.Mat;

import java.util.ArrayList;

// 不是 OpMode，不需要 @TeleOp
public class Odo_Locate {
    // ===== 需要自己量測 / 校正的常數 =====
    static final double TICKS_PER_REV = 2000;     // 例：goBILDA Odometry Pod 為 2000
    static final double WHEEL_DIAMETER_IN = 1.88; // 死輪直徑 (英吋)，依你的死輪修改
    static final double TRACK_WIDTH_IN = 12.0;    // 左右兩顆平行輪的間距
    static final double PERP_OFFSET_IN = -5.0;    // 垂直輪在機器中心的「前方」距離，在後方為負

    static final double IN_PER_TICK = Math.PI * WHEEL_DIAMETER_IN / TICKS_PER_REV;

    // ===== 驅動馬達 =====//
    private DcMotor FL, FR, BL, BR;
    //====== 視覺辨識 ====//
    private VisionPortal Camera;
    private AprilTagProcessor AprilTagProcessor1;
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    // ===== 死輪編碼器 =====//
    private DcMotorEx leftOdo, rightOdo, perpOdo;
    private int lastLeft, lastRight, lastPerp;

    // ===== 位置 (場地座標) =====
    public double x = 0, y = 0;        // 英吋
    public double heading = 0;         // 弧度，逆時針為正

    public void init(HardwareMap hwM) {
        FL = hwM.get(DcMotor.class, "FL");
        FR = hwM.get(DcMotor.class, "FR");
        BL = hwM.get(DcMotor.class, "BL");
        BR = hwM.get(DcMotor.class, "BR");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOdo = hwM.get(DcMotorEx.class, "leftOdo");
        rightOdo = hwM.get(DcMotorEx.class, "rightOdo");
        perpOdo = hwM.get(DcMotorEx.class, "perpOdo");

        // 方向要實測：機器前進時左右平行輪都要增加，向左平移時垂直輪要增加
        // leftOdo.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightOdo.setDirection(DcMotorSimple.Direction.REVERSE);
        // perpOdo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        perpOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastLeft = leftOdo.getCurrentPosition();
        lastRight = rightOdo.getCurrentPosition();
        lastPerp = perpOdo.getCurrentPosition();
    }

    /** 每個迴圈都要呼叫一次 */
    public void updateOdometry() {
        int l = leftOdo.getCurrentPosition();
        int r = rightOdo.getCurrentPosition();
        int p = perpOdo.getCurrentPosition();

        double dL = (l - lastLeft) * IN_PER_TICK;
        double dR = (r - lastRight) * IN_PER_TICK;
        double dP = (p - lastPerp) * IN_PER_TICK;
        lastLeft = l;
        lastRight = r;
        lastPerp = p;

        // 朝向變化：右輪比左輪多走 = 逆時針轉
        double dTheta = (dR - dL) / TRACK_WIDTH_IN;
        // 前進量 = 兩顆平行輪平均
        double dForward = (dL + dR) / 2.0;
        // 平移量 = 垂直輪扣掉旋轉造成的讀數
        double dStrafe = dP - PERP_OFFSET_IN * dTheta;

        // 圓弧積分 (小角度時退化成直線)
        double s, c;
        if (Math.abs(dTheta) < 1e-6) {
            s = 1.0 - dTheta * dTheta / 6.0;
            c = dTheta / 2.0;
        } else {
            s = Math.sin(dTheta) / dTheta;
            c = (1.0 - Math.cos(dTheta)) / dTheta;
        }
        double dxRobot = s * dForward - c * dStrafe;
        double dyRobot = c * dForward + s * dStrafe;

        // 轉到場地座標
        x += dxRobot * Math.cos(heading) - dyRobot * Math.sin(heading);
        y += dxRobot * Math.sin(heading) + dyRobot * Math.cos(heading);
        heading += dTheta;
    }

    /** 把目前朝向當作 0 (取代 imu.resetYaw()) */
    public void resetHeading() {
        heading = 0;
    }

    public void drive(double forward, double strafe, double rotate) {
        double FLPower = forward + strafe + rotate;
        double BLPower = forward - strafe + rotate;
        double FRPower = forward - strafe - rotate;
        double BRPower = forward + strafe - rotate;

        double MaxPower = 1.0;
        double MaxSpeed = 1.0;

        MaxPower = Math.max(MaxPower, Math.abs(FLPower));
        MaxPower = Math.max(MaxPower, Math.abs(FRPower));
        MaxPower = Math.max(MaxPower, Math.abs(BLPower));
        MaxPower = Math.max(MaxPower, Math.abs(BRPower));

        FL.setPower(MaxSpeed * (FLPower / MaxPower));
        FR.setPower(MaxSpeed * (FRPower / MaxPower));
        BL.setPower(MaxSpeed * (BLPower / MaxPower));
        BR.setPower(MaxSpeed * (BRPower / MaxPower));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        // 原本用 IMU yaw，現在改用死輪算出的 heading
        theta = theta - heading;
        double new_forward = r * Math.sin(theta);
        double new_strafe = r * Math.cos(theta);
        this.drive(new_forward, new_strafe, rotate);
    }
    public void Vision_init(){
        AprilTagProcessor1 =new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()) // 載入官方賽事 Tag 庫
                .setDrawTagID(true)               // 顯示 ID
                .setCameraPose(cameraPosition, cameraOrientation) // 設定相機姿態
                .setDrawTagOutline(true)          // 繪製外框線
                .setDrawAxes(true)                // 繪製坐標軸
                .setDrawCubeProjection(true)      // 繪製 3D 虛擬立方體
                .build();
        Camera =new VisionPortal.Builder()
                .addProcessors(AprilTagProcessor1)
                .setCameraResolution(new Size(1280, 720)) // 標準高清解析度
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();
    }
    public void Vision_Locate(){

    }
}