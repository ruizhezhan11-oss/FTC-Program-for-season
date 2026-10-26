package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.externalhardware.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

@TeleOp(name="Auto Aim & Odometry TeleOp", group="Competition")
public class AutoAimTeleOp extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    // 三輪惰輪 odometry 物件
    // 註記：如果等下跑 vision 迴圈變得很卡，這個可能要想辦法拉去開 Thread 算，不然手動會延遲
    ThreeWheelOdometry odometry = new ThreeWheelOdometry();

    // ==========================================
    // 參數區：這裡可以根據實際上車測試的狀況自己調！
    // ==========================================
    final int DESIRED_TAG_ID = 11;          // 我們要對準的 AprilTag ID (視今年場地標籤而定)
    final double DESIRED_DISTANCE = 12.0;   // 停在距離目標多遠的地方 (吋)
    final double SPEED_GAIN  =  0.02;       // 前後移動靈敏度 (車太慢就調大，會暴衝就調小)
    final double STRAFE_GAIN =  0.015;      // 左右平移靈敏度
    final double TURN_GAIN   =  0.01;       // 轉向靈敏度
    final double MAX_AUTO_SPEED = 0.5;      // 自動對齊的最高速度限制 (防撞)
    // ==========================================

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // 之後接好線記得把下面這行解除註解
        // odometry.init(robot.leftEncoder, robot.rightEncoder, robot.strafeEncoder);

        telemetry.addLine("好了沒，按 START 準備開跑啦！");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // odometry.update(); // 怕迴圈太慢先註解掉，之後獨立執行緒再開

            double drive = 0, strafe = 0, turn = 0;

            // 按住手把 A 鍵就進入自動對齊模式
            if (gamepad1.a) {
                boolean targetFound = false;
                AprilTagDetection targetTag = null;

                List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();

                for (AprilTagDetection detection : currentDetections) {
                    // 檢查有沒有抓到我們要的 ID 而且 Pose 不是空的
                    if (detection.id == DESIRED_TAG_ID && detection.ftcPose != null) {
                        targetFound = true;
                        targetTag = detection;
                        break;
                    }
                }

                if (targetFound) {
                    // 算一下距離跟角度的誤差
                    double rangeError   = targetTag.ftcPose.range - DESIRED_DISTANCE; // 前後距離差
                    double xError       = targetTag.ftcPose.x;                        // 左右偏多少
                    double headingError = targetTag.ftcPose.bearing;                  // 角度偏多少

                    // 帶入公式算出馬達要跑的速度，並限制在安全範圍內
                    drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                    // 如果實際測試發現左右平移方向相反，記得把 xError 前面加個負號 (-)
                    strafe = Range.clip(xError * STRAFE_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);

                    telemetry.addLine("抓到目標 ID " + DESIRED_TAG_ID + "！自動對齊中...");
                    telemetry.addData("Range 誤差", "%.2f in", rangeError);
                    telemetry.addData("X 誤差", "%.2f in", xError);
                    telemetry.addData("轉向誤差", "%.2f deg", headingError);
                } else {
                    telemetry.addLine("還沒看到目標標籤勒...尋找中...");
                    drive = 0;
                    strafe = 0;
                    turn = 0;
                }
            } else {
                // 放開 A 鍵就是正常手動開 (搖桿死區防一下抖動)
                drive  = -applyDeadzone(gamepad1.left_stick_y);  // FTC 搖桿往上是負的，所以加負號
                strafe = applyDeadzone(gamepad1.left_stick_x);
                turn   = applyDeadzone(gamepad1.right_stick_x);

                telemetry.addLine("手動駕駛模式");
            }

            // 把計算好的數值丟給馬達去跑
            moveRobot(drive, strafe, turn);

            // 如果有開里程計，就把這兩行解開看座標
            // telemetry.addData("Global X", "%.1f in", odometry.getX());
            // telemetry.addData("Global Y", "%.1f in", odometry.getY());
            telemetry.update();
        }
    }

    // 麥克納姆輪基本的底盤運動學
    private void moveRobot(double drive, double strafe, double turn) {
        double flPower = drive + strafe + turn;
        double frPower = drive - strafe - turn;
        double blPower = drive - strafe + turn;
        double brPower = drive + strafe - turn;

        // 如果馬達出力超過 1.0 就把它等比例縮小，才不會爆掉
        double max = Math.max(Math.abs(flPower), Math.max(Math.abs(frPower), Math.max(Math.abs(blPower), Math.abs(brPower))));
        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        robot.FL.setPower(flPower);
        robot.FR.setPower(frPower);
        robot.BL.setPower(blPower);
        robot.BR.setPower(brPower);
    }

    private double applyDeadzone(double input) {
        return Math.abs(input) > 0.05 ? input : 0.0;
    }
}
