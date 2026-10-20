package com.example.teamcode.ftclib.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 서브시스템 = 로봇의 "부품 하나". 이건 메카넘 바퀴 4개.
 *
 * 서브시스템은 "무엇을 할 수 있는지"(drive, stop)만 가지고 있고,
 * "언제 할지"는 커맨드와 버튼 연결이 정함.
 *
 * 모터 이름과 방향은 mecanum_drive.java와 같음.
 */
public class DriveSubsystem extends SubsystemBase {

    private final DcMotor l_f, r_f, l_b, r_b;

    public DriveSubsystem(HardwareMap hardwareMap) {
        r_f = hardwareMap.get(DcMotor.class, "motor1");
        r_b = hardwareMap.get(DcMotor.class, "motor2");
        l_b = hardwareMap.get(DcMotor.class, "motor3");
        l_f = hardwareMap.get(DcMotor.class, "motor4");

        l_f.setDirection(DcMotor.Direction.REVERSE);
        l_b.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{l_f, r_f, l_b, r_b}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * @param strafe  오른쪽 +, 왼쪽 - (-1~1)
     * @param forward 앞 +, 뒤 - (-1~1)
     * @param turn    시계 방향 + (-1~1)
     * @param speed   전체 속도 배율 (0~1)
     */
    public void drive(double strafe, double forward, double turn, double speed) {
        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        // 1.0을 넘으면 비율은 유지하고 전체를 줄임
        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf),
                     Math.max(Math.abs(lb), Math.abs(rb)))));

        l_f.setPower(lf / max * speed);
        r_f.setPower(rf / max * speed);
        l_b.setPower(lb / max * speed);
        r_b.setPower(rb / max * speed);
    }

    public void stop() {
        drive(0, 0, 0, 0);
    }
}
