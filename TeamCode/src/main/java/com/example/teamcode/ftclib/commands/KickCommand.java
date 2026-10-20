package com.example.teamcode.ftclib.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.example.teamcode.ftclib.subsystems.KickerSubsystem;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 직접 만든 커맨드 예제: 다리를 휘둘러서 공 차기.
 *
 * 커맨드는 4개의 함수로 동작함.
 *   initialize() : 시작할 때 한 번
 *   execute()    : 실행 중에 매 루프
 *   isFinished() : true를 돌려주면 끝
 *   end()        : 끝날 때 한 번 (끝까지 갔든, 중간에 끊겼든)
 */
public class KickCommand extends CommandBase {

    private final KickerSubsystem kicker;
    private final double power;
    private final double targetDeg;
    private final double maxSeconds;
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * @param power      휘두르는 파워 (0~1)
     * @param targetDeg  이 각도에 닿으면 파워 끊음
     * @param maxSeconds 각도에 못 가도 이 시간이 지나면 끝 (막혔을 때 대비)
     */
    public KickCommand(KickerSubsystem kicker, double power, double targetDeg, double maxSeconds) {
        this.kicker = kicker;
        this.power = power;
        this.targetDeg = targetDeg;
        this.maxSeconds = maxSeconds;

        // "이 커맨드가 도는 동안 kicker는 내 거" → 다른 커맨드가 동시에 kicker를 못 씀
        addRequirements(kicker);
    }

    @Override
    public void initialize() {
        timer.reset();
        kicker.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return kicker.getAngleDeg() >= targetDeg || timer.seconds() >= maxSeconds;
    }

    @Override
    public void end(boolean interrupted) {
        kicker.stop(); // BRAKE라서 서서히 멈춤
    }
}
