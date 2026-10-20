package com.example.teamcode.ftclib.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.example.teamcode.ftclib.subsystems.KickerSubsystem;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 찬 다음 다리를 천천히 내리는 커맨드.
 *
 * 다리가 아직 올라가는 중이면 기다렸다가(기어 보호),
 * 멈추면 degPerSec 속도로 0° 근처까지 내림.
 */
public class LowerLegCommand extends CommandBase {

    static final double HOME_DEG = 5.0; // 0°에서 이만큼 안이면 다 내려온 것

    private final KickerSubsystem kicker;
    private final double degPerSec;
    private final double maxSeconds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean lowering;

    public LowerLegCommand(KickerSubsystem kicker, double degPerSec, double maxSeconds) {
        this.kicker = kicker;
        this.degPerSec = degPerSec;
        this.maxSeconds = maxSeconds;
        addRequirements(kicker);
    }

    @Override
    public void initialize() {
        timer.reset();
        lowering = false;
        kicker.stop();
    }

    @Override
    public void execute() {
        if (!lowering && kicker.getSpeedDegPerSec() <= 0) {
            lowering = true; // 꼭대기에서 멈춤 → 이제 내리기 시작
        }
        if (lowering) {
            kicker.setSpeedDegPerSec(-degPerSec);
        }
    }

    @Override
    public boolean isFinished() {
        return kicker.getAngleDeg() <= HOME_DEG || timer.seconds() >= maxSeconds;
    }

    @Override
    public void end(boolean interrupted) {
        kicker.stop();
    }
}
