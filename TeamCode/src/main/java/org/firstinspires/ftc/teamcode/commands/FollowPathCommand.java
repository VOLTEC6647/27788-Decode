package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

// Credit to team 21802 is noted.
public class FollowPathCommand extends CommandBase {
    private final Follower follower;
    private final PathChain pathChain;
    private boolean holdEnd = true;

    public FollowPathCommand(Follower follower, PathChain path) {
        this.follower = follower;
        this.pathChain = path;
    }

    /**
     * Decides whether or not to make the robot maintain its position once the path ends.
     *
     * @param holdEnd If the robot should maintain its ending position.
     * @return This command for compatibility in command groups.
     */
    public FollowPathCommand setHoldEnd(boolean holdEnd) {
        this.holdEnd = holdEnd;
        return this;
    }

    @Override
    public void initialize() {
        follower.followPath(pathChain, holdEnd);
    }

    @Override
    public void execute() {
        follower.update();
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}