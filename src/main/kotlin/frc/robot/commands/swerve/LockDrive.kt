package frc.robot.commands.swerve

import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearVelocity
import frc.robot.utils.metersPerSecond
import frc.robot.utils.radiansPerSecond

class LockDrive : DriveManager.DriveRequestBase() {
    override var vx: LinearVelocity? = 0.0.metersPerSecond
    override var vy: LinearVelocity? = 0.0.metersPerSecond
    override var omega: AngularVelocity? = 0.0.radiansPerSecond
    override var doLock: Boolean? = true
    override val priority: Int = DriverPriority.LOCK.ordinal
}
