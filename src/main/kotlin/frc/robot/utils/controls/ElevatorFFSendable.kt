package frc.robot.utils.controls

import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder

/** Exposes the gains of the same feedforward instance used by the controller. */
class ElevatorFFSendable(private val feedforward: ElevatorFeedforward) : Sendable {
    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("ElevatorFeedforward")
        builder.addDoubleProperty("kS", feedforward::getKs, feedforward::setKs)
        builder.addDoubleProperty("kG", feedforward::getKg, feedforward::setKg)
        builder.addDoubleProperty("kV", feedforward::getKv, feedforward::setKv)
    }
}
