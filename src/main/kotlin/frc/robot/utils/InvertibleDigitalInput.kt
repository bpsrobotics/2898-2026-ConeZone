package frc.robot.utils

import edu.wpi.first.wpilibj.DigitalInput

class InvertibleDigitalInput(channel: Int, val inverted: Boolean = false) : DigitalInput(channel) {
    override fun get(): Boolean = if (inverted) !super.get() else super.get()
}
