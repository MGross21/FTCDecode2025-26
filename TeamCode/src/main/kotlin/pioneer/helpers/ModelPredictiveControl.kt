package pioneer.helpers

import kotlin.math.*

class ModelPredictiveControl(
    private val timeHorizon: Double,
    private val timeStep: Double,
    private val maxVelocity: Double,
    private val maxAcceleration: Double
) {
    data class State(val x: Double, val y: Double, val heading: Double, val vx: Double, val vy: Double, val omega: Double)
    data class Control(val ax: Double, val ay: Double, val alpha: Double)

    fun optimizePath(initialState: State, targetState: State): List<Control> {
        val controls = mutableListOf<Control>()
        var currentState = initialState

        for (t in 0 until (timeHorizon / timeStep).toInt()) {
            val control = calculateOptimalControl(currentState, targetState)
            controls.add(control)
            currentState = applyControl(currentState, control, timeStep)
        }

        return controls
    }

    private fun calculateOptimalControl(current: State, target: State): Control {
        val dx = target.x - current.x
        val dy = target.y - current.y
        val dHeading = target.heading - current.heading

        val desiredVx = dx / timeStep
        val desiredVy = dy / timeStep
        val desiredOmega = dHeading / timeStep

        val ax = (desiredVx - current.vx).coerceIn(-maxAcceleration, maxAcceleration)
        val ay = (desiredVy - current.vy).coerceIn(-maxAcceleration, maxAcceleration)
        val alpha = (desiredOmega - current.omega).coerceIn(-maxAcceleration, maxAcceleration)

        return Control(ax, ay, alpha)
    }

    private fun applyControl(state: State, control: Control, dt: Double): State {
        val newVx = (state.vx + control.ax * dt).coerceIn(-maxVelocity, maxVelocity)
        val newVy = (state.vy + control.ay * dt).coerceIn(-maxVelocity, maxVelocity)
        val newOmega = (state.omega + control.alpha * dt).coerceIn(-maxVelocity, maxVelocity)

        val newX = state.x + newVx * dt
        val newY = state.y + newVy * dt
        val newHeading = state.heading + newOmega * dt

        return State(newX, newY, newHeading, newVx, newVy, newOmega)
    }
}