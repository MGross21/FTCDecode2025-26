package pioneer.pathing.motionprofile

import pioneer.helpers.MathUtils
import pioneer.pathing.motionprofile.constraints.AccelerationConstraint
import pioneer.pathing.motionprofile.constraints.VelocityConstraint
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.max
import kotlin.math.sqrt

private data class EvaluatedConstraint(
    val maxVel: Double,
    val maxAccel: Double,
)

/**
 * Singleton object for generating motion profiles.
 */
object MotionProfileGenerator {
    /**
     * Generates a motion profile from the given start and end states,
     * applying the specified velocity and acceleration constraints.
     * Accelerates as much as possible within the constraints,
     * Uses a forward and backward pass to ensure the constraints.
     */
    fun generateMotionProfile(
        startState: MotionState,
        endState: MotionState,
        velocityConstraint: VelocityConstraint,
        accelerationConstraint: AccelerationConstraint,
        resolution: Double = 0.1, // Length of each segment in cm
    ): MotionProfile {
        val distance = endState.x - startState.x
        val direction = if (distance >= 0) 1.0 else -1.0
        val length = abs(distance)

        if (length < 1e-6) {
            // No translation required, keep pose but zero dynamics
            return MotionProfile(listOf(MotionSegment(startState.stationary(), 0.0)))
        }

        // Work in a positive-displacement frame then map back using direction
        val samples = max(2, ceil(length / resolution).toInt() + 1)
        val displacements = MathUtils.linspace(0.0, length, samples)
        val constraintsList =
            displacements.map { s ->
                // Evaluate constraints at the signed displacement from the original start
                val queryS = startState.x + direction * s
                EvaluatedConstraint(
                    max(0.0, velocityConstraint[queryS]),
                    max(0.0, accelerationConstraint[queryS]),
                )
            }

        val forwardVelocities = DoubleArray(samples)
        val backwardVelocities = DoubleArray(samples)

        // Seed start velocity (projected along the travel direction)
        forwardVelocities[0] = max(0.0, startState.v * direction)
        for (i in 1 until samples) {
            val dx = displacements[i] - displacements[i - 1]
            val maxAccel = constraintsList[i - 1].maxAccel
            val reachable = sqrt(forwardVelocities[i - 1] * forwardVelocities[i - 1] + 2 * maxAccel * dx)
            forwardVelocities[i] = minOf(reachable, constraintsList[i].maxVel)
        }

        // Seed end velocity (projected along the travel direction)
        backwardVelocities[samples - 1] = minOf(max(0.0, endState.v * direction), constraintsList.last().maxVel)
        for (i in samples - 2 downTo 0) {
            val dx = displacements[i + 1] - displacements[i]
            val maxAccel = constraintsList[i].maxAccel
            val reachable = sqrt(backwardVelocities[i + 1] * backwardVelocities[i + 1] + 2 * maxAccel * dx)
            backwardVelocities[i] = minOf(forwardVelocities[i], reachable, constraintsList[i].maxVel)
        }

        // Build segments using the tighter of forward/backward reachable velocities
        val segments = mutableListOf<MotionSegment>()
        for (i in 0 until samples - 1) {
            val dx = displacements[i + 1] - displacements[i]
            val vStart = backwardVelocities[i]
            val vEnd = backwardVelocities[i + 1]
            val accel =
                if (dx < 1e-9) {
                    0.0
                } else {
                    (vEnd * vEnd - vStart * vStart) / (2 * dx)
                }
            val avgVel = vStart + vEnd
            val maxAccel = constraintsList[i].maxAccel
            val dt =
                when {
                    abs(dx) < 1e-9 -> 0.0
                    abs(avgVel) > 1e-9 -> 2 * dx / avgVel
                    maxAccel > 1e-9 -> sqrt(2 * dx / maxAccel)
                    else -> 0.0
                }

            val start = MotionState(
                x = startState.x + direction * displacements[i],
                v = direction * vStart,
                a = direction * accel,
            )
            segments.add(MotionSegment(start, dt))
        }

        return MotionProfile(segments)
    }
}
