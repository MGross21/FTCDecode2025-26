package pioneer.helpers

import kotlin.math.*

class ProjectileMotion(
    private val gravity: Double = 981.0 // gravity in cm/s², default Earth gravity
) {

    var initialVelocity: Double? = null
        private set

    var launchAngleRadians: Double? = null
        private set

    /**
     * Updates the internal state with the launch angle and minimum initial velocity
     * required to hit the target from the start pose within the given time interval.
     *
     * @param startPose Starting pose including position, velocity, acceleration.
     * @param targetPose Target pose including position, velocity, acceleration.
     * @param deltaTime Time interval for relative motion compensation.
     * @param minAngleRad Minimum launch angle to consider (default 5 degrees).
     * @param maxAngleRad Maximum launch angle to consider (default 85 degrees).
     * @param tolerance Convergence tolerance for the search (default 1e-5 radians).
     * @param maxIterations Maximum iterations for the search (default 100).
     */
    fun update(
        startPose: Pose,
        targetPose: Pose,
        deltaTime: Double,
        minAngleRad: Double = 5.0 * PI / 180.0,
        maxAngleRad: Double = 85.0 * PI / 180.0,
        tolerance: Double = 1e-5,
        maxIterations: Int = 100
    ) {
        val result = findOptimalLaunchAngle(
            startPose,
            targetPose,
            deltaTime,
            minAngleRad,
            maxAngleRad,
            tolerance,
            maxIterations
        )

        if (result == null) {
            initialVelocity = null
            launchAngleRadians = null
        } else {
            launchAngleRadians = result.first
            initialVelocity = result.second
        }
    }

    /**
     * Uses ternary search to find the launch angle that minimizes required initial velocity.
     * Returns a Pair of (launch angle in radians, minimal initial velocity) or null if no solution.
     */
    private fun findOptimalLaunchAngle(
        startPose: Pose,
        targetPose: Pose,
        deltaTime: Double,
        angleMinRad: Double,
        angleMaxRad: Double,
        tolerance: Double,
        maxIterations: Int
    ): Pair<Double, Double>? {
        // Velocity calculation function returning a large number if no solution exists
        fun velocityAt(angleRad: Double): Double {
            return computeInitialVelocity(startPose, targetPose, angleRad, deltaTime) ?: Double.MAX_VALUE
        }

        var low = angleMinRad
        var high = angleMaxRad
        var bestAngle = low
        var bestVelocity = velocityAt(low)

        var iterations = 0
        while (high - low > tolerance && iterations < maxIterations) {
            val mid1 = low + (high - low) / 3.0
            val mid2 = high - (high - low) / 3.0

            val v1 = velocityAt(mid1)
            val v2 = velocityAt(mid2)

            if (v1 < bestVelocity) {
                bestVelocity = v1
                bestAngle = mid1
            }
            if (v2 < bestVelocity) {
                bestVelocity = v2
                bestAngle = mid2
            }

            // Narrow search space based on function comparison (ternary search for unimodal function)
            if (v1 < v2) {
                high = mid2
            } else {
                low = mid1
            }
            iterations++
        }

        return if (bestVelocity == Double.MAX_VALUE) null else Pair(bestAngle, bestVelocity)
    }

    /**
     * Computes the initial velocity required to hit the target for a given launch angle.
     * Returns null if no valid projectile solution exists.
     */
    private fun computeInitialVelocity(
        startPose: Pose,
        targetPose: Pose,
        launchAngleRad: Double,
        deltaTime: Double
    ): Double? {
        val deltaX = targetPose.x - startPose.x
        val deltaY = targetPose.y - startPose.y
        val deltaVx = targetPose.vx - startPose.vx
        val deltaVy = targetPose.vy - startPose.vy
        val deltaAx = targetPose.ax - startPose.ax
        val deltaAy = targetPose.ay - startPose.ay

        // Adjust position for relative velocity and acceleration over deltaTime
        val adjustedX = deltaX - deltaVx * deltaTime - 0.5 * deltaAx * deltaTime * deltaTime
        val adjustedY = deltaY - deltaVy * deltaTime - 0.5 * deltaAy * deltaTime * deltaTime

        val cosAngle = cos(launchAngleRad)
        val tanAngle = tan(launchAngleRad)

        if (abs(cosAngle) < 1e-8) return null // avoid division by zero near vertical launches

        val denominator = 2 * (adjustedY - tanAngle * adjustedX) * cosAngle * cosAngle
        if (denominator <= 0.0) return null // no feasible solution exists

        val numerator = gravity * adjustedX * adjustedX
        val velocitySquared = numerator / denominator

        return if (velocitySquared < 0.0) null else sqrt(velocitySquared)
    }
}