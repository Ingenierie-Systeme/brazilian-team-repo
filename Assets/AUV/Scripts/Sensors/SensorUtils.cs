using System;                                         // DateTimeOffset, Math
using UnityEngine;                                    // Vector3, Quaternion
using RosMessageTypes.Std;                            // HeaderMsg
using RosMessageTypes.BuiltinInterfaces;              // TimeMsg

// Helpers shared by the simulated sensors: Unity → ROS frame conversion, Gaussian noise and message headers.
// Frames: Unity is left-handed (X right, Y up, Z forward); ROS uses REP 103 "FLU"
// (x forward, y left, z up, right-handed). Vectors convert as (x, y, z)_ROS = (Z, -X, Y)_Unity.
public static class SensorUtils
{
    // Ordinary vector (position, velocity, acceleration, magnetic field) from Unity axes to ROS FLU axes
    public static Vector3 ToRos(Vector3 u) => new Vector3(u.z, -u.x, u.y);

    // Axial vector (angular velocity): the left/right-handed flip also reverses the rotation sense
    public static Vector3 AxialToRos(Vector3 u) => -ToRos(u);

    // Orientation from Unity to ROS FLU (same convention as the ROS-TCP-Connector; q and -q are the same rotation)
    public static Quaternion QuaternionToRos(Quaternion q) => new Quaternion(q.z, -q.x, q.y, -q.w);

    // Standard ROS header stamped with the current UTC time
    public static HeaderMsg Header(string frameId)
    {
        DateTimeOffset now = DateTimeOffset.UtcNow;                                  // wall-clock time
        var stamp = new TimeMsg((int)now.ToUnixTimeSeconds(), (uint)(now.Millisecond * 1_000_000)); // seconds + nanoseconds
        return new HeaderMsg(stamp, frameId);                                        // ROS 2 header (no seq field)
    }

    // 3×3 covariance matrix (row-major) with the same variance on the diagonal
    public static double[] Diagonal(double variance) => new[] { variance, 0, 0, 0, variance, 0, 0, 0, variance };

    // One sample of a standard normal distribution (Box-Muller transform)
    public static float Gaussian(System.Random rng)
    {
        double u1 = 1.0 - rng.NextDouble();                                          // (0, 1]: avoids log(0)
        double u2 = rng.NextDouble();                                                // [0, 1)
        return (float)(Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2)); // N(0, 1)
    }

    // Vector of independent Gaussian samples with the given standard deviation
    public static Vector3 GaussianVector(System.Random rng, float std) =>
        new Vector3(Gaussian(rng), Gaussian(rng), Gaussian(rng)) * std;               // each axis ~ N(0, std²)
}
