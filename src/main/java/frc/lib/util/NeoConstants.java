package frc.lib.util;

public class NeoConstants {
    public static final double NOMINAL_VOLTAGE = 12.0; // [V]
    public static final double STALL_CURRENT = 105; // [A]
    public static final double STALL_TORQUE = 2.6; // [N*m]
    public static final double FREE_SPEED = 5676; // [rad/s]
    public static final double Kt = STALL_TORQUE / STALL_CURRENT; // [N*m/A]
    public static final double Kv = FREE_SPEED / NOMINAL_VOLTAGE; // [rad/s/V]
    public static final double R = NOMINAL_VOLTAGE / STALL_CURRENT; // [Ohm]
}
