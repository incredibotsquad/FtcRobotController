def calculate_wrist_servo_position(specimen_angle, turret_angle):
    """
    Calculates the wrist servo position so the wrist is perpendicular to the specimen,
    accounting for turret rotation and servo mapping.

    Args:
        specimen_angle (float): The angle of the specimen in degrees.
        turret_angle (float): The turret's current angle in degrees.

    Returns:
        float: The normalized wrist servo position (0.0 to 1.0).
    """
    desired_wrist_angle = specimen_angle - 90 - turret_angle
    wrist_servo_pos = 0.475 + ((desired_wrist_angle - 180) / 300.0)
    wrist_servo_pos = max(0.0, min(1.0, wrist_servo_pos))
    return wrist_servo_pos

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 3:
        print("Usage: python test.py <specimen_angle> <turret_angle>")
        sys.exit(1)
    specimen_angle = float(sys.argv[1])
    turret_angle = float(sys.argv[2])
    wrist_pos = calculate_wrist_servo_position(specimen_angle, turret_angle)
    print(f"Wrist servo position: {wrist_pos:.4f}")