import math

def compute_angle(steps_x, steps_y):
    """
    Computes the angles for each leg based on the X and Y steps from the PID controller.
    
    Args:
        steps_x (int): Number of steps to move in the X direction.
        steps_y (int): Number of steps to move in the Y direction.
    
    Returns:
        dict: Dictionary containing angles for each leg ('A', 'B', 'C') in degrees.
    """
    # Define machine parameters (example values, replace with your system's specifics)
    d = 10  # Distance from platform center to each motor
    e = 5   # Arm length
    f = 15  # Length of connecting rod
    g = 20  # Length to end effector

    # Kinematic calculations for each leg
    leg_angles = {}

    # Calculate normal vector from steps (for simplicity, assume normalized values map to nx, ny)
    nx = steps_x / max(abs(steps_x), abs(steps_y), 1)  # Normalized direction X
    ny = steps_y / max(abs(steps_x), abs(steps_y), 1)  # Normalized direction Y
    nz = math.sqrt(max(0, 1 - nx**2 - ny**2))  # Assuming unit normal vector

    # Calculate angle for Leg A
    leg_angles['A'] = _compute_leg_angle('A', d, e, f, g, nx, ny, nz)

    # Calculate angle for Leg B
    leg_angles['B'] = _compute_leg_angle('B', d, e, f, g, nx, ny, nz)

    # Calculate angle for Leg C
    leg_angles['C'] = _compute_leg_angle('C', d, e, f, g, nx, ny, nz)

    return leg_angles

def _compute_leg_angle(leg, d, e, f, g, nx, ny, nz):
    """
    Helper function to compute the angle for a specific leg.
    
    Args:
        leg (str): Identifier for the leg ('A', 'B', or 'C').
        d, e, f, g (float): Machine-specific parameters.
        nx, ny, nz (float): Components of the unit normal vector.
    
    Returns:
        float: Angle for the specified leg in degrees.
    """
    if leg == 'A':
        y = d + (e / 2) * (1 - (nx**2 + 3 * nz**2 + 3 * nz) / (nz + 1 - nx**2 + (nx**4 - 3 * nx**2 * ny**2) / ((nz + 1) * (nz + 1 - nx**2))))
        z = e * ny
    elif leg == 'B':
        x = (math.sqrt(3) / 2) * (e * (1 - (nx**2 + math.sqrt(3) * nx * ny) / (nz + 1)) - d)
        y = x / math.sqrt(3)
        z = -(e / 2) * (math.sqrt(3) * nx + ny)
    elif leg == 'C':
        x = (math.sqrt(3) / 2) * (d - e * (1 - (nx**2 - math.sqrt(3) * nx * ny) / (nz + 1)))
        y = -x / math.sqrt(3)
        z = (e / 2) * (math.sqrt(3) * nx - ny)
    else:
        raise ValueError("Invalid leg identifier. Use 'A', 'B', or 'C'.")

    # Magnitude and angle calculation
    mag = math.sqrt(x**2 + y**2 + z**2)
    angle = math.acos((mag**2 + f**2 - g**2) / (2 * mag * f))

    return math.degrees(angle)  # Convert to degrees
