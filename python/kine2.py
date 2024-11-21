import math

class Kinematics:
    A = 0
    B = 1
    C = 2
    def __init__(self, d, e, f, g):
        """
        Initialize the kinematic parameters.
        
        Args:
            d (float): Platform offset distance.
            e (float): Arm length.
            f (float): Connecting rod length.
            g (float): Length to end effector.
        """
        self.d = d
        self.e = e
        self.f = f
        self.g = g

    def compute_angle(self, leg, hz, nx, ny):
        """
        Compute the angle for the specified leg based on inputs.
        
        Args:
            leg (str): Identifier for the leg ('A', 'B', or 'C').
            hz (float): Height offset.
            nx (float): Normal vector X component.
            ny (float): Normal vector Y component.
        
        Returns:
            float: Angle for the specified leg in degrees.
        """
        # Create unit normal vector
        nmag = math.sqrt(nx**2 + ny**2 + 1)
        if nmag == 0:  # Prevent division by zero
            raise ValueError("Invalid normal vector magnitude. Check nx and ny inputs.")

        nx /= nmag
        ny /= nmag
        nz = 1 / nmag

        # Safeguard for avoiding division by zero in complex calculations
        epsilon = 1e-9

        # Calculate angle A, B, or C
        if leg == A:  # Leg A
            term1 = max(epsilon, nz + 1 - nx**2)
            term2 = max(epsilon, (nz + 1) * (nz + 1 - nx**2))
            y = self.d + (self.e / 2) * (1 - (nx**2 + 3 * nz**2 + 3 * nz) / (term1 + (nx**4 - 3 * nx**2 * ny**2) / term2))
            z = hz + self.e * ny
            mag = math.sqrt(y**2 + z**2)
            angle = math.acos(max(-1, min(1, y / mag))) + math.acos(max(-1, min(1, (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f))))
        elif leg == B:  # Leg B
            term1 = max(epsilon, nz + 1)
            x = (math.sqrt(3) / 2) * (self.e * (1 - (nx**2 + math.sqrt(3) * nx * ny) / term1) - self.d)
            y = x / math.sqrt(3)
            z = hz - (self.e / 2) * (math.sqrt(3) * nx + ny)
            mag = math.sqrt(x**2 + y**2 + z**2)
            angle = math.acos(max(-1, min(1, (math.sqrt(3) * x + y) / (-2 * mag)))) + math.acos(max(-1, min(1, (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f))))
        elif leg == C:  # Leg C
            term1 = max(epsilon, nz + 1)
            x = (math.sqrt(3) / 2) * (self.d - self.e * (1 - (nx**2 - math.sqrt(3) * nx * ny) / term1))
            y = -x / math.sqrt(3)
            z = hz + (self.e / 2) * (math.sqrt(3) * nx - ny)
            mag = math.sqrt(x**2 + y**2 + z**2)
            angle = math.acos(max(-1, min(1, (math.sqrt(3) * x - y) / (2 * mag)))) + math.acos(max(-1, min(1, (mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f))))
        else:
            raise ValueError("Invalid leg identifier. Use 'A', 'B', or 'C'.")

        return math.degrees(angle)  # Convert angle to degrees and return
