import math

class Machine:
    def __init__(self, _d, _e, _f, _g):
        self.d = _d
        self.e = _e
        self.f = _f
        self.g = _g

    def theta(self, leg, hz, nx, ny):
        # Create unit normal vector
        nmag = math.sqrt(nx**2 + ny**2 + 1)
        nx /= nmag
        ny /= nmag
        nz = 1 / nmag

        # Calculate angle A, B, or C
        if leg == 'A':  # Leg A
            y = self.d + (self.e / 2) * (1 - (nx**2 + 3 * nz**2 + 3 * nz) / (nz + 1 - nx**2 + (nx**4 - 3 * nx**2 * ny**2) / ((nz + 1) * (nz + 1 - nx**2))))
            z = hz + self.e * ny
            mag = math.sqrt(y**2 + z**2)
            angle = math.acos(y / mag) + math.acos((mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f))
        elif leg == 'B':  # Leg B
            x = (math.sqrt(3) / 2) * (self.e * (1 - (nx**2 + math.sqrt(3) * nx * ny) / (nz + 1)) - self.d)
            y = x / math.sqrt(3)
            z = hz - (self.e / 2) * (math.sqrt(3) * nx + ny)
            mag = math.sqrt(x**2 + y**2 + z**2)
            angle = math.acos((math.sqrt(3) * x + y) / (-2 * mag)) + math.acos((mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f))
        elif leg == 'C':  # Leg C
            x = (math.sqrt(3) / 2) * (self.d - self.e * (1 - (nx**2 - math.sqrt(3) * nx * ny) / (nz + 1)))
            y = -x / math.sqrt(3)
            z = hz + (self.e / 2) * (math.sqrt(3) * nx - ny)
            mag = math.sqrt(x**2 + y**2 + z**2)
            angle = math.acos((math.sqrt(3) * x - y) / (2 * mag)) + math.acos((mag**2 + self.f**2 - self.g**2) / (2 * mag * self.f))
        else:
            raise ValueError("Invalid leg identifier. Use 'A', 'B', or 'C'.")

        return angle * (180 / math.pi)  # Convert angle to degrees and return the value