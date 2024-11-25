import math

d = float(0)
e = float(0)
f = float(0)
g = float(0)

nmag = float(0)
nz = float(0)

x = float(0)
y = float(0)
z = float(0)

mag = float(0)
angle = float(0)

class Machine:
    A = 0
    B = 1
    C = 2

    def __init__(self, _d, _e, _f, _g):
        global d, e, f, g
        d = _d
        e = _e
        f = _f
        g = _g
    
    def theta(self, leg, hz, nx, ny):
        global nmag, nz, x, y, z, mag, angle, d, e, f, g
        # Create unit normal vector
        nmag = math.sqrt(math.pow(nx, 2) + math.pow(ny, 2) + 1)

        if nmag == 0:  # Prevent division by zero
            raise ValueError("Invalid normal vector magnitude. Check nx and ny inputs.")
        
        nx /= nmag
        ny /= nmag
        nz = 1 / nmag

        if leg == Machine.A:
            y = d + (e / 2) * (1 - (math.pow(nx, 2) + 3 * math.pow(nz, 2) + 3 * nz) / (nz + 1 - math.pow(nx,2) + (math.pow(nx, 4) - 3 * math.pow(nx, 2) * math.pow(ny, 2)) / ((nz + 1) * (nz + 1 - math.pow(nx, 2)))))

            z = hz + e * ny

            mag = math.sqrt(math.pow(y, 2) + math.pow(z, 2))

            angle = math.acos(y / mag) + math.acos((math.pow(mag, 2) + math.pow(f, 2) - math.pow(g, 2)) / (2 * mag * f))

        elif leg == Machine.B:
            x = (math.sqrt(3) / 2) * (e * (1 - (math.pow(nx, 2) + math.sqrt(3) * nx * ny) / (nz + 1)) - d)

            y = x / math.sqrt(3)

            z = hz - (e / 2) * (math.sqrt(3) * nx + ny)

            mag = math.sqrt(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2))

            angle = math.acos((math.sqrt(3) * x + y) / (-2 * mag)) + math.acos((math.pow(mag, 2) + math.pow(f, 2) - math.pow(g, 2)) / (2 * mag * f))

        elif leg == Machine.C:
            x = (math.sqrt(3) / 2) * (d - e * (1 - (math.pow(nx, 2) - math.sqrt(3) * nx * ny) / (nz + 1)))

            y = -x / math.sqrt(3)

            z = hz + (e / 2) * (math.sqrt(3) * nx - ny)

            mag = math.sqrt(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2))

            angle = math.acos((math.sqrt(3) * x - y) / (2 * mag)) + math.acos((math.pow(mag, 2) + math.pow(f, 2) - math.pow(g, 2)) / (2 * mag * f))

        #converts angle to degrees and returns the value
        return (angle * (180 / math.pi))
