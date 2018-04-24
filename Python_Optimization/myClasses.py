# MyClasses
import numpy as np
import cv2


class stru:
    def __init__(self):
        self.filename = None
        self.raw = None
        self.gray = None
        self.corners = None
        self.rvec = None
        self.tvec = None
        self.pix = None
        self.worldPoints = None
        self.xypix = None


class MyPoint3D:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None


class MyRodrigues:
    def __init__(self):
        self.r1 = None
        self.r2 = None
        self.r3 = None


class MyTransform(MyRodrigues, MyPoint3D):
    def __init__(self, rvec, tvec):
        MyPoint3D.__init__(self)
        self.x = tvec[0]
        self.y = tvec[1]
        self.z = tvec[2]
        MyRodrigues.__init__(self)
        self.r1 = rvec[0]
        self.r2 = rvec[1]
        self.r3 = rvec[2]

    def getT(self):
        """Transformation matrix
            Gets 4x4 Tranform from internal xyz and Rodriges
        """
        T = np.eye(4)
        dxyz = np.array([self.x, self.y,
                         self.z], dtype=np.float)
        T[0:3, 3] = dxyz.transpose()
        rod = np.array([self.r1, self.r2, self.r3], dtype=np.float)
        DCM = cv2.Rodrigues(rod)
        T[0:3, 0:3] = DCM[0]
        return T

    def setPointRod(self, T):
        """Transformation matrix
            Sets point and rod values from 4x4 Tranform
        """
        dxyz = T[0:3, 3]
        dxyz = dxyz.transpose()
        rods, _ = cv2.Rodrigues(T[0:3, 0:3])
        rods = rods.transpose()
        rod = rods[0]
        self.x = dxyz[0]
        self.y = dxyz[1]
        self.z = dxyz[2]
        self.r1 = rod[0]
        self.r2 = rod[1]
        self.r3 = rod[2]

    def printValues(self):
        return "xyz = " + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + " rod= " + str(self.r1) + ", " + str(self.r2) + ", " + str(self.r3)


class MyDetection(MyTransform):
    def __init__(self, rvec, tvec, cam, ar):
        MyTransform.__init__(self, rvec, tvec)
        self.camera = cam
        self.aruco = ar

    def __str__(self):
        return "Detection from camera " + str(self.camera) + " of aruco " + str(self.aruco)


class MyCamera(MyTransform):
    def __init__(self, T, id):
        MyTransform.setPointRod(self, T)
        self.id = id


class MyAruco(MyTransform):
    def __init__(self, T, id):
        MyTransform.setPointRod(self, T)
        self.id = id


class MyX:
    def __init__(self):
        self.cameras = []
        self.arucos = []
        self.v = []

    def plotArucosIn3D(self, ax3D):

        # Define global reference marker
        l = 0.082
        Pc = np.array([[-l/2, l/2, 0], [l/2, l/2, 0],
                       [l/2, -l/2, 0], [-l/2, -l/2, 0]])

        # Referential of arucos
        s = 0.150
        P = np.array([[0, 0, 0], [s, 0, 0], [0, s, 0], [0, 0, s]])

        for aruco in self.arucos:
            T = aruco.getT()
            rot = T[0:3, 0:3]
            trans = T[0: 3, 3]

            Ptransf = []
            wp = np.zeros((4, 3))

            for p in P:
                Ptransf.append(rot.dot(p) + trans)

            i = 0
            for p in Pc:
                wp[i, :] = rot.dot(p) + trans
                i = i + 1

            # Draw aruco axis
            ax3D.plot([Ptransf[0][0], Ptransf[1][0]], [Ptransf[0][1], Ptransf[1][1]], [
                      Ptransf[0][2], Ptransf[1][2]], 'r-')
            ax3D.plot([Ptransf[0][0], Ptransf[2][0]], [Ptransf[0][1], Ptransf[2][1]], [
                      Ptransf[0][2], Ptransf[2][2]], 'g-')
            ax3D.plot([Ptransf[0][0], Ptransf[3][0]], [Ptransf[0][1], Ptransf[3][1]], [
                      Ptransf[0][2], Ptransf[3][2]], 'b-')

            # Draw aruco point 3D
            wp = wp.transpose()
            ax3D.plot(wp[0, :], wp[1, :], wp[2, :], 'k.')

    def toVector(self):
        for i in range(len(self.cameras)):
            self.v.extend([self.cameras[i].r1, self.cameras[i].r2, self.cameras[i].r3,
                           self.cameras[i].x, self.cameras[i].y, self.cameras[i].z])

        for i in range(len(self.arucos)):
            self.v.extend([self.arucos[i].r1, self.arucos[i].r2, self.arucos[i].r3,
                           self.arucos[i].x, self.arucos[i].y, self.arucos[i].z])
        print self.v

    def fromVector(self, v):
        n_cameras = len(self.cameras)
        for i in range(n_cameras):
            self.cameras[i].r1 = v[i*6]
            self.cameras[i].r2 = v[i*6+1]
            self.cameras[i].r3 = v[i*6+2]
            self.cameras[i].x = v[i*6+3]
            self.cameras[i].y = v[i*6+4]
            self.cameras[i].z = v[i*6+5]
        for i in range(len(self.arucos)):
            self.arucos[i].r1 = v[i*6+n_cameras*6]
            self.arucos[i].r2 = v[i*6+1+n_cameras*6]
            self.arucos[i].r3 = v[i*6+2+n_cameras*6]
            self.arucos[i].x = v[i*6+3+n_cameras*6]
            self.arucos[i].y = v[i*6+4+n_cameras*6]
            self.arucos[i].z = v[i*6+5+n_cameras*6]

# myt = MyTransform()
# T = np.array([[0, 1, 0, 2], [-1, 0, 0, 0], [0, 0, 1, 0],
#               [0, 0, 0, 1]], dtype=np.float)
# print(T)
# myt.setPointRod(T)
# myt.printValues()
# print(myt.getT())

# exit(0)
