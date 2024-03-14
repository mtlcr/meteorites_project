
import math
from math import *

# import sys
# sys.setrecursionlimit(2000)

class matrix:

    # implements basic operations of a matrix class

    def __init__(self, value):
        self.value = value
        self.dimx = len(value)
        self.dimy = len(value[0])
        if value == [[]]:
            self.dimx = 0

    def zero(self, dimx, dimy):
        # check if valid dimensions
        if dimx < 1 or dimy < 1:
            raise ValueError("Invalid size of matrix")
        else:
            self.dimx = dimx
            self.dimy = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        # check if valid dimension
        if dim < 1:
            raise ValueError("Invalid size of matrix")
        else:
            self.dimx = dim
            self.dimy = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1

    def show(self):
        for i in range(self.dimx):
            print(self.value[i])
        print(' ')

    def __add__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError("Matrices must be of equal dimensions to add")
        else:
            # add if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    def __sub__(self, other):
        # check if correct dimensions
        if self.dimx != other.dimx or self.dimy != other.dimy:
            raise ValueError("Matrices must be of equal dimensions to subtract")
        else:
            # subtract if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    def __mul__(self, other):
        # check if correct dimensions
        if self.dimy != other.dimx:
            raise ValueError("Matrices must be m*n and n*p to multiply")
        else:
            # multiply if correct dimensions
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
            return res

    def transpose(self):
        # compute transpose
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res

    # Thanks to Ernesto P. Adorio for use of Cholesky and CholeskyInverse functions

    def Cholesky(self, ztol=1.0e-5):
        # Computes the upper triangular Cholesky factorization of
        # a positive definite matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        for i in range(self.dimx):
            S = sum([(res.value[k][i]) ** 2 for k in range(i)])
            d = self.value[i][i] - S
            if abs(d) < ztol:
                res.value[i][i] = 0.0
            else:
                if d < 0.0:
                    raise ValueError("Matrix not positive-definite")
                res.value[i][i] = sqrt(d)
            for j in range(i + 1, self.dimx):
                S = sum([res.value[k][i] * res.value[k][j] for k in range(self.dimx)])
                if abs(S) < ztol:
                    S = 0.0
                try:
                    res.value[i][j] = (self.value[i][j] - S) / res.value[i][i]
                except:
                    raise ValueError("Zero diagonal")
        return res

    def CholeskyInverse(self):
        # Computes inverse of matrix given its Cholesky upper Triangular
        # decomposition of matrix.
        res = matrix([[]])
        res.zero(self.dimx, self.dimx)

        # Backward step for inverse.
        for j in reversed(range(self.dimx)):
            tjj = self.value[j][j]
            S = sum([self.value[j][k] * res.value[j][k] for k in range(j + 1, self.dimx)])
            res.value[j][j] = 1.0 / tjj ** 2 - S / tjj
            for i in reversed(range(j)):
                res.value[j][i] = res.value[i][j] = -sum(
                    [self.value[i][k] * res.value[k][j] for k in range(i + 1, self.dimx)]) / self.value[i][i]
        return res

    def inverse(self):
        aux = self.Cholesky()
        res = aux.CholeskyInverse()
        return res

    def __repr__(self):
        return repr(self.value)


########################################
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib
    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


class Turret(object):
    """The laser used to defend against invading Meteorites."""

    def __init__(self, init_pos, max_angle_change,
                 dt):
        """Initialize the Turret."""
        self.x_pos = init_pos['x']
        self.y_pos = init_pos['y']
        self.max_angle_change = max_angle_change
        self.dt = dt
        self.meteorite_dict = {}
        self.meteorite_dict_copy = {}
        self.target_meteorite_set = {}
        self.lowest_meteorite = None
        self.lowest_y = None
        self.target_meteor_id = ''
        self.timer = 0
        self.meteor_list = []
    def run_kf_single_meteorite(self, meteorite_x_obs, meteorite_y_obs):
        """Observe meteorite locations and predict their positions at time t+1.

        This function is Part 1a of the Meteorites project, which tests your
        Kalman Filter implementation on its ability to predict the location of
        a meteorite at time t+1 given the x- and y-coordinates of its position
        at time t. (No noise is applied to the meteorite coordinate
        observations in this part of the project, but your KF will need to be
        able to deal with noisy measurements starting in Part 1b.)

        Parameters
        ----------
        self = a reference to the current object, the Turret
        meteorite_x_obs = the meteorite's observed x-coordinate at time t
        meteorite_y_obs = the meteorite's observed y-coordinate at time t

        Returns
        -------
        meteorite_x_t_plus_1, meteorite_y_t_plus_1 = the x- and y-coordinates
        of the meteorite's location estimate at time t+1

        Notes
        -----
        You will want to create matrices within your Turret class that this
        function will use in your Kalman Filter calculations. See the project
        PDF's FAQ question on "How do I share data between
        `observe_and_estimate`, `get_laser_action`, `run_kf_single_meteorite`,
        and other functions in my Turret class?" for a demonstration of how to
        create a class variable that can be accessed from anywhere within the
        class.
        You may re-use those class variables in the other parts of this project
        if you wish.
        This function, however, is not intended to be used in parts of the
        project other than 1a. Please use it in Part 1a to test whether your Kalman
        Filter implementation works correctly in a scenario with no noise. You
        may then copy and paste code from this function into other functions to
        use in other parts of the project.

        """

        # x: initial state (location, velocity, and acceleration)
        x = matrix([[0.], [0.], [0.], [0.], [0.]])
        # P: initial uncertainty
        P = matrix([[1., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0.],
                    [0., 0., 1., 0., 0.],
                    [0., 0., 0., 1., 0.],
                    [0., 0., 0., 0., 1.]])
        # F: next state function
        F = matrix([[1., 0., 0.1, 0, 0.01 / 6],
                    [0., 1., 0., 0.1, 0.01 / 2],
                    [0., 0., 1., 0., 0.1 / 3],
                    [0., 0., 0., 1., 0.1],
                    [0., 0., 0., 0., 1.]])
        # u: external motion
        u = matrix([[0.], [0.], [0.], [0.], [0.]])
        # H: measurement function
        H = matrix([[1., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0.]])

        # R: measurement uncertainty
        R = matrix([[0., 0.],
                    [0., 0.]])
        # identity matrix
        I = matrix([[1., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0.],
                    [0., 0., 1., 0., 0.],
                    [0., 0., 0., 1., 0.],
                    [0., 0., 0., 0., 1.]])
        # measurement update
        z = matrix([[meteorite_x_obs], [meteorite_y_obs]])
        y = z - (H * x)
        s = H * P * H.transpose() + R
        k = P * H.transpose() * s.inverse()
        x = x + (k * y)
        P = (I - (k * H)) * P

        # prediction
        x = (F * x) + u
        P = F * P * F.transpose()
        meteorite_x_t_plus_1 = x.value[0][0]
        meteorite_y_t_plus_1 = x.value[1][0]
        print('meteorite_x_t_plus_1:')
        print(meteorite_x_t_plus_1)
        print('meteorite_y_t_plus_1:')
        print(meteorite_y_t_plus_1)
        return meteorite_x_t_plus_1, meteorite_y_t_plus_1



    def observe_and_estimate(self, noisy_meteorite_observations):
        """Observe meteorite locations and predict their positions at time t+1.

        Parameters
        ----------
        self = a reference to the current object, the Turret
        noisy_meteorite_observations = a list of noisy observations of
            meteorite locations, taken at time t

        Returns
        -------
        A tuple or list of tuples containing (i, x, y), where i, x, and y are:
        i = the meteorite's ID
        x = the estimated x-coordinate of meteorite i's position for time t+1
        y = the estimated y-coordinate of meteorite i's position for time t+1

        Return format hint:
        For a tuple of tuples, this would look something like
        ((1, 0.4, 0.381), (2, 0.77, 0.457), ...)
        For a list of tuples, this would look something like
        [(1, 0.4, 0.381), (2, 0.77, 0.457), ...]

        Notes
        -----
        Each observation in noisy_meteorite_observations is a tuple
        (i, x, y), where i is the unique ID for an meteorite, and x, y are the
        x, y locations (with noise) of the current observation of that
        meteorite at this timestep. Only meteorites that are currently
        'in-bounds' will appear in this list, so be sure to use the meteorite
        ID, and not the position/index within the list to identify specific
        meteorites.
        The list/tuple of tuples you return may change in size as meteorites
        move in and out of bounds.
        """
        # TODO: Update the Turret's estimate of where the meteorites are
        # located at the current timestep and return the updated estimates

        # meteorite_dict{Meteor ID: y_loc, x_loc, P, x}
        return_list = []

        # F: next state function
        F = matrix([[1., 0., 0.1, 0, 0.01 / 2],
                    [0., 1., 0., 0.1, 0.01 / 6],
                    [0., 0., 1., 0., 0.1],
                    [0., 0., 0., 1., 0.1/3],
                    [0., 0., 0., 0., 1.]])

        # u: external motion
        u = matrix([[0.], [0.], [0.], [0.], [0.]])

        # H: measurement function
        H = matrix([[1., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0.]])

        # R: measurement uncertainty
        R = matrix([[10., 0.],
                    [0.,10.]])

        # identity matrix
        I = matrix([[1., 0., 0., 0., 0.],
                    [0., 1., 0., 0., 0.],
                    [0., 0., 1., 0., 0.],
                    [0., 0., 0., 1., 0.],
                    [0., 0., 0., 0., 1.]])

        # Looping through each meteorite:
        for i in range(len(noisy_meteorite_observations)):
            meteor_id = noisy_meteorite_observations[i][0]
            # print("meteor")
            # print(noisy_meteorite_observations)
            # If meteor is not in the dictionary
            if meteor_id not in self.meteorite_dict:
                # print("New initialization ")
                # x: initial state (x location,y location, x velocity, y velocity, and acceleration)
                x = matrix([ [noisy_meteorite_observations[i][2]],[noisy_meteorite_observations[i][1]], [0.0], [0.0], [0.0]])
                # P: initial uncertainty
                p = matrix([[4., 0., 0., 0., 0.],
                            [0., 4., 0., 0., 0.],
                            [0., 0., 4., 0., 0.],
                            [0., 0., 0., 4., 0.],
                            [0., 0., 0., 0., 4.]])
                # print("Before measurement update")
                # print("x")
                # print(x)
                # print("p")
                # print(p)
            else:
                #  Meteorite already in dictionary
                # print("Existing meteor")
                x = self.meteorite_dict[meteor_id][3]
                p = self.meteorite_dict[meteor_id][2]

            # measurement update
            z = matrix([ [noisy_meteorite_observations[i][2]],[noisy_meteorite_observations[i][1]]])
            y = z - (H * x)
            s = H * p * H.transpose() + R
            k = p * H.transpose() * s.inverse()
            x = x + (k * y)
            p = (I - (k * H)) * p
            # print("After measurement update")
            # print("z")
            # print(z)
            # print("y")
            # print(y)
            # print("s")
            # print(s)
            # print("k")
            # print(k)
            # print("x")
            # print(x)
            # print("p")
            # print(p)

            # prediction
            x = (F * x) + u
            p = F * p * F.transpose()
            # print("After prediction")
            # print("x")
            # print(x)
            # print("p")
            # print(p)

            # Assign new values (x_loc, y_loc, P matrix, x matrix) to Meteorite with meteor_id
            self.meteorite_dict[meteor_id] = [ x.value[1][0],x.value[0][0], p, x]
            return_list.append([meteor_id, x.value[1][0] ,x.value[0][0]])
            # print("meteor dictionary")
            # print(self.meteorite_dict[meteor_id])
        self.meteor_list = noisy_meteorite_observations

        # print(return_list)
        return return_list

    def get_laser_action(self, current_aim_rad):
        """Return the laser's action; it can change its aim angle or fire.

        Parameters
        ----------
        self = a reference to the current object, the Turret
        current_aim_rad = the laser turret's current aim angle, in radians,
            provided by the simulation.



        Returns
        -------
        Float (desired change in laser aim angle, in radians), OR
            String 'fire' to fire the laser

        Notes
        -----
        The laser can aim in the range [0.0, pi].

        The maximum amount the laser's aim angle can change in a given timestep
        is self.max_angle_change radians. Larger change angles will be
        clamped to self.max_angle_change, but will keep the same sign as the
        returned desired angle change (e.g. an angle change of -3.0 rad would
        be clamped to -self.max_angle_change).

        If the laser is aimed at 0.0 rad, it will point horizontally to the
        right; if it is aimed at pi rad, it will point to the left.

        If the value returned from this function is the string 'fire' instead
        of a numerical angle change value, the laser will fire instead of
        moving.
        """
        # TODO: Update the change in the laser aim angle, in radians, based
        # on where the meteorites are currently, OR return 'fire' to fire the
        # laser at a meteorite
        print('------------------------------------------------------')
        print('------------------------------------------------------')
        print('------------------------------------------------------')
        # Set initial parameters:
        delta_angle = 0.2
        trial_counter = 0
        threshold = -0.9
        self.timer += self.dt
        if self.lowest_y == None:
            self.lowest_y = 1.
        # Remove all meteorite below threshold or shot down
        # self.meteorite_dict_copy = self.meteorite_dict
        # for key in self.meteorite_dict_copy:
        #     if key not in self.meteor_list:
        #         print('deleting ', key)
        #         del self.meteorite_dict[key]
        # Remove meteorite from dictionary id it disappears:
        # for key in [key for key in self.meteorite_dict if key not in self.meteor_list]:
        #     del self.meteorite_dict[key]

        # Make copy of meteorite dictionary
        meteorite_mirror = self.meteorite_dict
        # Remove meteorite from dictionary id it disappears:
        for key in [key for key in meteorite_mirror if key not in self.meteor_list]:
            del meteorite_mirror[key]

        # if self.lowest_meteorite == None:
        #     self.lowest_meteorite = self.meteorite_dict[1002]
            # first = next(iter(self.meteorite_dict))
            # print('first ', first)
            # self.lowest_meteorite = self.meteorite_dict[first]

        # Find the lowest meteor by looping through all meteorite that are above the threshold
        for i in self.meteorite_dict:
            if i == -1 or threshold >= self.meteorite_dict[i][1]:
                continue
            else:
                print(i, ' y_loc ', self.meteorite_dict[i][1])
                print('lowest y ',   self.lowest_y)
                if (threshold <= self.meteorite_dict[i][1] < self.lowest_y) or self.lowest_meteorite is None:
                    print('true')
                    self.lowest_meteorite = self.meteorite_dict[i]
                    self.target_meteor_id = i
                    self.lowest_y = self.meteorite_dict[i][1]
                else:
                    print('false')
        # Determine the angle for the lowest meteor
        print('after loop')
        print('lowest y ',   self.lowest_y)
        print('lowest_meteorite ID', self.target_meteor_id)
        print(self.lowest_meteorite[1])
        if self.lowest_meteorite[0] > 0:
                # If x coordinate > 1, then alpha angle = math.atan((y+1)/x). Else x coordinate = pi - math.atan((y+1)/x)
            alpha = math.atan((self.lowest_meteorite[1]+1)/self.lowest_meteorite[0])
            print('alpha')
            print(alpha)
        else:
            alpha = math.pi + math.atan((self.lowest_meteorite[1]+1)/self.lowest_meteorite[0])
            print('alpha')
            print(alpha)
        # Determine the delta change in turret angle
        print('y coord ', self.lowest_meteorite[1])
        print('x coord ', self.lowest_meteorite[0])
        delta_angle = alpha - current_aim_rad
        # if alpha >= current_aim_rad:
        #     delta_angle = alpha - current_aim_rad
        # else:
        #     delta_angle = current_aim_rad - alpha
        print('delta_angle')
        print(delta_angle)
        print('curent aim rad')
        print(current_aim_rad)
        print('dt ', self.timer)
        trial_counter += trial_counter
        print(self.meteor_list)
        if abs(delta_angle) <= 0.001 and self.timer >= 3 and self.lowest_meteorite is not None and self.target_meteor_id is not None:
            print('firing')
            self.lowest_meteorite = None
            self.target_meteor_id = None
            self.lowest_y = 1
            # del self.meteorite_dict[self.target_meteor_id]
            return 'fire'
        else:
            print('changing angle')
            return delta_angle
        # return delta_angle


