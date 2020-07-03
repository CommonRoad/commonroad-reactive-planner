from scipy.spatial import distance


class evaluation:

    #dynarry hierarchy:     1. egovehicle / other vehicle
    #                       2. time steps
    #                       3. vehicle numbers
    #                       4. position/acceleration/velocity/orientation
    #                       5. x/y coordinate for position

    @staticmethod
    def closest_distance(dynarray):
        if len(dynarray[0][0]) < 1:                             # no ego vehicle
            return 0                                            # no distance possible
        a = 9001
        i = 0
        while i < len(dynarray[0]):                             # going through all timesteps
            for j in dynarray[0][i]:                            # iterating through all ego vehicles
                for k in dynarray[1][i]:                        # iterating through all other vehicles
                    b = distance.euclidean(j[0], k[0])          # computing the distance
                    if b < a:                                   # shorter distance found
                        a = b                                   # updating the shortest distance
            i = i + 1
        return a

    @staticmethod
    def strongestbreake(dynarray, dist):
        if len(dynarray[0][0]) < 1:                             # no ego vehicle
            return 0                                            # no distance possible
        a = 0
        i = 0
        while i < len(dynarray[0]):                             # going through all timesteps
            for j in dynarray[0][i]:                            # iterating through all ego vehicles
                for k in dynarray[1][i]:                        # iterating through all other vehicles
                    if distance.euclidean(j[0], k[0])<dist:     # computing the distance
                        if k[1] < a:                            # lower acceleration found
                            a = k[1]                            # updating
            i = i + 1
        return a


    @staticmethod
    def avgspeed(dynarray, dist):
        if len(dynarray[0][0]) < 1:                             # no ego vehicle
            return 0                                            # no distance possible
        a = 0                                                   # total speed
        b = 0                                                   # total number of data evaluated
        i = 0
        while i < len(dynarray[0]):                             # going through all timesteps
            for j in dynarray[0][i]:                            # iterating through all ego vehicles
                for k in dynarray[1][i]:                        # iterating through all other vehicles
                    if distance.euclidean(j[0], k[0]) < dist:   # computing the distance
                        a = a + k[2]                            # updating
                        b = b + 1                               # updating
            i = i + 1
        if b > 0:                                               # no data evaluated, maybe dist, was to short
            return a/b
        else:
            return 0

    @staticmethod
    def test_max_speed(dynarray):                               # for testing dynarrays without ego vehicles
        a = 0                                                   # current max speed
        i = 0
        while i < len(dynarray[1]):                             # going through all timesteps
            for k in dynarray[1][i]:                            # iterating through all other vehicles
                if k[2] > a:                                    # higher speed found
                    a = k[2]                                    # updating
            i = i + 1
        return a




