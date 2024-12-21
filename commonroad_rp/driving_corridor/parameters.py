class Parameters:

    def __init__(self):
        pass

    a_lat_max = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    a_lat_min = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    a_lon_max = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    a_lon_min = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    v_lat_max = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    v_lat_min = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    v_lon_max = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    v_lon_min = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default