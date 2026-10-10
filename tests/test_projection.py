from observability.projection import body_height_projection


def test_floor_excluded_but_any_low_body_obstacle_remains():
    cells={(0,0,-1):{'state':'occupied'},(0,0,0):{'state':'free'},(0,0,1):{'state':'free'},(0,0,2):{'state':'free'},(1,0,0):{'state':'occupied'}}
    free,blocked=body_height_projection(cells,.25,.33,.28)
    assert free==[[0,0]] and blocked==[[1,0]]


def test_partial_known_column_is_unknown_not_free():
    free,blocked=body_height_projection({(0,0,0):{'state':'free'}},.25,.33,.28)
    assert not free and not blocked
