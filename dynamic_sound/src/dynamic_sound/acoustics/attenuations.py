
def geometric(distance:float) -> float:
    """Compute the geometric attenuation based on distance.

    Args:
        distance (float): Distance between source and receiver in meters.

    Returns:
        float: Geometric attenuation factor.
    """
    if distance == 0:
        return 1.0
    return 1.0 / distance
