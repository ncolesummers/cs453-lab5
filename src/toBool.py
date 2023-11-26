def to_bool(value) -> bool:
    """
    Converts 'something' to boolean. Raises exception if it gets a string it doesn't handle.
    Case is ignored for strings. These string values are handled:
    True: 'True', "1", "TRue", "yes", "y", "t"
    False: "", "0", "faLse", "no", "n", "f"
    :param value:  something you want to evaluate as True or False
    :return: Boolean representation of the value
    """
    if isinstance(value, bool):
        return value
    if not value:
        return False
    if isinstance(value, str):
        value = value.lower().strip()
        if value in ["true", "1", "yes", "y", "t"]:
            return True
        if value in ["false", "0", "no", "n", "f"]:
            return False
        raise Exception(f"Invalid value for boolean conversion: {value}")
    raise Exception(f"Invalid value for boolean conversion: {value}")
