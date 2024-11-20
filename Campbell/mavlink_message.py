class MavlinkMessage:
    """
    MavlinkMessage: Represents a MAVLink message type.

    This class encapsulates the details of a specific MAVLink message, including its type, ID, 
    fields, and current data. It provides a structured way to store and access the different 
    parameters of a MAVLink message.

    Attributes:
        type (str): The type or name of the MAVLink message (e.g., "ESC_TELEMETRY_1_TO_4").
        id (int): The unique ID for the MAVLink message.
        fields (list): A list of field names representing the data points in the message.
        data (dict): A dictionary to store the current values for each field in the message.
    """

    def __init__(self, type, id, fields):
        """
        Initializes a MavlinkMessage instance.

        The constructor sets up the message type, ID, and the fields that describe the data
        contained in the message. It also initializes the data dictionary where each field's 
        value is set to `None`.

        Args:
            type (str): The name/type of the MAVLink message.
            id (int): The unique identifier for the message.
            fields (list): A list of field names describing the data points in the message.
        """
        self.type = type
        self.id = id
        self.fields = fields
        self.data = {field: None for field in fields}

ESC_TELEMETRY = MavlinkMessage("ESC_TELEMETRY_1_TO_4",
                               11030,
                               ["temperature", "voltage", "current", "rpm"])