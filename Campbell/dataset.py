import numpy as np

class DataSet:
    """
    DataSet: Dynamic storage for a dataset (timestamp and value) to be displayed or plotted.

    Attributes:
        label (str): Descriptive name for the dataset.
        unit (str): Unit of measurement for the dataset (e.g., seconds, meters).
        scale (int): Scale factor for the dataset values.
        t (numpy.array): Array storing the timestamps of the data points.
        y (numpy.array): Array storing the values corresponding to the timestamps.
        update_function (callable, optional): A function to automatically update the dataset with new data.
    """

    def __init__(self, label, unit, update_function=None):
        """
        Initializes a DataSet instance.

        Args:
            label (str): A descriptive label for the dataset.
            unit (str): Unit of measurement for the dataset values.
            update_function (callable, optional): A function to automatically update the dataset.
        """
        self.label = label
        self.unit = unit
        self.scale = 0
        self.t = np.array([])
        self.y = np.array([])
        self.update_function = update_function

    def add_data(self, t, y):
        """
        Adds new data values to the dataset.

        If the value exceeds 100 times the current scale, the scale is adjusted automatically
        to keep the values manageable for display.

        Args:
            t (float): Timestamp for the new data point.
            y (float): Value corresponding to the timestamp.
        """
        if y / (10 ** self.scale) > 100:
            self.scale = np.ceil(np.log10(y / 100))

        self.t = np.append(self.t, t)
        self.y = np.append(self.y, y)
    
    def update(self):
        """
        Adds new data values to the dataset using the update function (if provided).

        If an update function is defined, it calls the function to obtain the new data 
        and adds it to the dataset.
        """
        if self.update_function is not None:
            t, y = self.update_function()
            self.add_data(t, y)
    
    def clear_data(self):
        """
        Clears all data from the dataset, resetting the timestamps and values.
        """
        self.scale = 0
        self.t = np.array([])
        self.y = np.array([])

    def get_label(self):
        """
        Returns the label of the dataset.

        Returns:
            str: The dataset label.
        """
        return self.label
    
    def get_unit(self):
        """
        Returns the unit of measurement for the dataset.

        Returns:
            str: The unit of the dataset.
        """
        return self.unit

    def get_scale(self):
        """
        Returns the current scale factor of the dataset.

        Returns:
            int: The scale factor of the dataset.
        """
        return self.scale
    
    def get_data(self):
        """
        Returns the entire dataset, including timestamps and values.

        Returns:
            tuple: A tuple containing the timestamps (t) and values (y).
        """
        return self.t, self.y
    
    def get_data_scaled(self):
        """
        Returns the scaled dataset values.

        The values are divided by 10 to the power of the scale, which adjusts the
        display units for better visualization.

        Returns:
            tuple: A tuple containing the timestamps (t) and scaled values (y).
        """
        return self.t, self.y / (10 ** self.scale)
    
    def get_value(self):
        """
        Returns the last recorded value in the dataset.

        If the dataset is empty, it returns 0.

        Returns:
            float: The most recent value in the dataset, or 0 if empty.
        """
        return self.y[-1] if len(self.y) > 0 else 0


class DataSetManager:
    """
    DataSetManager: Manager for handling multiple DataSet instances.

    Attributes:
        datasets (dict): A dictionary to store and manage multiple datasets, where the key is the dataset label.
    """

    def __init__(self):
        """
        Initializes a DataSetManager instance.

        The datasets attribute is an empty dictionary that stores all DataSet instances.
        """
        self.datasets = {}
    
    def add_dataset(self, label, unit, update_function=None):
        """
        Adds a new DataSet to the manager.

        If a dataset with the same label already exists, it will not be added again.

        Args:
            label (str): A descriptive label for the dataset.
            unit (str): The unit of measurement for the dataset values.
            update_function (callable, optional): A function to automatically update the dataset.
        """
        if label not in self.datasets:
            self.datasets[label] = DataSet(label, unit, update_function)
        else:
            print(f"DATA: Dataset '{label}' already exists.")
    
    def update_datasets(self):
        """
        Updates all datasets that have an update function.

        This method iterates through all datasets and calls their `update()` method,
        which collects new data points from the update function.
        """
        for dataset in self.datasets.values():
            dataset.update()

    def update_dataset(self, label, t, y):
        """
        Updates a specific dataset by adding new data points.

        If the dataset is not found, an error message is printed.

        Args:
            label (str): The label of the dataset to update.
            t (float): The timestamp for the new data point.
            y (float): The value corresponding to the timestamp.
        """
        if label in self.datasets:
            dataset = self.get_dataset(label)
            dataset.add_data(t, y)
        else:
            print(f"DATA: Dataset '{label}' not found.")

    def clear_datasets(self):
        """
        Clears all data from all datasets managed by the DataSetManager.
        """
        for dataset in self.datasets.values():
            dataset.clear_data()

    def get_dataset(self, label):
        """
        Retrieves a specific DataSet instance by label.

        If the dataset is not found, an error message is printed.

        Args:
            label (str): The label of the dataset to retrieve.

        Returns:
            DataSet: The dataset instance, or None if not found.
        """
        if label in self.datasets:
            return self.datasets.get(label)
        else:
            print(f"DATA: Dataset '{label}' not found.")

    def get_labels(self):
        """
        Returns a list of all dataset labels currently managed.

        Returns:
            list: A list of dataset labels.
        """
        return list(self.datasets.keys())