from tkinter import *

class DataBox:
    """
    DataBox: A GUI widget for displaying and selecting a dataset.

    This class provides a Tkinter-based graphical widget that allows users to select 
    a dataset and display its most recent value. Users can open a selection window to 
    choose which dataset they wish to display by double-clicking on the DataBox frame.

    Attributes:
        root (Tk): The root Tkinter window.
        dataset_manager (DataSetManager): Manages multiple datasets.
        dataset (DataSet): The currently selected dataset to display.
        frame (Frame): The Tkinter frame that holds the labels for displaying dataset information.
        label (Label): Label that displays the dataset name.
        value (Label): Label that displays the current value of the selected dataset.
    """

    def __init__(self, root, dataset_manager, relx, rely, width=200, height=200, color="lightblue"):
        """
        Initializes the DataBox instance.

        Creates the frame and labels to display dataset information, with an event
        handler to open the dataset selection window on double-click.

        Args:
            root (Tk): The root Tkinter window.
            dataset_manager (DataSetManager): Manages multiple datasets.
            relx (float): Relative x-position in the parent window for the DataBox frame.
            rely (float): Relative y-position in the parent window for the DataBox frame.
            width (int, optional): Width of the DataBox frame. Defaults to 200.
            height (int, optional): Height of the DataBox frame. Defaults to 200.
            color (str, optional): Background color of the DataBox frame. Defaults to "lightblue".
        """
        self.root = root
        self.dataset_manager = dataset_manager
        self.dataset = None

        # Create databox frame and labels
        self.frame = Frame(self.root, width=width, height=height, bg=color, relief=RAISED, bd=2)
        self.frame.place(relx=relx, rely=rely, anchor=CENTER)

        self.label = Label(self.frame, text="Select Dataset", font=("Arial", 12), bg=color)
        self.label.place(relx=0.5, rely=0.1, anchor=CENTER)

        self.value = Label(self.frame, text="0", font=("Arial", 36), bg=color)
        self.value.place(relx=0.5, rely=0.6, anchor=CENTER)

        # Opens dataset selection window when double clicked
        self.frame.bind("<Double-Button-1>", self.open_selection_window)

    def select_dataset(self, dataset):
        """
        Selects a dataset to display in the DataBox.

        Updates the label and value displayed in the DataBox based on the selected dataset.

        Args:
            dataset (DataSet): The dataset selected to display.
        """
        if dataset is not None:
            self.dataset = dataset
            self.label.config(text=self.dataset.get_label())
            self.update_value()

    def update_value(self):
        """
        Updates the displayed value of the currently selected dataset.

        The displayed value is updated to reflect the most recent value in the dataset,
        along with the unit of measurement.
        """
        if self.dataset is not None:
            self.value.config(text=f"{self.dataset.get_value()} {self.dataset.get_unit()}")

    def open_selection_window(self, event):
        """
        Opens a new window to allow the user to select a dataset.

        The window contains a listbox with available datasets from the DataSetManager.
        A dataset can be selected by double-clicking, and the window will close after selection.

        Args:
            event (Event): The Tkinter event that triggers this method (double-click on the DataBox).
        """
        selection_window = Toplevel(self.root)
        selection_window.title("Select Dataset")

        dataset_list = Listbox(selection_window, height=6)
        dataset_list.pack(fill=BOTH, expand=True)

        for label in self.dataset_manager.get_labels():
            dataset_list.insert(END, label)
    
        def close_selection_window(event):
            """
            Selects a dataset from the list and closes the selection window.

            Once the user double-clicks on a dataset label, the corresponding dataset 
            is selected, and the window is closed.

            Args:
                event (Event): The Tkinter event triggered by double-clicking a dataset.
            """
            index = dataset_list.curselection()
            if index is not None:
                label = dataset_list.get(index)
                dataset = self.dataset_manager.get_dataset(label)
                self.select_dataset(dataset)
            selection_window.destroy()
        
        # Closes dataset selection window when dataset selected with double click
        dataset_list.bind("<Double-Button-1>", close_selection_window)
