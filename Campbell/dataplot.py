from tkinter import *

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation

class DataPlot:
    """
    DataPlot: Configurable plot for multiple DataSets.

    This class is responsible for managing and displaying multiple datasets in a 
    graphical plot using Matplotlib within a Tkinter GUI. It provides functionality
    to add and remove datasets dynamically, update plot lines in real-time, and 
    open a selection window for toggling datasets.

    Attributes:
        root (Tk): The root Tkinter window.
        dataset_manager (DataSetManager): Manages multiple datasets for plotting.
        datasets (dict): Dictionary holding currently plotted datasets, keyed by dataset label.
        lines (dict): Stores Matplotlib Line2D objects for each dataset.
        scales (dict): Stores the scale of each dataset for display adjustments.
        frame (Frame): Tkinter Frame to hold the Matplotlib plot.
        fig (Figure): Matplotlib Figure for plotting datasets.
        ax (Axes): Matplotlib Axes for rendering the plot.
        canvas (FigureCanvasTkAgg): The canvas to display the plot in the Tkinter window.
        anim (FuncAnimation): Animation object to continuously update the plot.
    """

    def __init__(self, root, dataset_manager, relx, rely, width=500, height=500, bg_color="white"):
        """
        Initializes the DataPlot instance.

        Args:
            root (Tk): The root Tkinter window.
            dataset_manager (DataSetManager): Instance of DataSetManager to handle datasets.
            relx (float): Relative x-position in the parent window for the plot frame.
            rely (float): Relative y-position in the parent window for the plot frame.
            width (int, optional): Width of the plot frame. Defaults to 500.
            height (int, optional): Height of the plot frame. Defaults to 500.
            bg_color (str, optional): Background color of the plot frame. Defaults to "white".
        """
        self.root = root
        self.dataset_manager = dataset_manager
        self.datasets = {}
        self.lines = {}
        self.scales = {}

        self.frame = Frame(self.root, width=width, height=height, bg=bg_color, relief=RAISED, bd=2)
        self.frame.place(relx=relx, rely=rely, anchor=CENTER)

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel("Time (s)")
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.frame)
        canvas_widget = self.canvas.get_tk_widget()
        canvas_widget.pack(fill=BOTH, expand=True)

        self.anim = FuncAnimation(self.fig, self.update, interval=100, cache_frame_data=False)

        # Open dataset selection window when plot double clicked
        canvas_widget.bind("<Double-Button-1>", self.open_selection_window)
    
    def update(self, frame):
        """
        Updates the plot lines with the latest dataset values.

        This method is called periodically by the FuncAnimation object to refresh the
        displayed data. It adjusts each dataset's plot line based on its current scale and
        updates the legend accordingly.

        Args:
            frame: Animation frame argument (required for FuncAnimation compatibility).
        """
        for label, dataset in self.datasets.items():
            self.lines[label].set_data(dataset.get_data_scaled())

            if self.scales[label] != dataset.get_scale():
                if dataset.get_scale() == 0:
                    new_label = f"{dataset.label} ({dataset.unit})"
                elif dataset.get_scale() == 1:
                    new_label = f"{dataset.label} x 10 ({dataset.unit})"
                else:
                    new_label = f"{dataset.label} x 10^{dataset.get_scale()} ({dataset.unit})"

                self.lines[label].set_label(new_label)
                self.scales[label] = dataset.get_scale()
                self.ax.legend()

        self.ax.relim()
        self.ax.autoscale_view()

        self.canvas.draw()

    def add_dataset(self, dataset):
        """
        Adds a dataset to the plot.

        If the dataset is not already being plotted, a new plot line is created for it
        and it is added to the plot. The dataset's label is scaled as necessary.

        Args:
            dataset (DataSet): The dataset to be added to the plot.
        """
        if dataset.label not in self.datasets:
            if dataset.scale == 0:
                label = f"{dataset.label} ({dataset.unit})"
            elif dataset.get_scale() == 1:
                label = f"{dataset.label} x 10 ({dataset.unit})"
            else:
                label = f"{dataset.label} x10^{dataset.scale} ({dataset.unit})"

            self.datasets[dataset.label] = dataset
            line, = self.ax.plot([], [], label=label, linestyle='-', marker=None)
            self.lines[dataset.label] = line
            self.scales[dataset.label] = dataset.scale

            self.ax.legend()
            self.ax.legend(loc='upper right')
        else:
            print(f"DATA: Dataset '{dataset.label}' already plotting.")
    
    def remove_dataset(self, dataset):
        """
        Removes a dataset from the plot.

        If the dataset is currently being plotted, it is removed along with its corresponding
        plot line.

        Args:
            dataset (DataSet): The dataset to be removed from the plot.
        """
        if dataset.label in self.datasets:
            self.datasets.pop(dataset.label)
            line = self.lines.pop(dataset.label)
            line.remove()
            self.scales.pop(dataset.label)

            self.ax.legend()
        else:
            print(f"DATA: Dataset '{dataset.label}' not plotting.")

    def open_selection_window(self, event):
        """
        Opens a dataset selection window to toggle datasets on or off.

        A new Tkinter window appears with a list of available datasets from the 
        DataSetManager. Users can select which datasets to display on the plot.

        Args:
            event (Event): Tkinter event that triggers the selection window.
        """
        selection_window = Toplevel(self.root)
        selection_window.title("Select Datasets")

        dataset_list = Listbox(selection_window, selectmode=MULTIPLE, height=6)
        dataset_list.pack(fill=BOTH, expand=True)

        for label in self.dataset_manager.get_labels():
            dataset_list.insert(END, label)

        def apply_selection():
            """
            Toggles datasets on or off based on the user's selection.

            Datasets that are selected are added to the plot, and datasets that are 
            unselected are removed from the plot.
            """
            selected_indices = dataset_list.curselection()
            selected_labels = [dataset_list.get(i) for i in selected_indices]

            for label in selected_labels:
                if label not in self.datasets:
                    dataset = self.dataset_manager.get_dataset(label)
                    self.add_dataset(dataset)

            for label in list(self.datasets.keys()):
                if label not in selected_labels:
                    dataset = self.dataset_manager.get_dataset(label)
                    self.remove_dataset(dataset)

            selection_window.destroy()

        apply_button = Button(selection_window, text="Apply", command=apply_selection)
        apply_button.pack(pady=10)
