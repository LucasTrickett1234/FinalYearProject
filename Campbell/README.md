# MAVLink Communication and Dataset Management

This repository contains several Python modules for communicating with a MAVLink flight controller and managing datasets for visualization and analysis. It includes classes for managing telemetry data, performing motor tests, and plotting datasets using a GUI built with Tkinter and Matplotlib.

## Table of Contents
- [Files Overview](#files-overview)
- [Requirements](#requirements)
- [Setup](#setup)
- [Usage](#usage)
- [Class Descriptions](#class-descriptions)
  - [DataSet](#dataset-class)
  - [DataSetManager](#datasetmanager-class)
  - [DataPlot](#dataplot-class)
  - [DataBox](#databox-class)
  - [Mavlink](#mavlink-class)
- [License](#license)

## Files Overview

### 1. `databox.py`
This module contains the `DataBox` class, which creates a GUI widget for selecting and displaying the latest value from a dataset. It uses Tkinter for the graphical interface.

### 2. `dataplot.py`
This module contains the `DataPlot` class, which is responsible for plotting multiple datasets in real time using Matplotlib. It provides functionality for selecting datasets and configuring the plot display.

### 3. `dataset.py`
This module defines two classes:
- `DataSet`: A class for storing and managing dataset values (e.g., timestamps and values).
- `DataSetManager`: A class that manages multiple `DataSet` instances.

### 4. `mavlink.py`
This module implements the `Mavlink` class, which handles communication with a MAVLink flight controller. It provides functions to connect, send commands, and receive telemetry data from the flight controller. It also supports motor testing and custom message rates.

### 5. `mavlink_messages.py`
This module defines various MAVLink messages (e.g., `ESC_TELEMETRY`) that are used for interacting with the flight controller. It provides a structured way to manage and store data from the MAVLink messages.

## Requirements
To run these scripts, you will need:
- Python 3.x
- The following Python packages:
  - `pymavlink`
  - `matplotlib`
  - `numpy`
  - `tkinter`
