import scipy as sp
import numpy as np
import re

# List of regular expressions from the data files
rx_dict = {
    'time' : re.compile(r"time=(\d+)"),
    'm0' : re.compile(r"0\) (-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+)"),
    'm1' : re.compile(r"1\) (-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+)"),
    'm2' : re.compile(r"2\) (-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+)"),
    'm3' : re.compile(r"3\) (-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+)"),
    'm4' : re.compile(r"4\) (-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+)"),
    'm5' : re.compile(r"5\) (-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+)"),
    'm6' : re.compile(r"6\) (-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+)"),
    'm7' : re.compile(r"7\) (-?\d+.\d+),(-?\d+.\d+),(-?\d+.\d+)")
}

# Function that checks a line for a regular expression
def _parse_line(line):
    for key, rx in rx_dict.items():
        match = rx.search(line)
        if match:
            return key, match
    return None, None                       # return nothing if no matches


filepath = "logfile4.txt"
timeVec = []
marker0 = []
marker1 = []
marker2 = []
marker3 = []
marker4 = []
marker5 = []
marker6 = []
marker7 = []

NaN = float("NaN")

with open(filepath, 'r') as file_object:
    line = file_object.readline()       # Read the first line
    while line:
        # Check for regex match in line
        key, match = _parse_line(line)
        if key == 'time':
            currentTime = match.group(1)
            timeVec.append(currentTime)
            # Update markers with NaN, which will be overridden if they have data at this sample
            marker0.append([NaN, NaN, NaN])
            marker1.append([NaN, NaN, NaN])
            marker2.append([NaN, NaN, NaN])
            marker3.append([NaN, NaN, NaN])
            marker4.append([NaN, NaN, NaN])
            marker5.append([NaN, NaN, NaN])
            marker6.append([NaN, NaN, NaN])
            marker7.append([NaN, NaN, NaN])

        if key == 'm0':
            marker0[-1] = [match.group(1), match.group(2), match.group(3)]

        if key == 'm1':
            marker1[-1] = [match.group(1), match.group(2), match.group(3)]

        if key == 'm2':
            marker2[-1] = [match.group(1), match.group(2), match.group(3)]

        if key == 'm3':
            marker3[-1] = [match.group(1), match.group(2), match.group(3)]

        if key == 'm4':
            marker4[-1] = [match.group(1), match.group(2), match.group(3)]

        if key == 'm5':
            marker5[-1] = [match.group(1), match.group(2), match.group(3)]

        if key == 'm6':
            marker6[-1] = [match.group(1), match.group(2), match.group(3)]

        if key == 'm7':
            marker7[-1] = [match.group(1), match.group(2), match.group(3)]

        line = file_object.readline()   # Read the next line


timeVec = np.array(timeVec, dtype=np.float64)
marker0 = np.array(marker0, dtype=np.float64)
marker1 = np.array(marker1, dtype=np.float64)
marker2 = np.array(marker2, dtype=np.float64)
marker3 = np.array(marker3, dtype=np.float64)
marker4 = np.array(marker4, dtype=np.float64)
marker5 = np.array(marker5, dtype=np.float64)
marker6 = np.array(marker6, dtype=np.float64)
marker7 = np.array(marker7, dtype=np.float64)

sp.io.savemat("mocapData.mat", dict(timeVec=timeVec, marker0=marker0, marker1=marker1,
                                    marker2=marker2, marker3=marker3, marker4=marker4,
                                    marker5=marker5, marker6=marker6, marker7=marker7))
