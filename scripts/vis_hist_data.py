#!/usr/bin/env python3

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import numpy as np
import rospy
from measurement_msgs.msg import AnchorErrorHistogramArray
# flow 
# setup subscriber
# 
import matplotlib.pyplot as plt

pg.setConfigOption('background', 'w')

N_BINS = 20

class MobileWidget(pg.GraphicsWindow):

    def __init__(self, parent=None):
        super(MobileWidget, self).__init__(parent=parent)

        self.mainLayout = QtWidgets.QVBoxLayout()
        self.setLayout(self.mainLayout)

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(1000) # in milliseconds
        self.timer.start()
        self.timer.timeout.connect(self.on_new_data)

        self.row = 0
        self.col = 0

        self.range_data = {}
        self.anchor_plots = {}

        self.anchor_hist_sub = rospy.Subscriber("anchor_hist_data", AnchorErrorHistogramArray, self.anchor_hist_cb)

    def set_data(self, anchor_name):
        y1, x1 = self.range_data[anchor_name]
        opts = {}
        opts["x0"] = x1[:-1]
        opts["height"] = y1
        opts["bins"] = N_BINS
        opts["width"] = x1[1] - x1[0]
        self.anchor_plots[anchor_name].setOpts(**opts)

    def on_new_data(self):
        for anchor_name in self.range_data.keys():
            if anchor_name not in self.anchor_plots:
                plotItem = self.addPlot(row=self.row,col=self.col, title=anchor_name)
                plotItem.showGrid(x = True, y = True, alpha=1.0)
                self.anchor_plots[anchor_name] = pg.BarGraphItem(x=[], height=[], width = 1.0, brush ='b')
                plotItem.addItem(self.anchor_plots[anchor_name])
                if self.col >= 3:
                    self.row += 1
                    self.col = 0
                else:
                    self.col += 1

            self.set_data(anchor_name)

    def anchor_hist_cb(self, msg):
        for anc_info in msg.anchor_data:
            x, y = np.histogram(anc_info.data, bins=N_BINS)
            self.range_data[anc_info.name] = (x, y)
 
def main():
    app = QtWidgets.QApplication([])
    pg.setConfigOptions(antialias=False) # True seems to work as well
    win = MobileWidget()
    win.show()
    win.raise_()
    app.exec_()

if __name__ == "__main__":
    rospy.init_node("dynamic_range_plot")
    main()
    rospy.spin()
