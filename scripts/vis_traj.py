import numpy as onp
import sys

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

BUFFER = 0.05

def plot3DWGT(x,y,z,x_gt,y_gt,z_gt,color,color_gt,label,label_gt,xlabel,ylabel,zlabel,title):
	global BUFFER
	fig = plt.figure(0)
	ax = fig.gca(projection='3d')
	ax.plot(x,y,z,color=color,label=label)
	ax.plot(x_gt,y_gt,z_gt,color=color_gt,label=label_gt, alpha=0.3, linewidth=2.0)
	xlim = [min([min(x),min(x_gt)]) - BUFFER, max([max(x),max(x_gt)]) + BUFFER]
	ylim = [min([min(y),min(y_gt)]) - BUFFER, max([max(y),max(y_gt)]) + BUFFER]
	zlim = [min([min(z),min(z_gt)]) - BUFFER, max([max(z),max(z_gt)]) + BUFFER]
	ax.set_xlim(xlim)
	ax.set_ylim(ylim)
	ax.set_zlim(zlim)
	ax.set_xlabel(xlabel, linespacing=3.2)
	ax.set_ylabel(ylabel, linespacing=3.2)
	ax.set_zlabel(zlabel, linespacing=3.2)
	ax.set_title(title)
	ax.legend(loc='upper right')
	ax.xaxis.labelpad=30
	ax.yaxis.labelpad=30
	ax.zaxis.labelpad=30
	return fig

def plot2D(x, x_gt, idx):
	fig = plt.figure(idx)
	t = onp.arange(len(x))
	plt.plot(t, x-x_gt)
	plt.grid()
	return fig

def drawTraj(est_traj, gt_traj):
	x = est_traj[:,1]
	y = est_traj[:,2]
	z = est_traj[:,3]
	x_gt = gt_traj[:,1]
	y_gt = gt_traj[:,2]
	z_gt = gt_traj[:,3]
	fig = plot3DWGT(x, y, z, x_gt, y_gt, z_gt, "g", "r", "EST", "GT", "x(m)", "y(m)", "z(m)", "3D Position Plot")

	err_x = x[:] - x_gt[1:]
	err_y = y[:] - y_gt[1:]
	err_z = z[:] - z_gt[1:]
	dxrms = onp.sqrt((err_x**2).mean(axis=0))
	dyrms = onp.sqrt((err_y**2).mean(axis=0))
	dzrms = onp.sqrt((err_z**2).mean(axis=0))
	dprms = onp.sqrt(dxrms**2 + dyrms**2 + dzrms**2)
	print("Position RMSE:[%.3f]"%dprms)
	# figx = plot2D(x[1:], x_gt[:], 1)
	# figy = plot2D(y[1:], y_gt[:], 2)
	# figz = plot2D(z[1:], z_gt[:], 3)
	plt.show()

if __name__ == "__main__":
    # TODO: Add help function
    est_traj_file = sys.argv[1]
    gt_traj_file = sys.argv[2]
    #
    est_traj = onp.loadtxt(est_traj_file, delimiter=",")
    gt_traj = onp.loadtxt(gt_traj_file, delimiter=",")
    #
    drawTraj(est_traj, gt_traj)
