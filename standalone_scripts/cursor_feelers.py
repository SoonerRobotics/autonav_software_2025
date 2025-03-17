import sys
import numpy as np
import sympy as sp

# importing the library 
import matplotlib 
matplotlib.use('QtAgg')
import matplotlib.pyplot as plt 

from PyQt6 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
  
class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100, lb=-1, hb=1):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        self.axes.set_ylim(lb, hb)
        self.axes.set_xlim(lb, hb)
        super().__init__(fig)


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle('cursed feelers')
        # Create the maptlotlib FigureCanvas object,
        # which defines a single set of axes as self.axes.
        self.lb = -100
        self.hb = 100
        self.canvas = MplCanvas(self, width=10, height=10, dpi=100, lb=self.lb, hb=self.hb)
        self.setCentralWidget(self.canvas)

        self.feeler_basis = None
        self.control_point_sets = [0] * 2
        self.control_point_sets[0] = sp.Matrix([[-90, -90],    
                                    [-90, -90],
                                    [-80, 75],
                                    [-75, 75],
                                    [0, 100],
                                    [75, 75],
                                    [80, 75],
                                    [90, -90],
                                    [90, -90]])
        
        self.control_point_sets[1] = sp.Matrix([[-70, -90],    
                                    [-60, -90],
                                    [-55, 25],
                                    [-45, 25],
                                    [0, 50],
                                    [45, 25],
                                    [55, 25],
                                    [60, -90],
                                    [70, -90]])
        
        self.path_curves = self.generate_path_curves(len(self.control_point_sets), control_point_sets=self.control_point_sets)

        self.canvas.mpl_connect('motion_notify_event', self.mouse_moved)

        self.show()


    def mouse_moved(self, event):
        self.cursor_x, self.cursor_y = event.xdata, event.ydata

        if self.cursor_x is None or self.cursor_y is None:
            return
        
        self.redraw_canvas()


    def redraw_canvas(self):
        self.canvas.axes.cla()
        self.canvas.axes.set_ylim(-100, 100)
        self.canvas.axes.set_xlim(-100, 100) 

        # draw a dot on the cursor
        self.canvas.axes.scatter(self.cursor_x, self.cursor_y, color='blue')
        
        # generate & draw initial feelers
        angles = np.arange(4, 360, 15) # offset because we want to solve for intersections along x, so vertical feelers are bad
        self.feelers = [0] * len(angles)
        initial_radius = 1000
        self.radii = [initial_radius] * len(angles)

        for idx, angle in enumerate(angles):
            self.feelers[idx] = self.compute_feeler(self.radii[idx], angle=angle, optimized=True)
            # self.canvas.axes.plot(self.feelers[idx][:, 0], self.feelers[idx][:, 1], color='green')

        # draw path curves
        for idx, curve in enumerate(self.path_curves):
            self.canvas.axes.plot(self.path_curves[idx][:, 0], self.path_curves[idx][:, 1])
        
        # intersect feelers with path curves
        all_intersecting_points, all_intersection_distances = self.numpy_intersections(initial_radius=initial_radius)

        # redraw feelers using intersection distances as radii
        for idx, angle in enumerate(angles):
            self.feelers[idx] = self.compute_feeler(all_intersection_distances[idx], angle=angle, optimized=True)
            self.canvas.axes.plot(self.feelers[idx][:, 0], self.feelers[idx][:, 1], color='blue')

        self.canvas.axes.scatter(all_intersecting_points[:, 0], all_intersecting_points[:, 1])
        self.canvas.axes.plot(1, 1)
        self.canvas.draw()


    def numpy_intersections(self, initial_radius, over_x = True, over_y = False):
        all_intersecting_points = np.empty((0, 2))
        all_intersection_distances = [initial_radius] * len(self.feelers)

        for idx, feeler in enumerate(self.feelers):
            intersecting_point = None
            closest_intersection_distance = None
            for jdx, curve in enumerate(self.path_curves):
                # solve over domain, to add density where dy << dx
                tolerance = 1
                feeler_resolution = int(1e1)
                if over_x:
                    feeler_xs = np.interp(np.linspace(0, 1, len(curve[:, 0])), np.linspace(0, 1, feeler_resolution), np.array(feeler[:, 0]).flatten())
                    curve_xs = np.array(curve[:, 0])

                    try:
                        x_common = np.linspace(max(np.min(feeler_xs), np.min(curve_xs)), min(np.max(feeler_xs), np.max(curve_xs)), int(1e4))
                        print(x_common)
                    except:
                        continue #unfeasible
                    
                    feeler_ys = np.interp(np.linspace(0, 1, len(curve[:, 1])), np.linspace(0, 1, feeler_resolution), np.array(feeler[:, 1]).flatten())
                    curve_ys = np.array(curve[:, 1])
                    
                    # np.interp assumes sorted xp, yp..
                    sorted_indices = np.argsort(feeler_xs)
                    feeler_xs = feeler_xs[sorted_indices]
                    feeler_ys = feeler_ys[sorted_indices]

                    sorted_indices = np.argsort(curve_xs)
                    curve_xs = curve_xs[sorted_indices]
                    curve_ys = curve_ys[sorted_indices]

                    feeler_ys = np.interp(x_common, feeler_xs, feeler_ys)
                    curve_ys = np.interp(x_common, curve_xs, curve_ys)
                    
                    intersections = np.abs(feeler_ys - curve_ys) < tolerance

                    print(intersections)
                    intersection_xs = x_common[intersections]
                    intersection_ys = curve_ys[intersections]

                    # self.canvas.axes.scatter(x_common, curve_ys)
                    # self.canvas.axes.scatter(x_common, feeler_ys)

                    # all_intersecting_points = np.append(all_intersecting_points, np.array([intersection_xs, intersection_ys]).T, axis=0)

                    if len(intersection_xs) < 1 or len(intersection_ys) < 1: # no intersections
                        continue

                    print(intersection_xs)
                    print(intersection_ys)
                    for i in range(len(intersection_xs)):
                        distance = float(np.sqrt((self.cursor_x - intersection_xs[i])**2 + (self.cursor_y - intersection_ys[i])**2))
                        if closest_intersection_distance is None:
                            closest_intersection_distance = distance
                            intersecting_point = np.expand_dims(np.array([intersection_xs[i], intersection_ys[i]]), axis=0)
                        elif distance < closest_intersection_distance:
                            closest_intersection_distance = distance
                            intersecting_point = np.expand_dims(([intersection_xs[i], intersection_ys[i]]), axis=0)
                    

                if over_y:
                    # reset
                    feeler_xs = np.interp(np.linspace(0, 1, len(curve[:, 1])), np.linspace(0, 1, feeler_resolution), np.array(feeler[:, 0]).flatten())
                    curve_xs = np.array(curve[:, 0])

                    feeler_ys = np.interp(np.linspace(0, 1, len(curve[:, 1])), np.linspace(0, 1, feeler_resolution), np.array(feeler[:, 1]).flatten())
                    curve_ys = np.array(curve[:, 1])

                    # now solve over range, to add density where dx << dy
                    feeler_ys = np.interp(np.linspace(0, 1, len(curve[:, 0])), np.linspace(0, 1, feeler_resolution), np.array(feeler[:, 1]).flatten())
                    curve_ys = np.array(curve[:, 1])

                    try:
                        y_common = np.linspace(max(np.min(feeler_ys), np.min(curve_ys)), min(np.max(feeler_ys), np.max(curve_ys)), int(1e2))
                        print(y_common)
                    except:
                        continue #unfeasible

                    feeler_xs = np.interp(np.linspace(0, 1, len(curve[:, 1])), np.linspace(0, 1, feeler_resolution), np.array(feeler[:, 0]).flatten())
                    curve_xs = np.array(curve[:, 0])

                    # np.interp assumes sorted xp, yp..
                    sorted_indices = np.argsort(feeler_ys)
                    feeler_ys = feeler_ys[sorted_indices]
                    feeler_xs = feeler_xs[sorted_indices]

                    sorted_indices = np.argsort(curve_ys)
                    curve_xs = curve_xs[sorted_indices]
                    curve_ys = curve_ys[sorted_indices]

                    # self.canvas.axes.scatter(curve_xs, curve_ys)

                    feeler_xs = np.interp(y_common, feeler_ys, feeler_xs)
                    curve_xs_interp = np.interp(y_common, curve_ys, curve_xs) # issue here!

                    tolerance = 1
                    intersections = np.abs(feeler_xs - curve_xs_interp) < tolerance

                    print(intersections)
                    intersection_ys = y_common[intersections]
                    intersection_xs = curve_xs_interp[intersections]
                    
                    # self.canvas.axes.scatter(curve_xs_interp, y_common)
                    # self.canvas.axes.scatter(feeler_xs, y_common)
                    
                    # all_intersecting_points = np.append(all_intersecting_points, np.array([intersection_xs, intersection_ys]).T, axis=0)

                    if len(intersection_xs) < 1 or len(intersection_ys) < 1: # no intersections
                        continue

                    for i in range(len(intersection_xs)):
                        distance = float(np.sqrt((self.cursor_x - intersection_xs[i])**2 + (self.cursor_y - intersection_ys[i])**2))
                        if closest_intersection_distance is None:
                            closest_intersection_distance = distance
                            intersecting_point = np.expand_dims(np.array([intersection_xs[i], intersection_ys[i]]), axis=0)

                        elif distance < closest_intersection_distance:
                            closest_intersection_distance = distance
                            intersecting_point = np.expand_dims(([intersection_xs[i], intersection_ys[i]]), axis=0)

            if intersecting_point is None:
                continue
            
            all_intersecting_points = np.append(all_intersecting_points, intersecting_point, axis=0)
            all_intersection_distances[idx] = closest_intersection_distance

        return all_intersecting_points, all_intersection_distances
            

    def generate_path_curves(self, num_path_curves, control_point_sets: list[sp.Matrix]):
        if len(control_point_sets) != num_path_curves:
            print("must have the same number of controls point sets as path curves")
            return
        
        path_curves = [0] * num_path_curves
        curve_segments = []
        for idx, curve in enumerate(path_curves):
            path_curves[idx] = self.generate_b_spline(control_points=control_point_sets[idx])
            # curve_segments = curve_segments + self.generate_b_spline(control_points=control_point_sets[idx])

        return path_curves
        # return curve_segments


    def generate_b_spline(self, control_points: sp.Matrix, b_spline_resolution=int(1e2)):
        # quadratic uniform b-splines
        u = sp.symbols('u')
        U = sp.Matrix([[u**2, u, 1]])
        M = .5 * sp.Matrix([[1, -2, 1],
                            [-2, 2, 0], 
                            [1, 1, 0]])
        sp.pretty_print(M)
        
        n = control_points.shape[0] # number of rows, that is, control points
        imax = n-2
        curve_segments = [0] * (imax)

        curve_eval = np.empty((0, 2))
        segment_evals = [0] * (imax)
        for i in range(1, imax+1):
            idx = i-1
            Gseg = control_points[i-1:i+2, :]
            curve_segments[idx] = U * M * Gseg

            # plot the curve segments
            np_us = np.linspace(0, 1, b_spline_resolution)

            segment_func = sp.lambdify(u, sp.Add(curve_segments[idx], sp.Matrix([[1e-10*u**2 + 1e-10*u, 1e-10*u**2 + 1e-10*u]]), evaluate=False)) # this hack avoids the bug #5642
            segment_evals[idx] = segment_func(np_us)[0].T

            curve_eval = np.append(curve_eval, segment_evals[idx], axis=0) # append the segment along the rows

        return curve_eval
        # return segment_evals
    

    def compute_feeler(self, radius, angle, optimized=True) -> np.ndarray:
        if optimized:
            # optimized
            if self.feeler_basis is None:
                self.feeler_basis = self.compute_feeler_basis(evaluation_resolution=int(1e1))

            second_point_x = radius * np.cos(np.radians(angle))
            second_point_y = radius * np.sin(np.radians(angle))

            Gf = np.matrix([[self.cursor_x, self.cursor_y],
                            [self.cursor_x + second_point_x, self.cursor_y + second_point_y]])
            
            feeler = self.feeler_basis * Gf

            return feeler

        else:
            # unoptimized
            u = sp.symbols('u')
            U = sp.Matrix([[u, 1]])
            Nf_inv = sp.Matrix([[0, 1],
                            [1, 1]])
            
            Nf = Nf_inv ** -1

            second_point_x = radius * np.cos(np.radians(angle))
            second_point_y = radius * np.sin(np.radians(angle))

            Gf = sp.Matrix([[self.cursor_x, self.cursor_y],
                            [self.cursor_x + second_point_x, self.cursor_y + second_point_y]])

            Pf = U * Nf * Gf
            np_us = np.linspace(0, 1, int(1e2))

            sp.pretty_print(Pf)
            # Pf_func = np.vectorize(sp.lambdify(u, Pf, modules="numpy"), signature='()->(1e2)')
            Pf_func = sp.lambdify(u, sp.Add(Pf, sp.Matrix([[1e-10*u, 1e-10*u]]), evaluate=False)) # this hack avoids the bug
            Pf_eval = Pf_func(np_us)[0].T # tranpose the numpy convention, gives evaluation along the rows 
            # THERE'S A BUG HERE #5642, so without the hack this breaks when any polynomial evaluates to a constant.
            # because of this we just won't use the unoptimized version.

            return Pf_eval
    

    def compute_feeler_basis(self, evaluation_resolution=int(1e2)) -> np.ndarray:
        print('computing the basis only once, this operation is costly')
        u = sp.symbols('u')
        U = sp.Matrix([[u, 1]])
        Nf_inv = sp.Matrix([[0, 1],
                        [1, 1]])
        
        Nf = Nf_inv ** -1
        Bf = U * Nf

        np_us = np.linspace(0, 1, evaluation_resolution)
        Bf_eval = sp.lambdify(u, Bf)(np_us)[0].T # tranpose the numpy convention, 
                                                # we want leftmost axis to preserve evaluation since we're going to transform G by B i.e. Pf = Bf * Gf Pf = (1000, 2) = (1000, 2) * (2, 2)

        return Bf_eval
    
    def broadcast(self, fun):
        return lambda *x: np.broadcast_arrays(fun(*x), *x)[0]


app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
app.exec()



