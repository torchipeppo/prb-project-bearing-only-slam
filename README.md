# Compiling the project

```
mkdir build
cd build
cmake ..
make
```
<!--TODO: give command for only the bearing_only_slam executable?-->

# Executing the project

The compiled executable should be located in `<repo_root>/build/executables/bearing_only_slam`.

Usage: `bearing_only_slam <dataset_fname>`

The executable is interactive: the image shows the current state,
and the user can either advance the iterations one by one or
execute a bunch of iterations all at once.

On the given data, the algorithm converged in ~20 iterations, and it
definitely converged in one Tab press (which does 50 iterations
at once).

# Legend

* Red circles: Poses. A line shows the orientation.
* Blue squares: landmarks.
* Green lines: bearing measures.
* Purple lines: odometry measures.
* Purple bar at the side of the image: no meaning at all.
  It only serves as a visual element that changes as iterations pass, to
  make sure the program is actually doing something even if the state
  remains the same.

# Controls

Note: some buttons may not work if the user doesn't click on the image
first.

* Any key other than the listed ones: advance one iteration
* Tab/PgDn/Shift: advance many iterations (note: the image will freeze and
  not update until the whole batch of iterations is done)
* B: toggle bearing observation display
* O: toggle odometry observation display
* Esc: close