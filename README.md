# Compiling the project

```
mkdir build
cd build
cmake ..
make bearing_only_slam
```

*(Use `make` instead to compile everything, including all the tests. I suggest the `-j` option in that case.)*

# Executing the project

The compiled executable should be located in `<repo_root>/build/executables/bearing_only_slam`.

Usage: `bearing_only_slam <dataset_fname>`

The executable is interactive: the image shows the current state,
and the user can either advance the iterations one by one or
execute a bunch of iterations all at once.

On the given data, the algorithm converged in ~20 iterations, and it
definitely converged in one Tab press (which does 50 iterations
at once).

## Controls

*Note: some buttons may not work if the user doesn't click on the image
first.*

* **Any key other than the listed ones**: advance one iteration
* **Tab/PgDn/Shift**: advance many iterations (note: the image will freeze and
  not update until the whole batch of iterations is done)
* **B**: toggle bearing observation display
* **O**: toggle odometry observation display
* **Esc**: close

*Note: closing the window does **NOT** terminate the program. If the window is accidentally closed, use `Ctrl+C` on the terminal.*

## Legend

* **Red circles**: Poses. A line shows the orientation.
* **Blue squares**: Landmarks.
* **Green lines**: Bearing measures.
* **Purple lines**: Odometry measures.
* *Purple bar at the side of the image*: no meaning at all.
  It only serves as a visual element that changes as iterations pass, to
  make sure the program is actually doing something even if the state
  remains the same.

# Folder structure

* The source for the executables is here:
  * `executables`: compile and execute this to run the project (see above)
  * `tests`: some test executables I did, not really part of the project
* The main source code is here:
  * `framework`: data structures
  * `slam`: algorithm
  * `utils`: miscellanea
* The dataset is here:
  * `data`: self-explanatory
