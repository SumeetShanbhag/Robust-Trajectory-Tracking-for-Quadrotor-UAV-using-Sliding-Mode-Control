# Robot-Control
Wrote a script in Python to generate the quintic trajectories and design boundary layer-based sliding mode control laws for the z, φ, θ, ψ coordinates of the quadrotor to track desired trajectories zd, φd, θd, and ψd from the equations of motion for this quadrotor. A ROS node was then implemented to evaluate the performance of the controller design
Since the question provides us with 5 waypoints to track and the time it should
take between 2 successive waypoints, we can use the given data to generate a
quintic trajectory for the quadrotor to track. Below is the formula to obtain the
quintic trajectory.








a0
a2
a2
a3
a4
a5








=








1 t0 t2
0 t3
0 t4
0 t5
0
0 1 2t0 3t2
0 4t3
0 5t4
0
0 0 2 6t0 12t2
0 20t3
0
1 tf t2
f t3
f t4
f t5
f
0 1 2tf 3t2
f 4t3
f 5t4
f
0 0 2 6tf 12t2
f 20t3
f








−1 







q0
 ̇q0
 ̈q0
qf
 ̇qf
 ̈qf








Where a0, a1, a2, a3, a4, and a5 are the coefficients of the quintic polynomial
equation.
q(t) = at5 + bt4 + ct3 + dt2 + et + f
Here, q0,  ̇q0,  ̈q0, qf ,  ̇qf , and  ̈qf are the initial and final positions, velocities,
and accelerations, respectively, and t0 and tf are the start and end times.
