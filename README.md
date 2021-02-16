# ME699 Robotics Assignment 1
## Author: Josh Ashley

This project is a completion of [assignment 1](https://github.com/hpoonawala/rmc-s21/tree/master/assignments/assignment1). 

This project requires Julia and packages are specified in the provided manifest. It is built from the startup example in the [odes](https://github.com/hpoonawala/rmc-s21/blob/master/julia/odes/startup.jl) folder and uses the same packages (RigidBodyDynamics, MeshCatMechanisms).

## Usage
To run this project, simply open a julia REPL and include the startup.jl.

```sh
julia
julia> include("startup.jl")
```

Once this has compiled the robotic model can be viewed on http://localhost:8700/. 

From there, you can use the provided function "move_endeffector!(body::RigidBody, x::Float64, y::Float64, z::Float64)" to start moving the end effector using inverse kinematics!

```sh
move_endeffector!(right_body, -0.7, 0.7, 0.7) ; # move right end effector (blue)
move_endeffector!(left_body, -0.7, 0.7, 0.7) ; # move left end effector (red)
```

The result should look something like this!

<img src="https://github.com/jashley2017/ME699Assign1/move_endeffector_result.JPG" width = "400">
