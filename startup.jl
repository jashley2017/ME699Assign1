import Pkg;
Pkg.activate(@__DIR__);
using LinearAlgebra, StaticArrays
using RigidBodyDynamics, RigidBodySim
using MeshCat, MeshCatMechanisms
vis = Visualizer();open(vis)
#import PandaRobot # for visualizing Panda

function display_urdf(urdfPath,vis)
    # Displays mechanism at config all zeros
    # urdfPath must be a string
    mechanism = parse_urdf(Float64,urdfPath)
    state = MechanismState(mechanism)
    zero_configuration!(state);
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
    manipulate!(state) do x
        set_configuration!(mvis, configuration(x))
    end
    return mvis, mechanism
end

function jacobian_transpose_ik!(state::MechanismState,
    body::RigidBody,
    point::Point3D,
    desired::Point3D;
    a=0.15,
    iterations=200)
  mechanism = state.mechanism
  world = root_frame(mechanism)

  # Compute the joint path from world to our target body
  p = path(mechanism, root_body(mechanism), body)
  # Allocate the point jacobian (we'll update this in-place later)
  Jp = point_jacobian(state, p, transform(state, point, world))

  q = copy(configuration(state))
  qs = typeof(configuration(state))[]
  timescale = 3 
  ts = [i*(timescale/iterations) for i=1:iterations]

  for i in 1:iterations
    # Update the position of the point
    point_in_world = transform(state, point, world)
    # Update the point's jacobian
    point_jacobian!(Jp, state, p, point_in_world)
    # Compute an update in joint coordinates using the jacobian transpose
    dq = a * Array(Jp)' * (transform(state, desired, world) - point_in_world).v
    # Apply the update
    q .= configuration(state) .+ dq
    set_configuration!(state, q)
    push!(qs, copy(configuration(state)))
  end
  return qs, ts
end

function animate_from_goal(state::MechanismState, body::RigidBody, goalpoint::Point3D, mvis::MechanismVisualizer)
  setelement!(mvis, goalpoint, 0.07)
  endpoint = Point3D(default_frame(body), 0., 0, 0)
  qs, ts = jacobian_transpose_ik!(state, body, endpoint, goalpoint)
  animation = Animation(mvis, ts, qs)
  setanimation!(mvis, animation)
end

mvis, mechanism = display_urdf("nonplanar10R2E.urdf",vis)
state = MechanismState(mechanism)
right_body = findbody(mechanism, "right_endeffector")
left_body = findbody(mechanism, "left_endeffector")

move_endeffector! = (body::RigidBody, x::Float64, y::Float64, z::Float64) -> animate_from_goal(state, body, Point3D(root_frame(mechanism), x, y, z), mvis)
