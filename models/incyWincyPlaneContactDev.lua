-- incy wincy spider for the benchmarking 
-- external dependencies
math = require('math')

-- variables
local radiusBody = 0.25
local densityBody = 1.0

local radiusJoint = 0.1
local densityJoint = 1.0

local radiusLimb = 0.0725
local lengthLimb = 0.5
local densityLimb = 1.0

-- inertia and mass of a spheres
function inertiaSphere(r, rho) -- radius r, density rho
  local thetaS = 8.0/15.0*math.pi*rho*math.pow(r,5)
  return thetaS
end

function massSphere(r, rho) -- radius r, density rho
  local mS = 4.0/3.0*math.pi*rho*math.pow(r,3)
  return mS
end

-- inertia and mass of a cylinders
function inertiaCylinderX(r, l, rho) -- radius r, length l, density rho
  local thetaCX = 1.0/12.0*math.pi*rho*l*math.pow(r,2)*(3*math.pow(r, 2) + math.pow(l, 2))
  return thetaCX
end

function inertiaCylinderY(r, l, rho) -- radius r, length l, density rho
  local thetaCY = 1.0/12.0*math.pi*rho*l*math.pow(r,2)*(3*math.pow(r, 2) + math.pow(l, 2))
  return thetaCY
end

function inertiaCylinderZ(r, l, rho) -- radius r, length l, density rho
  local thetaCZ = 1.0/2.0*math.pi*rho*l*math.pow(r, 4)
  return thetaCZ
end

function massCylinder(r, l, rho) -- radius r, length l, density rho
  local mC = math.pi*rho*l*math.pow(r,2)
  return mC
end

-- meshes
meshes = {
  body = {
    name = "Body",
    dimensions = { 2.0*radiusBody, 2.0*radiusBody, 2.0*radiusBody },
    color = { 0.8, 0.8, 0.2 },
    mesh_center = { 0.0, 0.0, 0.0 },
    src = "unit_sphere_medres.obj",
  },
  limb = {
    name = "Limb",
    dimensions = { 2*radiusLimb, 2*radiusLimb, lengthLimb },
    color = { 0.2, 0.2, 0.2 },
    mesh_center = { 0.0, 0.0, -lengthLimb/2.0 },
    src = "cylinder.obj",
  },
  joint = {
    name = "Joint",
    dimensions = { 2.0*radiusJoint, 2.0*radiusJoint, 2.0*radiusJoint },
    color = { 0.8, 0.1, 0.1 },
    mesh_center = { 0.0, 0.0, 0.0 },
    src = "unit_sphere_medres.obj",
  },
  foot = {
    name = "Foot",
    dimensions = { 2.0*radiusJoint, 2.0*radiusJoint, 2.0*radiusJoint },
    color = { 0.8, 0.1, 0.1 },
    mesh_center = { 0.0, 0.0, -lengthLimb },
    src = "unit_sphere_medres.obj",
  },
} -- meshes

-- joints
joints = {
  revolute_z = {
    { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 },
  },
  revolute_y = {
    { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 },
  },
  spherical = {
    { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
    { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0},
  },
} -- joints

-- dynamics
local massBody = massSphere(radiusBody, densityBody)
local inertiaBody = inertiaSphere(radiusBody, densityBody)

local massJoint = massSphere(radiusJoint, densityJoint)
local inertiaJoint = inertiaSphere(radiusJoint, densityJoint)

local massLimb = massCylinder(radiusLimb, lengthLimb, densityLimb)
local inertiaLimbX = inertiaCylinderX(radiusLimb, lengthLimb, densityLimb)
local inertiaLimbY = inertiaCylinderY(radiusLimb, lengthLimb, densityLimb)
local inertiaLimbZ = inertiaCylinderZ(radiusLimb, lengthLimb, densityLimb)

dynamics = {
  body = {
    mass = massBody,
    com = { 0.0, 0.0, 0.0 },
    inertia = {
      { inertiaBody, 0.0, 0.0 },
      { 0.0, inertiaBody, 0.0 },
      { 0.0, 0.0, inertiaBody },
    },
  },
  limb = {
    mass = massLimb,
    com = { 0.0, 0.0, -lengthLimb/2.0 },
    inertia = {
      { inertiaLimbX, 0.0, 0.0 },
      { 0.0, inertiaLimbY, 0.0 },
      { 0.0, 0.0, inertiaLimbZ },
    },
  },
  joint = {
    mass = massJoint,
    com = { 0.0, 0.0, 0.0 },
    inertia = {
      { inertiaJoint, 0.0, 0.0 },
      { 0.0, inertiaJoint, 0.0 },
      { 0.0, 0.0, inertiaJoint },
    },
  },   
} -- dynamics

-- model
model = {
  animation_settings = {
    force_scale = 0.1,
    torque_scale = 1,
    force_color = {1., 1., 0.},
    torque_color = {0., 1., 0.},
    force_transparency = 0.5,
    torque_transparency = 0.5,
  },
  identityMatrix = {
      {1.,0.,0.},
      {0.,1.,0.},
      {0.,0.,1.},       
  },
  gravity = {0., 0., -9.81},
  configuration = {
    axis_front  = { 1, 0, 0,},
    axis_right  = { 0, -1, 0,},
    axis_up     = { 0, 0, 1,},
  },
  frames = {
    {
      name = "Body",
      parent = "ROOT",
      visuals = {
        meshes.body,
      },
      body = dynamics.body,
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
        E = {{0., -1., 0.}, 
             {1.,  0., 0.}, 
             {0.,  0., 1.}}
      },
      joint = joints.spherical,
    },

    -- frontal left leg
    {
      name = "FrontalLeftBodyJoint",
      parent = "Body",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { -radiusBody, radiusBody, 0.0 },
      },
      joint = joints.revolute_z,
    },
    {
      name = "FrontalLeftUpperLimb",
      parent = "FrontalLeftBodyJoint",
      visuals = {
        meshes.limb,
      },
      body = dynamics.limb,
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
      },
      joint = joints.revolute_y,
    },
    {
      name = "FrontalLeftUpperJoint",
      parent = "FrontalLeftUpperLimb",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { 0.0, 0.0, -lengthLimb },
      },
      --joint = joints.revolute_y,
    },
    {
      name = "FrontalLeftLowerLimb",
      parent = "FrontalLeftUpperJoint",
      visuals = {
        meshes.limb,
      },
      body = dynamics.limb,
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
      },
      joint = joints.revolute_y,
    },
    {
      name = "FrontalLeftFoot",
      parent = "FrontalLeftLowerLimb",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { 0.0, 0.0, -lengthLimb },
      },
      --joint = joints.revolute_y,
    },

    -- frontal right leg
    {
      name = "FrontalRightBodyJoint",
      parent = "Body",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { radiusBody, radiusBody, 0.0 },
      },
      joint = joints.revolute_z,
    },
    {
      name = "FrontalRightUpperLimb",
      parent = "FrontalRightBodyJoint",
      visuals = {
        meshes.limb,
      },
      body = dynamics.limb,
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
      },
      joint = joints.revolute_y,
    },
    {
      name = "FrontalRightUpperJoint",
      parent = "FrontalRightUpperLimb",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { 0.0, 0.0, -lengthLimb },
      },
      --joint = joints.revolute_y,
    },
    {
      name = "FrontalRightLowerLimb",
      parent = "FrontalRightUpperJoint",
      visuals = {
        meshes.limb,
      },
      body = dynamics.limb,
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
      },
      joint = joints.revolute_y,
    },
    {
      name = "FrontalRightFoot",
      parent = "FrontalRightLowerLimb",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { 0.0, 0.0, -lengthLimb },
      },
      --joint = joints.revolute_y,
    },

    -- dorsal left leg
    {
      name = "DorsalLeftBodyJoint",
      parent = "Body",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { -radiusBody, -radiusBody, 0.0 },
      },
      joint = joints.revolute_z,
    },
    {
      name = "DorsalLeftUpperLimb",
      parent = "DorsalLeftBodyJoint",
      visuals = {
        meshes.limb,
      },
      body = dynamics.limb,
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
      },
      joint = joints.revolute_y,
    },
    {
      name = "DorsalLeftUpperJoint",
      parent = "DorsalLeftUpperLimb",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { 0.0, 0.0, -lengthLimb },
      },
      --joint = joints.revolute_y,
    },
    {
      name = "DorsalLeftLowerLimb",
      parent = "DorsalLeftUpperJoint",
      visuals = {
        meshes.limb,
      },
      body = dynamics.limb,
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
      },
      joint = joints.revolute_y,
    },
    {
      name = "DorsalLeftFoot",
      parent = "DorsalLeftLowerLimb",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { 0.0, 0.0, -lengthLimb },
      },
      --joint = joints.revolute_y,
    },

    -- dorsal right leg
    {
      name = "DorsalRightBodyJoint",
      parent = "Body",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { radiusBody, -radiusBody, 0.0 },
      },
      joint = joints.revolute_z,
    },
    {
      name = "DorsalRightUpperLimb",
      parent = "DorsalRightBodyJoint",
      visuals = {
        meshes.limb,
      },
      body = dynamics.limb,
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
      },
      joint = joints.revolute_y,
    },
    {
      name = "DorsalRightUpperJoint",
      parent = "DorsalRightUpperLimb",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { 0.0, 0.0, -lengthLimb },
      },
      --joint = joints.revolute_y,
    },
    {
      name = "DorsalRightLowerLimb",
      parent = "DorsalRightUpperJoint",
      visuals = {
        meshes.limb,
      },
      body = dynamics.limb,
      joint_frame = {
        r = { 0.0, 0.0, 0.0 },
      },
      joint = joints.revolute_y,
    },
    {
      name = "DorsalRightFoot",
      parent = "DorsalRightLowerLimb",
      visuals = {
        meshes.joint,
      },
      body = dynamics.joint,
      joint_frame = {
        r = { 0.0, 0.0, -lengthLimb },
      },
      --joint = joints.revolute_y,
    },
  },
} -- model

return model
