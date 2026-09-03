# PREP-A-02 smoke level, built headlessly via UE 5.3 Python editor scripting
# following HoloOcean's create-env.rst: water plane at z=0 tagged WaterSurface
# (movable, no collision), two PostProcessVolumes tagged WaterPPV (tint + the
# MM_Fog_Water_Simple fog material), landscape substitute below z=0, the
# standard light set, and a cylinder as a sonar target.
import unreal

LEVEL = "/Game/UwSmokeLevel"
M = 100.0  # UE units per metre

def log(msg):
    unreal.log_warning("UW_LEVEL " + msg)

def spawn(cls, loc_m, rot=(0, 0, 0)):
    return unreal.EditorLevelLibrary.spawn_actor_from_class(
        cls, unreal.Vector(loc_m[0] * M, loc_m[1] * M, loc_m[2] * M), unreal.Rotator(rot[0], rot[1], rot[2]))

def mesh_actor(label, mesh_path, loc_m, scale, material=None, collision=True, movable=False, tags=()):
    a = spawn(unreal.StaticMeshActor, loc_m)
    a.set_actor_label(label)
    comp = a.static_mesh_component
    if movable:
        comp.set_mobility(unreal.ComponentMobility.MOVABLE)
    comp.set_static_mesh(unreal.load_asset(mesh_path))
    a.set_actor_scale3d(unreal.Vector(*scale))
    if material:
        mat = unreal.load_asset(material)
        if mat is None:
            log("material missing: " + material)
        else:
            comp.set_material(0, mat)
    if not collision:
        comp.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    if tags:
        a.set_editor_property("tags", [unreal.Name(t) for t in tags])
    return a

les = unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
ok = les.new_level(LEVEL)
log("new_level -> %s" % ok)

# Seabed 60x60 m at z=-10 m (a 1 m cube scaled), enclosing walls so sonar has something to see.
mesh_actor("Seabed", "/Engine/BasicShapes/Cube", (0, 0, -10.5), (60, 60, 1))
mesh_actor("WallNorth", "/Engine/BasicShapes/Cube", (30, 0, -5), (1, 60, 12))
mesh_actor("WallSouth", "/Engine/BasicShapes/Cube", (-30, 0, -5), (1, 60, 12))
mesh_actor("WallEast", "/Engine/BasicShapes/Cube", (0, 30, -5), (60, 1, 12))
mesh_actor("WallWest", "/Engine/BasicShapes/Cube", (0, -30, -5), (60, 1, 12))
# Sonar target: 0.5 m diameter, 6 m tall piling 10 m ahead of the spawn point.
mesh_actor("PilingTarget", "/Engine/BasicShapes/Cylinder", (10, 0, -6), (0.5, 0.5, 6))

# Water plane at z=0 (a 1 m plane scaled to 80 m), movable, no collision, tagged WaterSurface.
water_mat = "/Game/StarterContent/Materials/M_TranslucentBlue_Water"
if unreal.EditorAssetLibrary.does_asset_exist("/Game/WeatherContent/WaterRipples/MM_Ocean_Ripples_OpenWater"):
    water_mat = "/Game/WeatherContent/WaterRipples/MM_Ocean_Ripples_OpenWater"
mesh_actor("WaterSurface", "/Engine/BasicShapes/Plane", (0, 0, 0), (80, 80, 1),
           material=water_mat, collision=False, movable=True, tags=("WaterSurface",))

# PostProcessVolume 1: colour tint (Dam uses AAD9C8FF).
ppv1 = spawn(unreal.PostProcessVolume, (0, 0, -5))
ppv1.set_actor_label("WaterPPV_Tint")
ppv1.set_editor_property("unbound", True)
ppv1.set_editor_property("tags", [unreal.Name("WaterPPV")])
s1 = ppv1.get_editor_property("settings")
s1.set_editor_property("override_scene_color_tint", True)
s1.set_editor_property("scene_color_tint", unreal.LinearColor(0.667, 0.851, 0.784, 1.0))
ppv1.set_editor_property("settings", s1)

# PostProcessVolume 2: underwater fog material.
fog_mat = unreal.load_asset("/Game/WeatherContent/Fog/MM_Fog_Water_Simple")
ppv2 = spawn(unreal.PostProcessVolume, (0, 0, -5))
ppv2.set_actor_label("WaterPPV_Fog")
ppv2.set_editor_property("unbound", True)
ppv2.set_editor_property("tags", [unreal.Name("WaterPPV")])
if fog_mat is None:
    log("fog material missing")
else:
    s2 = ppv2.get_editor_property("settings")
    blend = unreal.WeightedBlendables(array=[unreal.WeightedBlendable(weight=1.0, object=fog_mat)])
    s2.set_editor_property("weighted_blendables", blend)
    ppv2.set_editor_property("settings", s2)

# Lights per create-env.rst.
sun = spawn(unreal.DirectionalLight, (0, 0, 20), rot=(-50, 30, 0)); sun.set_actor_label("Sun")
spawn(unreal.SkyLight, (0, 0, 20)).set_actor_label("SkyLight")
spawn(unreal.SkyAtmosphere, (0, 0, 20)).set_actor_label("SkyAtmosphere")
spawn(unreal.VolumetricCloud, (0, 0, 20)).set_actor_label("VolumetricCloud")
fog = spawn(unreal.ExponentialHeightFog, (0, 0, 0)); fog.set_actor_label("HeightFog")
fc = fog.get_editor_property("component")
try:
    fc.set_editor_property("fog_inscattering_luminance", unreal.LinearColor(0, 0, 0, 1))
    fc.set_editor_property("directional_inscattering_luminance", unreal.LinearColor(0, 0, 0, 1))
except Exception as e:  # property names differ across versions
    log("height fog colour properties: " + str(e))

# Player start so the map has a valid default spawn.
spawn(unreal.PlayerStart, (0, 0, -3)).set_actor_label("PlayerStart")

saved = les.save_current_level()
log("save_current_level -> %s" % saved)
actors = unreal.EditorLevelLibrary.get_all_level_actors()
log("actors: " + ", ".join(sorted(a.get_actor_label() for a in actors)))
log("asset exists: %s" % unreal.EditorAssetLibrary.does_asset_exist(LEVEL))
log("DONE")
