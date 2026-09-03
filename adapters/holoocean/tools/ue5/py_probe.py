import unreal
unreal.log_warning("UW_PY_PROBE ok, engine=" + unreal.SystemLibrary.get_engine_version())
unreal.log_warning("UW_PY_PROBE classes: " + str([hasattr(unreal, n) for n in ("EditorLevelLibrary","EditorAssetLibrary","PostProcessVolume","SkyAtmosphere","VolumetricCloud","ExponentialHeightFog","LevelEditorSubsystem")]))
