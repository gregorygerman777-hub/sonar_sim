"""Run once inside Unreal Editor to create the native map and simple materials."""
import unreal
assets = unreal.AssetToolsHelpers.get_asset_tools()
lib = unreal.MaterialEditingLibrary

def material(name, color, roughness, metallic=0., translucent=False):
    path='/Game/Materials/'+name
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        return unreal.load_asset(path)
    mat=assets.create_asset(name,'/Game/Materials',unreal.Material,unreal.MaterialFactoryNew())
    rgb=lib.create_material_expression(mat,unreal.MaterialExpressionConstant3Vector)
    rgb.set_editor_property('constant',unreal.LinearColor(*color,1.))
    lib.connect_material_property(rgb,'',unreal.MaterialProperty.MP_BASE_COLOR)
    for prop,value in [(unreal.MaterialProperty.MP_ROUGHNESS,roughness),(unreal.MaterialProperty.MP_METALLIC,metallic)]:
        node=lib.create_material_expression(mat,unreal.MaterialExpressionConstant)
        node.set_editor_property('r',value);lib.connect_material_property(node,'',prop)
    if translucent:
        mat.set_editor_property('blend_mode',unreal.BlendMode.BLEND_TRANSLUCENT)
        mat.set_editor_property('two_sided',True)
        lib.connect_material_property(rgb,'',unreal.MaterialProperty.MP_EMISSIVE_COLOR)
        opacity=lib.create_material_expression(mat,unreal.MaterialExpressionConstant)
        opacity.set_editor_property('r',.25)
        lib.connect_material_property(opacity,'',unreal.MaterialProperty.MP_OPACITY)
    lib.recompile_material(mat)
    unreal.EditorAssetLibrary.save_asset(path)
    return mat

material('M_Rock',(.065,.09,.105),.94)
material('M_Rust',(.19,.105,.055),.88,.35)
material('M_Submarine',(.45,.5,.43),.32,.65)
material('M_Pulse',(.08,.6,.45),.4,translucent=True)
level=unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
if not unreal.EditorAssetLibrary.does_asset_exist('/Game/Maps/Canyon'):
    level.new_level('/Game/Maps/Canyon')
    actors=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
    actors.spawn_actor_from_class(unreal.PlayerStart,unreal.Vector(0,0,0))
    level.save_current_level()
unreal.log('Abyss Sonar: bootstrap complete. Open /Game/Maps/Canyon and press Play.')
