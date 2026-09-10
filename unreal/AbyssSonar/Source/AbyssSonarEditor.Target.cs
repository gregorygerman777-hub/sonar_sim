using UnrealBuildTool;
public class AbyssSonarEditorTarget : TargetRules
{
    public AbyssSonarEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.V5;
        ExtraModuleNames.Add("AbyssSonar");
    }
}
