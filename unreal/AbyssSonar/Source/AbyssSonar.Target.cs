using UnrealBuildTool;
public class AbyssSonarTarget : TargetRules
{
    public AbyssSonarTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.V5;
        ExtraModuleNames.Add("AbyssSonar");
    }
}
