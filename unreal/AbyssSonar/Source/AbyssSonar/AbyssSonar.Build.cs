using UnrealBuildTool;
using System.IO;
public class AbyssSonar : ModuleRules
{
    public AbyssSonar(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        bUseUnity = false;
        bEnableExceptions = true; // Existing OBJ loader reports malformed meshes with exceptions.
        CppStandard = CppStandardVersion.Cpp20;
        PublicDependencyModuleNames.AddRange(new [] {"Core", "CoreUObject", "Engine", "InputCore", "ProceduralMeshComponent"});
        PublicIncludePaths.Add(Path.GetFullPath(Path.Combine(ModuleDirectory,"../../../../core")));
        foreach (string File in Directory.GetFiles(Path.Combine(ModuleDirectory,"../../Content/Targets")))
            RuntimeDependencies.Add(File, StagedFileType.NonUFS);
    }
}
