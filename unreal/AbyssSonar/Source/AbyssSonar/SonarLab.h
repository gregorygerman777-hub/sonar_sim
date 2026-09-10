#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/HUD.h"
#include "GameFramework/GameModeBase.h"
#include "ResearchBridge.h"
#include "SonarLab.generated.h"
class UCameraComponent;
class USpringArmComponent;
class UProceduralMeshComponent;
class UStaticMeshComponent;

UCLASS()
class ABYSSSONAR_API ASonarLab : public APawn
{
    GENERATED_BODY()
public:
    ASonarLab();
    virtual void BeginPlay() override;
    virtual void Tick(float Dt) override;
    void SetExperiment(int Index);
    void RebuildScene();
    void Ping();
    void FlipElevation();
    void CarveCurrentView();
    void ResetVolume();
    void ExportPing();
    UPROPERTY(VisibleAnywhere) TObjectPtr<USpringArmComponent> Boom;
    UPROPERTY(VisibleAnywhere) TObjectPtr<UCameraComponent> Camera;
    UPROPERTY(VisibleAnywhere) TObjectPtr<UProceduralMeshComponent> Wave;
    UPROPERTY(VisibleAnywhere) TObjectPtr<UStaticMeshComponent> Housing;
    UPROPERTY() TArray<TObjectPtr<USceneComponent>> Geometry;
    Scene AcousticScene;
    SonarConfig Config;
    ResearchBridge::Frame Latest;
    std::vector<double> BeforeFlip;
    std::vector<Vec3> Voxels;
    std::vector<unsigned char> Kept;
    int Experiment=1, PingNumber=0, ViewCount=0, SelectedBearing=48;
    bool Running=true, Speckle=false, Paths=true, Naive=false, Focus=false;
    float Time=0, PingClock=10, Sweep=1, GainDb=0, DynamicDb=60, RenderMs=0;
    double AmbiguityError=-1;
    FString Notice;
private:
    Pose CurrentPose() const;
};

UCLASS()
class ABYSSSONAR_API ASonarLabHUD : public AHUD
{
    GENERATED_BODY()
public:
    virtual void DrawHUD() override;
private:
    void Text(const FString& Value,float X,float Y,FLinearColor Color,float Scale=1);
    bool Button(const FString& Value,float X,float Y,float W,bool Active=false);
};

UCLASS()
class ABYSSSONAR_API ASonarLabMode : public AGameModeBase
{
    GENERATED_BODY()
public:
    ASonarLabMode();
    virtual void BeginPlay() override;
};
