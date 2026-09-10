#include "SonarLab.h"
#include "Modules/ModuleManager.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Components/LightComponent.h"
#include "Engine/ExponentialHeightFog.h"
#include "Engine/DirectionalLight.h"
#include "Materials/MaterialInterface.h"
#include "ProceduralMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "InputCoreTypes.h"
#include "physics.h"

IMPLEMENT_PRIMARY_GAME_MODULE(FDefaultGameModuleImpl,AbyssSonar,"AbyssSonar");
namespace
{
FVector UE(const Vec3& P) { const auto V=ResearchBridge::ToUnreal(P);return FVector(V.x,V.y,V.z); }
FLinearColor Cyan(0.27,0.85,0.78,1), White(0.88,0.9,0.88,1), Muted(0.48,0.57,0.6,1),
    Amber(0.94,0.65,0.43,1), Panel(0.018,0.027,0.036,0.98);
const TCHAR* Names[]={TEXT("Elevation ambiguity"),TEXT("Highlights & shadows"),TEXT("Surface multipath"),TEXT("Concave geometry")};
const TCHAR* Lessons[]={TEXT("Flip the target: two elevations, one sonar image."),TEXT("Move around the sphere. Watch the first-hit shadow change."),TEXT("Roll the sonar. Reflected sound can produce extra returns."),TEXT("Capture several views. A feasible volume is not unique geometry.")};
constexpr double Pi=3.14159265358979323846;
}
ASonarLab::ASonarLab()
{
    PrimaryActorTick.bCanEverTick=true;
    SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("SonarPose")));
    Housing=CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Housing"));Housing->SetupAttachment(RootComponent);
    Housing->SetRelativeScale3D(FVector(.65,.25,.25));Housing->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Boom=CreateDefaultSubobject<USpringArmComponent>(TEXT("OrbitCamera"));Boom->SetupAttachment(RootComponent);
    Boom->TargetArmLength=650;Boom->SetRelativeRotation(FRotator(-22,-25,0));Boom->bDoCollisionTest=false;
    Boom->SocketOffset=FVector(150,0,100);
    Camera=CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));Camera->SetupAttachment(Boom);Camera->FieldOfView=75;
    Wave=CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("SlowPing"));Wave->SetupAttachment(RootComponent);
    Wave->SetCollisionEnabled(ECollisionEnabled::NoCollision);Wave->SetCastShadow(false);
    auto* Light=CreateDefaultSubobject<USpotLightComponent>(TEXT("Searchlight"));Light->SetupAttachment(RootComponent);
    Light->SetRelativeLocation(FVector(40,0,0));Light->SetIntensity(180000);Light->SetAttenuationRadius(1800);
    Light->SetOuterConeAngle(42);Light->SetInnerConeAngle(25);Light->SetVolumetricScatteringIntensity(2);
}
void ASonarLab::BeginPlay()
{
    Super::BeginPlay();
    Housing->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Sphere.Sphere")));
    Housing->SetMaterial(0,LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/Materials/M_Submarine.M_Submarine")));
    Wave->SetMaterial(0,LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/Materials/M_Pulse.M_Pulse")));
    Wave->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
    if(auto* PC=Cast<APlayerController>(GetController())) { PC->bShowMouseCursor=true;PC->SetInputMode(FInputModeGameAndUI()); }
    SetExperiment(1);
}
Pose ASonarLab::CurrentPose() const
{
    const FVector P=GetActorLocation(),F=GetActorForwardVector(),R=GetActorRightVector(),U=GetActorUpVector();
    Pose p;p.position=ResearchBridge::FromUnreal(P.X,P.Y,P.Z);
    p.x_axis=ResearchBridge::DirectionFromUnreal(R.X,R.Y,R.Z);
    p.y_axis=ResearchBridge::DirectionFromUnreal(F.X,F.Y,F.Z);
    p.z_axis=ResearchBridge::DirectionFromUnreal(U.X,U.Y,U.Z);
    return p;
}
void ASonarLab::SetExperiment(int Index)
{
    Experiment=FMath::Clamp(Index,0,3);Config=ResearchBridge::DefaultConfig();
    Config.multipath_enabled=Experiment==2;Speckle=false;Naive=false;AmbiguityError=-1;BeforeFlip.clear();
    SetActorLocationAndRotation(Experiment==2?FVector(0,0,-43):FVector::ZeroVector,FRotator::ZeroRotator);
    try { AcousticScene=ResearchBridge::Experiment(Experiment,TCHAR_TO_UTF8(*FPaths::ConvertRelativePathToFull(FPaths::ProjectContentDir()/TEXT("Targets")))); }
    catch(const std::exception& e) { Notice=UTF8_TO_TCHAR(e.what());Running=false;return; }
    Notice=Lessons[Experiment];ResetVolume();RebuildScene();Ping();
}
void ASonarLab::RebuildScene()
{
    for(auto* C:Geometry) if(C) C->DestroyComponent();Geometry.Empty();
    auto* Rock=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/Materials/M_Rock.M_Rock"));
    auto* Metal=LoadObject<UMaterialInterface>(nullptr,TEXT("/Game/Materials/M_Rust.M_Rust"));
    auto Shape=[&](const TCHAR* Path,FVector Position,FVector Scale,UMaterialInterface* Mat)
    {
        auto* M=NewObject<UStaticMeshComponent>(this);M->RegisterComponent();M->SetWorldLocation(Position);
        M->SetStaticMesh(LoadObject<UStaticMesh>(nullptr,Path));M->SetWorldScale3D(Scale);M->SetMaterial(0,Mat);
        M->SetCollisionEnabled(ECollisionEnabled::NoCollision);Geometry.Add(M);
    };
    for(const auto& S:AcousticScene.spheres) Shape(TEXT("/Engine/BasicShapes/Sphere.Sphere"),UE(S.centre),FVector(S.radius*2),Metal);
    for(const auto& P:AcousticScene.planes) Shape(TEXT("/Engine/BasicShapes/Cube.Cube"),UE(P.point)+FVector(500,0,-5),FVector(30,30,.1),Rock);
    for(const auto& Mesh:AcousticScene.meshes)
    {
        auto* M=NewObject<UProceduralMeshComponent>(this);M->RegisterComponent();
        TArray<FVector> V,N;TArray<int32> T;TArray<FVector2D> UV;
        for(size_t j=0;j<Mesh.triangles.size();++j)
        {
            const auto& F=Mesh.triangles[j];int32 Base=V.Num();
            // Swapping axes changes handedness. Reverse winding for Unreal's renderer.
            for(int K:{0,2,1}) {V.Add(UE(Mesh.vertices[F[K]]));N.Add(UE(Mesh.normals[j]).GetSafeNormal());UV.Add(FVector2D(V.Last().X*.01,V.Last().Y*.01));}
            T.Append({Base,Base+1,Base+2});
        }
        M->CreateMeshSection_LinearColor(0,V,T,N,UV,TArray<FLinearColor>(),TArray<FProcMeshTangent>(),false);
        M->SetMaterial(0,Metal);Geometry.Add(M);
    }
}
void ASonarLab::Ping()
{
    const double Begin=FPlatformTime::Seconds();
    auto C=Config;if(Naive) C.num_elevation_subrays=1;
    Latest=ResearchBridge::Ping(AcousticScene,CurrentPose(),C,Speckle,42+PingNumber);
    RenderMs=(FPlatformTime::Seconds()-Begin)*1000;++PingNumber;PingClock=0;Sweep=0;
}
void ASonarLab::FlipElevation()
{
    if(Experiment!=0) SetExperiment(0);
    BeforeFlip=Latest.raw;
    AcousticScene.spheres[0].centre.z*=-1;RebuildScene();Ping();
    double Peak=0,Error=0;
    for(size_t I=0;I<BeforeFlip.size();++I){Peak=std::max(Peak,BeforeFlip[I]);Error=std::max(Error,std::abs(BeforeFlip[I]-Latest.raw[I]));}
    AmbiguityError=Error/std::max(Peak,1e-30);
}
void ASonarLab::ResetVolume()
{
    Voxels.clear();Kept.clear();ViewCount=0;
    const Vec3 C=Experiment==0?Vec3{0,4,0}:Experiment==1?Vec3{0,4,-.5}:Experiment==2?Vec3{0,2,-.15}:Vec3{0,4,-.6};
    for(int X=0;X<14;++X) for(int Y=0;Y<14;++Y) for(int Z=0;Z<14;++Z)
        Voxels.push_back(C+Vec3{(X-6.5)*.12,(Y-6.5)*.12,(Z-6.5)*.12});
    Kept.assign(Voxels.size(),1);
}
void ASonarLab::CarveCurrentView()
{
    if(Latest.raw.empty()) return;
    HighlightMask Hi;ShadowMask Shadow;ResearchBridge::Masks(Latest,.03,Hi,Shadow);
    SonarPose P;P.pose=Latest.pose;P.config=Latest.config;P.range_tolerance_bins=P.bearing_tolerance_bins=1;
    carve_voxels(Voxels,P,Hi,Shadow,Kept.data());++ViewCount;
    Notice=TEXT("Captured known pose. Farther cells are feasible, not measured shadows.");
}
void ASonarLab::ExportPing()
{
    const FString Dir=FPaths::ProjectSavedDir()/TEXT("Pings")/(FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S"))+TEXT("_")+FGuid::NewGuid().ToString());
    IFileManager::Get().MakeDirectory(*Dir,true);
    FString Csv=TEXT("bearing_bin,range_bin,intensity\n");
    for(int A=0;A<Latest.config.num_azimuth_bins;++A) for(int R=0;R<Latest.config.num_range_bins;++R)
        Csv+=FString::Printf(TEXT("%d,%d,%.17g\n"),A,R,Latest.raw[A*Latest.config.num_range_bins+R]);
    const FString Metadata=FString::Printf(TEXT("range_m=%.9g\nfov_deg=%.9g\nfrequency_hz=%.9g\nsubrays=%d\nposition_m=%.9g,%.9g,%.9g\nraw_intensity=true\n"),
        Latest.config.max_range_m,Latest.config.horizontal_fov_deg,Latest.config.frequency_hz,Latest.config.num_elevation_subrays,
        Latest.pose.position.x,Latest.pose.position.y,Latest.pose.position.z);
    const bool Ok=FFileHelper::SaveStringToFile(Csv,*(Dir/TEXT("raw.csv"))) && FFileHelper::SaveStringToFile(Metadata,*(Dir/TEXT("parameters.txt")));
    Notice=Ok?TEXT("Raw ping saved under Saved/Pings"):TEXT("Export failed: could not write Saved/Pings");
}
void ASonarLab::Tick(float Dt)
{
    Super::Tick(Dt);Dt=FMath::Min(Dt,.1f);
    auto* PC=Cast<APlayerController>(GetController());if(!PC)return;
    if(PC->WasInputKeyJustPressed(EKeys::SpaceBar))Running=!Running;
    if(PC->WasInputKeyJustPressed(EKeys::Enter))Ping();
    if(PC->WasInputKeyJustPressed(EKeys::R))SetExperiment(Experiment);
    if(PC->WasInputKeyJustPressed(EKeys::One))SetExperiment(0);
    if(PC->WasInputKeyJustPressed(EKeys::Two))SetExperiment(1);
    if(PC->WasInputKeyJustPressed(EKeys::Three))SetExperiment(2);
    if(PC->WasInputKeyJustPressed(EKeys::Four))SetExperiment(3);
    if(PC->WasInputKeyJustPressed(EKeys::E))FlipElevation();
    if(PC->WasInputKeyJustPressed(EKeys::K))CarveCurrentView();
    if(PC->WasInputKeyJustPressed(EKeys::P))ExportPing();
    if(PC->WasInputKeyJustPressed(EKeys::F))PC->ConsoleCommand(TEXT("r.SetRes 1440x900wf"));
    if(PC->IsInputKeyDown(EKeys::RightMouseButton))
    {
        float X,Y;PC->GetInputMouseDelta(X,Y);auto R=Boom->GetRelativeRotation();R.Yaw+=X*.3;R.Pitch=FMath::Clamp(R.Pitch-Y*.3,-75.,-5.);Boom->SetRelativeRotation(R);
    }
    if(Running) {
    Time+=Dt;PingClock+=Dt;Sweep=FMath::Min(1.f,Sweep+Dt/1.2f);
    FVector Move=(GetActorForwardVector()*(float(PC->IsInputKeyDown(EKeys::Up))-float(PC->IsInputKeyDown(EKeys::Down)))+
        FVector(0,0,float(PC->IsInputKeyDown(EKeys::PageUp))-float(PC->IsInputKeyDown(EKeys::PageDown))))*30*Dt;
    AddActorWorldOffset(Move);
    AddActorWorldRotation(FRotator(0,(float(PC->IsInputKeyDown(EKeys::Right))-float(PC->IsInputKeyDown(EKeys::Left)))*15*Dt,
        (float(PC->IsInputKeyDown(EKeys::Period))-float(PC->IsInputKeyDown(EKeys::Comma)))*20*Dt));
    if(PingClock>1.6)Ping();
    }
    // The display ring advances in 1.2 s for inspection. Physical round trip is 2r/c.
    TArray<FVector> V,N;TArray<FVector2D> UV;TArray<int32> T;
    for(int A=0;A<=60;++A)
    {
        const double Theta=(-.5+A/60.)*Latest.config.horizontal_fov_deg*Pi/180;
        for(int Edge=0;Edge<2;++Edge)
        {
            const double Range=Latest.config.max_range_m*Sweep*(Edge?1:.98);
            V.Add(UE(Latest.pose.position+(Latest.pose.x_axis*std::sin(Theta)+Latest.pose.y_axis*std::cos(Theta))*Range));
            N.Add(FVector::UpVector);UV.Add(FVector2D(A/60.,Edge));
        }
        if(A<60){int I=A*2;T.Append({I,I+2,I+1,I+1,I+2,I+3});}
    }
    Wave->SetVisibility(Sweep<1);Wave->CreateMeshSection_LinearColor(0,V,T,N,UV,TArray<FLinearColor>(),TArray<FProcMeshTangent>(),false);
    if(Paths && !AcousticScene.spheres.empty())
    {
        const auto& Target=AcousticScene.spheres[0].centre;const auto& P=Latest.pose.position;
        DrawDebugLine(GetWorld(),UE(P),UE(Target),FColor(70,220,180),false,0,0,1);
        if(Config.multipath_enabled)
        {
            const Vec3 Mirrored{P.x,P.y,2*Config.surface_z-P.z};
            const double U=(Config.surface_z-Target.z)/(Mirrored.z-Target.z);
            const Vec3 Bounce=Target+(Mirrored-Target)*U;
            DrawDebugLine(GetWorld(),UE(P),UE(Bounce),FColor(235,168,100),false,0,0,1);
            DrawDebugLine(GetWorld(),UE(Bounce),UE(Target),FColor(235,168,100),false,0,0,1);
        }
    }
    if(ViewCount)for(size_t I=0;I<Voxels.size();++I)if(Kept[I])DrawDebugPoint(GetWorld(),UE(Voxels[I]),3,FColor(80,210,180),false,0);
}
ASonarLabMode::ASonarLabMode(){DefaultPawnClass=ASonarLab::StaticClass();HUDClass=ASonarLabHUD::StaticClass();}
void ASonarLabMode::BeginPlay()
{
    Super::BeginPlay();
    auto* Fog=GetWorld()->SpawnActor<AExponentialHeightFog>();Fog->GetComponent()->SetFogDensity(.02);
    Fog->GetComponent()->SetFogInscatteringColor(FLinearColor(.02,.06,.085));Fog->GetComponent()->SetVolumetricFog(true);
    auto* Light=GetWorld()->SpawnActor<ADirectionalLight>(FVector(0,0,1000),FRotator(-45,-20,0));
    Light->GetLightComponent()->SetIntensity(1.2);Light->GetLightComponent()->SetLightColor(FLinearColor(.4,.6,.7));
}
void ASonarLabHUD::Text(const FString& V,float X,float Y,FLinearColor C,float Scale)
{DrawText(V,C,X,Y,GEngine->GetMediumFont(),Scale,false);}
bool ASonarLabHUD::Button(const FString& V,float X,float Y,float W,bool Active)
{
    float MX=0,MY=0;PlayerOwner->GetMousePosition(MX,MY);const bool Hover=MX>=X&&MX<X+W&&MY>=Y&&MY<Y+40;
    DrawRect(Active?FLinearColor(.12,.28,.26,1):Hover?FLinearColor(.09,.13,.15,1):Panel,X,Y,W,40);
    DrawLine(X,Y+39,X+W,Y+39,Active?Cyan:Muted,1);Text(V,X+8,Y+12,Active?Cyan:White,.8);
    return Hover&&PlayerOwner->WasInputKeyJustPressed(EKeys::LeftMouseButton);
}
void ASonarLabHUD::DrawHUD()
{
    Super::DrawHUD();auto* S=Cast<ASonarLab>(PlayerOwner->GetPawn());if(!S||!Canvas)return;
    if(S->Latest.raw.empty()){Text(S->Notice,24,24,Amber);return;}
    const float W=Canvas->SizeX,H=Canvas->SizeY,L=212,R=FMath::Clamp(W*.32f,320.f,500.f),RX=W-R;
    DrawRect(Panel,0,0,W,136);DrawRect(Panel,0,136,L,H-136);DrawRect(Panel,RX,136,R,H-136);
    Text(TEXT("SONAR / OBSERVATORY"),24,20,White,1.2);Text(TEXT("THE LABORATORY     /     SYNTHETIC DATA"),W-440,24,Muted,.8);
    Text(TEXT("Watch sound reveal a scene."),24,56,White,1.5);
    Text(FString::Printf(TEXT("0%d  /  %s"),S->Experiment+1,Lessons[S->Experiment]),24,104,Amber,.9);
    Text(TEXT("01 / THE EXPERIMENT"),16,160,Muted,.75);
    for(int I=0;I<4;++I)if(Button(Names[I],16,192+I*48,L-32,I==S->Experiment))S->SetExperiment(I);
    if(Button(S->Running?TEXT("Pause"):TEXT("Run"),16,408,84,S->Running))S->Running=!S->Running;
    if(Button(TEXT("Step ping"),108,408,88))S->Ping();
    if(Button(TEXT("Reset"),16,456,180))S->SetExperiment(S->Experiment);
    if(Button(TEXT("Flip elevation"),16,520,180))S->FlipElevation();
    if(Button(TEXT("Capture & carve"),16,568,180))S->CarveCurrentView();
    if(Button(TEXT("Export raw ping"),16,616,180))S->ExportPing();
    Text(TEXT("02 / 3-D WORLD"),L+24,160,White,.8);
    Text(TEXT("Right-drag: orbit camera"),L+24,192,Muted,.8);
    DrawRect(Panel,L,H-144,RX-L,144);
    Text(FString::Printf(TEXT("PING %04d     %.1f ms / render     %d poses"),S->PingNumber,S->RenderMs,S->ViewCount),L+24,H-124,White,.9);
    Text(TEXT("Arrows: move / turn   PgUp/PgDn: rise / dive   , / .: roll"),L+24,H-92,Muted,.75);
    Text(TEXT("SPACE pause   ENTER ping   1-4 experiments   R reset"),L+24,H-68,Muted,.75);
    Text(TEXT("Slowed visual sweep. Real round trip: 2 x range / sound speed."),L+24,H-40,Amber,.7);
    Text(TEXT("03 / RANGE + BEARING"),RX+24,160,White,.9);
    const float Radius=FMath::Min(R-64.f,H*.39f);const FVector2D O(RX+R/2,220+Radius);
    auto Point=[&](double Angle,double Distance){return O+FVector2D(std::sin(Angle)*Distance,-std::cos(Angle)*Distance);};
    const auto& C=S->Latest.config;
    const double Fov=C.horizontal_fov_deg*Pi/180;
    double Peak=*std::max_element(S->Latest.display.begin(),S->Latest.display.end());
    for(int A=0;A<C.num_azimuth_bins;++A)for(int B=0;B<C.num_range_bins;++B)
    {
        const double Level=ResearchBridge::DisplayLevel(S->Latest.display[A*C.num_range_bins+B],Peak,S->GainDb,S->DynamicDb);
        if(Level<=0 || (B+.5)/C.num_range_bins>S->Sweep)continue;
        const auto P=Point(((A+.5)/C.num_azimuth_bins-.5)*Fov,(B+.5)/C.num_range_bins*Radius);
        DrawRect(FLinearColor(.1+Level*.5,.3+Level*.65,.25+Level*.55,1),P.X,P.Y,2.2,2.2);
    }
    for(int Ring=1;Ring<=4;++Ring)
    {
        for(int A=0;A<60;++A)
        {
            auto P=Point((A/60.-.5)*Fov,Radius*Ring/4),Q=Point(((A+1)/60.-.5)*Fov,Radius*Ring/4);
            DrawLine(P.X,P.Y,Q.X,Q.Y,FLinearColor(.14,.26,.28,.55),1);
        }
        Text(FString::Printf(TEXT("%.1f m"),C.max_range_m*Ring/4),O.X+4,O.Y-Radius*Ring/4,Muted,.7);
    }
    for(int A=-1;A<=1;++A)
    {
        const auto P=Point(A*Fov/2,Radius);DrawLine(O.X,O.Y,P.X,P.Y,Muted,1);
        Text(FString::Printf(TEXT("%+.0f"),A*C.horizontal_fov_deg/2),P.X-12,P.Y-20,Muted,.7);
    }
    float MX,MY;PlayerOwner->GetMousePosition(MX,MY);
    if(PlayerOwner->WasInputKeyJustPressed(EKeys::LeftMouseButton)&&MX>RX&&MY>200&&MY<O.Y)
    {
        const double Angle=std::atan2(MX-O.X,O.Y-MY);
        S->SelectedBearing=FMath::Clamp(int((Angle/Fov+.5)*C.num_azimuth_bins),0,C.num_azimuth_bins-1);
    }
    const float CY=O.Y+24;
    if(Button(S->Naive?TEXT("1 ray: naive"):TEXT("Elevation integrated"),RX+24,CY,R-48,S->Naive)){S->Naive=!S->Naive;S->Ping();}
    if(Button(S->Config.multipath_enabled?TEXT("Multipath ON"):TEXT("Multipath OFF"),RX+24,CY+48,(R-56)/2,S->Config.multipath_enabled))
        {S->Config.multipath_enabled=!S->Config.multipath_enabled;S->Ping();}
    if(Button(S->Speckle?TEXT("Speckle ON"):TEXT("Speckle OFF"),RX+32+(R-56)/2,CY+48,(R-56)/2,S->Speckle)){S->Speckle=!S->Speckle;S->Ping();}
    const float AY=CY+120;
    Text(TEXT("SELECTED BEARING / RAW POWER"),RX+24,AY-16,Muted,.7);
    for(int B=1;B<C.num_range_bins;++B)
    {
        const int I=S->SelectedBearing*C.num_range_bins+B;
        const double A=ResearchBridge::DisplayLevel(S->Latest.raw[I-1],Peak,0,60),V=ResearchBridge::DisplayLevel(S->Latest.raw[I],Peak,0,60);
        DrawLine(RX+24+(R-48)*(B-1)/C.num_range_bins,AY+64-A*56,RX+24+(R-48)*B/C.num_range_bins,AY+64-V*56,Cyan,1);
    }
    if(S->AmbiguityError>=0)Text(FString::Printf(TEXT("Flip relative difference: %.3g"),S->AmbiguityError),RX+24,AY+88,Amber,.8);
    else Text(FString::Printf(TEXT("%.0f kHz / %d sub-rays / %.3f mm wavelength"),C.frequency_hz/1000,C.num_elevation_subrays,C.speed_of_sound_mps/C.frequency_hz*1000),RX+24,AY+88,Muted,.7);
    Text(TEXT("Diffuse / point-scatterer approximation"),RX+24,AY+116,Muted,.7);
    Text(TEXT("No calibration or ghost-removal algorithm"),RX+24,AY+140,Muted,.7);
}
