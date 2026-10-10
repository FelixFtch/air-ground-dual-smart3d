#include "UavKinematicActor.h"
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Actor/ActorDescription.h"

#include <util/ue-header-guard-begin.h>
#include "Components/BoxComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/World.h"
#include <util/ue-header-guard-end.h>

namespace
{
constexpr double SkinCm = 0.05;
constexpr double StepCm = 2.5;
constexpr double StepYawDegrees = 2.0;
constexpr double MaxRequestYawDegrees = 45.0;
constexpr double PositionToleranceCm = 0.001;
bool IsLevel(const FRotator &R)
{
  return !R.ContainsNaN() && FMath::Abs(FRotator::NormalizeAxis(R.Pitch)) <= 0.01 &&
      FMath::Abs(FRotator::NormalizeAxis(R.Roll)) <= 0.01;
}
}

AUavKinematicActor::AUavKinematicActor()
{
  PrimaryActorTick.bCanEverTick = false;
  Body = CreateDefaultSubobject<UBoxComponent>(TEXT("UavBody"));
  SetRootComponent(Body);
  Body->SetMobility(EComponentMobility::Movable);
  Body->InitBoxExtent(HalfExtentCm);
  Body->SetCollisionProfileName(TEXT("BlockAllDynamic"));
  Body->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
  Body->SetCollisionResponseToAllChannels(ECR_Block);
  Body->SetSimulatePhysics(false);
  Body->SetGenerateOverlapEvents(false);
  Visual = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("UavAppearance"));
  Visual->SetupAttachment(Body);
  Visual->SetMobility(EComponentMobility::Movable);
  Visual->SetCollisionEnabled(ECollisionEnabled::NoCollision);
  Visual->SetSimulatePhysics(false);
  // First phase is a rigid reference pose, not independently moving parts.
  Visual->SetComponentTickEnabled(false);
}

bool AUavKinematicActor::Configure(const FActorDescription &D)
{
  using ABFL = UActorBlueprintFunctionLibrary;
  auto Number = [&](const TCHAR *Key, double Default) {
    return static_cast<double>(ABFL::RetrieveActorAttributeToFloat(Key, D.Variations, Default));
  };
  HalfExtentCm = 100.0 * FVector(Number(TEXT("body_half_x_m"), .48),
      Number(TEXT("body_half_y_m"), .48), Number(TEXT("body_half_z_m"), .34));
  FlightMinCm = 100.0 * FVector(Number(TEXT("flight_min_x_m"), -10.0),
      Number(TEXT("flight_min_y_m"), -10.0), Number(TEXT("flight_min_z_m"), .2));
  FlightMaxCm = 100.0 * FVector(Number(TEXT("flight_max_x_m"), 10.0),
      Number(TEXT("flight_max_y_m"), 10.0), Number(TEXT("flight_max_z_m"), 6.2));
  MaxSpeedCm = 100.0 * Number(TEXT("max_speed_mps"), 1.5);
  MaxAccelerationCm = 100.0 * Number(TEXT("max_acceleration_mps2"), 2.0);
  MaxYawRateDegrees = FMath::RadiansToDegrees(Number(TEXT("max_yaw_rate_rps"), .8));
  MaxYawAccelerationDegrees = FMath::RadiansToDegrees(Number(TEXT("max_yaw_acceleration_rps2"), 2.0));
  MaxRequestCm = 100.0 * Number(TEXT("max_request_translation_m"), .15);
  const double VisualScale = Number(TEXT("visual_scale"), .35);
  const double VisualYaw = Number(TEXT("visual_yaw_degrees"), -90.0);
  if (HalfExtentCm.ContainsNaN() || FlightMinCm.ContainsNaN() || FlightMaxCm.ContainsNaN() ||
      !(HalfExtentCm.GetMin() > 1.0 && HalfExtentCm.GetMax() <= 200.0) ||
      !((FlightMaxCm - FlightMinCm).GetMin() > 2.0 * HalfExtentCm.GetMax()) ||
      !(MaxSpeedCm > 0.0 && MaxSpeedCm <= 500.0) ||
      !(MaxAccelerationCm > 0.0 && MaxAccelerationCm <= 1000.0) ||
      !(MaxYawRateDegrees > 0.0 && MaxYawRateDegrees <= 180.0) ||
      !(MaxYawAccelerationDegrees > 0.0 && MaxYawAccelerationDegrees <= 720.0) ||
      !(MaxRequestCm > 0.0 && MaxRequestCm <= 50.0) ||
      !(VisualScale > 0.0 && VisualScale <= 2.0) || !FMath::IsFinite(VisualYaw))
  {
    LastMoveReason = TEXT("invalid_uav_profile"); return false;
  }
  auto *Mesh = LoadObject<USkeletalMesh>(nullptr,
      TEXT("/Game/Robot_Bee/meshes/SK_Robot_Bee.SK_Robot_Bee"));
  if (!Mesh) { LastMoveReason = TEXT("fighter_mesh_unavailable"); return false; }
  const FBoxSphereBounds Bounds = Mesh->GetBounds();
  const FQuat VisualRotation = FRotator(0.0, VisualYaw, 0.0).Quaternion();
  const FVector ScaledExtent = Bounds.BoxExtent * VisualScale;
  const FVector RotatedExtent = VisualRotation.GetAxisX().GetAbs() * ScaledExtent.X +
      VisualRotation.GetAxisY().GetAbs() * ScaledExtent.Y +
      VisualRotation.GetAxisZ().GetAbs() * ScaledExtent.Z;
  if (Bounds.Origin.ContainsNaN() || RotatedExtent.ContainsNaN() ||
      RotatedExtent.X > HalfExtentCm.X || RotatedExtent.Y > HalfExtentCm.Y ||
      RotatedExtent.Z > HalfExtentCm.Z)
  {
    LastMoveReason = TEXT("visual_mesh_exceeds_collision_body"); return false;
  }
  Visual->SetSkeletalMeshAsset(Mesh);
  Visual->SetRelativeScale3D(FVector(VisualScale));
  Visual->SetRelativeRotation(VisualRotation);
  Visual->SetRelativeLocation(-VisualRotation.RotateVector(Bounds.Origin * VisualScale));
  Body->SetBoxExtent(HalfExtentCm, true);
  if (!IsLevel(GetActorRotation()) || !GetActorScale3D().Equals(FVector::OneVector, 1.e-6) ||
      !InsideFlightBounds(GetActorTransform()))
  {
    LastMoveReason = TEXT("spawn_outside_flight_bounds_or_level_pose"); return false;
  }
  FCollisionQueryParams Params(SCENE_QUERY_STAT(UavSpawn), false, this);
  const FCollisionResponseParams Response(Body->GetCollisionResponseToChannels());
  if (GetWorld()->OverlapBlockingTestByChannel(GetActorLocation(), GetActorQuat(),
      Body->GetCollisionObjectType(), FCollisionShape::MakeBox(HalfExtentCm + FVector(SkinCm)),
      Params, Response))
  {
    LastMoveReason = TEXT("spawn_body_overlaps_world"); return false;
  }
  Configured = true;
  LastMoveSucceeded = true;
  LastMoveReason = TEXT("spawned");
  Tags.AddUnique(TEXT("AirGroundUavRigid"));
  UE_LOG(LogCarla, Display, TEXT("UAV configured body_half_cm=%s visual_scale=%.3f visual_yaw=%.1f visual_offset_cm=%s"),
      *HalfExtentCm.ToCompactString(), VisualScale, VisualYaw, *Visual->GetRelativeLocation().ToCompactString());
  return true;
}

bool AUavKinematicActor::InsideFlightBounds(const FTransform &Pose) const
{
  const FQuat Q = Pose.GetRotation();
  const FVector Aabb = Q.GetAxisX().GetAbs() * HalfExtentCm.X +
      Q.GetAxisY().GetAbs() * HalfExtentCm.Y + Q.GetAxisZ().GetAbs() * HalfExtentCm.Z + FVector(SkinCm);
  const FVector Lo = Pose.GetLocation() - Aabb;
  const FVector Hi = Pose.GetLocation() + Aabb;
  return Lo.X >= FlightMinCm.X && Lo.Y >= FlightMinCm.Y && Lo.Z >= FlightMinCm.Z &&
      Hi.X <= FlightMaxCm.X && Hi.Y <= FlightMaxCm.Y && Hi.Z <= FlightMaxCm.Z;
}

bool AUavKinematicActor::Reject(const TCHAR *Reason, const FTransform &Initial, double Dt)
{
  LastMoveSucceeded = false; LastMoveReason = Reason;
  ActualVelocityCm = Dt > 0.0 ? (GetActorLocation() - Initial.GetLocation()) / Dt : FVector::ZeroVector;
  ActualYawVelocityDegrees = Dt > 0.0 ? FMath::FindDeltaAngleDegrees(
      Initial.Rotator().Yaw, GetActorRotation().Yaw) / Dt : 0.0;
  LastMotionSimTime = GetWorld() ? GetWorld()->GetTimeSeconds() : -1.0;
  UE_LOG(LogCarla, Verbose, TEXT("UAV move rejected: %s actual_cm=%s"), Reason, *GetActorLocation().ToCompactString());
  return false;
}

bool AUavKinematicActor::TryMove(const FTransform &Requested)
{
  const FTransform Initial = GetActorTransform();
  if (!Configured || !GetWorld() || !Body->IsRegistered() || !Body->IsPhysicsStateCreated() ||
      !Body->IsQueryCollisionEnabled() || Body->IsSimulatingPhysics() || Body->GetAttachParent())
    return Reject(TEXT("query_body_unavailable"), Initial, 0.0);
  if (Initial.ContainsNaN() || Requested.ContainsNaN() ||
      !Initial.GetScale3D().Equals(FVector::OneVector, 1.e-6) ||
      !Requested.GetScale3D().Equals(FVector::OneVector, 1.e-6) ||
      !IsLevel(Initial.Rotator()) || !IsLevel(Requested.Rotator()))
    return Reject(TEXT("finite_unit_scale_level_pose_required"), Initial, 0.0);
  const FVector WantedDelta = Requested.GetLocation() - Initial.GetLocation();
  const double WantedYaw = FMath::FindDeltaAngleDegrees(Initial.Rotator().Yaw, Requested.Rotator().Yaw);
  if (WantedDelta.Size() > MaxRequestCm + PositionToleranceCm ||
      FMath::Abs(WantedYaw) > MaxRequestYawDegrees + .0001)
    return Reject(TEXT("request_exceeds_bounded_move"), Initial, 0.0);
  const double Now = GetWorld()->GetTimeSeconds();
  const double Gap = LastCommandSimTime >= 0.0 ? Now - LastCommandSimTime : GetWorld()->GetDeltaSeconds();
  if (!(Gap > 0.000001)) return Reject(TEXT("same_simulation_tick"), Initial, 0.0);
  const double Dt = FMath::Clamp(Gap, 0.001, 0.1);
  LastCommandSimTime = Now;
  if (Gap > .2) { ActualVelocityCm = FVector::ZeroVector; ActualYawVelocityDegrees = 0.0; }
  FVector WantedVelocity = (WantedDelta / Dt).GetClampedToMaxSize(MaxSpeedCm);
  FVector Velocity = ActualVelocityCm + (WantedVelocity - ActualVelocityCm).GetClampedToMaxSize(MaxAccelerationCm * Dt);
  FVector Delta = (Velocity * Dt).GetClampedToMaxSize(WantedDelta.Size());
  // No residual integration: requesting a stationary pose stops immediately.
  if (WantedDelta.IsNearlyZero(PositionToleranceCm)) Delta = FVector::ZeroVector;
  const double WantedYawRate = FMath::Clamp(WantedYaw / Dt, -MaxYawRateDegrees, MaxYawRateDegrees);
  const double YawRate = ActualYawVelocityDegrees + FMath::Clamp(WantedYawRate - ActualYawVelocityDegrees,
      -MaxYawAccelerationDegrees * Dt, MaxYawAccelerationDegrees * Dt);
  double YawDelta = FMath::Clamp(YawRate * Dt, -FMath::Abs(WantedYaw), FMath::Abs(WantedYaw));
  if (FMath::Abs(WantedYaw) < .0001) YawDelta = 0.0;
  const int32 Steps = FMath::Max(1, FMath::Max(FMath::CeilToInt(Delta.Size() / StepCm),
      FMath::CeilToInt(FMath::Abs(YawDelta) / StepYawDegrees)));
  if (Steps > 32) return Reject(TEXT("too_many_substeps"), Initial, Dt);
  FCollisionQueryParams Params(SCENE_QUERY_STAT(UavKinematicMove), false, this);
  TArray<AActor *> Attached; GetAttachedActors(Attached, true, true); Params.AddIgnoredActors(Attached);
  const FCollisionResponseParams Response(Body->GetCollisionResponseToChannels());
  const auto Channel = Body->GetCollisionObjectType();
  const double Radius = FVector2D(HalfExtentCm.X, HalfExtentCm.Y).Size();
  const double AngularInflation = 2.0 * Radius * FMath::Sin(FMath::DegreesToRadians(FMath::Abs(YawDelta) / Steps) * .25);
  const FCollisionShape Envelope = FCollisionShape::MakeBox(HalfExtentCm +
      FVector(AngularInflation + SkinCm, AngularInflation + SkinCm, SkinCm));
  FTransform Previous = Initial;
  for (int32 I = 1; I <= Steps; ++I)
  {
    const double Alpha = static_cast<double>(I) / Steps;
    const FQuat Rotation = FRotator(0.0, Initial.Rotator().Yaw + Alpha * YawDelta, 0.0).Quaternion();
    const FQuat MidRotation = FRotator(0.0, Initial.Rotator().Yaw + (Alpha - .5 / Steps) * YawDelta, 0.0).Quaternion();
    const FTransform Candidate(Rotation, Initial.GetLocation() + Alpha * Delta, FVector::OneVector);
    // Test endpoints and the rotation-inflated continuous XYZ sweep.
    if (!InsideFlightBounds(Candidate)) return Reject(TEXT("flight_bounds"), Initial, Dt);
    // Endpoint containment alone can miss an intermediate yaw extremum.
    // The same rotation-inflated envelope must remain inside the flight box.
    const FVector EnvelopeHalf = HalfExtentCm + FVector(
        AngularInflation + SkinCm, AngularInflation + SkinCm, SkinCm);
    const FVector WorldHalf = MidRotation.GetAxisX().GetAbs() * EnvelopeHalf.X +
        MidRotation.GetAxisY().GetAbs() * EnvelopeHalf.Y + MidRotation.GetAxisZ().GetAbs() * EnvelopeHalf.Z;
    for (const FVector Center : {Previous.GetLocation(), Candidate.GetLocation()})
    {
      const FVector Lo = Center - WorldHalf;
      const FVector Hi = Center + WorldHalf;
      if (Lo.X < FlightMinCm.X || Lo.Y < FlightMinCm.Y || Lo.Z < FlightMinCm.Z ||
          Hi.X > FlightMaxCm.X || Hi.Y > FlightMaxCm.Y || Hi.Z > FlightMaxCm.Z)
        return Reject(TEXT("flight_bounds_swept_envelope"), Initial, Dt);
    }
    FHitResult Hit;
    if (GetWorld()->OverlapBlockingTestByChannel(Previous.GetLocation(), MidRotation, Channel, Envelope, Params, Response) ||
        GetWorld()->SweepSingleByChannel(Hit, Previous.GetLocation(), Candidate.GetLocation(), MidRotation, Channel, Envelope, Params, Response) ||
        GetWorld()->OverlapBlockingTestByChannel(Candidate.GetLocation(), MidRotation, Channel, Envelope, Params, Response))
      return Reject(TEXT("swept_body_blocked"), Initial, Dt);
    SetActorLocationAndRotation(Candidate.GetLocation(), Rotation, true, &Hit, ETeleportType::None);
    const FTransform Actual = GetActorTransform();
    if (Hit.bBlockingHit || !Actual.GetLocation().Equals(Candidate.GetLocation(), .01) ||
        !Actual.GetRotation().Equals(Rotation, 1.e-5))
      return Reject(TEXT("root_sweep_clamped"), Initial, Dt);
    Previous = Actual;
  }
  ActualVelocityCm = (GetActorLocation() - Initial.GetLocation()) / Dt;
  ActualYawVelocityDegrees = FMath::FindDeltaAngleDegrees(Initial.Rotator().Yaw, GetActorRotation().Yaw) / Dt;
  LastMotionSimTime = Now; LastMoveSucceeded = true;
  LastMoveReason = Delta.Equals(WantedDelta, .01) && FMath::IsNearlyEqual(YawDelta, WantedYaw, .001)
      ? TEXT("reached_requested_step") : TEXT("rate_limited");
  return true;
}

FVector AUavKinematicActor::GetVelocity() const
{
  return GetWorld() && GetWorld()->GetTimeSeconds() - LastMotionSimTime <= .15 ? ActualVelocityCm : FVector::ZeroVector;
}

FVector AUavKinematicActor::GetKinematicAngularVelocityDegrees() const
{
  return GetWorld() && GetWorld()->GetTimeSeconds() - LastMotionSimTime <= .15
      ? FVector(0.0, 0.0, ActualYawVelocityDegrees) : FVector::ZeroVector;
}
