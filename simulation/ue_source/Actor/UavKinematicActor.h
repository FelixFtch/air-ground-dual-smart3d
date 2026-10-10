// AirGroundLab: independent, collision-aware XYZ/yaw actor for CARLA.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "UavKinematicActor.generated.h"

class UBoxComponent;
class USkeletalMeshComponent;
struct FActorDescription;

UCLASS()
class CARLA_API AUavKinematicActor : public AActor
{
  GENERATED_BODY()
public:
  AUavKinematicActor();
  bool Configure(const FActorDescription &Description);
  // UE centimetres/degrees, world-local pose. Rate clipping is successful;
  // rejected commands/collisions are reported by the existing batch RPC.
  bool TryMove(const FTransform &Requested);
  virtual FVector GetVelocity() const override;
  FVector GetKinematicAngularVelocityDegrees() const;
  FVector GetBodyHalfExtent() const { return HalfExtentCm; }
  const FString &GetLastMoveReason() const { return LastMoveReason; }
  bool DidLastMoveSucceed() const { return LastMoveSucceeded; }

private:
  UPROPERTY() TObjectPtr<UBoxComponent> Body;
  UPROPERTY() TObjectPtr<USkeletalMeshComponent> Visual;
  FVector HalfExtentCm = FVector(48.0, 48.0, 34.0);
  FVector FlightMinCm = FVector(-1000.0, -1000.0, 20.0);
  FVector FlightMaxCm = FVector(1000.0, 1000.0, 620.0);
  FVector ActualVelocityCm = FVector::ZeroVector;
  double ActualYawVelocityDegrees = 0.0;
  double MaxSpeedCm = 150.0;
  double MaxAccelerationCm = 200.0;
  double MaxYawRateDegrees = 45.83662361;
  double MaxYawAccelerationDegrees = 114.5915590;
  double MaxRequestCm = 15.0;
  double LastMotionSimTime = -1.0;
  double LastCommandSimTime = -1.0;
  bool Configured = false;
  bool LastMoveSucceeded = false;
  FString LastMoveReason = TEXT("not_configured");
  bool InsideFlightBounds(const FTransform &Pose) const;
  bool Reject(const TCHAR *Reason, const FTransform &Initial, double Dt);
};
