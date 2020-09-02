// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "HomingProjectileMotion.generated.h"

// Initialization params for trajectory calculations
USTRUCT(BlueprintType)
struct FProjectileParams
{
	GENERATED_BODY()

public:
	FProjectileParams()
		: LaunchLocation(FVector::ZeroVector)
		, TargetLocation(FVector::ZeroVector)
		, GravityDirection(FVector::DownVector)
		, fGravity(0.0f)
		, ArcParameter(0.0f)
	{
	}

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HPM)
	FVector LaunchLocation;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HPM)
	FVector TargetLocation;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HPM)
	FVector GravityDirection;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HPM)
	float	fGravity;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HPM)
	float	ArcParameter;
};

// Denotes current state of projectile at any given moment.
USTRUCT(BlueprintType)
struct FProjectileState
{
	GENERATED_BODY()

public:
	FProjectileState()
		: LinearDirection(FVector::ForwardVector)
		, LaunchVelocity(FVector::ZeroVector)
		, NetVelocity(FVector::ZeroVector)
		, fLinearSpeed(0.0f)
		, fTotalTime(0.0f)
		, fTimeLeft(0.0f)
	{
	}

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HPM)
	FVector LinearDirection;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HPM)
	FVector LaunchVelocity;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HPM)
	FVector NetVelocity;

	// Cache
	FVector XVel;
	FVector YVel;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HPM)
	float	fLinearSpeed;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HPM)
	float	fTotalTime;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = HPM)
	float	fTimeLeft;

};

// This class takes in Launch parameters and updates velocity and acceleration to calculate final velocity of projectile for trajectory duration.
UCLASS(BlueprintType)
class YOUR_PROJECT_API UHomingProjectileMotion : public UObject
{
	GENERATED_BODY()

public:
	// (1) Before using getting projectile state, initialize launch parameters.
	UFUNCTION(BlueprintCallable, Category = Trajectory)
	void SetupHPM(UObject* WorldContext, const FProjectileParams LaunchParams);

	// (2) Update projectile motion every frame. This updates projectile velocity and provides additional information for gameplay purposes.
	UFUNCTION(BlueprintCallable, Category = Trajectory)
	FProjectileState UpdateHPM(FVector CurrentLocation, FVector CurrentTargetLocation, float TimeStamp);

	//void CalculateRequiredAcceleration(float dX);

	UFUNCTION(BlueprintPure, Category = Trajectory)
	FProjectileState GetProjectileState() const { return m_ProjectileState; }

	// This function generates vector perpendicular to given direction.
	// Alpha defines direction of generated vector. Accepts value 0 - 1 (0 - 360 degree)
	UFUNCTION(BlueprintPure, Category = Math)
	static FVector GetPerpendicularVector(FVector const& Dir, float Alpha);

private:
	// Gets projected linear direction on gravity direction plane from current projectile location to current target location.
	FVector GetLinearDirection(const FVector& CurrentLoacation, const FVector& CurrentTargetLocation);
	// Gets current linear speed required to reach transient target location.
	float GetHomingLinearSpeed(const FVector& CurrentLoacation, const FVector& CurrnetTargetLocation);
	// Calculates Launch velocity. Used when setting up for launch.
	FVector CalculateLaunchVelocity();
	// Gives rotation of Linear direction (new X) and Gravity(new -Z) Axes.
	FRotator GetAxesRotation();

	float GetTrajectoryDuration();

	// Helper functions to provide vector components for linear and gravity direction.
	FORCEINLINE FVector GetGravityComponent(FVector InVector);
	FORCEINLINE FVector GetLinearComponent(FVector InVector);

	//void DebugDraw();

	UObject* WorldContextObj;
	FProjectileParams m_Params;
	FProjectileState m_ProjectileState;
};
