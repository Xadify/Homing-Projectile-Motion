// Fill out your copyright notice in the Description page of Project Settings.

#include "HomingProjectileMotion.h"
#include <Kismet/GameplayStatics.h>

void UHomingProjectileMotion::SetupHPM(UObject * WorldContext, const FProjectileParams LaunchParams)
{
	if (!WorldContext)
	{
		return;
	}
	WorldContextObj	= WorldContext;

	m_Params = LaunchParams;
	// Reset to init values
	m_ProjectileState = FProjectileState();

	m_ProjectileState.LinearDirection	= GetLinearDirection(m_Params.LaunchLocation, m_Params.TargetLocation);

	m_ProjectileState.LaunchVelocity	= CalculateLaunchVelocity();

	// Cached velocity components.
	m_ProjectileState.YVel = GetGravityComponent(m_ProjectileState.LaunchVelocity);
	m_ProjectileState.XVel = GetLinearComponent(m_ProjectileState.LaunchVelocity);

	m_ProjectileState.fLinearSpeed = m_ProjectileState.XVel.Size();

	m_ProjectileState.fTotalTime = m_ProjectileState.fTimeLeft = GetTrajectoryDuration();
}

FProjectileState UHomingProjectileMotion::UpdateHPM(FVector CurrentLocation, FVector CurrentTargetLocation, float TimeStamp)
{
	if (!WorldContextObj || !WorldContextObj->GetWorld() || m_ProjectileState.fTimeLeft <= 0.0f)
	{
		return FProjectileState();
	}

	// Update linear direction in case target is moving
	m_ProjectileState.LinearDirection = GetLinearDirection(CurrentLocation, CurrentTargetLocation);
	// Update linear speed required to reach target distance
	// This ensures we can track forward and backward moving projectile in X,Y plane.
	m_ProjectileState.fLinearSpeed	= GetHomingLinearSpeed(CurrentLocation, CurrentTargetLocation);
	
	FVector ProjectedLine = FVector::PointPlaneProject(CurrentTargetLocation, CurrentLocation, m_ProjectileState.LinearDirection) - CurrentLocation;
	float dX = FVector::DotProduct(m_Params.GravityDirection, ProjectedLine);
	float V = FVector::DotProduct(m_ProjectileState.YVel, m_Params.GravityDirection);

	// Update gravity to track moving target in Z axis.
	// dX = ut + (0.5f * a * t^2) ----> a = 2 * (dX - ut) / t^2
	float MutatedGravity = (2.0f * (dX - (V * m_ProjectileState.fTimeLeft))) / (m_ProjectileState.fTimeLeft * m_ProjectileState.fTimeLeft);

	float DeltaTime = WorldContextObj->GetWorld()->GetDeltaSeconds();
	
	// Y component responsible for moving up/down.
	m_ProjectileState.YVel = m_ProjectileState.YVel + (MutatedGravity * m_Params.GravityDirection * DeltaTime);
	// X component responsible for moving forward.
	m_ProjectileState.XVel = m_ProjectileState.LinearDirection * m_ProjectileState.fLinearSpeed;

	m_ProjectileState.NetVelocity = m_ProjectileState.XVel + m_ProjectileState.YVel;

	// IMP - don't forget to decrement time of flight consumed.
	// this allows correct calculation of mutated linear speed and gravity for target tracking.
	m_ProjectileState.fTimeLeft -= DeltaTime;

	return m_ProjectileState;
}

FVector UHomingProjectileMotion::GetPerpendicularVector(FVector const& Dir, float Alpha)
{
	const float ConeHalfAngleRad = 90.0f;

	float const RandU = Alpha;
	float const RandV = 0.5;

	// Get spherical coords that have an even distribution over the unit sphere
	// Method described at http://mathworld.wolfram.com/SpherePointPicking.html	
	float Theta = 2.f * PI * RandU;
	float Phi = FMath::Acos((2.f * RandV) - 1.f);

	// restrict phi to [0, ConeHalfAngleRad]
	// this gives an even distribution of points on the surface of the cone
	// centered at the origin, pointing upward (z), with the desired angle
	Phi = FMath::Fmod(Phi, ConeHalfAngleRad);

	// get axes we need to rotate around
	FMatrix const DirMat = FRotationMatrix(Dir.Rotation());
	// note the axis translation, since we want the variation to be around X
	FVector const DirZ = DirMat.GetScaledAxis(EAxis::X);
	FVector const DirY = DirMat.GetScaledAxis(EAxis::Y);

	FVector Result = Dir.RotateAngleAxis(Phi * 180.f / PI, DirY);
	Result = Result.RotateAngleAxis(Theta * 180.f / PI, DirZ);

	// ensure it's a unit vector (might not have been passed in that way)
	Result = Result.GetSafeNormal();

	return Result;
}

FVector UHomingProjectileMotion::GetLinearDirection(const FVector& CurrentLoacation, const FVector& CurrentTargetLocation)
{
	// We are projecting projectile location and target location on plane with gravity as normal.
	// This ensures we get no up/down component on linear direction.
	FVector vDirection = (FVector::PointPlaneProject(CurrentTargetLocation, m_Params.LaunchLocation, m_Params.GravityDirection) - FVector::PointPlaneProject(CurrentLoacation, m_Params.LaunchLocation, m_Params.GravityDirection)).GetSafeNormal();
	if (vDirection.SizeSquared() > 0.0f)
	{
		return vDirection;
	}
	// This will happen when target location is exactly above or below us. (Based on gravity direction as our 'Down vector')
	// Since projectile will move only up or down at launch, we don't care about linear direction.
	// Just need it to be perpendicular to gravity direction. That ensures other calculations to work properly.
	else 
	{		
		return GetPerpendicularVector(m_Params.GravityDirection, 0.0f);
	}
}

float UHomingProjectileMotion::GetHomingLinearSpeed(const FVector & CurrentLoacation, const FVector & CurrentTargetLocation)
{
	float CurrentDistance = (FVector::PointPlaneProject(CurrentTargetLocation, m_Params.LaunchLocation, m_Params.GravityDirection)
							- FVector::PointPlaneProject(CurrentLoacation, m_Params.LaunchLocation, m_Params.GravityDirection)).Size();
	// V = dX / T
	return CurrentDistance / m_ProjectileState.fTimeLeft;
}

FVector UHomingProjectileMotion::CalculateLaunchVelocity()
{
	FVector LaunchVel = FVector::ZeroVector;

	FVector vLine = (m_Params.TargetLocation - m_Params.LaunchLocation).GetSafeNormal();
	FVector vCrossproduct = FVector::CrossProduct(vLine, m_Params.GravityDirection);

	// Check if target location is not exactly above or below us.
	if (vCrossproduct.SizeSquared() > 0.001f)
	{
		FRotator AxesRotation = GetAxesRotation();

		// [SuggestProjectileVelocity_CustomArc] only works with gravity acting on Z axis.
		// To work with that, transform current axes (linear direction and gravity) to match UE XYZ axes.
		FVector TranrformedTarget = m_Params.LaunchLocation + AxesRotation.UnrotateVector(m_Params.TargetLocation - m_Params.LaunchLocation);

		UGameplayStatics::SuggestProjectileVelocity_CustomArc(WorldContextObj, LaunchVel, m_Params.LaunchLocation, TranrformedTarget, -m_Params.fGravity, m_Params.ArcParameter);

		// Once we have launch velocity in UE XYZ axis, transform it back to original axes we have to get correct launch velocity.
		LaunchVel = AxesRotation.RotateVector(LaunchVel);
	}
	else
	{
		// Directly above, calculate initial velocity based on travel distance.
		if (FVector::DotProduct(vLine, m_Params.GravityDirection) < 0.0f)
		{
			// Final velocity will be 0 once we reach at target location.
			// v^2 (0.0f) = u^2 + 2ad. -----> u^2 = -2ad.
			LaunchVel = -m_Params.GravityDirection * FMath::Sqrt(2.0f * m_Params.fGravity * (m_Params.TargetLocation - m_Params.LaunchLocation).Size());
		}
		// if directly below us, launch velocity = 0 and let gravity do its work.
	}

	return LaunchVel;
}

FRotator UHomingProjectileMotion::GetAxesRotation()
{
	FVector vLine = (m_Params.TargetLocation - m_Params.LaunchLocation).GetSafeNormal();
	FVector vProjectedLine = FVector::VectorPlaneProject(vLine, m_Params.GravityDirection);

	FVector Up		= -m_Params.GravityDirection;
	FVector Fwd		= (vProjectedLine.SizeSquared() > 0.0001) ? vProjectedLine : m_ProjectileState.LinearDirection;
	FVector Right	= FVector::CrossProduct(Up, Fwd);

	FMatrix RotMatrix(Fwd, Right, Up, FVector::ZeroVector);

	return RotMatrix.Rotator();
}

float UHomingProjectileMotion::GetTrajectoryDuration()
{
	float T = 0.0f;
	// With linear speed > 0, we can use T = dX/V formula over Xaxis of trajectory to calculate time of flight.
	if (m_ProjectileState.fLinearSpeed > 0.001)
	{
		float dX = GetLinearComponent(m_Params.TargetLocation - m_Params.LaunchLocation).Size();
		float V = m_ProjectileState.fLinearSpeed;
		T = dX / V;
	}
	// if linear speed is 0, this means target is exactly above or below us.
	// Here we calculate time based on Y axis of trajectory using
	// v^2 = u^2 + 2ad ---> T = (V - U)/A
	else
	{
		float D = GetGravityComponent(m_Params.TargetLocation - m_Params.LaunchLocation).Size();
		float U = m_ProjectileState.LaunchVelocity.Size();
		float A = m_Params.fGravity * FVector::DotProduct((m_Params.TargetLocation - m_Params.LaunchLocation).GetSafeNormal(), m_Params.GravityDirection);

		float V = FMath::Sqrt(U*U + 2.0f * A * D);

		T = (V - U) / A;
	}
	
	return T;
}

FVector UHomingProjectileMotion::GetGravityComponent(FVector InVector)
{
	if (InVector.SizeSquared() > SMALL_NUMBER)
	{
		return InVector.ProjectOnTo(m_Params.GravityDirection);
	}
	else
	{
		return FVector::ZeroVector;
	}
}

FVector UHomingProjectileMotion::GetLinearComponent(FVector InVector)
{
	if (InVector.SizeSquared() > SMALL_NUMBER)
	{
		return InVector.ProjectOnTo(m_ProjectileState.LinearDirection);
	}
	else
	{
		return FVector::ZeroVector;
	}
}