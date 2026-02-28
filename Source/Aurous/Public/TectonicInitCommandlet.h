#pragma once

#include "CoreMinimal.h"
#include "Commandlets/Commandlet.h"
#include "TectonicInitCommandlet.generated.h"

UCLASS()
class AUROUS_API UTectonicInitCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	UTectonicInitCommandlet();

	virtual int32 Main(const FString& Params) override;
};
