#ifndef CONTEXT_H
#define CONTEXT_H

#include <stdint.h>

typedef struct ContextInfo
{
	bool mps;
	uint64_t countMPS;
	uint64_t totalCount;
} CONTEXTINFO;

#endif // CONTEXT_H