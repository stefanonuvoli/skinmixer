#ifndef SKINMIXER_OPERATION_H
#define SKINMIXER_OPERATION_H

#include <nvl/nuvolib.h>

#include <vector>

namespace skinmixer {

enum OperationType { NONE, REMOVE, DETACH, REPLACE, ATTACH };
enum MergeMode { FIELD, BOOLEAN };

}

#endif // SKINMIXER_OPERATION_H
