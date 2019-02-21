#ifndef DARTPY_TYPES_H_
#define DARTPY_TYPES_H_

namespace dart {
namespace python {

struct JointType {
    enum Enum {
        PRISMATIC,
        REVOLUTE,
        SCREW,
        WELD,
        UNIVERSAL,
        BALL,
        EULER,
        PLANAR,
        TRANSLATIONAL,
        FREE,
    };
};

}
}

#endif
