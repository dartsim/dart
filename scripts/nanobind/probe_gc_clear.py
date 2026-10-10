"""Call the actual GC clear slots in both orders, including a dead IK node."""

import ctypes
import gc
import json
import weakref

import dartpy as dart
from probe_gc import KINDS, case

get_slot = ctypes.pythonapi.PyType_GetSlot
get_slot.argtypes = (ctypes.py_object, ctypes.c_int)
get_slot.restype = ctypes.c_void_p


def clear(obj):
    slot = get_slot(type(obj), 51)  # CPython's public Py_tp_clear constant.
    assert slot, type(obj)
    callback = ctypes.PYFUNCTYPE(ctypes.c_int, ctypes.py_object)(slot)
    assert callback(obj) == 0


if __name__ == "__main__":
    rows = []
    for kind in KINDS:
        for reverse in (False, True):
            for _ in range(20):
                child, owner = case(kind)
                refs = weakref.ref(child), weakref.ref(owner)
                objects = [child, owner] if reverse else [owner, child]
                for obj in objects:
                    clear(obj)
                # Idempotence matters when owners share parts of a native graph.
                clear(owner)
                del obj, objects, child, owner
                gc.collect()
                assert all(ref() is None for ref in refs), kind
        rows.append({"kind": kind, "orders": 2, "trials_per_order": 20})
    for _ in range(20):

        class Frame(dart.dynamics.SimpleFrame):
            pass

        skeleton = dart.dynamics.Skeleton()
        ik = skeleton.createFreeJointAndBodyNodePair()[1].getOrCreateIK()
        target = Frame()
        target.owner = ik
        ik.setTarget(target)
        skeleton_ref = weakref.ref(skeleton)
        del skeleton
        assert skeleton_ref() is None
        clear(ik)
        target.owner = None
        del target, ik
        gc.collect()
    print(
        json.dumps(
            {"owner_clear": rows, "ik_target_after_node_destruction": 20}, indent=2
        )
    )
