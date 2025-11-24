from __future__ import annotations
import dartpy.common
import dartpy.dynamics
import dartpy.math
import dartpy.simulation
import numpy
import typing
__all__: list[str] = ['BodyNodeDnD', 'CameraMode', 'DragAndDrop', 'GUIActionAdapter', 'GUIEventAdapter', 'GUIEventHandler', 'GridVisual', 'ImGuiHandler', 'ImGuiViewer', 'ImGuiWidget', 'InteractiveFrame', 'InteractiveFrameDnD', 'InteractiveTool', 'PolyhedronVisual', 'RealTimeWorldNode', 'ShadowMap', 'ShadowTechnique', 'SimpleFrameDnD', 'SimpleFrameShapeDnD', 'Viewer', 'ViewerAttachment', 'WorldNode', 'osgViewer']
class BodyNodeDnD(DragAndDrop):
    @typing.overload
    def __init__(self, viewer: Viewer, bn: dartpy.dynamics.BodyNode) -> None:
        ...
    @typing.overload
    def __init__(self, viewer: Viewer, bn: dartpy.dynamics.BodyNode, useExternalIK: bool) -> None:
        ...
    @typing.overload
    def __init__(self, viewer: Viewer, bn: dartpy.dynamics.BodyNode, useExternalIK: bool, useWholeBody: bool) -> None:
        ...
    def getJointRestrictionModKey(self) -> GUIEventAdapter.ModKeyMask:
        ...
    def getPreserveOrientationModKey(self) -> GUIEventAdapter.ModKeyMask:
        ...
    def isUsingExternalIK(self) -> bool:
        ...
    def isUsingWholeBody(self) -> bool:
        ...
    def move(self) -> None:
        ...
    def release(self) -> None:
        ...
    def saveState(self) -> None:
        ...
    def setJointRestrictionModKey(self, modkey: GUIEventAdapter.ModKeyMask) -> None:
        ...
    def setPreserveOrientationModKey(self, modkey: GUIEventAdapter.ModKeyMask) -> None:
        ...
    def update(self) -> None:
        ...
    def useExternalIK(self, external: bool) -> None:
        ...
    def useWholeBody(self, wholeBody: bool) -> None:
        ...
class CameraMode:
    """
    Members:
    
      RGBA
    
      DEPTH
    """
    DEPTH: typing.ClassVar[CameraMode]  # value = <CameraMode.DEPTH: 1>
    RGBA: typing.ClassVar[CameraMode]  # value = <CameraMode.RGBA: 0>
    __members__: typing.ClassVar[dict[str, CameraMode]]  # value = {'RGBA': <CameraMode.RGBA: 0>, 'DEPTH': <CameraMode.DEPTH: 1>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class DragAndDrop(dartpy.common.Observer, dartpy.common.Subject):
    class RotationOption:
        """
        Members:
        
          HOLD_MODKEY
        
          ALWAYS_ON
        
          ALWAYS_OFF
        """
        ALWAYS_OFF: typing.ClassVar[DragAndDrop.RotationOption]  # value = <RotationOption.ALWAYS_OFF: 2>
        ALWAYS_ON: typing.ClassVar[DragAndDrop.RotationOption]  # value = <RotationOption.ALWAYS_ON: 1>
        HOLD_MODKEY: typing.ClassVar[DragAndDrop.RotationOption]  # value = <RotationOption.HOLD_MODKEY: 0>
        __members__: typing.ClassVar[dict[str, DragAndDrop.RotationOption]]  # value = {'HOLD_MODKEY': <RotationOption.HOLD_MODKEY: 0>, 'ALWAYS_ON': <RotationOption.ALWAYS_ON: 1>, 'ALWAYS_OFF': <RotationOption.ALWAYS_OFF: 2>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    ALWAYS_OFF: typing.ClassVar[DragAndDrop.RotationOption]  # value = <RotationOption.ALWAYS_OFF: 2>
    ALWAYS_ON: typing.ClassVar[DragAndDrop.RotationOption]  # value = <RotationOption.ALWAYS_ON: 1>
    HOLD_MODKEY: typing.ClassVar[DragAndDrop.RotationOption]  # value = <RotationOption.HOLD_MODKEY: 0>
    def constrainToLine(self, slope: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def constrainToPlane(self, normal: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def getConstrainedDx(self) -> numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def getConstrainedRotation(self) -> dartpy.math.AngleAxis:
        ...
    def getRotationModKey(self) -> GUIEventAdapter.ModKeyMask:
        ...
    def getRotationOption(self) -> ...:
        ...
    def isMoving(self) -> bool:
        ...
    def isObstructable(self) -> bool:
        ...
    def move(self) -> None:
        ...
    def release(self) -> None:
        ...
    def saveState(self) -> None:
        ...
    def setObstructable(self, obstructable: bool) -> None:
        ...
    def setRotationModKey(self, rotationModKey: GUIEventAdapter.ModKeyMask) -> None:
        ...
    def setRotationOption(self, option: ...) -> None:
        ...
    def unconstrain(self) -> None:
        ...
    def update(self) -> None:
        ...
class GUIActionAdapter:
    pass
class GUIEventAdapter:
    class EventType:
        """
        Members:
        
          NONE
        
          PUSH
        
          RELEASE
        
          DOUBLECLICK
        
          DRAG
        
          MOVE
        
          KEYDOWN
        
          KEYUP
        
          FRAME
        
          RESIZE
        
          SCROLL
        
          PEN_PRESSURE
        
          PEN_ORIENTATION
        
          PEN_PROXIMITY_ENTER
        
          PEN_PROXIMITY_LEAVE
        
          CLOSE_WINDOW
        
          QUIT_APPLICATION
        
          USER
        """
        CLOSE_WINDOW: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.CLOSE_WINDOW: 16384>
        DOUBLECLICK: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.DOUBLECLICK: 4>
        DRAG: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.DRAG: 8>
        FRAME: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.FRAME: 128>
        KEYDOWN: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.KEYDOWN: 32>
        KEYUP: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.KEYUP: 64>
        MOVE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.MOVE: 16>
        NONE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.NONE: 0>
        PEN_ORIENTATION: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.PEN_ORIENTATION: 2048>
        PEN_PRESSURE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.PEN_PRESSURE: 1024>
        PEN_PROXIMITY_ENTER: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.PEN_PROXIMITY_ENTER: 4096>
        PEN_PROXIMITY_LEAVE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.PEN_PROXIMITY_LEAVE: 8192>
        PUSH: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.PUSH: 1>
        QUIT_APPLICATION: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.QUIT_APPLICATION: 32768>
        RELEASE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.RELEASE: 2>
        RESIZE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.RESIZE: 256>
        SCROLL: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.SCROLL: 512>
        USER: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.USER: 65536>
        __members__: typing.ClassVar[dict[str, GUIEventAdapter.EventType]]  # value = {'NONE': <EventType.NONE: 0>, 'PUSH': <EventType.PUSH: 1>, 'RELEASE': <EventType.RELEASE: 2>, 'DOUBLECLICK': <EventType.DOUBLECLICK: 4>, 'DRAG': <EventType.DRAG: 8>, 'MOVE': <EventType.MOVE: 16>, 'KEYDOWN': <EventType.KEYDOWN: 32>, 'KEYUP': <EventType.KEYUP: 64>, 'FRAME': <EventType.FRAME: 128>, 'RESIZE': <EventType.RESIZE: 256>, 'SCROLL': <EventType.SCROLL: 512>, 'PEN_PRESSURE': <EventType.PEN_PRESSURE: 1024>, 'PEN_ORIENTATION': <EventType.PEN_ORIENTATION: 2048>, 'PEN_PROXIMITY_ENTER': <EventType.PEN_PROXIMITY_ENTER: 4096>, 'PEN_PROXIMITY_LEAVE': <EventType.PEN_PROXIMITY_LEAVE: 8192>, 'CLOSE_WINDOW': <EventType.CLOSE_WINDOW: 16384>, 'QUIT_APPLICATION': <EventType.QUIT_APPLICATION: 32768>, 'USER': <EventType.USER: 65536>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class KeySymbol:
        """
        Members:
        
          KEY_Space
        
          KEY_0
        
          KEY_1
        
          KEY_2
        
          KEY_3
        
          KEY_4
        
          KEY_5
        
          KEY_6
        
          KEY_7
        
          KEY_8
        
          KEY_9
        
          KEY_A
        
          KEY_B
        
          KEY_C
        
          KEY_D
        
          KEY_E
        
          KEY_F
        
          KEY_G
        
          KEY_H
        
          KEY_I
        
          KEY_J
        
          KEY_K
        
          KEY_L
        
          KEY_M
        
          KEY_N
        
          KEY_O
        
          KEY_P
        
          KEY_Q
        
          KEY_R
        
          KEY_S
        
          KEY_T
        
          KEY_U
        
          KEY_V
        
          KEY_W
        
          KEY_X
        
          KEY_Y
        
          KEY_Z
        
          KEY_Exclaim
        
          KEY_Quotedbl
        
          KEY_Hash
        
          KEY_Dollar
        
          KEY_Ampersand
        
          KEY_Quote
        
          KEY_Leftparen
        
          KEY_Rightparen
        
          KEY_Asterisk
        
          KEY_Plus
        
          KEY_Comma
        
          KEY_Minus
        
          KEY_Period
        
          KEY_Slash
        
          KEY_Colon
        
          KEY_Semicolon
        
          KEY_Less
        
          KEY_Equals
        
          KEY_Greater
        
          KEY_Question
        
          KEY_At
        
          KEY_Leftbracket
        
          KEY_Backslash
        
          KEY_Rightbracket
        
          KEY_Caret
        
          KEY_Underscore
        
          KEY_Backquote
        
          KEY_BackSpace
        
          KEY_Tab
        
          KEY_Linefeed
        
          KEY_Clear
        
          KEY_Return
        
          KEY_Pause
        
          KEY_Scroll_Lock
        
          KEY_Sys_Req
        
          KEY_Escape
        
          KEY_Delete
        
          KEY_Home
        
          KEY_Left
        
          KEY_Up
        
          KEY_Right
        
          KEY_Down
        
          KEY_Prior
        
          KEY_Page_Up
        
          KEY_Next
        
          KEY_Page_Down
        
          KEY_End
        
          KEY_Begin
        
          KEY_Select
        
          KEY_Print
        
          KEY_Execute
        
          KEY_Insert
        
          KEY_Undo
        
          KEY_Redo
        
          KEY_Menu
        
          KEY_Find
        
          KEY_Cancel
        
          KEY_Help
        
          KEY_Break
        
          KEY_Mode_switch
        
          KEY_Script_switch
        
          KEY_Num_Lock
        
          KEY_KP_Space
        
          KEY_KP_Tab
        
          KEY_KP_Enter
        
          KEY_KP_F1
        
          KEY_KP_F2
        
          KEY_KP_F3
        
          KEY_KP_F4
        
          KEY_KP_Home
        
          KEY_KP_Left
        
          KEY_KP_Up
        
          KEY_KP_Right
        
          KEY_KP_Down
        
          KEY_KP_Prior
        
          KEY_KP_Page_Up
        
          KEY_KP_Next
        
          KEY_KP_Page_Down
        
          KEY_KP_End
        
          KEY_KP_Begin
        
          KEY_KP_Insert
        
          KEY_KP_Delete
        
          KEY_KP_Equal
        
          KEY_KP_Multiply
        
          KEY_KP_Add
        
          KEY_KP_Separator
        
          KEY_KP_Subtract
        
          KEY_KP_Decimal
        
          KEY_KP_Divide
        
          KEY_KP_0
        
          KEY_KP_1
        
          KEY_KP_2
        
          KEY_KP_3
        
          KEY_KP_4
        
          KEY_KP_5
        
          KEY_KP_6
        
          KEY_KP_7
        
          KEY_KP_8
        
          KEY_KP_9
        
          KEY_F1
        
          KEY_F2
        
          KEY_F3
        
          KEY_F4
        
          KEY_F5
        
          KEY_F6
        
          KEY_F7
        
          KEY_F8
        
          KEY_F9
        
          KEY_F10
        
          KEY_F11
        
          KEY_F12
        
          KEY_F13
        
          KEY_F14
        
          KEY_F15
        
          KEY_F16
        
          KEY_F17
        
          KEY_F18
        
          KEY_F19
        
          KEY_F20
        
          KEY_F21
        
          KEY_F22
        
          KEY_F23
        
          KEY_F24
        
          KEY_F25
        
          KEY_F26
        
          KEY_F27
        
          KEY_F28
        
          KEY_F29
        
          KEY_F30
        
          KEY_F31
        
          KEY_F32
        
          KEY_F33
        
          KEY_F34
        
          KEY_F35
        
          KEY_Shift_L
        
          KEY_Shift_R
        
          KEY_Control_L
        
          KEY_Control_R
        
          KEY_Caps_Lock
        
          KEY_Shift_Lock
        
          KEY_Meta_L
        
          KEY_Meta_R
        
          KEY_Alt_L
        
          KEY_Alt_R
        
          KEY_Super_L
        
          KEY_Super_R
        
          KEY_Hyper_L
        
          KEY_Hyper_R
        """
        KEY_0: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_0: 48>
        KEY_1: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_1: 49>
        KEY_2: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_2: 50>
        KEY_3: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_3: 51>
        KEY_4: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_4: 52>
        KEY_5: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_5: 53>
        KEY_6: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_6: 54>
        KEY_7: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_7: 55>
        KEY_8: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_8: 56>
        KEY_9: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_9: 57>
        KEY_A: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_A: 97>
        KEY_Alt_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Alt_L: 65513>
        KEY_Alt_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Alt_R: 65514>
        KEY_Ampersand: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Ampersand: 38>
        KEY_Asterisk: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Asterisk: 42>
        KEY_At: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_At: 64>
        KEY_B: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_B: 98>
        KEY_BackSpace: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_BackSpace: 65288>
        KEY_Backquote: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Backquote: 96>
        KEY_Backslash: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Backslash: 92>
        KEY_Begin: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Begin: 65368>
        KEY_Break: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Break: 65387>
        KEY_C: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_C: 99>
        KEY_Cancel: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Cancel: 65385>
        KEY_Caps_Lock: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Caps_Lock: 65509>
        KEY_Caret: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Caret: 94>
        KEY_Clear: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Clear: 65291>
        KEY_Colon: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Colon: 58>
        KEY_Comma: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Comma: 44>
        KEY_Control_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Control_L: 65507>
        KEY_Control_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Control_R: 65508>
        KEY_D: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_D: 100>
        KEY_Delete: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Delete: 65535>
        KEY_Dollar: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Dollar: 36>
        KEY_Down: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Down: 65364>
        KEY_E: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_E: 101>
        KEY_End: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_End: 65367>
        KEY_Equals: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Equals: 61>
        KEY_Escape: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Escape: 65307>
        KEY_Exclaim: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Exclaim: 33>
        KEY_Execute: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Execute: 65378>
        KEY_F: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F: 102>
        KEY_F1: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F1: 65470>
        KEY_F10: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F10: 65479>
        KEY_F11: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F11: 65480>
        KEY_F12: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F12: 65481>
        KEY_F13: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F13: 65482>
        KEY_F14: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F14: 65483>
        KEY_F15: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F15: 65484>
        KEY_F16: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F16: 65485>
        KEY_F17: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F17: 65486>
        KEY_F18: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F18: 65487>
        KEY_F19: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F19: 65488>
        KEY_F2: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F2: 65471>
        KEY_F20: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F20: 65489>
        KEY_F21: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F21: 65490>
        KEY_F22: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F22: 65491>
        KEY_F23: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F23: 65492>
        KEY_F24: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F24: 65493>
        KEY_F25: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F25: 65494>
        KEY_F26: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F26: 65495>
        KEY_F27: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F27: 65496>
        KEY_F28: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F28: 65497>
        KEY_F29: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F29: 65498>
        KEY_F3: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F3: 65472>
        KEY_F30: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F30: 65499>
        KEY_F31: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F31: 65500>
        KEY_F32: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F32: 65501>
        KEY_F33: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F33: 65502>
        KEY_F34: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F34: 65503>
        KEY_F35: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F35: 65504>
        KEY_F4: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F4: 65473>
        KEY_F5: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F5: 65474>
        KEY_F6: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F6: 65475>
        KEY_F7: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F7: 65476>
        KEY_F8: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F8: 65477>
        KEY_F9: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F9: 65478>
        KEY_Find: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Find: 65384>
        KEY_G: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_G: 103>
        KEY_Greater: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Greater: 62>
        KEY_H: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_H: 104>
        KEY_Hash: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Hash: 35>
        KEY_Help: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Help: 65386>
        KEY_Home: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Home: 65360>
        KEY_Hyper_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Hyper_L: 65517>
        KEY_Hyper_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Hyper_R: 65518>
        KEY_I: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_I: 105>
        KEY_Insert: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Insert: 65379>
        KEY_J: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_J: 106>
        KEY_K: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_K: 107>
        KEY_KP_0: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_0: 65456>
        KEY_KP_1: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_1: 65457>
        KEY_KP_2: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_2: 65458>
        KEY_KP_3: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_3: 65459>
        KEY_KP_4: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_4: 65460>
        KEY_KP_5: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_5: 65461>
        KEY_KP_6: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_6: 65462>
        KEY_KP_7: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_7: 65463>
        KEY_KP_8: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_8: 65464>
        KEY_KP_9: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_9: 65465>
        KEY_KP_Add: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Add: 65451>
        KEY_KP_Begin: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Begin: 65437>
        KEY_KP_Decimal: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Decimal: 65454>
        KEY_KP_Delete: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Delete: 65439>
        KEY_KP_Divide: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Divide: 65455>
        KEY_KP_Down: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Down: 65433>
        KEY_KP_End: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_End: 65436>
        KEY_KP_Enter: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Enter: 65421>
        KEY_KP_Equal: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Equal: 65469>
        KEY_KP_F1: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_F1: 65425>
        KEY_KP_F2: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_F2: 65426>
        KEY_KP_F3: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_F3: 65427>
        KEY_KP_F4: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_F4: 65428>
        KEY_KP_Home: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Home: 65429>
        KEY_KP_Insert: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Insert: 65438>
        KEY_KP_Left: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Left: 65430>
        KEY_KP_Multiply: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Multiply: 65450>
        KEY_KP_Next: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Next: 65435>
        KEY_KP_Page_Down: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Next: 65435>
        KEY_KP_Page_Up: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Prior: 65434>
        KEY_KP_Prior: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Prior: 65434>
        KEY_KP_Right: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Right: 65432>
        KEY_KP_Separator: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Separator: 65452>
        KEY_KP_Space: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Space: 65408>
        KEY_KP_Subtract: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Subtract: 65453>
        KEY_KP_Tab: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Tab: 65417>
        KEY_KP_Up: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Up: 65431>
        KEY_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_L: 108>
        KEY_Left: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Left: 65361>
        KEY_Leftbracket: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Leftbracket: 91>
        KEY_Leftparen: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Leftparen: 40>
        KEY_Less: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Less: 60>
        KEY_Linefeed: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Linefeed: 65290>
        KEY_M: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_M: 109>
        KEY_Menu: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Menu: 65383>
        KEY_Meta_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Meta_L: 65511>
        KEY_Meta_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Meta_R: 65512>
        KEY_Minus: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Minus: 45>
        KEY_Mode_switch: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Mode_switch: 65406>
        KEY_N: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_N: 110>
        KEY_Next: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Next: 65366>
        KEY_Num_Lock: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Num_Lock: 65407>
        KEY_O: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_O: 111>
        KEY_P: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_P: 112>
        KEY_Page_Down: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Next: 65366>
        KEY_Page_Up: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Prior: 65365>
        KEY_Pause: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Pause: 65299>
        KEY_Period: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Period: 46>
        KEY_Plus: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Plus: 43>
        KEY_Print: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Print: 65377>
        KEY_Prior: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Prior: 65365>
        KEY_Q: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Q: 113>
        KEY_Question: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Question: 63>
        KEY_Quote: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Quote: 39>
        KEY_Quotedbl: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Quotedbl: 34>
        KEY_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_R: 114>
        KEY_Redo: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Redo: 65382>
        KEY_Return: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Return: 65293>
        KEY_Right: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Right: 65363>
        KEY_Rightbracket: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Rightbracket: 93>
        KEY_Rightparen: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Rightparen: 41>
        KEY_S: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_S: 115>
        KEY_Script_switch: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Mode_switch: 65406>
        KEY_Scroll_Lock: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Scroll_Lock: 65300>
        KEY_Select: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Select: 65376>
        KEY_Semicolon: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Semicolon: 59>
        KEY_Shift_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Shift_L: 65505>
        KEY_Shift_Lock: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Shift_Lock: 65510>
        KEY_Shift_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Shift_R: 65506>
        KEY_Slash: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Slash: 47>
        KEY_Space: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Space: 32>
        KEY_Super_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Super_L: 65515>
        KEY_Super_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Super_R: 65516>
        KEY_Sys_Req: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Sys_Req: 65301>
        KEY_T: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_T: 116>
        KEY_Tab: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Tab: 65289>
        KEY_U: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_U: 117>
        KEY_Underscore: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Underscore: 95>
        KEY_Undo: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Undo: 65381>
        KEY_Up: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Up: 65362>
        KEY_V: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_V: 118>
        KEY_W: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_W: 119>
        KEY_X: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_X: 120>
        KEY_Y: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Y: 121>
        KEY_Z: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Z: 122>
        __members__: typing.ClassVar[dict[str, GUIEventAdapter.KeySymbol]]  # value = {'KEY_Space': <KeySymbol.KEY_Space: 32>, 'KEY_0': <KeySymbol.KEY_0: 48>, 'KEY_1': <KeySymbol.KEY_1: 49>, 'KEY_2': <KeySymbol.KEY_2: 50>, 'KEY_3': <KeySymbol.KEY_3: 51>, 'KEY_4': <KeySymbol.KEY_4: 52>, 'KEY_5': <KeySymbol.KEY_5: 53>, 'KEY_6': <KeySymbol.KEY_6: 54>, 'KEY_7': <KeySymbol.KEY_7: 55>, 'KEY_8': <KeySymbol.KEY_8: 56>, 'KEY_9': <KeySymbol.KEY_9: 57>, 'KEY_A': <KeySymbol.KEY_A: 97>, 'KEY_B': <KeySymbol.KEY_B: 98>, 'KEY_C': <KeySymbol.KEY_C: 99>, 'KEY_D': <KeySymbol.KEY_D: 100>, 'KEY_E': <KeySymbol.KEY_E: 101>, 'KEY_F': <KeySymbol.KEY_F: 102>, 'KEY_G': <KeySymbol.KEY_G: 103>, 'KEY_H': <KeySymbol.KEY_H: 104>, 'KEY_I': <KeySymbol.KEY_I: 105>, 'KEY_J': <KeySymbol.KEY_J: 106>, 'KEY_K': <KeySymbol.KEY_K: 107>, 'KEY_L': <KeySymbol.KEY_L: 108>, 'KEY_M': <KeySymbol.KEY_M: 109>, 'KEY_N': <KeySymbol.KEY_N: 110>, 'KEY_O': <KeySymbol.KEY_O: 111>, 'KEY_P': <KeySymbol.KEY_P: 112>, 'KEY_Q': <KeySymbol.KEY_Q: 113>, 'KEY_R': <KeySymbol.KEY_R: 114>, 'KEY_S': <KeySymbol.KEY_S: 115>, 'KEY_T': <KeySymbol.KEY_T: 116>, 'KEY_U': <KeySymbol.KEY_U: 117>, 'KEY_V': <KeySymbol.KEY_V: 118>, 'KEY_W': <KeySymbol.KEY_W: 119>, 'KEY_X': <KeySymbol.KEY_X: 120>, 'KEY_Y': <KeySymbol.KEY_Y: 121>, 'KEY_Z': <KeySymbol.KEY_Z: 122>, 'KEY_Exclaim': <KeySymbol.KEY_Exclaim: 33>, 'KEY_Quotedbl': <KeySymbol.KEY_Quotedbl: 34>, 'KEY_Hash': <KeySymbol.KEY_Hash: 35>, 'KEY_Dollar': <KeySymbol.KEY_Dollar: 36>, 'KEY_Ampersand': <KeySymbol.KEY_Ampersand: 38>, 'KEY_Quote': <KeySymbol.KEY_Quote: 39>, 'KEY_Leftparen': <KeySymbol.KEY_Leftparen: 40>, 'KEY_Rightparen': <KeySymbol.KEY_Rightparen: 41>, 'KEY_Asterisk': <KeySymbol.KEY_Asterisk: 42>, 'KEY_Plus': <KeySymbol.KEY_Plus: 43>, 'KEY_Comma': <KeySymbol.KEY_Comma: 44>, 'KEY_Minus': <KeySymbol.KEY_Minus: 45>, 'KEY_Period': <KeySymbol.KEY_Period: 46>, 'KEY_Slash': <KeySymbol.KEY_Slash: 47>, 'KEY_Colon': <KeySymbol.KEY_Colon: 58>, 'KEY_Semicolon': <KeySymbol.KEY_Semicolon: 59>, 'KEY_Less': <KeySymbol.KEY_Less: 60>, 'KEY_Equals': <KeySymbol.KEY_Equals: 61>, 'KEY_Greater': <KeySymbol.KEY_Greater: 62>, 'KEY_Question': <KeySymbol.KEY_Question: 63>, 'KEY_At': <KeySymbol.KEY_At: 64>, 'KEY_Leftbracket': <KeySymbol.KEY_Leftbracket: 91>, 'KEY_Backslash': <KeySymbol.KEY_Backslash: 92>, 'KEY_Rightbracket': <KeySymbol.KEY_Rightbracket: 93>, 'KEY_Caret': <KeySymbol.KEY_Caret: 94>, 'KEY_Underscore': <KeySymbol.KEY_Underscore: 95>, 'KEY_Backquote': <KeySymbol.KEY_Backquote: 96>, 'KEY_BackSpace': <KeySymbol.KEY_BackSpace: 65288>, 'KEY_Tab': <KeySymbol.KEY_Tab: 65289>, 'KEY_Linefeed': <KeySymbol.KEY_Linefeed: 65290>, 'KEY_Clear': <KeySymbol.KEY_Clear: 65291>, 'KEY_Return': <KeySymbol.KEY_Return: 65293>, 'KEY_Pause': <KeySymbol.KEY_Pause: 65299>, 'KEY_Scroll_Lock': <KeySymbol.KEY_Scroll_Lock: 65300>, 'KEY_Sys_Req': <KeySymbol.KEY_Sys_Req: 65301>, 'KEY_Escape': <KeySymbol.KEY_Escape: 65307>, 'KEY_Delete': <KeySymbol.KEY_Delete: 65535>, 'KEY_Home': <KeySymbol.KEY_Home: 65360>, 'KEY_Left': <KeySymbol.KEY_Left: 65361>, 'KEY_Up': <KeySymbol.KEY_Up: 65362>, 'KEY_Right': <KeySymbol.KEY_Right: 65363>, 'KEY_Down': <KeySymbol.KEY_Down: 65364>, 'KEY_Prior': <KeySymbol.KEY_Prior: 65365>, 'KEY_Page_Up': <KeySymbol.KEY_Prior: 65365>, 'KEY_Next': <KeySymbol.KEY_Next: 65366>, 'KEY_Page_Down': <KeySymbol.KEY_Next: 65366>, 'KEY_End': <KeySymbol.KEY_End: 65367>, 'KEY_Begin': <KeySymbol.KEY_Begin: 65368>, 'KEY_Select': <KeySymbol.KEY_Select: 65376>, 'KEY_Print': <KeySymbol.KEY_Print: 65377>, 'KEY_Execute': <KeySymbol.KEY_Execute: 65378>, 'KEY_Insert': <KeySymbol.KEY_Insert: 65379>, 'KEY_Undo': <KeySymbol.KEY_Undo: 65381>, 'KEY_Redo': <KeySymbol.KEY_Redo: 65382>, 'KEY_Menu': <KeySymbol.KEY_Menu: 65383>, 'KEY_Find': <KeySymbol.KEY_Find: 65384>, 'KEY_Cancel': <KeySymbol.KEY_Cancel: 65385>, 'KEY_Help': <KeySymbol.KEY_Help: 65386>, 'KEY_Break': <KeySymbol.KEY_Break: 65387>, 'KEY_Mode_switch': <KeySymbol.KEY_Mode_switch: 65406>, 'KEY_Script_switch': <KeySymbol.KEY_Mode_switch: 65406>, 'KEY_Num_Lock': <KeySymbol.KEY_Num_Lock: 65407>, 'KEY_KP_Space': <KeySymbol.KEY_KP_Space: 65408>, 'KEY_KP_Tab': <KeySymbol.KEY_KP_Tab: 65417>, 'KEY_KP_Enter': <KeySymbol.KEY_KP_Enter: 65421>, 'KEY_KP_F1': <KeySymbol.KEY_KP_F1: 65425>, 'KEY_KP_F2': <KeySymbol.KEY_KP_F2: 65426>, 'KEY_KP_F3': <KeySymbol.KEY_KP_F3: 65427>, 'KEY_KP_F4': <KeySymbol.KEY_KP_F4: 65428>, 'KEY_KP_Home': <KeySymbol.KEY_KP_Home: 65429>, 'KEY_KP_Left': <KeySymbol.KEY_KP_Left: 65430>, 'KEY_KP_Up': <KeySymbol.KEY_KP_Up: 65431>, 'KEY_KP_Right': <KeySymbol.KEY_KP_Right: 65432>, 'KEY_KP_Down': <KeySymbol.KEY_KP_Down: 65433>, 'KEY_KP_Prior': <KeySymbol.KEY_KP_Prior: 65434>, 'KEY_KP_Page_Up': <KeySymbol.KEY_KP_Prior: 65434>, 'KEY_KP_Next': <KeySymbol.KEY_KP_Next: 65435>, 'KEY_KP_Page_Down': <KeySymbol.KEY_KP_Next: 65435>, 'KEY_KP_End': <KeySymbol.KEY_KP_End: 65436>, 'KEY_KP_Begin': <KeySymbol.KEY_KP_Begin: 65437>, 'KEY_KP_Insert': <KeySymbol.KEY_KP_Insert: 65438>, 'KEY_KP_Delete': <KeySymbol.KEY_KP_Delete: 65439>, 'KEY_KP_Equal': <KeySymbol.KEY_KP_Equal: 65469>, 'KEY_KP_Multiply': <KeySymbol.KEY_KP_Multiply: 65450>, 'KEY_KP_Add': <KeySymbol.KEY_KP_Add: 65451>, 'KEY_KP_Separator': <KeySymbol.KEY_KP_Separator: 65452>, 'KEY_KP_Subtract': <KeySymbol.KEY_KP_Subtract: 65453>, 'KEY_KP_Decimal': <KeySymbol.KEY_KP_Decimal: 65454>, 'KEY_KP_Divide': <KeySymbol.KEY_KP_Divide: 65455>, 'KEY_KP_0': <KeySymbol.KEY_KP_0: 65456>, 'KEY_KP_1': <KeySymbol.KEY_KP_1: 65457>, 'KEY_KP_2': <KeySymbol.KEY_KP_2: 65458>, 'KEY_KP_3': <KeySymbol.KEY_KP_3: 65459>, 'KEY_KP_4': <KeySymbol.KEY_KP_4: 65460>, 'KEY_KP_5': <KeySymbol.KEY_KP_5: 65461>, 'KEY_KP_6': <KeySymbol.KEY_KP_6: 65462>, 'KEY_KP_7': <KeySymbol.KEY_KP_7: 65463>, 'KEY_KP_8': <KeySymbol.KEY_KP_8: 65464>, 'KEY_KP_9': <KeySymbol.KEY_KP_9: 65465>, 'KEY_F1': <KeySymbol.KEY_F1: 65470>, 'KEY_F2': <KeySymbol.KEY_F2: 65471>, 'KEY_F3': <KeySymbol.KEY_F3: 65472>, 'KEY_F4': <KeySymbol.KEY_F4: 65473>, 'KEY_F5': <KeySymbol.KEY_F5: 65474>, 'KEY_F6': <KeySymbol.KEY_F6: 65475>, 'KEY_F7': <KeySymbol.KEY_F7: 65476>, 'KEY_F8': <KeySymbol.KEY_F8: 65477>, 'KEY_F9': <KeySymbol.KEY_F9: 65478>, 'KEY_F10': <KeySymbol.KEY_F10: 65479>, 'KEY_F11': <KeySymbol.KEY_F11: 65480>, 'KEY_F12': <KeySymbol.KEY_F12: 65481>, 'KEY_F13': <KeySymbol.KEY_F13: 65482>, 'KEY_F14': <KeySymbol.KEY_F14: 65483>, 'KEY_F15': <KeySymbol.KEY_F15: 65484>, 'KEY_F16': <KeySymbol.KEY_F16: 65485>, 'KEY_F17': <KeySymbol.KEY_F17: 65486>, 'KEY_F18': <KeySymbol.KEY_F18: 65487>, 'KEY_F19': <KeySymbol.KEY_F19: 65488>, 'KEY_F20': <KeySymbol.KEY_F20: 65489>, 'KEY_F21': <KeySymbol.KEY_F21: 65490>, 'KEY_F22': <KeySymbol.KEY_F22: 65491>, 'KEY_F23': <KeySymbol.KEY_F23: 65492>, 'KEY_F24': <KeySymbol.KEY_F24: 65493>, 'KEY_F25': <KeySymbol.KEY_F25: 65494>, 'KEY_F26': <KeySymbol.KEY_F26: 65495>, 'KEY_F27': <KeySymbol.KEY_F27: 65496>, 'KEY_F28': <KeySymbol.KEY_F28: 65497>, 'KEY_F29': <KeySymbol.KEY_F29: 65498>, 'KEY_F30': <KeySymbol.KEY_F30: 65499>, 'KEY_F31': <KeySymbol.KEY_F31: 65500>, 'KEY_F32': <KeySymbol.KEY_F32: 65501>, 'KEY_F33': <KeySymbol.KEY_F33: 65502>, 'KEY_F34': <KeySymbol.KEY_F34: 65503>, 'KEY_F35': <KeySymbol.KEY_F35: 65504>, 'KEY_Shift_L': <KeySymbol.KEY_Shift_L: 65505>, 'KEY_Shift_R': <KeySymbol.KEY_Shift_R: 65506>, 'KEY_Control_L': <KeySymbol.KEY_Control_L: 65507>, 'KEY_Control_R': <KeySymbol.KEY_Control_R: 65508>, 'KEY_Caps_Lock': <KeySymbol.KEY_Caps_Lock: 65509>, 'KEY_Shift_Lock': <KeySymbol.KEY_Shift_Lock: 65510>, 'KEY_Meta_L': <KeySymbol.KEY_Meta_L: 65511>, 'KEY_Meta_R': <KeySymbol.KEY_Meta_R: 65512>, 'KEY_Alt_L': <KeySymbol.KEY_Alt_L: 65513>, 'KEY_Alt_R': <KeySymbol.KEY_Alt_R: 65514>, 'KEY_Super_L': <KeySymbol.KEY_Super_L: 65515>, 'KEY_Super_R': <KeySymbol.KEY_Super_R: 65516>, 'KEY_Hyper_L': <KeySymbol.KEY_Hyper_L: 65517>, 'KEY_Hyper_R': <KeySymbol.KEY_Hyper_R: 65518>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class ModKeyMask:
        """
        Members:
        
          MODKEY_LEFT_SHIFT
        
          MODKEY_RIGHT_SHIFT
        
          MODKEY_LEFT_CTRL
        
          MODKEY_RIGHT_CTRL
        
          MODKEY_LEFT_ALT
        
          MODKEY_RIGHT_ALT
        
          MODKEY_LEFT_META
        
          MODKEY_RIGHT_META
        
          MODKEY_LEFT_SUPER
        
          MODKEY_RIGHT_SUPER
        
          MODKEY_LEFT_HYPER
        
          MODKEY_RIGHT_HYPER
        
          MODKEY_NUM_LOCK
        
          MODKEY_CAPS_LOCK
        
          MODKEY_CTRL
        
          MODKEY_SHIFT
        
          MODKEY_ALT
        
          MODKEY_META
        
          MODKEY_SUPER
        
          MODKEY_HYPER
        """
        MODKEY_ALT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_ALT: 48>
        MODKEY_CAPS_LOCK: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_CAPS_LOCK: 8192>
        MODKEY_CTRL: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_CTRL: 12>
        MODKEY_HYPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_HYPER: 3072>
        MODKEY_LEFT_ALT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_ALT: 16>
        MODKEY_LEFT_CTRL: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_CTRL: 4>
        MODKEY_LEFT_HYPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_HYPER: 1024>
        MODKEY_LEFT_META: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_META: 64>
        MODKEY_LEFT_SHIFT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_SHIFT: 1>
        MODKEY_LEFT_SUPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_SUPER: 256>
        MODKEY_META: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_META: 192>
        MODKEY_NUM_LOCK: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_NUM_LOCK: 4096>
        MODKEY_RIGHT_ALT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_ALT: 32>
        MODKEY_RIGHT_CTRL: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_CTRL: 8>
        MODKEY_RIGHT_HYPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_HYPER: 2048>
        MODKEY_RIGHT_META: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_META: 128>
        MODKEY_RIGHT_SHIFT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_SHIFT: 2>
        MODKEY_RIGHT_SUPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_SUPER: 512>
        MODKEY_SHIFT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_SHIFT: 3>
        MODKEY_SUPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_SUPER: 768>
        __members__: typing.ClassVar[dict[str, GUIEventAdapter.ModKeyMask]]  # value = {'MODKEY_LEFT_SHIFT': <ModKeyMask.MODKEY_LEFT_SHIFT: 1>, 'MODKEY_RIGHT_SHIFT': <ModKeyMask.MODKEY_RIGHT_SHIFT: 2>, 'MODKEY_LEFT_CTRL': <ModKeyMask.MODKEY_LEFT_CTRL: 4>, 'MODKEY_RIGHT_CTRL': <ModKeyMask.MODKEY_RIGHT_CTRL: 8>, 'MODKEY_LEFT_ALT': <ModKeyMask.MODKEY_LEFT_ALT: 16>, 'MODKEY_RIGHT_ALT': <ModKeyMask.MODKEY_RIGHT_ALT: 32>, 'MODKEY_LEFT_META': <ModKeyMask.MODKEY_LEFT_META: 64>, 'MODKEY_RIGHT_META': <ModKeyMask.MODKEY_RIGHT_META: 128>, 'MODKEY_LEFT_SUPER': <ModKeyMask.MODKEY_LEFT_SUPER: 256>, 'MODKEY_RIGHT_SUPER': <ModKeyMask.MODKEY_RIGHT_SUPER: 512>, 'MODKEY_LEFT_HYPER': <ModKeyMask.MODKEY_LEFT_HYPER: 1024>, 'MODKEY_RIGHT_HYPER': <ModKeyMask.MODKEY_RIGHT_HYPER: 2048>, 'MODKEY_NUM_LOCK': <ModKeyMask.MODKEY_NUM_LOCK: 4096>, 'MODKEY_CAPS_LOCK': <ModKeyMask.MODKEY_CAPS_LOCK: 8192>, 'MODKEY_CTRL': <ModKeyMask.MODKEY_CTRL: 12>, 'MODKEY_SHIFT': <ModKeyMask.MODKEY_SHIFT: 3>, 'MODKEY_ALT': <ModKeyMask.MODKEY_ALT: 48>, 'MODKEY_META': <ModKeyMask.MODKEY_META: 192>, 'MODKEY_SUPER': <ModKeyMask.MODKEY_SUPER: 768>, 'MODKEY_HYPER': <ModKeyMask.MODKEY_HYPER: 3072>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class MouseButtonMask:
        """
        Members:
        
          LEFT_MOUSE_BUTTON
        
          MIDDLE_MOUSE_BUTTON
        
          RIGHT_MOUSE_BUTTON
        """
        LEFT_MOUSE_BUTTON: typing.ClassVar[GUIEventAdapter.MouseButtonMask]  # value = <MouseButtonMask.LEFT_MOUSE_BUTTON: 1>
        MIDDLE_MOUSE_BUTTON: typing.ClassVar[GUIEventAdapter.MouseButtonMask]  # value = <MouseButtonMask.MIDDLE_MOUSE_BUTTON: 2>
        RIGHT_MOUSE_BUTTON: typing.ClassVar[GUIEventAdapter.MouseButtonMask]  # value = <MouseButtonMask.RIGHT_MOUSE_BUTTON: 4>
        __members__: typing.ClassVar[dict[str, GUIEventAdapter.MouseButtonMask]]  # value = {'LEFT_MOUSE_BUTTON': <MouseButtonMask.LEFT_MOUSE_BUTTON: 1>, 'MIDDLE_MOUSE_BUTTON': <MouseButtonMask.MIDDLE_MOUSE_BUTTON: 2>, 'RIGHT_MOUSE_BUTTON': <MouseButtonMask.RIGHT_MOUSE_BUTTON: 4>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class MouseYOrientation:
        """
        Members:
        
          Y_INCREASING_UPWARDS
        
          Y_INCREASING_DOWNWARDS
        """
        Y_INCREASING_DOWNWARDS: typing.ClassVar[GUIEventAdapter.MouseYOrientation]  # value = <MouseYOrientation.Y_INCREASING_DOWNWARDS: 1>
        Y_INCREASING_UPWARDS: typing.ClassVar[GUIEventAdapter.MouseYOrientation]  # value = <MouseYOrientation.Y_INCREASING_UPWARDS: 0>
        __members__: typing.ClassVar[dict[str, GUIEventAdapter.MouseYOrientation]]  # value = {'Y_INCREASING_UPWARDS': <MouseYOrientation.Y_INCREASING_UPWARDS: 0>, 'Y_INCREASING_DOWNWARDS': <MouseYOrientation.Y_INCREASING_DOWNWARDS: 1>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class ScrollingMotion:
        """
        Members:
        
          SCROLL_NONE
        
          SCROLL_LEFT
        
          SCROLL_RIGHT
        
          SCROLL_UP
        
          SCROLL_DOWN
        
          SCROLL_2D
        """
        SCROLL_2D: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_2D: 5>
        SCROLL_DOWN: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_DOWN: 4>
        SCROLL_LEFT: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_LEFT: 1>
        SCROLL_NONE: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_NONE: 0>
        SCROLL_RIGHT: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_RIGHT: 2>
        SCROLL_UP: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_UP: 3>
        __members__: typing.ClassVar[dict[str, GUIEventAdapter.ScrollingMotion]]  # value = {'SCROLL_NONE': <ScrollingMotion.SCROLL_NONE: 0>, 'SCROLL_LEFT': <ScrollingMotion.SCROLL_LEFT: 1>, 'SCROLL_RIGHT': <ScrollingMotion.SCROLL_RIGHT: 2>, 'SCROLL_UP': <ScrollingMotion.SCROLL_UP: 3>, 'SCROLL_DOWN': <ScrollingMotion.SCROLL_DOWN: 4>, 'SCROLL_2D': <ScrollingMotion.SCROLL_2D: 5>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class TabletPointerType:
        """
        Members:
        
          UNKNOWN
        
          PEN
        
          PUCK
        
          ERASER
        """
        ERASER: typing.ClassVar[GUIEventAdapter.TabletPointerType]  # value = <TabletPointerType.ERASER: 3>
        PEN: typing.ClassVar[GUIEventAdapter.TabletPointerType]  # value = <TabletPointerType.PEN: 1>
        PUCK: typing.ClassVar[GUIEventAdapter.TabletPointerType]  # value = <TabletPointerType.PUCK: 2>
        UNKNOWN: typing.ClassVar[GUIEventAdapter.TabletPointerType]  # value = <TabletPointerType.UNKNOWN: 0>
        __members__: typing.ClassVar[dict[str, GUIEventAdapter.TabletPointerType]]  # value = {'UNKNOWN': <TabletPointerType.UNKNOWN: 0>, 'PEN': <TabletPointerType.PEN: 1>, 'PUCK': <TabletPointerType.PUCK: 2>, 'ERASER': <TabletPointerType.ERASER: 3>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    class TouchPhase:
        """
        Members:
        
          TOUCH_UNKNOWN
        
          TOUCH_BEGAN
        
          TOUCH_MOVED
        
          TOUCH_STATIONERY
        
          TOUCH_ENDED
        """
        TOUCH_BEGAN: typing.ClassVar[GUIEventAdapter.TouchPhase]  # value = <TouchPhase.TOUCH_BEGAN: 1>
        TOUCH_ENDED: typing.ClassVar[GUIEventAdapter.TouchPhase]  # value = <TouchPhase.TOUCH_ENDED: 4>
        TOUCH_MOVED: typing.ClassVar[GUIEventAdapter.TouchPhase]  # value = <TouchPhase.TOUCH_MOVED: 2>
        TOUCH_STATIONERY: typing.ClassVar[GUIEventAdapter.TouchPhase]  # value = <TouchPhase.TOUCH_STATIONERY: 3>
        TOUCH_UNKNOWN: typing.ClassVar[GUIEventAdapter.TouchPhase]  # value = <TouchPhase.TOUCH_UNKNOWN: 0>
        __members__: typing.ClassVar[dict[str, GUIEventAdapter.TouchPhase]]  # value = {'TOUCH_UNKNOWN': <TouchPhase.TOUCH_UNKNOWN: 0>, 'TOUCH_BEGAN': <TouchPhase.TOUCH_BEGAN: 1>, 'TOUCH_MOVED': <TouchPhase.TOUCH_MOVED: 2>, 'TOUCH_STATIONERY': <TouchPhase.TOUCH_STATIONERY: 3>, 'TOUCH_ENDED': <TouchPhase.TOUCH_ENDED: 4>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    CLOSE_WINDOW: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.CLOSE_WINDOW: 16384>
    DOUBLECLICK: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.DOUBLECLICK: 4>
    DRAG: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.DRAG: 8>
    ERASER: typing.ClassVar[GUIEventAdapter.TabletPointerType]  # value = <TabletPointerType.ERASER: 3>
    FRAME: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.FRAME: 128>
    KEYDOWN: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.KEYDOWN: 32>
    KEYUP: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.KEYUP: 64>
    KEY_0: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_0: 48>
    KEY_1: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_1: 49>
    KEY_2: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_2: 50>
    KEY_3: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_3: 51>
    KEY_4: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_4: 52>
    KEY_5: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_5: 53>
    KEY_6: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_6: 54>
    KEY_7: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_7: 55>
    KEY_8: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_8: 56>
    KEY_9: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_9: 57>
    KEY_A: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_A: 97>
    KEY_Alt_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Alt_L: 65513>
    KEY_Alt_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Alt_R: 65514>
    KEY_Ampersand: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Ampersand: 38>
    KEY_Asterisk: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Asterisk: 42>
    KEY_At: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_At: 64>
    KEY_B: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_B: 98>
    KEY_BackSpace: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_BackSpace: 65288>
    KEY_Backquote: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Backquote: 96>
    KEY_Backslash: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Backslash: 92>
    KEY_Begin: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Begin: 65368>
    KEY_Break: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Break: 65387>
    KEY_C: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_C: 99>
    KEY_Cancel: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Cancel: 65385>
    KEY_Caps_Lock: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Caps_Lock: 65509>
    KEY_Caret: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Caret: 94>
    KEY_Clear: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Clear: 65291>
    KEY_Colon: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Colon: 58>
    KEY_Comma: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Comma: 44>
    KEY_Control_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Control_L: 65507>
    KEY_Control_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Control_R: 65508>
    KEY_D: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_D: 100>
    KEY_Delete: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Delete: 65535>
    KEY_Dollar: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Dollar: 36>
    KEY_Down: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Down: 65364>
    KEY_E: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_E: 101>
    KEY_End: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_End: 65367>
    KEY_Equals: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Equals: 61>
    KEY_Escape: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Escape: 65307>
    KEY_Exclaim: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Exclaim: 33>
    KEY_Execute: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Execute: 65378>
    KEY_F: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F: 102>
    KEY_F1: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F1: 65470>
    KEY_F10: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F10: 65479>
    KEY_F11: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F11: 65480>
    KEY_F12: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F12: 65481>
    KEY_F13: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F13: 65482>
    KEY_F14: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F14: 65483>
    KEY_F15: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F15: 65484>
    KEY_F16: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F16: 65485>
    KEY_F17: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F17: 65486>
    KEY_F18: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F18: 65487>
    KEY_F19: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F19: 65488>
    KEY_F2: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F2: 65471>
    KEY_F20: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F20: 65489>
    KEY_F21: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F21: 65490>
    KEY_F22: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F22: 65491>
    KEY_F23: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F23: 65492>
    KEY_F24: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F24: 65493>
    KEY_F25: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F25: 65494>
    KEY_F26: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F26: 65495>
    KEY_F27: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F27: 65496>
    KEY_F28: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F28: 65497>
    KEY_F29: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F29: 65498>
    KEY_F3: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F3: 65472>
    KEY_F30: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F30: 65499>
    KEY_F31: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F31: 65500>
    KEY_F32: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F32: 65501>
    KEY_F33: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F33: 65502>
    KEY_F34: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F34: 65503>
    KEY_F35: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F35: 65504>
    KEY_F4: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F4: 65473>
    KEY_F5: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F5: 65474>
    KEY_F6: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F6: 65475>
    KEY_F7: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F7: 65476>
    KEY_F8: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F8: 65477>
    KEY_F9: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_F9: 65478>
    KEY_Find: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Find: 65384>
    KEY_G: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_G: 103>
    KEY_Greater: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Greater: 62>
    KEY_H: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_H: 104>
    KEY_Hash: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Hash: 35>
    KEY_Help: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Help: 65386>
    KEY_Home: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Home: 65360>
    KEY_Hyper_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Hyper_L: 65517>
    KEY_Hyper_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Hyper_R: 65518>
    KEY_I: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_I: 105>
    KEY_Insert: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Insert: 65379>
    KEY_J: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_J: 106>
    KEY_K: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_K: 107>
    KEY_KP_0: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_0: 65456>
    KEY_KP_1: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_1: 65457>
    KEY_KP_2: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_2: 65458>
    KEY_KP_3: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_3: 65459>
    KEY_KP_4: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_4: 65460>
    KEY_KP_5: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_5: 65461>
    KEY_KP_6: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_6: 65462>
    KEY_KP_7: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_7: 65463>
    KEY_KP_8: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_8: 65464>
    KEY_KP_9: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_9: 65465>
    KEY_KP_Add: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Add: 65451>
    KEY_KP_Begin: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Begin: 65437>
    KEY_KP_Decimal: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Decimal: 65454>
    KEY_KP_Delete: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Delete: 65439>
    KEY_KP_Divide: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Divide: 65455>
    KEY_KP_Down: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Down: 65433>
    KEY_KP_End: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_End: 65436>
    KEY_KP_Enter: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Enter: 65421>
    KEY_KP_Equal: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Equal: 65469>
    KEY_KP_F1: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_F1: 65425>
    KEY_KP_F2: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_F2: 65426>
    KEY_KP_F3: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_F3: 65427>
    KEY_KP_F4: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_F4: 65428>
    KEY_KP_Home: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Home: 65429>
    KEY_KP_Insert: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Insert: 65438>
    KEY_KP_Left: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Left: 65430>
    KEY_KP_Multiply: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Multiply: 65450>
    KEY_KP_Next: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Next: 65435>
    KEY_KP_Page_Down: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Next: 65435>
    KEY_KP_Page_Up: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Prior: 65434>
    KEY_KP_Prior: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Prior: 65434>
    KEY_KP_Right: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Right: 65432>
    KEY_KP_Separator: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Separator: 65452>
    KEY_KP_Space: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Space: 65408>
    KEY_KP_Subtract: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Subtract: 65453>
    KEY_KP_Tab: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Tab: 65417>
    KEY_KP_Up: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_KP_Up: 65431>
    KEY_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_L: 108>
    KEY_Left: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Left: 65361>
    KEY_Leftbracket: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Leftbracket: 91>
    KEY_Leftparen: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Leftparen: 40>
    KEY_Less: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Less: 60>
    KEY_Linefeed: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Linefeed: 65290>
    KEY_M: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_M: 109>
    KEY_Menu: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Menu: 65383>
    KEY_Meta_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Meta_L: 65511>
    KEY_Meta_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Meta_R: 65512>
    KEY_Minus: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Minus: 45>
    KEY_Mode_switch: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Mode_switch: 65406>
    KEY_N: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_N: 110>
    KEY_Next: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Next: 65366>
    KEY_Num_Lock: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Num_Lock: 65407>
    KEY_O: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_O: 111>
    KEY_P: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_P: 112>
    KEY_Page_Down: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Next: 65366>
    KEY_Page_Up: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Prior: 65365>
    KEY_Pause: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Pause: 65299>
    KEY_Period: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Period: 46>
    KEY_Plus: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Plus: 43>
    KEY_Print: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Print: 65377>
    KEY_Prior: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Prior: 65365>
    KEY_Q: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Q: 113>
    KEY_Question: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Question: 63>
    KEY_Quote: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Quote: 39>
    KEY_Quotedbl: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Quotedbl: 34>
    KEY_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_R: 114>
    KEY_Redo: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Redo: 65382>
    KEY_Return: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Return: 65293>
    KEY_Right: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Right: 65363>
    KEY_Rightbracket: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Rightbracket: 93>
    KEY_Rightparen: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Rightparen: 41>
    KEY_S: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_S: 115>
    KEY_Script_switch: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Mode_switch: 65406>
    KEY_Scroll_Lock: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Scroll_Lock: 65300>
    KEY_Select: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Select: 65376>
    KEY_Semicolon: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Semicolon: 59>
    KEY_Shift_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Shift_L: 65505>
    KEY_Shift_Lock: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Shift_Lock: 65510>
    KEY_Shift_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Shift_R: 65506>
    KEY_Slash: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Slash: 47>
    KEY_Space: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Space: 32>
    KEY_Super_L: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Super_L: 65515>
    KEY_Super_R: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Super_R: 65516>
    KEY_Sys_Req: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Sys_Req: 65301>
    KEY_T: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_T: 116>
    KEY_Tab: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Tab: 65289>
    KEY_U: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_U: 117>
    KEY_Underscore: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Underscore: 95>
    KEY_Undo: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Undo: 65381>
    KEY_Up: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Up: 65362>
    KEY_V: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_V: 118>
    KEY_W: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_W: 119>
    KEY_X: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_X: 120>
    KEY_Y: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Y: 121>
    KEY_Z: typing.ClassVar[GUIEventAdapter.KeySymbol]  # value = <KeySymbol.KEY_Z: 122>
    LEFT_MOUSE_BUTTON: typing.ClassVar[GUIEventAdapter.MouseButtonMask]  # value = <MouseButtonMask.LEFT_MOUSE_BUTTON: 1>
    MIDDLE_MOUSE_BUTTON: typing.ClassVar[GUIEventAdapter.MouseButtonMask]  # value = <MouseButtonMask.MIDDLE_MOUSE_BUTTON: 2>
    MODKEY_ALT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_ALT: 48>
    MODKEY_CAPS_LOCK: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_CAPS_LOCK: 8192>
    MODKEY_CTRL: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_CTRL: 12>
    MODKEY_HYPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_HYPER: 3072>
    MODKEY_LEFT_ALT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_ALT: 16>
    MODKEY_LEFT_CTRL: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_CTRL: 4>
    MODKEY_LEFT_HYPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_HYPER: 1024>
    MODKEY_LEFT_META: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_META: 64>
    MODKEY_LEFT_SHIFT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_SHIFT: 1>
    MODKEY_LEFT_SUPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_LEFT_SUPER: 256>
    MODKEY_META: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_META: 192>
    MODKEY_NUM_LOCK: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_NUM_LOCK: 4096>
    MODKEY_RIGHT_ALT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_ALT: 32>
    MODKEY_RIGHT_CTRL: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_CTRL: 8>
    MODKEY_RIGHT_HYPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_HYPER: 2048>
    MODKEY_RIGHT_META: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_META: 128>
    MODKEY_RIGHT_SHIFT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_SHIFT: 2>
    MODKEY_RIGHT_SUPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_RIGHT_SUPER: 512>
    MODKEY_SHIFT: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_SHIFT: 3>
    MODKEY_SUPER: typing.ClassVar[GUIEventAdapter.ModKeyMask]  # value = <ModKeyMask.MODKEY_SUPER: 768>
    MOVE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.MOVE: 16>
    NONE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.NONE: 0>
    PEN: typing.ClassVar[GUIEventAdapter.TabletPointerType]  # value = <TabletPointerType.PEN: 1>
    PEN_ORIENTATION: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.PEN_ORIENTATION: 2048>
    PEN_PRESSURE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.PEN_PRESSURE: 1024>
    PEN_PROXIMITY_ENTER: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.PEN_PROXIMITY_ENTER: 4096>
    PEN_PROXIMITY_LEAVE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.PEN_PROXIMITY_LEAVE: 8192>
    PUCK: typing.ClassVar[GUIEventAdapter.TabletPointerType]  # value = <TabletPointerType.PUCK: 2>
    PUSH: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.PUSH: 1>
    QUIT_APPLICATION: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.QUIT_APPLICATION: 32768>
    RELEASE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.RELEASE: 2>
    RESIZE: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.RESIZE: 256>
    RIGHT_MOUSE_BUTTON: typing.ClassVar[GUIEventAdapter.MouseButtonMask]  # value = <MouseButtonMask.RIGHT_MOUSE_BUTTON: 4>
    SCROLL: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.SCROLL: 512>
    SCROLL_2D: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_2D: 5>
    SCROLL_DOWN: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_DOWN: 4>
    SCROLL_LEFT: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_LEFT: 1>
    SCROLL_NONE: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_NONE: 0>
    SCROLL_RIGHT: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_RIGHT: 2>
    SCROLL_UP: typing.ClassVar[GUIEventAdapter.ScrollingMotion]  # value = <ScrollingMotion.SCROLL_UP: 3>
    TOUCH_BEGAN: typing.ClassVar[GUIEventAdapter.TouchPhase]  # value = <TouchPhase.TOUCH_BEGAN: 1>
    TOUCH_ENDED: typing.ClassVar[GUIEventAdapter.TouchPhase]  # value = <TouchPhase.TOUCH_ENDED: 4>
    TOUCH_MOVED: typing.ClassVar[GUIEventAdapter.TouchPhase]  # value = <TouchPhase.TOUCH_MOVED: 2>
    TOUCH_STATIONERY: typing.ClassVar[GUIEventAdapter.TouchPhase]  # value = <TouchPhase.TOUCH_STATIONERY: 3>
    TOUCH_UNKNOWN: typing.ClassVar[GUIEventAdapter.TouchPhase]  # value = <TouchPhase.TOUCH_UNKNOWN: 0>
    UNKNOWN: typing.ClassVar[GUIEventAdapter.TabletPointerType]  # value = <TabletPointerType.UNKNOWN: 0>
    USER: typing.ClassVar[GUIEventAdapter.EventType]  # value = <EventType.USER: 65536>
    Y_INCREASING_DOWNWARDS: typing.ClassVar[GUIEventAdapter.MouseYOrientation]  # value = <MouseYOrientation.Y_INCREASING_DOWNWARDS: 1>
    Y_INCREASING_UPWARDS: typing.ClassVar[GUIEventAdapter.MouseYOrientation]  # value = <MouseYOrientation.Y_INCREASING_UPWARDS: 0>
    def __init__(self) -> None:
        ...
    def getEventType(self) -> ...:
        ...
    def getKey(self) -> int:
        ...
class GUIEventHandler(__GUIEventHandler__):
    def __init__(self) -> None:
        ...
class GridVisual(ViewerAttachment):
    class PlaneType:
        """
        Members:
        
          XY
        
          YZ
        
          ZX
        """
        XY: typing.ClassVar[GridVisual.PlaneType]  # value = <PlaneType.XY: 0>
        YZ: typing.ClassVar[GridVisual.PlaneType]  # value = <PlaneType.YZ: 1>
        ZX: typing.ClassVar[GridVisual.PlaneType]  # value = <PlaneType.ZX: 2>
        __members__: typing.ClassVar[dict[str, GridVisual.PlaneType]]  # value = {'XY': <PlaneType.XY: 0>, 'YZ': <PlaneType.YZ: 1>, 'ZX': <PlaneType.ZX: 2>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    def __init__(self) -> None:
        ...
    def setMinorLineStepSize(self, arg0: float) -> None:
        ...
    def setNumCells(self, arg0: int) -> None:
        ...
    def setNumMinorLinesPerMajorLine(self, arg0: int) -> None:
        ...
    def setOffset(self, arg0: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setPlaneType(self, arg0: ...) -> None:
        ...
class PolyhedronVisual(ViewerAttachment):
    def __init__(self) -> None:
        ...
    def clear(self) -> None:
        ...
    def display(self, arg0: bool) -> None:
        ...
    def displaySurface(self, arg0: bool) -> None:
        ...
    def displayWireframe(self, arg0: bool) -> None:
        ...
    def getSurfaceColor(self) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def getVertices(self) -> list[numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]]:
        ...
    def getWireframeColor(self) -> numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]]:
        ...
    def getWireframeWidth(self) -> float:
        ...
    def isDisplayed(self) -> bool:
        ...
    def isSurfaceDisplayed(self) -> bool:
        ...
    def isWireframeDisplayed(self) -> bool:
        ...
    def setSurfaceColor(self, arg0: numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setVertices(self, arg0: list[numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]]) -> None:
        ...
    def setVerticesMatrix(self, arg0: numpy.ndarray[typing.Any, numpy.dtype[numpy.float64]]) -> None:
        ...
    def setWireframeColor(self, arg0: numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setWireframeWidth(self, arg0: float) -> None:
        ...
class ImGuiHandler:
    def __init__(self) -> None:
        ...
    @typing.overload
    def addWidget(self, widget: ImGuiWidget) -> None:
        ...
    @typing.overload
    def addWidget(self, widget: ImGuiWidget, visible: bool) -> None:
        ...
    def handle(self, eventAdapter: GUIEventAdapter, actionAdapter: GUIActionAdapter, object: ..., nodeVisitor: ...) -> bool:
        ...
    def hasWidget(self, widget: ImGuiWidget) -> bool:
        ...
    def newFrame(self, renderInfo: ...) -> None:
        ...
    def removeAllWidget(self) -> None:
        ...
    def removeWidget(self, widget: ImGuiWidget) -> None:
        ...
    def render(self, renderInfo: ...) -> None:
        ...
    def setCameraCallbacks(self, camera: ...) -> None:
        ...
class ImGuiViewer(Viewer):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, clearColor: numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    @typing.overload
    def __init__(self, clearColor: ...) -> None:
        ...
    def getImGuiHandler(self) -> ImGuiHandler:
        ...
    def hideAbout(self) -> None:
        ...
    def showAbout(self) -> None:
        ...
class ImGuiWidget:
    def __init__(self) -> None:
        ...
    def hide(self) -> None:
        ...
    def isVisible(self) -> bool:
        ...
    def render(self) -> None:
        ...
    def setVisible(self, visible: bool) -> None:
        ...
    def show(self) -> None:
        ...
    def toggleVisible(self) -> None:
        ...
class InteractiveFrame(dartpy.dynamics.SimpleFrame):
    @typing.overload
    def __init__(self, referenceFrame: dartpy.dynamics.Frame) -> None:
        ...
    @typing.overload
    def __init__(self, referenceFrame: dartpy.dynamics.Frame, name: str) -> None:
        ...
    @typing.overload
    def __init__(self, referenceFrame: dartpy.dynamics.Frame, name: str, relativeTransform: dartpy.math.Isometry3) -> None:
        ...
    @typing.overload
    def __init__(self, referenceFrame: dartpy.dynamics.Frame, name: str, relativeTransform: dartpy.math.Isometry3, sizeScale: float) -> None:
        ...
    @typing.overload
    def __init__(self, referenceFrame: dartpy.dynamics.Frame, name: str, relativeTransform: dartpy.math.Isometry3, sizeScale: float, thicknessScale: float) -> None:
        ...
    @typing.overload
    def getShapeFrames(self) -> ...:
        ...
    @typing.overload
    def getShapeFrames(self) -> ...:
        ...
    def removeAllShapeFrames(self) -> None:
        ...
    @typing.overload
    def resizeStandardVisuals(self) -> None:
        ...
    @typing.overload
    def resizeStandardVisuals(self, sizeScale: float) -> None:
        ...
    @typing.overload
    def resizeStandardVisuals(self, sizeScale: float, thicknessScale: float) -> None:
        ...
class InteractiveFrameDnD(DragAndDrop):
    def __init__(self, viewer: Viewer, frame: InteractiveFrame) -> None:
        ...
    def move(self) -> None:
        ...
    def saveState(self) -> None:
        ...
    def update(self) -> None:
        ...
class InteractiveTool(dartpy.dynamics.SimpleFrame):
    class Type:
        """
        Members:
        
          LINEAR
        
          ANGULAR
        
          PLANAR
        
          NUM_TYPES
        """
        ANGULAR: typing.ClassVar[InteractiveTool.Type]  # value = <Type.ANGULAR: 1>
        LINEAR: typing.ClassVar[InteractiveTool.Type]  # value = <Type.LINEAR: 0>
        NUM_TYPES: typing.ClassVar[InteractiveTool.Type]  # value = <Type.NUM_TYPES: 3>
        PLANAR: typing.ClassVar[InteractiveTool.Type]  # value = <Type.PLANAR: 2>
        __members__: typing.ClassVar[dict[str, InteractiveTool.Type]]  # value = {'LINEAR': <Type.LINEAR: 0>, 'ANGULAR': <Type.ANGULAR: 1>, 'PLANAR': <Type.PLANAR: 2>, 'NUM_TYPES': <Type.NUM_TYPES: 3>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    ANGULAR: typing.ClassVar[InteractiveTool.Type]  # value = <Type.ANGULAR: 1>
    LINEAR: typing.ClassVar[InteractiveTool.Type]  # value = <Type.LINEAR: 0>
    NUM_TYPES: typing.ClassVar[InteractiveTool.Type]  # value = <Type.NUM_TYPES: 3>
    PLANAR: typing.ClassVar[InteractiveTool.Type]  # value = <Type.PLANAR: 2>
    def __init__(self, frame: ..., defaultAlpha: float, name: str) -> None:
        ...
    def getDefaultAlpha(self) -> float:
        ...
    def getEnabled(self) -> bool:
        ...
    @typing.overload
    def getShapeFrames(self) -> ...:
        ...
    @typing.overload
    def getShapeFrames(self) -> ...:
        ...
    def removeAllShapeFrames(self) -> None:
        ...
    def resetAlpha(self) -> None:
        ...
    def setAlpha(self, alpha: float) -> None:
        ...
    @typing.overload
    def setDefaultAlpha(self, alpha: float) -> None:
        ...
    @typing.overload
    def setDefaultAlpha(self, alpha: float, reset: bool) -> None:
        ...
    def setEnabled(self, enabled: bool) -> None:
        ...
class RealTimeWorldNode(WorldNode):
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, world: dartpy.simulation.World) -> None:
        ...
    @typing.overload
    def __init__(self, world: dartpy.simulation.World, shadower: ...) -> None:
        ...
    @typing.overload
    def __init__(self, world: dartpy.simulation.World, shadower: ..., targetFrequency: float) -> None:
        ...
    @typing.overload
    def __init__(self, world: dartpy.simulation.World, shadower: ..., targetFrequency: float, targetRealTimeFactor: float) -> None:
        ...
    def getHighestRealTimeFactor(self) -> float:
        ...
    def getLastRealTimeFactor(self) -> float:
        ...
    def getLowestRealTimeFactor(self) -> float:
        ...
    def getTargetFrequency(self) -> float:
        ...
    def getTargetRealTimeFactor(self) -> float:
        ...
    def refresh(self) -> None:
        ...
    def setTargetFrequency(self, targetFrequency: float) -> None:
        ...
    def setTargetRealTimeFactor(self, targetRTF: float) -> None:
        ...
class ShadowMap(ShadowTechnique):
    def __init__(self) -> None:
        ...
class ShadowTechnique:
    pass
class SimpleFrameDnD(DragAndDrop):
    def __init__(self, viewer: Viewer, frame: dartpy.dynamics.SimpleFrame) -> None:
        ...
    def move(self) -> None:
        ...
    def saveState(self) -> None:
        ...
class SimpleFrameShapeDnD(SimpleFrameDnD):
    def __init__(self, viewer: Viewer, frame: dartpy.dynamics.SimpleFrame, shape: dartpy.dynamics.Shape) -> None:
        ...
    def update(self) -> None:
        ...
class Viewer(osgViewer, dartpy.common.Subject):
    class LightingMode:
        """
        Members:
        
          NO_LIGHT
        
          HEADLIGHT
        
          SKY_LIGHT
        """
        HEADLIGHT: typing.ClassVar[Viewer.LightingMode]  # value = <LightingMode.HEADLIGHT: 1>
        NO_LIGHT: typing.ClassVar[Viewer.LightingMode]  # value = <LightingMode.NO_LIGHT: 0>
        SKY_LIGHT: typing.ClassVar[Viewer.LightingMode]  # value = <LightingMode.SKY_LIGHT: 2>
        __members__: typing.ClassVar[dict[str, Viewer.LightingMode]]  # value = {'NO_LIGHT': <LightingMode.NO_LIGHT: 0>, 'HEADLIGHT': <LightingMode.HEADLIGHT: 1>, 'SKY_LIGHT': <LightingMode.SKY_LIGHT: 2>}
        def __eq__(self, other: typing.Any) -> bool:
            ...
        def __getstate__(self) -> int:
            ...
        def __hash__(self) -> int:
            ...
        def __index__(self) -> int:
            ...
        def __init__(self, value: int) -> None:
            ...
        def __int__(self) -> int:
            ...
        def __ne__(self, other: typing.Any) -> bool:
            ...
        def __repr__(self) -> str:
            ...
        def __setstate__(self, state: int) -> None:
            ...
        def __str__(self) -> str:
            ...
        @property
        def name(self) -> str:
            ...
        @property
        def value(self) -> int:
            ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, clearColor: numpy.ndarray[tuple[typing.Literal[4], typing.Literal[1]], numpy.dtype[numpy.float32]]) -> None:
        ...
    @typing.overload
    def __init__(self, clearColor: ...) -> None:
        ...
    def addAttachment(self, attachment: ...) -> None:
        ...
    def addInstructionText(self, instruction: str) -> None:
        ...
    @typing.overload
    def addWorldNode(self, newWorldNode: WorldNode) -> None:
        ...
    @typing.overload
    def addWorldNode(self, newWorldNode: WorldNode, active: bool) -> None:
        ...
    def allowSimulation(self, allow: bool) -> None:
        ...
    def captureScreen(self, filename: str) -> None:
        ...
    def checkHeadlights(self) -> bool:
        ...
    @typing.overload
    def disableDragAndDrop(self, dnd: ...) -> bool:
        ...
    @typing.overload
    def disableDragAndDrop(self, dnd: ...) -> bool:
        ...
    @typing.overload
    def disableDragAndDrop(self, dnd: ...) -> bool:
        ...
    @typing.overload
    def disableDragAndDrop(self, dnd: ...) -> bool:
        ...
    @typing.overload
    def disableDragAndDrop(self, dnd: ...) -> bool:
        ...
    @typing.overload
    def enableDragAndDrop(self, frame: InteractiveFrame) -> ...:
        ...
    @typing.overload
    def enableDragAndDrop(self, frame: dartpy.dynamics.SimpleFrame) -> ...:
        ...
    @typing.overload
    def enableDragAndDrop(self, frame: dartpy.dynamics.SimpleFrame, shape: dartpy.dynamics.Shape) -> ...:
        ...
    @typing.overload
    def enableDragAndDrop(self, bodyNode: dartpy.dynamics.BodyNode, useExternalIK: bool = True, useWholeBody: bool = False) -> ...:
        ...
    @typing.overload
    def enableDragAndDrop(self, entity: dartpy.dynamics.Entity) -> ...:
        ...
    @typing.overload
    def frame(self) -> None:
        ...
    @typing.overload
    def frame(self, arg0: float) -> None:
        ...
    def getCameraMode(self) -> ...:
        ...
    def getInstructions(self) -> str:
        ...
    def getLightingMode(self) -> ...:
        ...
    def getVerticalFieldOfView(self) -> float:
        ...
    def isAllowingSimulation(self) -> bool:
        ...
    def isRecording(self) -> bool:
        ...
    def isSimulating(self) -> bool:
        ...
    def pauseRecording(self) -> None:
        ...
    @typing.overload
    def record(self, directory: str) -> None:
        ...
    @typing.overload
    def record(self, directory: str, prefix: str) -> None:
        ...
    @typing.overload
    def record(self, directory: str, prefix: str, restart: bool) -> None:
        ...
    @typing.overload
    def record(self, directory: str, prefix: str, restart: bool, digits: int) -> None:
        ...
    def removeAttachment(self, attachment: ...) -> None:
        ...
    @typing.overload
    def removeWorldNode(self, oldWorldNode: WorldNode) -> None:
        ...
    @typing.overload
    def removeWorldNode(self, oldWorld: dartpy.simulation.World) -> None:
        ...
    def run(self) -> int:
        ...
    def setCameraHomePosition(self, arg0: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], arg1: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]], arg2: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setCameraMode(self, mode: ...) -> None:
        ...
    def setLightingMode(self, lightingMode: ...) -> None:
        ...
    def setUpViewInWindow(self, arg0: int, arg1: int, arg2: int, arg3: int) -> None:
        ...
    @typing.overload
    def setUpwardsDirection(self, up: ...) -> None:
        ...
    @typing.overload
    def setUpwardsDirection(self, up: numpy.ndarray[tuple[typing.Literal[3], typing.Literal[1]], numpy.dtype[numpy.float64]]) -> None:
        ...
    def setVerticalFieldOfView(self, fov: float) -> None:
        ...
    @typing.overload
    def setWorldNodeActive(self, node: WorldNode) -> None:
        ...
    @typing.overload
    def setWorldNodeActive(self, node: WorldNode, active: bool) -> None:
        ...
    @typing.overload
    def setWorldNodeActive(self, world: dartpy.simulation.World) -> None:
        ...
    @typing.overload
    def setWorldNodeActive(self, world: dartpy.simulation.World, active: bool) -> None:
        ...
    def setupDefaultLights(self) -> None:
        ...
    def simulate(self, on: bool) -> None:
        ...
    def switchDefaultEventHandler(self, on: bool) -> None:
        ...
    def switchHeadlights(self, on: bool) -> None:
        ...
    def updateDragAndDrops(self) -> None:
        ...
    def updateViewer(self) -> None:
        ...
class ViewerAttachment:
    def refresh(self) -> None:
        ...
class WorldNode:
    @staticmethod
    def createDefaultShadowTechnique(viewer: ...) -> ...:
        ...
    @typing.overload
    def __init__(self) -> None:
        ...
    @typing.overload
    def __init__(self, world: dartpy.simulation.World) -> None:
        ...
    @typing.overload
    def __init__(self, world: dartpy.simulation.World, shadowTechnique: ...) -> None:
        ...
    def customPostRefresh(self) -> None:
        ...
    def customPostStep(self) -> None:
        ...
    def customPreRefresh(self) -> None:
        ...
    def customPreStep(self) -> None:
        ...
    def getNumStepsPerCycle(self) -> int:
        ...
    def getShadowTechnique(self) -> ...:
        ...
    def getWorld(self) -> dartpy.simulation.World:
        ...
    def isShadowed(self) -> bool:
        ...
    def isSimulating(self) -> bool:
        ...
    def refresh(self) -> None:
        ...
    def setNumStepsPerCycle(self, steps: int) -> None:
        ...
    @typing.overload
    def setShadowTechnique(self) -> None:
        ...
    @typing.overload
    def setShadowTechnique(self, shadowTechnique: ...) -> None:
        ...
    def setWorld(self, newWorld: dartpy.simulation.World) -> None:
        ...
    def simulate(self, on: bool) -> None:
        ...
class __GUIEventHandler__:
    def __init__(self) -> None:
        ...
class osgViewer:
    def __init__(self) -> None:
        ...
    def addEventHandler(self, eventHandler: __GUIEventHandler__) -> None:
        ...
