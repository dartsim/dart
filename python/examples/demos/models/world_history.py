"""Bounded playback of owning skeleton configurations and body states."""

import operator
import sys
from collections import deque

import dartpy as dart


def _size_t(value):
    try:
        value = operator.index(value)
    except BaseException:
        # Match nanobind's unsigned conversion, which clears protocol errors.
        raise TypeError("index must be an unsigned integer") from None
    if not 0 <= value <= 2 * sys.maxsize + 1:
        raise TypeError("index must fit an unsigned size_t")
    return value


class WorldHistory:
    """Record frame zero and retain the World for subsequent playback."""

    def __init__(self, world, *, max_frames=4000):
        if world is None:
            raise TypeError("world must be a World")
        max_frames = _size_t(max_frames)
        if not issubclass(type(world), dart.simulation.World):
            raise TypeError("world must be a World")
        if max_frames == 0:
            raise ValueError("max_frames must be positive")
        self._world = world
        self._max_frames = max_frames
        self._history = deque([self._grab_time_slice()])
        self._current_index = 0
        self._frames_recorded = 1

    @property
    def world(self):
        return self._world

    @property
    def current_index(self):
        return self._current_index

    @property
    def size(self):
        return len(self._history)

    @property
    def frames_recorded(self):
        return self._frames_recorded

    @property
    def max_frames(self):
        return self._max_frames

    def _grab_time_slice(self):
        return [
            (
                skeleton.getConfiguration(),
                [body.get_composite_state() for body in skeleton.getBodyNodes()],
            )
            for skeleton in (
                self._world.getSkeleton(i)
                for i in range(self._world.getNumSkeletons())
            )
        ]

    def record(self):
        while self._current_index + 1 < len(self._history):
            self._history.pop()
        self._history.append(self._grab_time_slice())
        self._frames_recorded += 1
        self._current_index += 1
        if len(self._history) > self._max_frames:
            self._history.popleft()
            self._current_index -= 1

    def move_to(self, index):
        index = min(_size_t(index), len(self._history) - 1)
        for i, (configuration, body_states) in enumerate(self._history[index]):
            if i >= self._world.getNumSkeletons():
                break
            skeleton = self._world.getSkeleton(i)
            skeleton.setConfiguration(configuration)
            for body, state in zip(skeleton.getBodyNodes(), body_states):
                body.set_composite_state(state)
        self._current_index = index

    def move_forward(self, delta=1):
        remaining = len(self._history) - 1 - self._current_index
        self.move_to(self._current_index + min(_size_t(delta), remaining))

    def move_backward(self, delta=1):
        self.move_to(max(0, self._current_index - _size_t(delta)))
