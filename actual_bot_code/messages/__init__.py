from enum import Enum
from .pacmanState_pb2 import PacmanState
from .lightState_pb2 import LightState
from .pacmanDirection_pb2 import PacmanDirection

class MsgType(Enum):
    LIGHT_STATE = 0
    PACMAN_LOCATION = 1
    FULL_STATE = 2
    PACMAN_DIRECTION = 3

message_buffers = {
    MsgType.FULL_STATE: PacmanState,
    MsgType.PACMAN_LOCATION: PacmanState.AgentState,
    MsgType.LIGHT_STATE: LightState,
    MsgType.PACMAN_DIRECTION: PacmanDirection
}


__all__ = ['MsgType', 'message_buffers', 'PacmanState', 'LightState', 'PacmanDirection']
