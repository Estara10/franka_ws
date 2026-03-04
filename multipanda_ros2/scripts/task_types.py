#!/usr/bin/env python3

#状态枚举
from enum import Enum


class TaskState(Enum):
    INIT_ENVIRONMENT = 1
    PLAN_APPROACH = 2
    EXECUTE_APPROACH = 3
    CLOSE_GRIPPERS = 4
    PLAN_SYNC_TRAJECTORY = 5
    EXECUTE_WITH_COMPLIANCE = 6
    OPEN_GRIPPERS = 7
    RETURN_TO_HOME = 8
    FINISHED = 9
    ERROR = 99
