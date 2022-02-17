#!/usr/bin/env python3

from abc import ABC, abstractmethod


class ActionCallbackPack(ABC):

    @abstractmethod
    def doneCallback(self, state, result): ...

    def activeCallback(self): ...

    def feedbackCallback(self, feedback): ...
