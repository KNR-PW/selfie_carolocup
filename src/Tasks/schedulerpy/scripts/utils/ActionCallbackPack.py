#!/usr/bin/env python3

from abc import ABC, abstractmethod


class ActionCallbackPack(ABC):

    @abstractmethod
    def doneCallback(self, state, result):
        pass

    def activeCallback(self):
        pass

    def feedbackCallback(self, feedback):
        pass
