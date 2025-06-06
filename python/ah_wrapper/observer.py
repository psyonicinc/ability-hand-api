from abc import ABC, abstractmethod


class Observable:
    def __init__(self):
        self.observers = []

    def add_observer(self, observer):
        if observer not in self.observers:
            self.observers.append(observer)

    def remove_observer(self, observer):
        if observer in self.observers:
            self.observers.remove(observer)

    def notify_observers(self, *args, **kwargs):
        for observer in self.observers:
            observer.update(self, *args, **kwargs)

    def notify_pos(self, position):
        for observer in self.observers:
            observer.update_pos(position)

    def notify_vel(self, velocity):
        for observer in self.observers:
            observer.update_vel(velocity)

    def notify_cur(self, current):
        for observer in self.observers:
            observer.update_cur(current)

    def notify_fsr(self, fsr):
        for observer in self.observers:
            observer.update_fsr(fsr)

    def notify_hot_cold(self, hot_cold):
        for observer in self.observers:
            observer.update_hot_cold(hot_cold)


class Observer(ABC):
    def update(self, observable, *args, **kwargs):
        pass

    @abstractmethod
    def update_pos(self, position):
        pass

    @abstractmethod
    def update_vel(self, velocity):
        pass

    @abstractmethod
    def update_cur(self, current):
        pass

    @abstractmethod
    def update_fsr(self, fsr):
        pass

    @abstractmethod
    def update_hot_cold(self, hot_cold):
        pass
