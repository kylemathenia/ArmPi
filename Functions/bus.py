"""Contains a class for a communication bus to pass messages."""

from readerwriterlock import rwlock
class Bus(object):
    def __init__(self):
        self.message = None
        self.lock = rwlock.RWLockWriteD()

    def write(self,msg):
        with self.lock.gen_wlock():
            self.message = msg

    def read(self):
        with self.lock.gen_rlock():
            return self.message